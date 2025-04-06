#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"


#define SCL_IO_PIN CONFIG_I2C_MASTER_SCL
#define SDA_IO_PIN CONFIG_I2C_MASTER_SDA
#define MASTER_FREQUENCY 100000
#define SLAVE_ADRESS_IMU 0x68
#define SLAVE_ADRESS_TEMPERATURE 0x55 // TODO

#define WINDOW_SIZE 3

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    
    //int16_t gyro_x;
    //int16_t gyro_y;
    //int16_t gyro_z;
} Mpu6050;

typedef struct {
    uint16_t temperature;
    Mpu6050 imu;
} Borda_package;

// Veri okuma taskı için context
typedef struct {
    i2c_master_dev_handle_t dev_handle_temperature;
    i2c_master_dev_handle_t dev_handle_imu;
    QueueHandle_t rawDataQueue;
} SensorConxtext;

// Filtre taskı için context
typedef struct {
    QueueHandle_t filteredDataQueue;
    QueueHandle_t rawDataQueue;
} FilterContext;


int compare(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

void debug_i2c(Borda_package borda) {
    printf("Temperature: %hu\n", borda.temperature);
    printf("IMU: \n\tacc_x = %u\nacc_y = %u\nacc_z = %u", borda.imu.acc_x, borda.imu.acc_y, borda.imu.acc_z);
}


//////////////////////////////////////////////////
// I2C Slave cihazları başlatma ve okuma işlemleri
//////////////////////////////////////////////////

/**
 * @brief MPU6050 sensörünü başlatır (uyandırır).
 */
void mpu_init(SensorConxtext* ctx) {
    uint8_t reg1 = 0x6B;
    uint8_t reg2 = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(ctx->dev_handle_imu, &reg1, 1, 1000));
    ESP_ERROR_CHECK(i2c_master_transmit(ctx->dev_handle_imu, &reg2, 1, 1000));
}

/**
 * @brief I2C slavelerden(Arduino ve Mpu6050) veri okuyup Queue'ya gönderir.
 */
void read_sensor_data(void* pvParameters) {
    SensorConxtext* s_context = (SensorConxtext*) pvParameters;
    
    uint8_t imu_reg = 0x3B;

    while(1) {
        uint8_t temperature_buffer[2] = {0};
        uint8_t imu_buffer[6] = {0}; //acc_x, acc_y and acc_z = 6 bytes

        // Sıcaklık okuma (arduino)
        ESP_ERROR_CHECK(i2c_master_receive(s_context->dev_handle_temperature, temperature_buffer, sizeof(temperature_buffer), 1000));
        uint16_t temperature = temperature_buffer[0] << 8 | temperature_buffer[1]; 

        // İvmeölçer okuma (MPU6050, acc_x, acc_y, acc_z)
        ESP_ERROR_CHECK(i2c_master_transmit(s_context->dev_handle_imu, &imu_reg, 1, 1000));
        ESP_ERROR_CHECK(i2c_master_receive(s_context->dev_handle_imu, imu_buffer, sizeof(imu_buffer), 1000));

        uint16_t acc_x = (imu_buffer[0] << 8) | imu_buffer[1];
        uint16_t acc_y = (imu_buffer[2] << 8) | imu_buffer[3];
        uint16_t acc_z = (imu_buffer[4] << 8) | imu_buffer[5];

        // Verilerin struct'a yerleştirilmesi
        Borda_package borda = {
            .temperature = temperature,
            .imu = {
                .acc_x = acc_x,
                .acc_y = acc_y,
                .acc_z = acc_z
            }
        };

        xQueueSend(s_context->rawDataQueue, &borda, portMAX_DELAY);

        // Periyot
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

//////////////////////////////////////////////////
// Filtre işlemi
//////////////////////////////////////////////////

/**
 * @brief Bir pencere içerisindeki sıcaklık değerinin medianını hesaplar.
 */
int temperature_median(uint16_t window[3]) {
    uint16_t temperature[] = {window[0], window[1], window[2]};
    qsort(temperature, sizeof(temperature) / sizeof(uint16_t), sizeof(uint16_t), compare);
    return temperature[1];
}

/**
 * @brief Raw data kuyruğundan(rawDataQueue) gelen verileri alır, pencere üzerinden filtre uygular
 *        ve filtrelenmiş veri kuyruğuna(filteredDataQueue) gönderir.
 */
void filter_data(void* pvParameters) {

    FilterContext* f_context = (FilterContext*) pvParameters;

    Borda_package window[WINDOW_SIZE] = {0};
    Borda_package rawData = {0};
    Borda_package filteredData = {0};
    uint32_t count = 0;
    uint32_t idx = 1;
    

    while (1) {
        if(xQueueReceive(f_context->rawDataQueue, &rawData, portMAX_DELAY) == pdTRUE) {

            if(count < 1) {
                window[idx] = rawData;
                count++;
                idx++;
                continue;
            }
            window[idx] = rawData;
            count++;
            idx = (idx + 1) % 3;

            filteredData.imu = rawData.imu;
            uint16_t temperature_window[3] = {window[0].temperature, window[1].temperature, window[2].temperature};
            filteredData.temperature = temperature_median(temperature_window);

            xQueueSend(f_context->filteredDataQueue, &filteredData, portMAX_DELAY);
            debug_i2c(filteredData);
        }
    }

}


void app_main(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    // I2c slave arduino
    i2c_device_config_t dev_cfg_temperature = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SLAVE_ADRESS_TEMPERATURE,
        .scl_speed_hz = MASTER_FREQUENCY
    };

    static i2c_master_dev_handle_t dev_handle_temperature;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_temperature, &dev_handle_temperature));

    //i2c slave MPU6050
    i2c_device_config_t dev_cfg_imu = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SLAVE_ADRESS_IMU,
        .scl_speed_hz = MASTER_FREQUENCY
    };

    static i2c_master_dev_handle_t dev_handle_imu;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_imu, &dev_handle_imu));
    /////////////////////////////////////////////////////////////////////////////// 
    static QueueHandle_t rawDataQueue;
    static QueueHandle_t filteredDataQueue;

    rawDataQueue = xQueueCreate(10, sizeof(Borda_package));
    filteredDataQueue = xQueueCreate(10, sizeof(Borda_package));

    static SensorConxtext s_context = {0};
    s_context.dev_handle_temperature = dev_handle_temperature;
    s_context.dev_handle_imu = dev_handle_imu;
    s_context.rawDataQueue = rawDataQueue;

    static FilterContext f_context = {0};
    f_context.filteredDataQueue = filteredDataQueue;
    f_context.rawDataQueue = rawDataQueue;

    mpu_init(&s_context);
   

    xTaskCreate(read_sensor_data, "get_sensor_data", 4096, (void*) &s_context, tskIDLE_PRIORITY, NULL);
    xTaskCreate(filter_data, "filter_data", 4096, (void*) &f_context, tskIDLE_PRIORITY, NULL);
    // TODO Bluetooth broadcast taskı 
}   


// TODO BLUETOOTH taskı yap
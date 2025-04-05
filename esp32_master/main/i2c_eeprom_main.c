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
#define SLAVE_ADRESS 0x55

#define STACK_SIZE 128

typedef struct {
    i2c_master_dev_handle_t temp_sensor;
    i2c_master_dev_handle_t imu_sensor;
    QueueHandle_t rawDataQueue;
} I2c_sensor_context;

typedef struct {
    QueueHandle_t filteredDataQueue;
} I2c_filter_context;

typedef struct {
    uint16_t temp;
    uint32_t imu;
} Borda_package;

// Producer
void get_sensor_data();
// Consumer
void filter_data();

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

    i2c_device_config_t dev_cfg_temp = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SLAVE_ADRESS,
        .scl_speed_hz = MASTER_FREQUENCY
    };

    i2c_master_dev_handle_t dev_handle_temp;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_temp, &dev_handle_temp));

    i2c_device_config_t dev_cfg_imu = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SLAVE_ADRESS, // TODO
        .scl_speed_hz = MASTER_FREQUENCY
    };

    i2c_master_dev_handle_t dev_handle_imu;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_imu, &dev_handle_imu));
    /////////////////////////////////////////////////////////////////////////////// 

    static I2c_sensor_context s_context = {0};
    s_context.temp_sensor = dev_handle_temp;
    s_context.imu_sensor = dev_handle_imu;
    s_context.rawDataQueue = xQueueCreate(10, sizeof(Borda_package));

    static I2c_filter_context f_context = {0};
    f_context.filteredDataQueue = xQueueCreate(10, sizeof(Borda_package));

    
    xTaskCreate(get_sensor_data, "get_sensor_data", 1024, (void*) &s_context, tskIDLE_PRIORITY, NULL);
    xTaskCreate(filter_data, "filter_data", 1024, (void*) &f_context, tskIDLE_PRIORITY, NULL);
}   

void get_sensor_data(void* param) {
    I2c_sensor_context* s_context = (I2c_sensor_context*) param;
    
    while(1) {
        uint8_t temp_buffer[2] = {0};
        uint8_t imu_buffer[3] = {0};

        ESP_ERROR_CHECK(i2c_master_receive(s_context->temp_sensor, temp_buffer, sizeof(temp_buffer), 1000));
        uint16_t temp = temp_buffer[0] << 8 | temp_buffer[1]; 

        ESP_ERROR_CHECK(i2c_master_receive(s_context->imu_sensor, imu_buffer, sizeof(imu_buffer), 1000));
        uint32_t imu = 0; // TODO

        Borda_package borda = {
            .temp = temp,
            .imu = imu
        };

        xQueueSend(s_context->rawDataQueue, &borda, portMAX_DELAY);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void filter_data(void* param) {

}
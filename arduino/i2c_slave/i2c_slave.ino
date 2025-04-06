#include <Wire.h>

#define SLAVE_ADRESS 0x55

byte fakeTemp[2] = {0};

int generateTemp() {
  int baseTemp = 25.0;
  int noise = random(-50, 50);

  return baseTemp + noise;
}

void I2C_Handler(void) {
  Wire.write(fakeTemp, 2);
}

void setup() {
  Wire.begin(SLAVE_ADRESS);
  Wire.onRequest(I2C_Handler);
}

void loop() {
  int temp = 0;
  int chance = random(100);
  if(chance < )
    temp = generateTemp();
  else
    temp = random(-500, 1000)
   
  fakeTemp[0] = (temp >> 8) & 0xFF;
  fakeTemp[1] = temp & 0xFF;
  
  delay(200);
  }

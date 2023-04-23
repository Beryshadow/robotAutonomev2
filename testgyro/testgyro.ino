#include <DFRobot_ICG20660L.h>
DFRobot_ICG20660L_IIC icg(/*addr=*/IIC_ADDR_SDO_H, &Wire);
float DPS = 3.1415926 / 180.0;

#ifdef ARDUINO_BBC_MICROBIT
#define CS_PIN 8 // The CS pin of sensor which is connected to the 8 digital io pin of micro:bit,and also can connected to other pin.
#else
#define CS_PIN 5 // The CS pin of sensor which is connected to the 5 digital io pin of MCU,and also can connected to other pin.
#endif

void setup() {
  Serial.begin(115200);
  while (icg.begin(/*mode=*/icg.eRegMode) != 0)
    {
        Serial.println("failed. Please check whether the hardware connection is wrong.");
        delay(1000);
        Serial.print("Initialization sensor...");
    }
    Serial.println("done.");
    Serial.print("ICG20660L Device ID: 0x");
    Serial.println(icg.readID(), HEX);
    icg.enableSensor(icg.eGyroAxisXYZ);
    icg.configGyro(icg.eFSR_G_250DPS, icg.eGyro_DLPF_8173_32KHZ);
    icg.setSampleDiv(19);
}

void loop() {
  Serial.println(icg.getGyroDataX());
  delay(100);
}
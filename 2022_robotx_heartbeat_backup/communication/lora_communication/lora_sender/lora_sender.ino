#include <SPI.h>
#include <LoRa.h>

#define SERIAL_RATE 115200;
#define LORA_RATE 915E6;

void setup() {
    Serial.begin(SERIAL_RATE);
    while (!Serial);

    Serial.println("LoRa Sender");

    if (!LoRa.begin(LORA_RATE)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
}

void loop() {
    while(Serial.available()){
        String cmd = Serial.readString();
        // send packet
        LoRa.beginPacket();
        LoRa.print(cmd);
        LoRa.endPacket();
    }
}

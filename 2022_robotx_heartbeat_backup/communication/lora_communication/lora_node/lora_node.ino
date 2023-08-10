#include <SPI.h>
#include <LoRa.h>

#define SERIAL_RATE 115200
#define LORA_RATE 915E6
#define LORA_BANDWIDTH 62E3


void setup() {
    Serial.begin(SERIAL_RATE);
    while (!Serial);

    Serial.println("LoRa Sender");

    if (!LoRa.begin(LORA_RATE)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    //LoRa.setSignalBandwidth(LORA_BANDWIDTH);
     // register the receive callback
    LoRa.onReceive(onReceive);

    // put the radio into receive mode
    LoRa.receive();
}

void onReceive(int packetSize) {
    String cmd;
    // read packet
    for (int i = 0; i < packetSize; i++) {
        cmd += (char)LoRa.read();
    }

    Serial.print(cmd);
}

void loop() {
    while(Serial.available()){
        String cmd = Serial.readString();
        // send packet
        LoRa.beginPacket();
        LoRa.print(cmd);
        LoRa.endPacket();
        LoRa.receive();
    }
}

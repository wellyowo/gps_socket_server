#include <SPI.h>
#include <LoRa.h>

#define SERIAL_RATE 115200;
#define LORA_RATE 915E6;

void setup() {
    Serial.begin(SERIAL_RATE);
    while (!Serial);

    Serial.println("LoRa Receiver Callback");

    if (!LoRa.begin(LORA_RATE)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    // register the receive callback
    LoRa.onReceive(onReceive);

    // put the radio into receive mode
    LoRa.receive();
}

void loop() {
  // do nothing
}

void onReceive(int packetSize) {
    String cmd;
    // read packet
    for (int i = 0; i < packetSize; i++) {
        cmd += (char)LoRa.read();
    }

    Serial.println(cmd);

}

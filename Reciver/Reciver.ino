/*
 * FS-CT6B to ESP32 Receiver V2.1 (IBUS ONLY)
 * Optimized for Betaflight/iNav/Ardupilot
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <esp_now.h>
#include <WiFi.h>

/* ================== CONFIGURATION ================== */
#define USE_LORA          false   // Set to true for LoRa, false for ESP-NOW
#define FAILSAFE_TIMEOUT  200     // 0.2 Seconds signal loss triggers failsafe

// Pin Definitions
#define PIN_IBUS_TX       17      // Connect this to RX pin on Flight Controller
#define SCK_PIN 5
#define MISO_PIN 19
#define MOSI_PIN 27
#define SS_PIN 18
#define RST_PIN 14
#define DIO0_PIN 26

/* ================== DATA STRUCT ================== */
typedef struct {
  uint16_t ch1; uint16_t ch2; uint16_t ch3; uint16_t ch4;
  uint16_t ch5; uint16_t ch6;
  uint8_t  swC; uint8_t  swD;
  uint16_t checksum;
} RCPacket;

RCPacket incomingData;
unsigned long lastRecvTime = 0;
bool isFailsafe = true;

/* ================== IBUS PROTOCOL SENDER ================== */
void sendIBUS() {
  uint8_t ibusPacket[32];
  
  // Header
  ibusPacket[0] = 0x20;
  ibusPacket[1] = 0x40;

  // Channel Mapping (iBUS uses 14 channels, 2 bytes each, Little Endian)
  uint16_t channels[14];
  channels[0] = incomingData.ch1;
  channels[1] = incomingData.ch2;
  channels[2] = incomingData.ch3;
  channels[3] = incomingData.ch4;
  channels[4] = incomingData.ch5;
  channels[5] = incomingData.ch6;
  channels[6] = incomingData.swC ? 2000 : 1000;
  channels[7] = incomingData.swD ? 2000 : 1000;
  
  // Fill remaining channels with neutral 1500
  for(int i=8; i<14; i++) channels[i] = 1500;

  // Load channels into packet
  for (int i = 0; i < 14; i++) {
    ibusPacket[2 + i * 2] = channels[i] & 0xFF;         // Low byte
    ibusPacket[3 + i * 2] = (channels[i] >> 8) & 0xFF;  // High byte
  }

  // Calculate Checksum: 0xFFFF - sum of first 30 bytes
  uint16_t chksum = 0xFFFF;
  for (int i = 0; i < 30; i++) {
    chksum -= ibusPacket[i];
  }

  // Append Checksum (Little Endian)
  ibusPacket[30] = chksum & 0xFF;
  ibusPacket[31] = (chksum >> 8) & 0xFF;

  // Send to Flight Controller
  Serial1.write(ibusPacket, 32);
}

/* ================== CALLBACK (CORE 3.X FIX) ================== */
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incoming, int len) {
  if (len == sizeof(RCPacket)) {
    memcpy(&incomingData, incoming, sizeof(RCPacket));
    
    // Checksum verification from transmitter
    uint16_t calc = incomingData.ch1 ^ incomingData.ch2 ^ incomingData.ch3 ^ incomingData.ch4;
    if(calc == incomingData.checksum) {
      lastRecvTime = millis();
      isFailsafe = false;
    }
  }
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200); // Debug to PC
  
  // Initialize Serial1 for iBUS (115200 Baud, 8N1)
  // Format: Serial1.begin(baud, config, RX_PIN, TX_PIN)
  Serial1.begin(115200, SERIAL_8N1, -1, PIN_IBUS_TX);

  if (USE_LORA) {
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
    if (!LoRa.begin(433E6)) {
      Serial.println("LoRa Init Failed!");
      while (1);
    }
    Serial.println("LoRa Mode Active");
  } else {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW Init Failed!");
      while (1);
    }
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW Mode Active");
  }
}

/* ================== LOOP ================== */
void loop() {
  // 1. Handle LoRa packets if enabled
  if (USE_LORA) {
    int packetSize = LoRa.parsePacket();
    if (packetSize == sizeof(RCPacket)) {
      LoRa.readBytes((uint8_t *)&incomingData, sizeof(RCPacket));
      lastRecvTime = millis();
      isFailsafe = false;
    }
  }

  // 2. Failsafe Check
  if (millis() - lastRecvTime > FAILSAFE_TIMEOUT) {
    if (!isFailsafe) {
      Serial.println("FAILSAFE: No data from transmitter!");
      isFailsafe = true;
    }
    // We stop sending Serial1 data entirely during failsafe.
    // This forces Betaflight to enter "RX_LOSS" mode.
  } else {
    // 3. Send iBUS data to Flight Controller
    sendIBUS();
  }

  // 4. Debug to Serial Monitor (Every 200ms)
  static unsigned long lastDebug = 0;
  if(millis() - lastDebug > 200) {
    if(isFailsafe) {
      Serial.println("Searching for Transmitter...");
    } else {
      Serial.printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d\n", 
                    incomingData.ch1, incomingData.ch2, 
                    incomingData.ch3, incomingData.ch4);
    }
    lastDebug = millis();
  }
  
  // iBUS usually sends every 7ms to 10ms. Delay to prevent flooding.
  delay(7); 
}
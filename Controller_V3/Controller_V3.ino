/*
 * FS-CT6B to ESP32 Transmitter Upgrade (FINAL)
 * Features:
 * - LoRa (433MHz)
 * - ESP-NOW (Max TX Power)
 * - Fancy Web UI
 * - Web-based Calibration
 * - Web-controlled Packet Rate (Hz)
 * - Smoothing + Persistent Settings
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp_wifi.h"

/* ================== PIN DEFINITIONS ================== */
// ADC1 (WiFi-safe)
#define PIN_THROTTLE 36
#define PIN_YAW      39
#define PIN_ROLL     32
#define PIN_PITCH    33
#define PIN_VR_A     34
#define PIN_VR_B     35

// Digital
#define PIN_SW_C     17
#define PIN_SW_D     16

// LoRa SX1278
#define SCK_PIN      5
#define MISO_PIN     19
#define MOSI_PIN     27
#define SS_PIN       18
#define RST_PIN      14
#define DIO0_PIN     26

/* ================== LORA SETTINGS ================== */
#define LORA_FREQUENCY 433E6
#define LORA_SF        7
#define LORA_TX_POWER  20
#define LORA_BANDWIDTH 125E3

/* ================== FILTER ================== */
#define FILTER_ALPHA 0.1

/* ================== DATA STRUCT ================== */
typedef struct {
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint16_t ch4;
  uint16_t ch5;
  uint16_t ch6;
  uint8_t  swC;
  uint8_t  swD;
  uint16_t checksum;
} RCPacket;

RCPacket myData;

/* ================== CALIBRATION ================== */
struct CalibData {
  int minVal[6];
  int maxVal[6];
  bool isCalibrated;
};

CalibData calibration = {
  {0,0,0,0,0,0},
  {4095,4095,4095,4095,4095,4095},
  false
};

/* ================== GLOBALS ================== */
Preferences preferences;
WebServer server(80);

int rawValues[6];
float smoothedValues[6];
bool calibrationMode = false;

uint16_t controlHz = 50;
unsigned long packetInterval = 20;

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

const int inputPins[6] = {
  PIN_ROLL, PIN_PITCH, PIN_THROTTLE,
  PIN_YAW, PIN_VR_A, PIN_VR_B
};

const char* channelNames[6] = {
  "Roll","Pitch","Throttle","Yaw","Knob A","Knob B"
};

/* ================== HTML ================== */
const char* htmlHeader = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RC Transmitter</title>
<style>
body{background:#0f172a;font-family:sans-serif;color:#e5e7eb;text-align:center;padding:20px}
.card{background:#020617;border-radius:14px;padding:16px;margin:12px auto;max-width:420px}
button{width:100%;padding:14px;font-size:18px;border-radius:10px;border:none;margin-top:10px}
.start{background:#22c55e}.stop{background:#ef4444}
.slider{width:100%}
.bar{height:14px;background:#1e293b;border-radius:8px;overflow:hidden}
.fill{height:100%;background:linear-gradient(90deg,#22d3ee,#38bdf8)}
small{color:#94a3b8}
h1{color:#38bdf8}
</style></head><body>
<h1>RC Transmitter</h1>
)rawliteral";

/* ================== WEB HANDLERS ================== */
void handleRoot() {
  String s = htmlHeader;

  s += "<div class='card'><h3>Status</h3>";
  s += calibrationMode ? "<p style='color:#facc15'>CALIBRATING</p>"
                       : "<p style='color:#22c55e'>NORMAL</p>";
  s += "</div>";

  s += "<div class='card'><h3>Calibration</h3>";
  s += calibrationMode
       ? "<a href='/stop_cal'><button class='stop'>Finish & Save</button></a>"
       : "<a href='/start_cal'><button class='start'>Start Calibration</button></a>";
  s += "</div>";

  s += "<div class='card'><h3>Control Rate</h3>";
  s += "<input type='range' min='10' max='200' value='" + String(controlHz) + "' class='slider' id='hz'>";
  s += "<p><span id='hzVal'>" + String(controlHz) + "</span> Hz</p></div>";

  s += "<div class='card'><h3>Live Channels</h3><div id='data'></div></div>";

  s += R"rawliteral(
<script>
const hz=document.getElementById('hz');
const hv=document.getElementById('hzVal');
hz.oninput=()=>{hv.innerText=hz.value;fetch('/set_hz?val='+hz.value);}
setInterval(()=>{fetch('/data').then(r=>r.text()).then(t=>data.innerHTML=t)},150);
</script></body></html>
)rawliteral";

  server.send(200,"text/html",s);
}

void handleStartCal() {
  calibrationMode = true;
  for(int i=0;i<6;i++){ calibration.minVal[i]=4095; calibration.maxVal[i]=0; }
  server.sendHeader("Location","/");
  server.send(303);
}

void handleStopCal() {
  calibrationMode=false;
  calibration.isCalibrated=true;
  preferences.begin("rc",false);
  preferences.putBytes("cal",&calibration,sizeof(calibration));
  preferences.end();
  server.sendHeader("Location","/");
  server.send(303);
}

void handleSetHz() {
  if(server.hasArg("val")){
    controlHz=constrain(server.arg("val").toInt(),10,200);
    packetInterval=1000/controlHz;
    preferences.begin("rc",false);
    preferences.putUShort("hz",controlHz);
    preferences.end();
  }
  server.send(200,"text/plain","OK");
}

void handleData() {
  String d="";
  uint16_t ch[6]={myData.ch1,myData.ch2,myData.ch3,myData.ch4,myData.ch5,myData.ch6};
  for(int i=0;i<6;i++){
    int p=map(ch[i],1000,2000,0,100);
    d+=channelNames[i];
    d+="<div class='bar'><div class='fill' style='width:"+String(p)+"%'></div></div>";
    d+="<small>"+String(ch[i])+"</small><br>";
  }
  server.send(200,"text/html",d);
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  pinMode(PIN_SW_C,INPUT_PULLUP);
  pinMode(PIN_SW_D,INPUT_PULLUP);

  preferences.begin("rc",true);
  if(preferences.isKey("cal"))
    preferences.getBytes("cal",&calibration,sizeof(calibration));
  if(preferences.isKey("hz")){
    controlHz=preferences.getUShort("hz",50);
    packetInterval=1000/controlHz;
  }
  preferences.end();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("RC_Transmitter_Setup","12345678");
  esp_wifi_set_max_tx_power(80);
  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_now_init();
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr,broadcastAddress,6);
  esp_now_add_peer(&peer);

  server.on("/",handleRoot);
  server.on("/start_cal",handleStartCal);
  server.on("/stop_cal",handleStopCal);
  server.on("/set_hz",handleSetHz);
  server.on("/data",handleData);
  server.begin();

  SPI.begin(SCK_PIN,MISO_PIN,MOSI_PIN,SS_PIN);
  LoRa.setPins(SS_PIN,RST_PIN,DIO0_PIN);
  LoRa.begin(LORA_FREQUENCY);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
}

/* ================== LOOP ================== */
void loop() {
  server.handleClient();

  for(int i=0;i<6;i++) rawValues[i]=analogRead(inputPins[i]);

  if(calibrationMode){
    for(int i=0;i<6;i++){
      calibration.minVal[i]=min(calibration.minVal[i],rawValues[i]);
      calibration.maxVal[i]=max(calibration.maxVal[i],rawValues[i]);
    }
  }

  for(int i=0;i<6;i++){
    smoothedValues[i]=(FILTER_ALPHA*rawValues[i])+((1-FILTER_ALPHA)*smoothedValues[i]);
    int v=constrain(smoothedValues[i],calibration.minVal[i],calibration.maxVal[i]);

    int m;
    if(i == 0) {
      // 🔁 Roll channel inverted
      m = map(v, calibration.minVal[i], calibration.maxVal[i], 2000, 1000);
    } else {
      m = map(v, calibration.minVal[i], calibration.maxVal[i], 1000, 2000);
    }

    (&myData.ch1)[i] = m;
  }

  myData.swC=!digitalRead(PIN_SW_C);
  myData.swD=!digitalRead(PIN_SW_D);
  myData.checksum=myData.ch1^myData.ch2^myData.ch3^myData.ch4;

  static unsigned long lastSend=0;
  if(millis()-lastSend>=packetInterval){
    esp_now_send(broadcastAddress,(uint8_t*)&myData,sizeof(myData));
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&myData,sizeof(myData));
    LoRa.endPacket();
    lastSend=millis();
  }
}

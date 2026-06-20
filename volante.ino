#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <GP94.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <HTTPClient.h>

#define RXD2 16
#define TXD2 17

HardwareSerial mySerial(2);
GP9 imu(mySerial);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* ssid     = "gateway_esteban";
const char* password = "shaman2026";

IPAddress gatewayIP(192, 168, 4, 1);
const uint16_t gatewayPort  = 5000;
const uint16_t localUdpPort = 4000;

WiFiUDP udp;

const int selectPin0 = 33;
const int selectPin1 = 25;
const int selectPin2 = 26;
const int zInput     = 32;

const unsigned long OLED_UPDATE_MS       = 750;
const unsigned int  PRINT_EVERY_N        = 0;
const unsigned int  SEND_EVERY_N         = 20;
const uint16_t      TX_QUEUE_LEN         = 64;
const size_t        UDP_PACKET_MAX_BYTES = 230;
const uint8_t       UDP_BATCH_MAX_LINES  = 2;
const unsigned long IMU_STALE_MS         = 1000;

unsigned long lastOledUpdate = 0;
bool oledOk = false;

unsigned long frameCount  = 0;
uint32_t      seqCounter  = 0;
unsigned long imuPacketCount = 0;
unsigned long lastImuPacketMs = 0;
bool imuAlive = false;

volatile unsigned long txQueued     = 0;
volatile unsigned long txSent       = 0;
volatile unsigned long txQueueDrop  = 0;
volatile unsigned long txUdpFail    = 0;
volatile unsigned long txPacketSent = 0;

struct TxMessage {
  char line[220];
};

QueueHandle_t txQueue = nullptr;
bool grabar = false;
uint32_t prueba_id = 0;

// Snapshot global de IMU actualizado SOLO dentro de imu.decode()
float imu_gyro_x = 0.0f;
float imu_accel_x = 0.0f;
int16_t imu_quat_a = 0;
int16_t imu_quat_b = 0;
int16_t imu_quat_c = 0;
int16_t imu_quat_d = 0;

void checkRecordingStatus() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 2000) return;
  lastCheck = millis();

  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin("http://192.168.4.1/api/check-recording.php");
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    if (payload.indexOf("grabando") > -1) {
      int grabPos = payload.indexOf("grabando");
      int colonPos = payload.indexOf(":", grabPos);
      int coma = payload.indexOf(",", colonPos);
      String grabStr = payload.substring(colonPos + 1, coma);
      grabStr.trim();
      grabar = (grabStr == "1");

      if (grabar) {
        int pidPos = payload.indexOf("prueba_id");
        int pidColon = payload.indexOf(":", pidPos);
        int pidEnd = payload.indexOf("}", pidColon);
        String pidStr = payload.substring(pidColon + 1, pidEnd);
        pidStr.trim();
        prueba_id = pidStr.toInt();
      }
    }
  }
  http.end();
}

void refreshImuAlive() {
  if (imuAlive && (millis() - lastImuPacketMs > IMU_STALE_MS)) {
    imuAlive = false;
  }
}

void updateOledStatus() {
  const unsigned long now = millis();
  if (!oledOk) return;
  if (now - lastOledUpdate < OLED_UPDATE_MS) return;

  static unsigned long lastFrameCount = 0;
  static unsigned long lastHzTime     = 0;
  static unsigned long imuHz          = 0;

  unsigned long elapsed = now - lastHzTime;
  if (elapsed >= OLED_UPDATE_MS) {
    unsigned long newFrames = frameCount - lastFrameCount;
    if (elapsed > 0) imuHz = (newFrames * 1000UL) / elapsed;
    lastFrameCount = frameCount;
    lastHzTime     = now;
  }

  lastOledUpdate = now;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.println("VOLANTE UDP");
  display.print("WiFi: ");
  display.println(WiFi.status() == WL_CONNECTED ? "OK" : "NO");
  display.print("GP9: ");
  display.println(imuAlive ? "OK" : "NO");
  display.print("Hz: ");
  display.println(imuHz);
  display.print("Pkts: ");
  display.println(imuPacketCount);
  display.print("REC: ");
  display.println(grabar ? "SI" : "NO");
  display.print("Sent: ");
  display.println(txSent);
  display.print("QDrop: ");
  display.println(txQueueDrop);

  display.display();
}

void udpTask(void *parameter) {
  (void)parameter;
  char packet[UDP_PACKET_MAX_BYTES + 1];
  unsigned long lastHb = 0;

  for (;;) {
    unsigned long nowMs = millis();
    if (nowMs - lastHb >= 1000 && WiFi.status() == WL_CONNECTED) {
      lastHb = nowMs;
      char hb[64];
      int hblen = snprintf(hb, sizeof(hb), "H,V,%d,%lu\n", imuAlive ? 1 : 0, imuPacketCount);
      if (udp.beginPacket(gatewayIP, gatewayPort)) {
        udp.write((const uint8_t*)hb, hblen);
        udp.endPacket();
      }
    }

    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    TxMessage msg;
    if (xQueueReceive(txQueue, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    size_t  used  = 0;
    uint8_t lines = 0;

    while (true) {
      size_t len = strnlen(msg.line, sizeof(msg.line));

      if (len == 0 || len >= sizeof(msg.line)) {
        txUdpFail++;
      } else if (used + len <= UDP_PACKET_MAX_BYTES) {
        memcpy(packet + used, msg.line, len);
        used += len;
        lines++;
      } else {
        break;
      }

      if (lines >= UDP_BATCH_MAX_LINES) break;
      if (xQueueReceive(txQueue, &msg, 0) != pdTRUE) break;
    }

    if (used == 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (udp.beginPacket(gatewayIP, gatewayPort) == 1) {
      size_t written = udp.write((const uint8_t*)packet, used);
      int    ended   = udp.endPacket();

      if (written == used && ended == 1) {
        txSent += lines;
        txPacketSent++;
      } else {
        txUdpFail++;
      }
    } else {
      txUdpFail++;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("VOLANTE UDP");
  Serial.println("VOLANTE CODEX v1");

  mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(selectPin0, OUTPUT);
  pinMode(selectPin1, OUTPUT);
  pinMode(selectPin2, OUTPUT);
  pinMode(zInput, INPUT);

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if (oledOk) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("VOLANTE UDP");
    display.println("Conectando...");
    display.display();
  }

  txQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(TxMessage));
  if (txQueue == nullptr) {
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid, password);

  Serial.print("Conectando a WiFi gateway");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi OK. IP volante: ");
  Serial.println(WiFi.localIP());

  udp.begin(localUdpPort);

  xTaskCreatePinnedToCore(udpTask, "udpTask", 4096, nullptr, 1, nullptr, 0);
}

bool pollImuNonBlocking() {
  bool gotPacket = false;

  while (mySerial.available() > 0) {
    const int raw = mySerial.read();
    if (raw < 0) break;

    if (imu.decode((byte)raw)) {
      gotPacket = true;
      imuPacketCount++;
      lastImuPacketMs = millis();
      imuAlive = true;

      imu_gyro_x = imu.gyro_x;
      imu_accel_x = imu.accel_x;
      imu_quat_a = imu.quat_a;
      imu_quat_b = imu.quat_b;
      imu_quat_c = imu.quat_c;
      imu_quat_d = imu.quat_d;
    }
  }

  return gotPacket;
}

void queueSample(
  int pot1ADC, int pot2ADC, int pot3ADC, int pot4ADC,
  unsigned long sampleTimeMs, uint32_t seq
) {
  if (txQueue == nullptr) return;

  TxMessage msg;
  snprintf(
    msg.line, sizeof(msg.line),
    "%lu,%lu,%d,%d,%d,%d,%.6f,%.6f,%d,%d,%d,%d,%lu\n",
    frameCount, sampleTimeMs,
    pot1ADC, pot2ADC, pot3ADC, pot4ADC,
    imu_gyro_x, imu_accel_x,
    imu_quat_a, imu_quat_b, imu_quat_c, imu_quat_d,
    (unsigned long)seq
  );

  if (xQueueSend(txQueue, &msg, 0) == pdPASS) {
    txQueued++;
  } else {
    TxMessage oldMsg;
    if (xQueueReceive(txQueue, &oldMsg, 0) == pdTRUE &&
        xQueueSend(txQueue, &msg, 0) == pdPASS) {
      txQueued++;
    }
    txQueueDrop++;
  }
}

void sendData() {
  refreshImuAlive();

  if (pollImuNonBlocking()) {
    digitalWrite(selectPin0, LOW);  digitalWrite(selectPin1, LOW);  digitalWrite(selectPin2, LOW);
    int pot1ADC = analogRead(zInput);

    digitalWrite(selectPin0, HIGH); digitalWrite(selectPin1, LOW);  digitalWrite(selectPin2, LOW);
    int pot2ADC = analogRead(zInput);

    digitalWrite(selectPin0, LOW);  digitalWrite(selectPin1, HIGH); digitalWrite(selectPin2, LOW);
    int pot3ADC = analogRead(zInput);

    digitalWrite(selectPin0, HIGH); digitalWrite(selectPin1, HIGH); digitalWrite(selectPin2, LOW);
    int pot4ADC = analogRead(zInput);

    frameCount++;

    bool shouldPrint = (PRINT_EVERY_N > 0 && frameCount % PRINT_EVERY_N == 0);
    bool shouldSend  = (frameCount % SEND_EVERY_N == 0);

    if (shouldPrint || (shouldSend && grabar)) {
      unsigned long sampleTimeMs = millis();
      uint32_t seq = ++seqCounter;
      queueSample(pot1ADC, pot2ADC, pot3ADC, pot4ADC, sampleTimeMs, seq);
      if (shouldPrint) {
        Serial.print("frame=");
        Serial.println(frameCount);
      }
    }
  }
}

void loop() {
  sendData();
  updateOledStatus();
  checkRecordingStatus();
}

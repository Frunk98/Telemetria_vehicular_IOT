#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <GP94.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define CAN0_INT 33
MCP_CAN CAN0(4);

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
const uint16_t gatewayPort  = 5001;
const uint16_t localUdpPort = 4001;

WiFiUDP udp;

const unsigned long OLED_UPDATE_MS      = 750;
const unsigned int  SEND_EVERY_N        = 20;
const uint16_t      TX_QUEUE_LEN        = 64;
const size_t        UDP_PACKET_MAX_BYTES = 230;
const uint8_t       UDP_BATCH_MAX_LINES  = 2;

long unsigned int rxId;
float vel, rpm, acel, chrg, tmp, gas, adj, tiempo;
unsigned char len = 0;
unsigned char rxBuf[8];
byte D_sol[8] = { 0x02, 0x01, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
int S_rxId;

bool grabar = false;
uint32_t prueba_id = 0;

// Snapshot global de IMU actualizado SOLO dentro de imu.decode()
float imu_accel_y = 0.0f;
float imu_accel_z = 0.0f;
float imu_gyro_y = 0.0f;
float imu_gyro_z = 0.0f;
double imu_lat = 0.0;
double imu_lon = 0.0;
double imu_alt = 0.0;
double imu_speed = 0.0;
int gps_valid_flag = 0;
unsigned long lastGpsFixMs = 0;
const unsigned long GPS_FIX_STALE_MS = 8000;
unsigned long lastImuPacketMs = 0;
unsigned long imuPacketCount = 0;
bool imuAlive = false;
const unsigned long IMU_STALE_MS = 3000;

unsigned long lastOledUpdate = 0;
bool oledOk = false;

unsigned long frameCount  = 0;
uint32_t      seqCounter  = 0;

volatile unsigned long txQueued     = 0;
volatile unsigned long txSent       = 0;
volatile unsigned long txQueueDrop  = 0;
volatile unsigned long txUdpFail    = 0;
volatile unsigned long txPacketSent = 0;

struct TxMessage {
  char line[220];
};

QueueHandle_t txQueue = nullptr;

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

  display.println("OBD UDP");
  display.print("WiFi: ");
  display.println(WiFi.status() == WL_CONNECTED ? "OK" : "NO");
  display.print("GP9: ");
  display.println(imuAlive ? "OK" : "NO");
  display.print("GPS: ");
  display.println(gps_valid_flag ? "OK" : "NO");
  display.print("Hz: ");
  display.println(imuHz);
  display.print("REC: ");
  display.println(grabar ? "SI" : "NO");
  display.print("Sent: ");
  display.println(txSent);
  display.print("QDrop: ");
  display.println(txQueueDrop);

  display.display();
}

void leer() {
  if (!digitalRead(CAN0_INT)) {
    if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      if (rxId == 0x7E8) {
        switch (rxBuf[2]) {
          case 0x0D: vel    = rxBuf[3]; break;
          case 0x0C: rpm    = 0.25 * ((rxBuf[3] * 256.0) + rxBuf[4]); break;
          case 0x49: acel   = rxBuf[3] / 2.55; break;
          case 0x04: chrg   = rxBuf[3] / 2.55; break;
          case 0x05: tmp    = rxBuf[3] - 40.0; break;
          case 0x0E: tiempo = (rxBuf[3] / 2.0) - 64.0; break;
          case 0x14: adj    = (rxBuf[4] / 1.28) - 100.0; break;
          case 0x2F: gas    = (1.0 / 2.55) * rxBuf[3]; break;
        }
      }
    }
  }
}

void enviar() {
  D_sol[2] = S_rxId;
  if (CAN0.sendMsgBuf(0x7DF, 0, 8, D_sol) != CAN_OK) {
  }
  leer();
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
      int hblen = snprintf(hb, sizeof(hb), "H,O,%.6f,%.6f,%d,%d,%lu\n",
        imu_lat, imu_lon, gps_valid_flag, imuAlive ? 1 : 0, imuPacketCount);
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
  Serial.println("OBD UDP");
  Serial.println("OBD CODEX SIN REBAUD v3");

  mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if (oledOk) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("OBD UDP");
    display.println("Conectando...");
    display.display();
  }

  while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    delay(1000);
  }
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);

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
  Serial.print("WiFi OK. IP OBD: ");
  Serial.println(WiFi.localIP());
  Serial.println("IMU UART baud: 115200");

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
      imu_accel_y = imu.accel_y;
      imu_accel_z = imu.accel_z;
      imu_gyro_y  = imu.gyro_y;
      imu_gyro_z  = imu.gyro_z;
      imu_lat   = imu.lattitude;
      imu_lon   = imu.longitude;
      imu_alt   = imu.altitude;
      imu_speed = imu.speed;
      if (imu.lattitude != 0.0 && imu.longitude != 0.0) {
        lastGpsFixMs = millis();
      }
    }
  }
  return gotPacket;
}

void refreshImuAlive() {
  if (imuAlive && (millis() - lastImuPacketMs > IMU_STALE_MS)) {
    imuAlive = false;
  }
}

void refreshGpsValidFlag() {
  const bool coordsNonZero = (imu_lat != 0.0 && imu_lon != 0.0);
  const bool fresh = (lastGpsFixMs > 0) && ((millis() - lastGpsFixMs) <= GPS_FIX_STALE_MS);
  gps_valid_flag = (coordsNonZero && fresh) ? 1 : 0;
}

void sendData() {
  refreshImuAlive();
  refreshGpsValidFlag();

  if (pollImuNonBlocking()) {
    refreshImuAlive();
    refreshGpsValidFlag();

    S_rxId = 0x0D; enviar();
    S_rxId = 0x0C; enviar();
    S_rxId = 0x49; enviar();
    S_rxId = 0x04; enviar();
    S_rxId = 0x05; enviar();
    S_rxId = 0x0E; enviar();
    S_rxId = 0x14; enviar();
    S_rxId = 0x2F; enviar();

    frameCount++;

    if (grabar && frameCount % SEND_EVERY_N == 0) {
      unsigned long sampleTimeMs = millis();
      uint32_t seq = ++seqCounter;

      TxMessage msg;
      snprintf(
        msg.line, sizeof(msg.line),
        "%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%lu\n",
        frameCount, sampleTimeMs,
        vel, rpm, acel, chrg, tmp, tiempo, adj, gas,
        imu_accel_y, imu_accel_z,
        imu_gyro_y,  imu_gyro_z,
        imu_lat, imu_lon, imu_alt, imu_speed,
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
  }
}

void loop() {
  sendData();
  updateOledStatus();
  checkRecordingStatus();
}

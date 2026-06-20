#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define SerialAT  Serial1

#define MODEM_RX     26
#define MODEM_TX     27
#define MODEM_PWRKEY 4

const char* ap_ssid     = "gateway_esteban";
const char* ap_password = "shaman2026";

IPAddress ap_ip(192, 168, 4, 1);
IPAddress ap_gateway(192, 168, 4, 1);
IPAddress ap_subnet(255, 255, 255, 0);

const uint16_t UDP_PORT_VOLANTE = 5000;
const uint16_t UDP_PORT_OBD     = 5001;
WiFiUDP udpVolante;
WiFiUDP udpObd;

WebServer httpServer(80);

const char apn[]      = "internet.itelcel.com";
const char gprsUser[] = "webgprs";
const char gprsPass[] = "webgprs2002";

TinyGsm modem(SerialAT);
TinyGsmClient cellClient(modem);

const char serverIp[]   = "142.44.162.246";
const char hostHeader[] = "shaman.thais.ovh";
const int  serverPort   = 80;
const char phpPath[]    = "/api/recibir.php";

const unsigned long POST_INTERVAL_MS         = 250;
const size_t        BATCH_BUFFER_SIZE        = 65536;
const size_t        POST_MAX_BYTES           = 8192;
const unsigned long HTTP_RESPONSE_TIMEOUT_MS = 8000;

SemaphoreHandle_t batchMutex = nullptr;
SemaphoreHandle_t modemMutex = nullptr;

char          batchBuffer[BATCH_BUFFER_SIZE];
size_t        batchLen   = 0;
unsigned long batchLines = 0;

// postAttempts se conserva: gprsKeepaliveTask lo usa para detectar
// actividad de envio y decidir si reconecta el celular.
volatile unsigned long postAttempts = 0;

volatile int      gateway_grabando  = 0;
volatile uint32_t gateway_prueba_id = 0;
volatile unsigned long volante_last_seen_ms = 0;
volatile unsigned long obd_last_seen_ms     = 0;
volatile int           obd_gps_valid        = 0;
volatile double        obd_lat              = 0;
volatile double        obd_lon              = 0;
volatile unsigned long obd_gps_last_hb_ms   = 0;
const unsigned long    OBD_GPS_STALE_MS     = 3000;
volatile bool          volante_ever_seen    = false;
volatile bool          obd_ever_seen        = false;

int getFreshGpsValid() {
  if (obd_gps_last_hb_ms == 0) return 0;
  if ((millis() - obd_gps_last_hb_ms) > OBD_GPS_STALE_MS) return 0;
  return obd_gps_valid ? 1 : 0;
}

void powerOnModem() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(5000);
}

bool setupCellular() {
  powerOnModem();
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  if (!modem.testAT()) return false;

  modem.sendAT("+CMNB=1");  modem.waitResponse(5000);
  modem.sendAT("+CFUN=0");  modem.waitResponse(10000); delay(5000);
  modem.sendAT("+CFUN=1");  modem.waitResponse(10000); delay(10000);

  if (!modem.waitForNetwork(60000L)) return false;

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) return false;

  return true;
}

void handleCheckRecording() {
  String response = "{\"grabando\":";
  response += String((int)gateway_grabando);
  response += ",\"prueba_id\":";
  response += String((uint32_t)gateway_prueba_id);
  response += "}";
  httpServer.send(200, "application/json", response);
}

void handleNotFound() {
  httpServer.send(404, "text/plain", "Not Found");
}

bool setupAPUdp() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  if (!WiFi.softAP(ap_ssid, ap_password)) return false;

  if (!udpVolante.begin(UDP_PORT_VOLANTE)) return false;
  if (!udpObd.begin(UDP_PORT_OBD))         return false;

  httpServer.on("/api/check-recording.php", handleCheckRecording);
  httpServer.onNotFound(handleNotFound);
  httpServer.begin();
  return true;
}

unsigned long countLinesInChunk(const char* data, size_t len) {
  unsigned long count = 0;
  for (size_t i = 0; i < len; i++) if (data[i] == '\n') count++;
  if (len > 0 && data[len - 1] != '\n') count++;
  return count;
}

size_t getChunkLength() {
  size_t limit = batchLen < POST_MAX_BYTES ? batchLen : POST_MAX_BYTES;
  size_t cut = 0;
  for (size_t i = 0; i < limit; i++) if (batchBuffer[i] == '\n') cut = i + 1;
  if (cut == 0) cut = (batchLen <= POST_MAX_BYTES) ? batchLen : POST_MAX_BYTES;
  return cut;
}

void removeSentChunk(size_t lenToRemove, unsigned long linesToRemove) {
  if (lenToRemove == 0 || lenToRemove > batchLen) return;
  size_t remaining = batchLen - lenToRemove;
  if (remaining > 0) memmove(batchBuffer, batchBuffer + lenToRemove, remaining);
  batchLen = remaining;
  batchBuffer[batchLen] = '\0';
  batchLines = (batchLines >= linesToRemove) ? batchLines - linesToRemove : 0;
}

void addPacketToBatch(const char* packet, size_t packetLen, char prefix) {
  if (packetLen == 0) return;

  if (xSemaphoreTake(batchMutex, portMAX_DELAY) != pdTRUE) return;

  size_t i = 0;
  while (i < packetLen) {
    size_t lineStart = i;
    while (i < packetLen && packet[i] != '\n') i++;
    size_t lineLen = i - lineStart;
    if (i < packetLen) i++;

    if (lineLen == 0) continue;

    size_t needed = 2 + lineLen + 1;
    if (batchLen + needed > BATCH_BUFFER_SIZE - 1) {
      continue;
    }

    batchBuffer[batchLen++] = prefix;
    batchBuffer[batchLen++] = ',';
    memcpy(&batchBuffer[batchLen], &packet[lineStart], lineLen);
    batchLen += lineLen;
    batchBuffer[batchLen++] = '\n';
    batchBuffer[batchLen]    = '\0';
    batchLines++;
  }

  xSemaphoreGive(batchMutex);
}

void parseServerResponse(const char* body) {
  const char* p = strstr(body, "\"grabando\"");
  if (p) {
    p = strchr(p, ':');
    if (p) {
      p++;
      while (*p == ' ') p++;
      gateway_grabando = atoi(p);
    }
  }
  p = strstr(body, "\"prueba_id\"");
  if (p) {
    p = strchr(p, ':');
    if (p) {
      p++;
      while (*p == ' ') p++;
      gateway_prueba_id = (uint32_t)atol(p);
    }
  }
}

void sendHeartbeatToServer() {
  if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(10000)) != pdTRUE) return;

  do {
    if (!modem.isGprsConnected()) break;

    unsigned long now = millis();
    long vol_age = volante_ever_seen ? (long)(now - volante_last_seen_ms) : -1;
    long obd_age = obd_ever_seen     ? (long)(now - obd_last_seen_ms)     : -1;

    int gps_ok_now = getFreshGpsValid();

    char body[256];
    int blen = snprintf(body, sizeof(body),
      "{\"vol_ms\":%ld,\"obd_ms\":%ld,\"gps_ok\":%d,\"lat\":%.6f,\"lon\":%.6f}",
      vol_age, obd_age, gps_ok_now, (double)obd_lat, (double)obd_lon);

    cellClient.stop();
    vTaskDelay(pdMS_TO_TICKS(1));

    if (!cellClient.connect(serverIp, serverPort)) break;

    cellClient.print("POST /api/recibir.php HTTP/1.1\r\n");
    cellClient.print("Host: "); cellClient.print(hostHeader); cellClient.print("\r\n");
    cellClient.print("Connection: close\r\n");
    cellClient.print("Content-Type: text/plain\r\n");
    cellClient.print("Content-Length: "); cellClient.print(blen); cellClient.print("\r\n\r\n");
    cellClient.print(body);

    unsigned long waitStart = millis();
    while (!cellClient.available()) {
      if (!cellClient.connected() || millis() - waitStart > 5000) {
        cellClient.stop();
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!cellClient.available()) break;

    char resp[512]; uint16_t bi = 0;
    bool headerEnded = false;
    int blankCount = 0;
    unsigned long bodyStart = millis();
    while (millis() - bodyStart < 3000 && bi < sizeof(resp) - 1) {
      while (cellClient.available() && bi < sizeof(resp) - 1) {
        char c = cellClient.read();
        if (!headerEnded) {
          if (c == '\r') continue;
          if (c == '\n') { blankCount++; if (blankCount >= 2) headerEnded = true; continue; }
          blankCount = 0;
        } else {
          resp[bi++] = c;
        }
      }
      if (!cellClient.connected() && !cellClient.available()) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    resp[bi] = '\0';
    cellClient.stop();

    parseServerResponse(resp);
  } while (false);

  xSemaphoreGive(modemMutex);
}

bool sendHttpBatch(const char* data, size_t len, unsigned long linesCount) {
  postAttempts++;
  if (len == 0) return false;

  if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(35000)) != pdTRUE) {
    return false;
  }

  bool ok = false;
  do {
    if (!modem.isGprsConnected()) {
      if (!modem.isNetworkConnected() && !modem.waitForNetwork(30000L)) break;
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) break;
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    cellClient.stop();
    vTaskDelay(pdMS_TO_TICKS(1));

    if (!cellClient.connect(serverIp, serverPort)) break;

    cellClient.print("POST "); cellClient.print(phpPath); cellClient.println(" HTTP/1.1");
    cellClient.print("Host: "); cellClient.println(hostHeader);
    cellClient.println("Connection: close");
    cellClient.println("Content-Type: text/plain");
    cellClient.print("Content-Length: "); cellClient.println(len);
    cellClient.println();

    size_t sent = 0;
    while (sent < len) {
      size_t chunk = (len - sent > 1024) ? 1024 : (len - sent);
      size_t written = cellClient.write((const uint8_t*)data + sent, chunk);
      if (written == 0) { cellClient.stop(); break; }
      sent += written;
    }
    if (sent < len) break;

    unsigned long waitStart = millis();
    while (!cellClient.available()) {
      if (!cellClient.connected() || millis() - waitStart > HTTP_RESPONSE_TIMEOUT_MS) {
        cellClient.stop(); break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!cellClient.available()) break;

    char statusLine[96]; uint16_t si = 0; bool got = false;
    while (millis() - waitStart <= HTTP_RESPONSE_TIMEOUT_MS) {
      while (cellClient.available()) {
        char c = cellClient.read();
        if (c == '\n') { statusLine[si] = '\0'; got = (si > 0); break; }
        if (c != '\r' && si < sizeof(statusLine) - 1) statusLine[si++] = c;
      }
      if (got || (!cellClient.connected() && !cellClient.available())) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!got) { cellClient.stop(); break; }

    int code = 0;
    const char* cs = strchr(statusLine, ' ');
    if (cs != nullptr) code = atoi(cs + 1);

    char body[512]; uint16_t bi = 0;
    unsigned long bodyStart = millis();
    while (millis() - bodyStart < 3000 && bi < sizeof(body) - 1) {
      while (cellClient.available() && bi < sizeof(body) - 1) {
        body[bi++] = cellClient.read();
      }
      if (!cellClient.connected() && !cellClient.available()) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    body[bi] = '\0';
    cellClient.stop();

    if (code == 200) {
      ok = true;
      parseServerResponse(body);
    }
  } while (false);

  xSemaphoreGive(modemMutex);
  return ok;
}

void udpReceiveTask(void *parameter) {
  (void)parameter;
  char packet[256];
  uint8_t burst = 0;

  for (;;) {
    bool any = false;

    int ps = udpVolante.parsePacket();
    if (ps > 0) {
      if (ps >= (int)sizeof(packet)) ps = sizeof(packet) - 1;
      int l = udpVolante.read(packet, ps);
      if (l > 0) {
        packet[l] = '\0';
        volante_last_seen_ms = millis();
        volante_ever_seen = true;
        if (packet[0] == 'H' && packet[1] == ',') {
          // Solo heartbeat, no agregar al batch
        } else {
          addPacketToBatch(packet, (size_t)l, 'V');
        }
        any = true;
      }
    }

    ps = udpObd.parsePacket();
    if (ps > 0) {
      if (ps >= (int)sizeof(packet)) ps = sizeof(packet) - 1;
      int l = udpObd.read(packet, ps);
      if (l > 0) {
        packet[l] = '\0';
        obd_last_seen_ms = millis();
        obd_ever_seen = true;
        if (packet[0] == 'H' && packet[1] == ',') {
          char* p = packet + 2;
          if (*p == 'O') p++;
          if (*p == ',') p++;
          obd_lat = atof(p);
          char* c1 = strchr(p, ',');
          if (c1) {
            obd_lon = atof(c1 + 1);
            char* c2 = strchr(c1 + 1, ',');
            if (c2) {
              obd_gps_valid = atoi(c2 + 1);
              obd_gps_last_hb_ms = millis();
            }
          }
        } else {
          addPacketToBatch(packet, (size_t)l, 'O');
        }
        any = true;
      }
    }

    if (any) {
      burst++;
      if (burst >= 16) { burst = 0; vTaskDelay(pdMS_TO_TICKS(1)); }
    } else {
      burst = 0;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void httpPostTask(void *parameter) {
  (void)parameter;
  static char sendBuffer[POST_MAX_BYTES + 1];

  for (;;) {
    size_t snapshotLen = 0;
    if (xSemaphoreTake(batchMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      snapshotLen = batchLen;
      xSemaphoreGive(batchMutex);
    }

    unsigned long waitMs = 0;
    if      (snapshotLen == 0)    waitMs = POST_INTERVAL_MS;
    else if (snapshotLen >= 1024) waitMs = 0;
    else                          waitMs = 10;
    if (waitMs > 0) vTaskDelay(pdMS_TO_TICKS(waitMs));

    size_t lenToSend = 0;
    unsigned long linesToSend = 0;

    if (xSemaphoreTake(batchMutex, portMAX_DELAY) == pdTRUE) {
      if (batchLen > 0) {
        lenToSend = getChunkLength();
        if (lenToSend > 0 && lenToSend <= POST_MAX_BYTES) {
          memcpy(sendBuffer, batchBuffer, lenToSend);
          sendBuffer[lenToSend] = '\0';
          linesToSend = countLinesInChunk(sendBuffer, lenToSend);
        }
      }
      xSemaphoreGive(batchMutex);
    }

    if (lenToSend > 0) {
      bool ok = sendHttpBatch(sendBuffer, lenToSend, linesToSend);
      if (ok) {
        if (xSemaphoreTake(batchMutex, portMAX_DELAY) == pdTRUE) {
          removeSentChunk(lenToSend, linesToSend);
          xSemaphoreGive(batchMutex);
        }
      }
    }
  }
}

void gprsKeepaliveTask(void *parameter) {
  (void)parameter;
  unsigned long lastSeen = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(60000));

    unsigned long now = postAttempts;
    if (now != lastSeen) { lastSeen = now; continue; }

    size_t backlog = 0;
    if (xSemaphoreTake(batchMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      backlog = batchLen;
      xSemaphoreGive(batchMutex);
    }
    if (backlog > POST_MAX_BYTES * 2) continue;

    if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(40000)) != pdTRUE) continue;

    if (!modem.isGprsConnected()) {
      if (!modem.isNetworkConnected()) modem.waitForNetwork(30000L);
      modem.gprsConnect(apn, gprsUser, gprsPass);
    }
    xSemaphoreGive(modemMutex);
  }
}

void webServerTask(void *parameter) {
  (void)parameter;
  for (;;) {
    httpServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void heartbeatTask(void *parameter) {
  (void)parameter;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    sendHeartbeatToServer();
  }
}

void setup() {
  batchBuffer[0] = '\0';
  batchMutex = xSemaphoreCreateMutex();
  modemMutex = xSemaphoreCreateMutex();
  if (batchMutex == nullptr || modemMutex == nullptr) {
    while (true) delay(1000);
  }

  setupCellular();
  setupAPUdp();

  xTaskCreatePinnedToCore(udpReceiveTask,    "udpRx",     4096,  nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(httpPostTask,      "httpPost",  12288, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(gprsKeepaliveTask, "ka",        4096,  nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(webServerTask,     "web",       4096,  nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(heartbeatTask,     "heartbeat", 4096,  nullptr, 1, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

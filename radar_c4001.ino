/*
  Radar ESP32 C4001 24 GHz - V6 ASCII $DFHPD/$DFDMD
  - WiFiManager pour config Wi-Fi sans recompiler
  - IHM HTML locale
  - Lecture UART C4001 en ASCII ($DFHPD presence, $DFDMD distance/vitesse) sans librairie DFRobot
  - Passage en mode distance/vitesse par commandes ASCII non bloquantes : sensorStop / setRunApp 1 / saveConfig / sensorStart
  - Envoi REST/JSON vers Raspberry/PHP/MySQL

  Câblage conseillé :
    C4001 VIN  -> ESP32 VIN / 5V
    C4001 GND  -> ESP32 GND
    C4001 TX   -> ESP32 RX2 / GPIO16
    C4001 RX   -> ESP32 TX2 / GPIO17
    C4001 OUT  -> ESP32 GPIO27 optionnel

  Librairie Arduino à installer :
    - WiFiManager by tzapu
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <WebServer.h>
#include <time.h>

// =====================================================
// BROCHES
// =====================================================
#define C4001_RX_PIN 16     // RX2 ESP32 <- TX C4001
#define C4001_TX_PIN 17     // TX2 ESP32 -> RX C4001
#define C4001_OUT_PIN 27    // Optionnel, si OUT est branché

// =====================================================
// C4001 UART ASCII
// =====================================================
// Le module reçu envoie des trames ASCII du type :
// $DFHPD,1, , , *
// Baudrate observe : 9600
#define C4001_UART_BAUD 9600
HardwareSerial RadarSerial(2);
String c4001LineBuffer = "";
String lastC4001Line = "";
unsigned long c4001FrameCount = 0;
unsigned long c4001BadFrameCount = 0;
unsigned long c4001LastFrameMs = 0;

// =====================================================
// CONFIG PORTAIL WIFI
// =====================================================
const char* CONFIG_AP_NAME = "Radar-C4001-Setup";
const char* CONFIG_AP_PASSWORD = "radar1234"; // 8 caractères minimum

// =====================================================
// VALEURS PAR DEFAUT
// =====================================================
const char* DEFAULT_API_URL = "http://192.168.0.23/Radar/radar_presence.php";
const char* DEFAULT_API_TOKEN = "56c3b1bd605dffac15f5efe9c337249995984d0474b70e432997bfe50af05bf2";
const char* DEFAULT_DEVICE_NAME = "radar_c4001_couloir_escalier";

// =====================================================
// VARIABLES CONFIGURABLES
// =====================================================
Preferences prefs;
WebServer server(80);

String apiUrl;
String apiToken;
String deviceName;

unsigned long heartbeatMs = 30000;
unsigned long stableDelayMs = 800;
unsigned long debugMs = 1000;
bool sendOnlyNight = false;
int nightStartHour = 22;
int nightEndHour = 6;

// Réglages C4001 en mode distance/vitesse
int detectMinCm = 30;       // 30 cm
int detectMaxCm = 1200;     // 12 m pour commencer, même avec version 25 m
int detectThreshold = 10;   // seuil DFRobot, unité dimensionless / 0.1 selon doc
bool frettingDetection = true;
// UART desactivable pour eviter de bloquer l'IHM si le module ne repond pas.
bool c4001UartEnabled = true;
bool autoSpeedMode = true;       // demande automatiquement le mode distance/vitesse au demarrage
bool pendingReinitC4001 = false;

// Commandes ASCII C4001 non bloquantes
enum C4001SeqType { SEQ_NONE, SEQ_SPEED_MODE, SEQ_PRESENCE_MODE, SEQ_START_ONLY, SEQ_RESET };
C4001SeqType activeSeq = SEQ_NONE;
String seqCmds[6];
int seqCmdCount = 0;
int seqCmdIndex = 0;
unsigned long seqNextMs = 0;
String lastC4001Command = "";
String lastC4001SeqText = "";

// =====================================================
// ETAT C4001
// =====================================================
bool c4001Ready = false;
bool rawPresence = false;
bool stablePresence = false;
bool lastRawPresence = false;

float targetSpeedMS = 0.0f;
float targetRangeM = 0.0f;
float targetEnergy = 0.0f;
int targetNumber = 0;
bool motionDetected = false;
String c4001FrameType = "";        // DFHPD ou DFDMD
String c4001WorkModeText = "inconnu";

unsigned long lastRawChangeMs = 0;
unsigned long presenceStartMs = 0;
unsigned long lastSendMs = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long lastRadarReadMs = 0;

int lastHttpCode = 0;
String lastHttpResponse = "";
String lastReason = "";

// =====================================================
// OUTILS HTML
// =====================================================
String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  return s;
}

String boolText(bool v) {
  return v ? "OUI" : "NON";
}

String uptimeText() {
  unsigned long total = millis() / 1000;
  unsigned long days = total / 86400;
  total %= 86400;
  unsigned long hours = total / 3600;
  total %= 3600;
  unsigned long minutes = total / 60;
  unsigned long seconds = total % 60;

  char buf[64];
  snprintf(buf, sizeof(buf), "%lu j %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  return String(buf);
}

// =====================================================
// CONFIGURATION MEMOIRE
// =====================================================
void loadConfig() {
  prefs.begin("radarC4001", true);

  apiUrl = prefs.getString("api_url", DEFAULT_API_URL);
  apiToken = prefs.getString("api_token", DEFAULT_API_TOKEN);
  deviceName = prefs.getString("device", DEFAULT_DEVICE_NAME);

  heartbeatMs = prefs.getULong("heartbeat", 30000);
  stableDelayMs = prefs.getULong("stable", 800);
  debugMs = prefs.getULong("debug", 1000);

  sendOnlyNight = prefs.getBool("only_night", false);
  nightStartHour = prefs.getInt("night_start", 22);
  nightEndHour = prefs.getInt("night_end", 6);

  detectMinCm = prefs.getInt("min_cm", 30);
  detectMaxCm = prefs.getInt("max_cm", 1200);
  detectThreshold = prefs.getInt("threshold", 10);
  frettingDetection = prefs.getBool("fretting", true);
  c4001UartEnabled = prefs.getBool("uart_on", true);
  autoSpeedMode = prefs.getBool("auto_speed", true);

  prefs.end();
}

void saveConfig() {
  prefs.begin("radarC4001", false);

  prefs.putString("api_url", apiUrl);
  prefs.putString("api_token", apiToken);
  prefs.putString("device", deviceName);

  prefs.putULong("heartbeat", heartbeatMs);
  prefs.putULong("stable", stableDelayMs);
  prefs.putULong("debug", debugMs);

  prefs.putBool("only_night", sendOnlyNight);
  prefs.putInt("night_start", nightStartHour);
  prefs.putInt("night_end", nightEndHour);

  prefs.putInt("min_cm", detectMinCm);
  prefs.putInt("max_cm", detectMaxCm);
  prefs.putInt("threshold", detectThreshold);
  prefs.putBool("fretting", frettingDetection);
  prefs.putBool("uart_on", c4001UartEnabled);
  prefs.putBool("auto_speed", autoSpeedMode);

  prefs.end();
}

// =====================================================
// NTP
// =====================================================
void setupTimeNTP() {
  configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", "fr.pool.ntp.org", "pool.ntp.org", "time.google.com");
  Serial.println("Synchronisation NTP...");

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 3000)) {
    Serial.printf("Heure locale : %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    Serial.println("NTP non disponible pour l'instant");
  }
}

bool isNightTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1000)) {
    return false;
  }

  int h = timeinfo.tm_hour;
  if (nightStartHour == nightEndHour) return true;
  if (nightStartHour < nightEndHour) {
    return h >= nightStartHour && h < nightEndHour;
  }
  return h >= nightStartHour || h < nightEndHour;
}

String localTimeText() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1000)) {
    return "NTP non synchronise";
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return String(buf);
}

// =====================================================
// WIFI MANAGER
// =====================================================
void setupWiFiPortal() {
  loadConfig();

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);

  char apiUrlBuffer[180];
  char apiTokenBuffer[120];
  char deviceBuffer[80];

  apiUrl.toCharArray(apiUrlBuffer, sizeof(apiUrlBuffer));
  apiToken.toCharArray(apiTokenBuffer, sizeof(apiTokenBuffer));
  deviceName.toCharArray(deviceBuffer, sizeof(deviceBuffer));

  WiFiManagerParameter custom_api_url("api_url", "URL endpoint Raspberry", apiUrlBuffer, sizeof(apiUrlBuffer));
  WiFiManagerParameter custom_api_token("api_token", "Token API", apiTokenBuffer, sizeof(apiTokenBuffer));
  WiFiManagerParameter custom_device("device", "Nom du capteur", deviceBuffer, sizeof(deviceBuffer));

  wm.addParameter(&custom_api_url);
  wm.addParameter(&custom_api_token);
  wm.addParameter(&custom_device);

  Serial.println("Connexion Wi-Fi ou portail config...");
  bool connected = wm.autoConnect(CONFIG_AP_NAME, CONFIG_AP_PASSWORD);
  if (!connected) {
    Serial.println("Echec Wi-Fi / timeout portail. Redemarrage...");
    delay(3000);
    ESP.restart();
  }

  apiUrl = String(custom_api_url.getValue());
  apiToken = String(custom_api_token.getValue());
  deviceName = String(custom_device.getValue());
  saveConfig();
  loadConfig();

  Serial.println("Wi-Fi connecté");
  Serial.print("IP ESP32 : "); Serial.println(WiFi.localIP());
  Serial.print("RSSI : "); Serial.println(WiFi.RSSI());
}

bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  Serial.println("Wi-Fi déconnecté, reconnexion...");
  WiFi.reconnect();
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

// =====================================================
// C4001 ASCII $DFHPD
// =====================================================
float parseFloatSafe(String s, float currentValue) {
  s.trim();
  if (s.length() == 0) return currentValue;
  return s.toFloat();
}

int parseIntSafe(String s, int currentValue) {
  s.trim();
  if (s.length() == 0) return currentValue;
  return s.toInt();
}

String getField(String line, int fieldIndex) {
  // fieldIndex 0 = prefix $DFHPD, 1 = presence, 2 = distance, etc.
  int start = 0;
  int current = 0;

  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line.charAt(i) == ',') {
      if (current == fieldIndex) {
        String f = line.substring(start, i);
        f.trim();
        return f;
      }
      current++;
      start = i + 1;
    }
  }

  return String("");
}

void parseC4001AsciiLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  lastC4001Line = line;

  bool isPresenceFrame = line.startsWith("$DFHPD");
  bool isSpeedFrame = line.startsWith("$DFDMD");

  if (!isPresenceFrame && !isSpeedFrame) {
    c4001BadFrameCount++;
    return;
  }

  c4001FrameCount++;
  c4001LastFrameMs = millis();
  c4001Ready = true;

  if (isPresenceFrame) {
    // Exemple observe : $DFHPD,1, , , *
    // Champ 1 = presence. Les autres champs sont souvent vides en mode presence.
    c4001FrameType = "DFHPD";
    c4001WorkModeText = "presence";

    String presenceField = getField(line, 1);
    bool asciiPresence = (presenceField.toInt() != 0);

    // OUT reste la reference physique ; l'UART peut confirmer la presence.
    rawPresence = (digitalRead(C4001_OUT_PIN) == HIGH) || asciiPresence;

    targetNumber = rawPresence ? 1 : 0;
    if (!rawPresence) {
      targetRangeM = 0.0f;
      targetSpeedMS = 0.0f;
      targetEnergy = 0.0f;
    }
    motionDetected = rawPresence;
    return;
  }

  if (isSpeedFrame) {
    // Format attendu par la librairie DFRobot :
    // $DFDMD,par1,par2,par3,par4,par5,par6,par7*
    // parts[1] = nombre de cibles
    // parts[3] = distance en metres
    // parts[4] = vitesse en m/s
    // parts[5] = energie
    c4001FrameType = "DFDMD";
    c4001WorkModeText = "distance/vitesse";

    String numberField = getField(line, 1);
    String rangeField  = getField(line, 3);
    String speedField  = getField(line, 4);
    String energyField = getField(line, 5);

    numberField.replace("*", ""); numberField.trim();
    rangeField.replace("*", "");  rangeField.trim();
    speedField.replace("*", "");  speedField.trim();
    energyField.replace("*", ""); energyField.trim();

    int n = numberField.toInt();
    targetNumber = n;

    if (n > 0) {
      if (rangeField.length() > 0) targetRangeM = rangeField.toFloat();
      if (speedField.length() > 0) targetSpeedMS = speedField.toFloat();
      if (energyField.length() > 0) targetEnergy = energyField.toFloat();
    } else {
      targetRangeM = 0.0f;
      targetSpeedMS = 0.0f;
      targetEnergy = 0.0f;
    }

    rawPresence = (digitalRead(C4001_OUT_PIN) == HIGH) || (targetNumber > 0);
    motionDetected = (targetNumber > 0) && (abs(targetSpeedMS) > 0.001f || targetEnergy > 0.0f);
    return;
  }
}
void readC4001Ascii() {
  if (!c4001UartEnabled) {
    return;
  }

  int safety = 0;
  while (RadarSerial.available() && safety < 128) {
    safety++;
    char c = (char)RadarSerial.read();

    if (c == '\n') {
      c4001LineBuffer.trim();
      if (c4001LineBuffer.length() > 0) {
        parseC4001AsciiLine(c4001LineBuffer);
      }
      c4001LineBuffer = "";
    } else if (c != '\r') {
      c4001LineBuffer += c;
      if (c4001LineBuffer.length() > 160) {
        c4001LineBuffer = "";
        c4001BadFrameCount++;
      }
    }
  }
}


// =====================================================
// COMMANDES ASCII C4001 NON BLOQUANTES
// =====================================================
void writeC4001Command(const String& cmd) {
  if (!c4001UartEnabled) return;
  Serial.print("Commande C4001 -> ");
  Serial.println(cmd);
  lastC4001Command = cmd;
  // La librairie officielle DFRobot envoie les commandes sans \r\n.
  RadarSerial.print(cmd);
}

void startC4001Sequence(C4001SeqType seq) {
  if (!c4001UartEnabled) {
    Serial.println("Sequence C4001 ignoree : UART desactive");
    return;
  }

  activeSeq = seq;
  seqCmdIndex = 0;
  seqCmdCount = 0;
  seqNextMs = millis();

  if (seq == SEQ_SPEED_MODE) {
    lastC4001SeqText = "passage distance/vitesse";
    seqCmds[seqCmdCount++] = "sensorStop";
    seqCmds[seqCmdCount++] = "setRunApp 1";
    seqCmds[seqCmdCount++] = "saveConfig";
    seqCmds[seqCmdCount++] = "sensorStart";
  } else if (seq == SEQ_PRESENCE_MODE) {
    lastC4001SeqText = "passage presence";
    seqCmds[seqCmdCount++] = "sensorStop";
    seqCmds[seqCmdCount++] = "setRunApp 0";
    seqCmds[seqCmdCount++] = "saveConfig";
    seqCmds[seqCmdCount++] = "sensorStart";
  } else if (seq == SEQ_START_ONLY) {
    lastC4001SeqText = "sensorStart";
    seqCmds[seqCmdCount++] = "sensorStart";
  } else if (seq == SEQ_RESET) {
    lastC4001SeqText = "resetSystem";
    seqCmds[seqCmdCount++] = "resetSystem";
  } else {
    activeSeq = SEQ_NONE;
    lastC4001SeqText = "";
    return;
  }

  Serial.print("Sequence C4001 lancee : ");
  Serial.println(lastC4001SeqText);
}

void processC4001Sequence() {
  if (activeSeq == SEQ_NONE) return;
  if (millis() < seqNextMs) return;

  if (seqCmdIndex < seqCmdCount) {
    writeC4001Command(seqCmds[seqCmdIndex]);
    seqCmdIndex++;

    // Delais prudents, mais non bloquants pour l'IHM.
    if (seqCmdIndex == 1) {
      seqNextMs = millis() + 700;  // apres sensorStop
    } else if (seqCmdIndex == 3) {
      seqNextMs = millis() + 900;  // apres saveConfig
    } else {
      seqNextMs = millis() + 250;
    }
  } else {
    Serial.print("Sequence C4001 terminee : ");
    Serial.println(lastC4001SeqText);
    activeSeq = SEQ_NONE;
    seqCmdCount = 0;
    seqCmdIndex = 0;
  }
}

void setupC4001() {
  Serial.println("Initialisation UART C4001 ASCII...");

  targetSpeedMS = 0.0f;
  targetRangeM = 0.0f;
  targetEnergy = 0.0f;
  targetNumber = 0;
  motionDetected = false;
  c4001Ready = false;

  rawPresence = (digitalRead(C4001_OUT_PIN) == HIGH);

  if (!c4001UartEnabled) {
    Serial.println("UART C4001 desactive. Mode OUT/GPIO uniquement.");
    return;
  }

  RadarSerial.end();
  delay(100);
  RadarSerial.begin(C4001_UART_BAUD, SERIAL_8N1, C4001_RX_PIN, C4001_TX_PIN);
  c4001LineBuffer = "";

  Serial.println("UART2 active : C4001 TX -> GPIO16 RX2, C4001 RX -> GPIO17 TX2, baud 9600");

  if (autoSpeedMode) {
    startC4001Sequence(SEQ_SPEED_MODE);
  } else {
    startC4001Sequence(SEQ_START_ONLY);
  }

  // On attend un peu une trame sans jamais bloquer l'IHM.
  unsigned long start = millis();
  while (millis() - start < 1500) {
    server.handleClient();
    processC4001Sequence();
    readC4001Ascii();
    if (c4001Ready) break;
    delay(10);
  }

  if (c4001Ready) {
    Serial.println("C4001 ASCII detecte");
    Serial.print("Derniere trame : ");
    Serial.println(lastC4001Line);
  } else {
    Serial.println("C4001 non detecte en ASCII pour l'instant");
    Serial.println("L'IHM reste disponible. Verifier RX/TX ou attendre les trames.");
  }
}

void readC4001() {
  // OUT/GPIO est toujours lu, non bloquant.
  bool outPresence = (digitalRead(C4001_OUT_PIN) == HIGH);
  rawPresence = outPresence;

  readC4001Ascii();

  // Si plus aucune trame UART recente, on ne marque pas KO definitif :
  // on garde la presence OUT, mais l'IHM montrera l'age de la derniere trame.
  if (c4001UartEnabled && c4001LastFrameMs > 0 && millis() - c4001LastFrameMs > 5000) {
    // On laisse c4001Ready à true pour indiquer qu'il a deja parle,
    // mais les valeurs distance/vitesse peuvent etre anciennes ou vides.
  }
}

int distanceCm() {
  if (targetRangeM <= 0.0f) return 0;
  return (int)(targetRangeM * 100.0f + 0.5f);
}

int speedCms() {
  return (int)(targetSpeedMS * 100.0f + (targetSpeedMS >= 0 ? 0.5f : -0.5f));
}

int getDurationSeconds(bool presence) {
  if (!presence || presenceStartMs == 0) return 0;
  return (int)((millis() - presenceStartMs) / 1000);
}

// =====================================================
// HTTP POST
// =====================================================
bool sendRadarData(bool presence, const char* reason) {
  if (sendOnlyNight && !isNightTime()) {
    Serial.println("Mode nocturne seulement : envoi ignore hors plage nuit");
    return true;
  }

  if (!ensureWiFiConnected()) {
    Serial.println("Abandon envoi : Wi-Fi indisponible");
    return false;
  }

  HTTPClient http;
  http.begin(apiUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Token", apiToken);

  String json = "{";
  json += "\"device\":\"" + deviceName + "\",";
  json += "\"radar_model\":\"C4001\",";
  json += "\"presence\":" + String(presence ? 1 : 0) + ",";
  json += "\"duration_s\":" + String(getDurationSeconds(presence)) + ",";
  json += "\"distance_cm\":" + String(distanceCm()) + ",";
  json += "\"speed_m_s\":" + String(targetSpeedMS, 3) + ",";
  json += "\"speed_cm_s\":" + String(speedCms()) + ",";
  json += "\"target_number\":" + String(targetNumber) + ",";
  json += "\"target_energy\":" + String(targetEnergy, 3) + ",";
  json += "\"motion_detected\":" + String(motionDetected ? 1 : 0) + ",";
  json += "\"uart_valid\":" + String(c4001Ready ? 1 : 0) + ",";
  json += "\"c4001_frame_type\":\"" + c4001FrameType + "\",";
  json += "\"c4001_mode\":\"" + c4001WorkModeText + "\",";
  json += "\"is_night\":" + String(isNightTime() ? 1 : 0) + ",";
  json += "\"rssi_wifi\":" + String(WiFi.RSSI()) + ",";
  json += "\"uptime_ms\":" + String(millis()) + ",";
  json += "\"reason\":\"" + String(reason) + "\"";
  json += "}";

  Serial.println();
  Serial.println("POST JSON :");
  Serial.println(json);

  int httpCode = http.POST(json);
  String response = http.getString();

  lastHttpCode = httpCode;
  lastHttpResponse = response;
  lastReason = String(reason);

  Serial.print("HTTP code : "); Serial.println(httpCode);
  Serial.print("Reponse : "); Serial.println(response);

  http.end();
  return httpCode >= 200 && httpCode < 300;
}

// =====================================================
// WEBSERVER
// =====================================================
void handleRoot() {
  String html;
  html += "<!DOCTYPE html><html lang='fr'><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Radar ESP32 C4001</title>";
  html += "<style>body{font-family:Arial;margin:20px;background:#f4f4f4;} .card{background:white;padding:16px;margin-bottom:14px;border-radius:8px;box-shadow:0 1px 4px #ccc;} input{width:100%;padding:8px;margin:4px 0 10px 0;} label{font-weight:bold;} button,.btn{display:inline-block;padding:10px 14px;margin:4px;background:#1769aa;color:white;text-decoration:none;border:0;border-radius:5px;} .danger{background:#b00020;} code{background:#eee;padding:2px 4px;}</style>";
  html += "</head><body>";
  html += "<h1>Radar ESP32 C4001 24 GHz</h1>";

  html += "<div class='card'><h2>Statut</h2>";
  html += "<p><b>Device :</b> " + htmlEscape(deviceName) + "</p>";
  html += "<p><b>IP ESP32 :</b> " + WiFi.localIP().toString() + "</p>";
  html += "<p><b>Wi-Fi :</b> " + String(WiFi.status() == WL_CONNECTED ? "connecte" : "deconnecte") + " / RSSI " + String(WiFi.RSSI()) + " dBm</p>";
  html += "<p><b>Heure locale :</b> " + localTimeText() + " / nuit=" + boolText(isNightTime()) + "</p>";
  html += "<p><b>Uptime :</b> " + uptimeText() + "</p>";
  html += "</div>";

  html += "<div class='card'><h2>C4001</h2>";
  html += "<p><b>C4001 UART ASCII :</b> " + String(c4001Ready ? "OK" : "NON DETECTE") + "</p>";
  html += "<p><b>Mode C4001 vu :</b> " + htmlEscape(c4001WorkModeText) + " / trame=" + htmlEscape(c4001FrameType) + "</p>";
  html += "<p><b>Derniere commande :</b> <code>" + htmlEscape(lastC4001Command) + "</code> / sequence=" + htmlEscape(lastC4001SeqText) + "</p>";
  html += "<p><b>Lecture UART active :</b> " + String(c4001UartEnabled ? "OUI - parser ASCII $DFHPD" : "NON - mode securise OUT/GPIO") + "</p>";
  html += "<p><b>Derniere trame UART :</b> <code>" + htmlEscape(lastC4001Line) + "</code></p>";
  html += "<p><b>Trames UART :</b> OK=" + String(c4001FrameCount) + " / erreurs=" + String(c4001BadFrameCount) + " / age=" + String(c4001LastFrameMs == 0 ? 0 : (millis() - c4001LastFrameMs)) + " ms</p>";
  html += "<p><b>Presence brute :</b> " + String(rawPresence ? "PRESENCE" : "ABSENCE") + "</p>";
  html += "<p><b>Presence stable :</b> " + String(stablePresence ? "PRESENCE" : "ABSENCE") + "</p>";
  html += "<p><b>Target number :</b> " + String(targetNumber) + "</p>";
  html += "<p><b>Distance :</b> " + String(targetRangeM, 2) + " m / " + String(distanceCm()) + " cm</p>";
  html += "<p><b>Vitesse :</b> " + String(targetSpeedMS, 3) + " m/s / " + String(speedCms()) + " cm/s</p>";
  html += "<p><b>Energie :</b> " + String(targetEnergy, 3) + "</p>";
  html += "<p><b>Motion detection :</b> " + boolText(motionDetected) + "</p>";
  html += "<p><b>Dernier HTTP :</b> " + String(lastHttpCode) + " / raison=" + htmlEscape(lastReason) + "</p>";
  html += "<p><b>Derniere reponse :</b> <code>" + htmlEscape(lastHttpResponse) + "</code></p>";
  html += "<p><a class='btn' href='/testpost'>Envoyer un test POST</a> <a class='btn' href='/reinit_c4001'>Reinitialiser C4001</a> <a class='btn' href='/mode_speed'>Mode distance/vitesse</a> <a class='btn' href='/mode_presence'>Mode presence</a></p>";
  html += "</div>";

  html += "<div class='card'><h2>Configuration</h2>";
  html += "<form method='POST' action='/save'>";
  html += "<label>Nom capteur</label><input name='device' value='" + htmlEscape(deviceName) + "'>";
  html += "<label>URL endpoint Raspberry</label><input name='api_url' value='" + htmlEscape(apiUrl) + "'>";
  html += "<label>Token API (laisser vide pour conserver)</label><input name='api_token' type='password' value=''>";
  html += "<label>Heartbeat ms</label><input name='heartbeat' type='number' value='" + String(heartbeatMs) + "'>";
  html += "<label>Delai stabilite ms</label><input name='stable' type='number' value='" + String(stableDelayMs) + "'>";
  html += "<label>Mode nocturne seulement</label><input name='only_night' type='checkbox' " + String(sendOnlyNight ? "checked" : "") + ">";
  html += "<label>Heure debut nuit</label><input name='night_start' type='number' min='0' max='23' value='" + String(nightStartHour) + "'>";
  html += "<label>Heure fin nuit</label><input name='night_end' type='number' min='0' max='23' value='" + String(nightEndHour) + "'>";
  html += "<label>Distance min C4001 cm</label><input name='min_cm' type='number' value='" + String(detectMinCm) + "'>";
  html += "<label>Distance max C4001 cm</label><input name='max_cm' type='number' value='" + String(detectMaxCm) + "'>";
  html += "<label>Seuil detection C4001</label><input name='threshold' type='number' value='" + String(detectThreshold) + "'>";
  html += "<label>Fretting / micro-mouvements</label><input name='fretting' type='checkbox' " + String(frettingDetection ? "checked" : "") + ">";
  html += "<label>Activer lecture UART C4001 distance/vitesse</label><input name='uart_on' type='checkbox' " + String(c4001UartEnabled ? "checked" : "") + ">";
  html += "<label>Demander automatiquement le mode distance/vitesse au demarrage</label><input name='auto_speed' type='checkbox' " + String(autoSpeedMode ? "checked" : "") + ">";
  html += "<button type='submit'>Sauvegarder</button>";
  html += "</form></div>";

  html += "<div class='card'><h2>Maintenance</h2>";
  html += "<a class='btn' href='/reboot'>Redemarrer</a> ";
  html += "<a class='btn danger' href='/resetwifi'>Reset Wi-Fi</a>";
  html += "</div>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("device")) deviceName = server.arg("device");
  if (server.hasArg("api_url")) apiUrl = server.arg("api_url");
  if (server.hasArg("api_token") && server.arg("api_token").length() > 0) apiToken = server.arg("api_token");

  if (server.hasArg("heartbeat")) heartbeatMs = server.arg("heartbeat").toInt();
  if (server.hasArg("stable")) stableDelayMs = server.arg("stable").toInt();
  if (server.hasArg("night_start")) nightStartHour = constrain(server.arg("night_start").toInt(), 0, 23);
  if (server.hasArg("night_end")) nightEndHour = constrain(server.arg("night_end").toInt(), 0, 23);

  sendOnlyNight = server.hasArg("only_night");

  if (server.hasArg("min_cm")) detectMinCm = server.arg("min_cm").toInt();
  if (server.hasArg("max_cm")) detectMaxCm = server.arg("max_cm").toInt();
  if (server.hasArg("threshold")) detectThreshold = server.arg("threshold").toInt();
  frettingDetection = server.hasArg("fretting");
  c4001UartEnabled = server.hasArg("uart_on");
  autoSpeedMode = server.hasArg("auto_speed");

  if (heartbeatMs < 5000) heartbeatMs = 5000;
  if (stableDelayMs < 100) stableDelayMs = 100;
  if (detectMinCm < 30) detectMinCm = 30;
  if (detectMaxCm < detectMinCm + 100) detectMaxCm = detectMinCm + 100;

  saveConfig();
  pendingReinitC4001 = true;

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleTestPost() {
  sendRadarData(stablePresence, "manual_test");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleReboot() {
  server.send(200, "text/plain", "Redemarrage ESP32...");
  delay(1000);
  ESP.restart();
}

void handleResetWiFi() {
  server.send(200, "text/plain", "Reset Wi-Fi. Redemarrage...");
  delay(1000);
  WiFiManager wm;
  wm.resetSettings();
  WiFi.disconnect(true, true);
  delay(1000);
  ESP.restart();
}

void handleReinitC4001() {
  pendingReinitC4001 = true;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleModeSpeed() {
  c4001UartEnabled = true;
  autoSpeedMode = true;
  saveConfig();
  startC4001Sequence(SEQ_SPEED_MODE);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleModePresence() {
  c4001UartEnabled = true;
  autoSpeedMode = false;
  saveConfig();
  startC4001Sequence(SEQ_PRESENCE_MODE);
  server.sendHeader("Location", "/");
  server.send(303);
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/testpost", HTTP_GET, handleTestPost);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.on("/resetwifi", HTTP_GET, handleResetWiFi);
  server.on("/reinit_c4001", HTTP_GET, handleReinitC4001);
  server.on("/mode_speed", HTTP_GET, handleModeSpeed);
  server.on("/mode_presence", HTTP_GET, handleModePresence);
  Serial.println("Demarrage serveur HTML sur port 80...");
  server.begin();
  Serial.println("Serveur HTML OK : ouvrir http://" + WiFi.localIP().toString() + "/");
}

// =====================================================
// DEBUG
// =====================================================
void printDebug() {
  if (millis() - lastDebugPrintMs < debugMs) return;
  lastDebugPrintMs = millis();

  Serial.print("C4001_ASCII="); Serial.print(c4001Ready ? "OK" : "KO");
  Serial.print(" | RAW="); Serial.print(rawPresence ? "PRESENCE" : "ABSENCE");
  Serial.print(" | STABLE="); Serial.print(stablePresence ? "PRESENCE" : "ABSENCE");
  Serial.print(" | N="); Serial.print(targetNumber);
  Serial.print(" | distance_cm="); Serial.print(distanceCm());
  Serial.print(" | speed_m_s="); Serial.print(targetSpeedMS, 3);
  Serial.print(" | energy="); Serial.print(targetEnergy, 3);
  Serial.print(" | RSSI="); Serial.print(WiFi.status() == WL_CONNECTED ? String(WiFi.RSSI()) : "NO_WIFI");
  Serial.print(" | night="); Serial.print(isNightTime() ? "1" : "0");
  Serial.print(" | uptime="); Serial.println(millis());
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("ESP32 + C4001 - Radar 24 GHz");
  Serial.println("Firmware V6 ASCII $DFHPD/$DFDMD - WebServer prioritaire");
  Serial.println("WiFiManager + WebServer + REST/JSON");
  Serial.println("=================================");

  pinMode(C4001_OUT_PIN, INPUT);

  Serial.println("[1/5] Wi-Fi / portail captif...");
  setupWiFiPortal();

  Serial.println("[2/5] Demarrage immediat du serveur Web...");
  setupWebServer();
  for (int i = 0; i < 10; i++) {
    server.handleClient();
    delay(50);
  }

  Serial.println("[3/5] NTP...");
  setupTimeNTP();

  Serial.println("[4/5] Initialisation radar C4001 ASCII...");
  setupC4001();

  Serial.println("[5/5] Etat initial + premier POST...");
  readC4001();
  lastRawPresence = rawPresence;
  stablePresence = rawPresence;
  lastRawChangeMs = millis();
  if (stablePresence) presenceStartMs = millis();

  sendRadarData(stablePresence, "boot");
  lastSendMs = millis();

  Serial.println("Setup termine.");
  Serial.print("IHM locale : http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

void loop() {
  server.handleClient();

  if (pendingReinitC4001) {
    pendingReinitC4001 = false;
    setupC4001();
  }

  unsigned long now = millis();

  processC4001Sequence();
  readC4001();
  printDebug();

  if (rawPresence != lastRawPresence) {
    lastRawPresence = rawPresence;
    lastRawChangeMs = now;
    Serial.print("Changement brut C4001 : ");
    Serial.println(rawPresence ? "PRESENCE" : "ABSENCE");
  }

  if ((now - lastRawChangeMs) >= stableDelayMs) {
    if (stablePresence != rawPresence) {
      stablePresence = rawPresence;

      if (stablePresence) {
        presenceStartMs = now;
        Serial.println("Etat stable : PRESENCE");
        sendRadarData(true, "presence_start");
      } else {
        Serial.println("Etat stable : ABSENCE");
        sendRadarData(false, "presence_end");
        presenceStartMs = 0;
      }

      lastSendMs = now;
    }
  }

  if ((now - lastSendMs) >= heartbeatMs) {
    if ((now - lastRawChangeMs) >= stableDelayMs) {
      sendRadarData(stablePresence, "heartbeat");
      lastSendMs = now;
    }
  }

  delay(50);
}

/*
 * HLK-LD2450 Radar Web Dashboard
 * für ESP32-C6 SuperMini
 *
 * Pinbelegung:
 *   Sensor TX  →  ESP32-C6 GPIO4  (RX)
 *   Sensor RX  →  ESP32-C6 GPIO5  (TX)
 *   Sensor 5V  →  5V
 *   Sensor GND →  GND
 *
 * Protokoll: UART 256000 Baud, 8N1
 * Datenframe: AA FF 03 00 ... 55 CC  (30 Bytes, 10 Hz)
 *
 * Benötigte Bibliotheken (Arduino Library Manager):
 *   - ESPAsyncWebServer  (by lacamera / ESP Async WebServer)
 *   - AsyncTCP           (by dvarrel)
 *
 * HTTP  ws://<IP>/     → Dashboard
 * WS    ws://<IP>/ws   → WebSocket (Port 80, kein separater Port nötig)
 * REST  http://<IP>/json
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>   // ESPAsyncWebServer + AsyncWebSocket
#include "arduino_secrets.h"

// ─── Konfiguration ────────────────────────────────────────────────────────────
const char* WIFI_SSID     = SECRET_SSID;
const char* WIFI_PASSWORD = SECRET_PASS;

#define RADAR_RX_PIN  4       // GPIO4  ← Sensor TX
#define RADAR_TX_PIN  5       // GPIO5  → Sensor RX
#define RADAR_BAUD    256000

// ─── Globale Objekte ──────────────────────────────────────────────────────────
HardwareSerial  radarSerial(1);
AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");

// ─── Target-Struktur ──────────────────────────────────────────────────────────
struct Target {
  int16_t  x;       // mm  (negativ = links, positiv = rechts)
  int16_t  y;       // mm  (vor dem Sensor, immer positiv)
  int16_t  speed;   // cm/s (negativ = entfernt sich, positiv = nähert sich)
  uint16_t res;     // Distanz-Auflösung mm
  bool     active;
};

Target   targets[3];
uint32_t lastFrameMs = 0;

// ─── JSON zusammenbauen ───────────────────────────────────────────────────────
void buildJson(char* buf, size_t len) {
  snprintf(buf, len,
    "{\"t\":["
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"active\":%d},"
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"active\":%d},"
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"active\":%d}"
    "]}",
    targets[0].x, targets[0].y, targets[0].speed, targets[0].active ? 1 : 0,
    targets[1].x, targets[1].y, targets[1].speed, targets[1].active ? 1 : 0,
    targets[2].x, targets[2].y, targets[2].speed, targets[2].active ? 1 : 0
  );
}

// ─── WebSocket broadcast ──────────────────────────────────────────────────────
void pushWebSocket() {
  if (ws.count() == 0) return;   // niemand verbunden → nichts tun
  char json[256];
  buildJson(json, sizeof(json));
  ws.textAll(json);
}

// ─── Frame-Parser ─────────────────────────────────────────────────────────────
// Frame-Aufbau (30 Bytes):
//  0- 3: Header   AA FF 03 00
//  4-11: Target1  X_L X_H Y_L Y_H Spd_L Spd_H Res_L Res_H
// 12-19: Target2
// 20-27: Target3
// 28-29: Footer   55 CC
void parseFrame(const uint8_t* buf) {
  for (int i = 0; i < 3; i++) {
    const uint8_t* d = buf + 4 + i * 8;
    uint16_t xRaw   = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    uint16_t yRaw   = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
    uint16_t spdRaw = (uint16_t)d[4] | ((uint16_t)d[5] << 8);
    uint16_t resRaw = (uint16_t)d[6] | ((uint16_t)d[7] << 8);

    targets[i].x     = (xRaw   & 0x8000) ? -(int16_t)(xRaw   & 0x7FFF) : (int16_t)(xRaw   & 0x7FFF);
    targets[i].y     =                                                      (int16_t)(yRaw   & 0x7FFF);
    targets[i].speed = (spdRaw & 0x8000) ?  (int16_t)(spdRaw & 0x7FFF) : -(int16_t)(spdRaw & 0x7FFF);
    targets[i].res   = resRaw;
    targets[i].active = (xRaw != 0 || yRaw != 0);
  }
  lastFrameMs = millis();

  // Alle 50 Frames (~5 s) eine Debug-Zeile
  static uint32_t frameCount = 0;
  frameCount++;
  if (frameCount % 50 == 1) {
    Serial.printf("[Frame %5lu] ", frameCount);
    bool any = false;
    for (int i = 0; i < 3; i++) {
      if (targets[i].active) {
        any = true;
        float dist = sqrt((float)targets[i].x * targets[i].x +
                          (float)targets[i].y * targets[i].y);
        Serial.printf("T%d: X=%5dmm Y=%5dmm Spd=%4dcm/s Dist=%5.0fmm  ",
                      i+1, targets[i].x, targets[i].y, targets[i].speed, dist);
      }
    }
    if (!any) Serial.print("(kein Target)");
    Serial.println();
  }
}

// ─── UART-Empfang mit State Machine ──────────────────────────────────────────
void readRadar() {
  static uint8_t buf[30];
  static uint8_t idx = 0;

  while (radarSerial.available()) {
    uint8_t b = radarSerial.read();

    if (idx == 0 && b != 0xAA) return;
    if (idx == 1 && b != 0xFF) { idx = 0; return; }
    if (idx == 2 && b != 0x03) { idx = 0; return; }
    if (idx == 3 && b != 0x00) { idx = 0; return; }

    buf[idx++] = b;

    if (idx == 30) {
      if (buf[28] == 0x55 && buf[29] == 0xCC) {
        parseFrame(buf);
        pushWebSocket();
      }
      idx = 0;
    }
  }
}

// ─── WebSocket-Ereignisse ─────────────────────────────────────────────────────
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u verbunden von %s\n",
                  client->id(), client->remoteIP().toString().c_str());
    // Sofort aktuellen Stand senden
    char json[256];
    buildJson(json, sizeof(json));
    client->text(json);
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u getrennt\n", client->id());
  }
}

// ─── HTML Dashboard ───────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HLK-LD2450 Radar</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700&display=swap');

  :root {
    --bg:    #050d12;
    --panel: #0a1a24;
    --border:#0e3347;
    --t1:    #00ffe7;
    --t2:    #ff6b35;
    --t3:    #a855f7;
    --text:  #7ecfea;
    --dim:   #2a5570;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Share Tech Mono', monospace;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 12px;
  }
  h1 {
    font-family: 'Orbitron', sans-serif;
    font-size: clamp(0.75rem, 3vw, 1.1rem);
    letter-spacing: 0.2em;
    color: var(--t1);
    text-shadow: 0 0 8px #00ffe7aa;
    margin-bottom: 4px;
  }
  #status {
    font-size: 0.7rem;
    color: var(--dim);
    margin-bottom: 12px;
    letter-spacing: 0.1em;
  }
  #status.ok  { color: #00ff88; }
  #status.err { color: #ff4444; }

  .layout {
    display: flex;
    flex-wrap: wrap;
    gap: 12px;
    justify-content: center;
    width: 100%;
    max-width: 900px;
  }
  .radar-wrap {
    position: relative;
    /* responsiv: auf Smartphones kleiner */
    width:  min(420px, 95vw);
    height: min(420px, 95vw);
    flex-shrink: 0;
  }
  #radarCanvas {
    width: 100%; height: 100%;
    border: 1px solid var(--border);
    background: var(--panel);
    border-radius: 4px;
    display: block;
  }
  .cards {
    display: flex;
    flex-direction: column;
    gap: 8px;
    flex: 1;
    min-width: 180px;
  }
  .card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 10px 14px;
    transition: border-color 0.2s, box-shadow 0.2s;
  }
  .card.active-1 { border-color: var(--t1); box-shadow: 0 0 8px #00ffe7aa; }
  .card.active-2 { border-color: var(--t2); box-shadow: 0 0 8px #ff6b35aa; }
  .card.active-3 { border-color: var(--t3); box-shadow: 0 0 8px #a855f7aa; }
  .card-title {
    font-family: 'Orbitron', sans-serif;
    font-size: 0.6rem;
    letter-spacing: 0.15em;
    margin-bottom: 8px;
  }
  .card-title.c1 { color: var(--t1); }
  .card-title.c2 { color: var(--t2); }
  .card-title.c3 { color: var(--t3); }
  .row { display: flex; justify-content: space-between; font-size: 0.72rem; margin-bottom: 3px; }
  .label { color: var(--dim); }
  .val   { color: var(--text); }
  .val.big { font-size: 1rem; font-family: 'Orbitron', sans-serif; }
  .inactive .val { color: var(--dim); }
  #fps-bar { margin-top: 10px; font-size: 0.62rem; color: var(--dim); letter-spacing: 0.1em; }
</style>
</head>
<body>
<h1>HLK-LD2450 · RADAR TRACKER</h1>
<div id="status">VERBINDE...</div>
<div class="layout">
  <div class="radar-wrap"><canvas id="radarCanvas" width="420" height="420"></canvas></div>
  <div class="cards" id="cards"></div>
</div>
<div id="fps-bar">— · · ·</div>

<script>
const COLORS = ['#00ffe7','#ff6b35','#a855f7'];
const RANGE  = 6000;
const canvas = document.getElementById('radarCanvas');
const ctx    = canvas.getContext('2d');
const W = canvas.width, H = canvas.height;
const CX = W / 2, CY = 30;
const SCALE = (H - 50) / RANGE;

let targets = [{},{},{}];
let lastFrameTime = 0, fpsCount = 0, fps = 0;

// ── Karten aufbauen ───────────────────────────────────────────────────────────
const cardsEl = document.getElementById('cards');
for (let i = 0; i < 3; i++) {
  cardsEl.innerHTML += `
  <div class="card" id="card${i}">
    <div class="card-title c${i+1}">TARGET ${i+1}</div>
    <div class="row"><span class="label">STATUS</span><span class="val" id="t${i}_st">—</span></div>
    <div class="row"><span class="label">DIST</span><span class="val big" id="t${i}_d">—</span></div>
    <div class="row"><span class="label">X</span><span class="val" id="t${i}_x">—</span></div>
    <div class="row"><span class="label">Y</span><span class="val" id="t${i}_y">—</span></div>
    <div class="row"><span class="label">SPEED</span><span class="val" id="t${i}_s">—</span></div>
    <div class="row"><span class="label">WINKEL</span><span class="val" id="t${i}_a">—</span></div>
  </div>`;
}

// ── Radar zeichnen ────────────────────────────────────────────────────────────
function drawRadar() {
  try {
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#0a1a24';
    ctx.fillRect(0, 0, W, H);

    // Sweep
    const sweepAngle = (Date.now() / 3000) % (Math.PI * 2);
    ctx.save();
    ctx.translate(CX, CY);
    ctx.rotate(sweepAngle);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, H, 0, Math.PI / 6);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0,255,231,0.05)';
    ctx.fill();
    ctx.restore();

    // Entfernungsringe
    ctx.lineWidth = 1;
    [1500, 3000, 4500, 6000].forEach(r => {
      const pr = r * SCALE;
      ctx.strokeStyle = '#0d2f45';
      ctx.beginPath();
      ctx.arc(CX, CY, pr, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = '#1a4a66';
      ctx.font = '9px Share Tech Mono';
      ctx.fillText(`${r/1000}m`, CX + pr - 22, CY + 11);
    });

    // Winkellinien ±60°
    ctx.strokeStyle = '#0d2f45';
    [-60, -30, 0, 30, 60].forEach(deg => {
      const rad = (deg - 90) * Math.PI / 180;
      ctx.beginPath();
      ctx.moveTo(CX, CY);
      ctx.lineTo(CX + Math.cos(rad) * RANGE * SCALE, CY + Math.sin(rad) * RANGE * SCALE);
      ctx.stroke();
    });

    // Sensor-Punkt
    ctx.fillStyle = '#00ffe7';
    ctx.shadowBlur = 8; ctx.shadowColor = '#00ffe7';
    ctx.beginPath(); ctx.arc(CX, CY, 4, 0, Math.PI * 2); ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#2a8aaa';
    ctx.font = '9px Share Tech Mono';
    ctx.fillText('SENSOR', CX - 22, CY - 8);

    // Targets zeichnen
    targets.forEach((t, i) => {
      if (!t.active) return;
      const px = CX + t.x * SCALE;
      const py = CY + t.y * SCALE;
      if (py < -10 || py > H + 10 || px < -10 || px > W + 10) return;

      // Trail
      const trailLen = Math.min(Math.abs(t.speed) * 0.4, 40);
      const trailDir = t.speed >= 0 ? -1 : 1;
      ctx.strokeStyle = COLORS[i] + '55';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py + trailLen * trailDir); ctx.stroke();

      // Punkt
      ctx.fillStyle = COLORS[i];
      ctx.shadowBlur = 12; ctx.shadowColor = COLORS[i];
      ctx.beginPath(); ctx.arc(px, py, 6, 0, Math.PI * 2); ctx.fill();
      ctx.shadowBlur = 0;

      // Label
      ctx.fillStyle = COLORS[i];
      ctx.font = 'bold 10px Share Tech Mono';
      ctx.fillText(`T${i+1}`, px + 9, py - 4);
    });
  } catch(e) { console.error('drawRadar:', e); }
  requestAnimationFrame(drawRadar);
}
requestAnimationFrame(drawRadar);

// ── Karten aktualisieren ──────────────────────────────────────────────────────
function updateCards(ts) {
  const now = Date.now();
  fpsCount++;
  if (now - lastFrameTime >= 1000) {
    fps = fpsCount; fpsCount = 0; lastFrameTime = now;
    document.getElementById('fps-bar').textContent =
      `FRAME RATE ${fps} Hz · LETZTER FRAME ${now - (window._lastRadar||now)} ms`;
  }
  ts.forEach((t, i) => {
    const dist = t.active ? Math.round(Math.sqrt(t.x*t.x + t.y*t.y)) : null;
    const ang  = t.active ? Math.round(Math.atan2(t.x, t.y) * 180 / Math.PI) : null;
    document.getElementById(`card${i}`).className =
      'card' + (t.active ? ` active-${i+1}` : ' inactive');
    document.getElementById(`t${i}_st`).textContent = t.active ? 'ERKANNT' : 'KEIN TARGET';
    document.getElementById(`t${i}_d`).textContent  = dist != null ? `${dist} mm` : '—';
    document.getElementById(`t${i}_x`).textContent  = t.active ? `${t.x} mm`     : '—';
    document.getElementById(`t${i}_y`).textContent  = t.active ? `${t.y} mm`     : '—';
    document.getElementById(`t${i}_s`).textContent  = t.active ? `${t.speed} cm/s` : '—';
    document.getElementById(`t${i}_a`).textContent  = ang != null ? `${ang}°`    : '—';
  });
}

// ── WebSocket (Port 80, Pfad /ws) ─────────────────────────────────────────────
const wsUrl    = `ws://${location.host}/ws`;
const statusEl = document.getElementById('status');
let   sock, reconnectTimer;

function connect() {
  sock = new WebSocket(wsUrl);
  sock.onopen  = () => {
    statusEl.textContent = 'VERBUNDEN · ' + location.hostname;
    statusEl.className   = 'ok';
  };
  sock.onclose = () => {
    statusEl.textContent = 'GETRENNT – VERBINDE NEU...';
    statusEl.className   = 'err';
    reconnectTimer = setTimeout(connect, 2000);
  };
  sock.onerror = () => sock.close();
  sock.onmessage = e => {
    window._lastRadar = Date.now();
    try {
      const data = JSON.parse(e.data);
      targets = data.t.map(t => ({ ...t, active: t.active === 1 }));
      updateCards(targets);
    } catch(ex) { console.error('JSON:', ex); }
  };
}
connect();
</script>
</body>
</html>
)rawhtml";

// ─── HTTP-Handler ─────────────────────────────────────────────────────────────
void handleJson(AsyncWebServerRequest* request) {
  char buf[256];
  buildJson(buf, sizeof(buf));
  request->send(200, "application/json", buf);
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);  // ESP32-C6 USB-CDC braucht kurz

  Serial.println();
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║   HLK-LD2450  Radar Web Dashboard    ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.printf ("Chip: ESP32-C6  |  CPU: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf ("Freier Heap: %d Bytes\n\n", ESP.getFreeHeap());

  // -- Radar-UART --
  Serial.println("[ 1/4 ] Radar UART initialisieren...");
  radarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  Serial.printf ("        RX = GPIO%d  (<- Sensor TX)\n", RADAR_RX_PIN);
  Serial.printf ("        TX = GPIO%d  (-> Sensor RX)\n", RADAR_TX_PIN);
  Serial.printf ("        Baudrate: %d\n", RADAR_BAUD);
  Serial.println("        OK\n");

  // -- WLAN --
  Serial.println("[ 2/4 ] WLAN verbinden...");
  Serial.printf ("        SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); Serial.print('.'); tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("        WLAN verbunden!");
    Serial.printf ("        IP-Adresse : http://%s\n", WiFi.localIP().toString().c_str());
    Serial.printf ("        Gateway    : %s\n",        WiFi.gatewayIP().toString().c_str());
    Serial.printf ("        RSSI       : %d dBm\n",    WiFi.RSSI());
    Serial.printf ("        MAC        : %s\n\n",      WiFi.macAddress().c_str());
  } else {
    Serial.println("        WLAN fehlgeschlagen - starte Access Point!");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("LD2450-Radar", "radar1234");
    delay(500);
    Serial.println("        AP-SSID    : LD2450-Radar");
    Serial.println("        AP-Passwort: radar1234");
    Serial.printf ("        AP-IP      : http://%s\n\n", WiFi.softAPIP().toString().c_str());
  }

  // -- WebSocket auf /ws --
  Serial.println("[ 3/4 ] WebSocket einrichten...");
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  Serial.println("        Pfad: /ws  (Port 80)  OK\n");

  // -- HTTP-Routen --
  Serial.println("[ 4/4 ] HTTP-Server starten...");
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.printf("[HTTP] GET /  von %s\n",
                  request->client()->remoteIP().toString().c_str());
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server.on("/json", HTTP_GET, handleJson);
  server.begin();
  Serial.println("        Port 80  OK\n");

  Serial.println("==========================================");
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf ("  Dashboard : http://%s\n", WiFi.localIP().toString().c_str());
  else
    Serial.printf ("  Dashboard : http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("  WebSocket : ws://<IP>/ws  (Port 80)");
  Serial.println("  REST      : http://<IP>/json");
  Serial.println("  Radar-Frames werden unten geloggt...");
  Serial.println("==========================================\n");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  readRadar();
  ws.cleanupClients();  // getrennte Clients aufräumen

  // Warnung wenn seit 5 s kein Frame ankam
  static uint32_t lastWarnMs = 0;
  if (lastFrameMs > 0 && millis() - lastFrameMs > 5000 && millis() - lastWarnMs > 5000) {
    Serial.printf("[WARN] Kein Radar-Frame seit %lu s!\n", (millis() - lastFrameMs) / 1000);
    lastWarnMs = millis();
  }
  static bool firstFrameSeen = false;
  if (!firstFrameSeen && lastFrameMs > 0) {
    firstFrameSeen = true;
    Serial.println("[INFO] Erster Radar-Frame empfangen - Sensor kommuniziert!");
  }
}
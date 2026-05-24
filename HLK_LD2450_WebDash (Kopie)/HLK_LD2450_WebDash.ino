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
 * Jedes Target: 8 Bytes (X lo/hi, Y lo/hi, Speed lo/hi, Resolution lo/hi)
 */
 
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>   // Bibliothek: "WebSockets" von Markus Sattler
#include "arduino_secrets.h"

// ─── Konfiguration ────────────────────────────────────────────────────────────
const char* WIFI_SSID     = SECRET_SSID;
const char* WIFI_PASSWORD = SECRET_PASS;

#define RADAR_RX_PIN  4    // ESP32-C6 GPIO4  ← Sensor TX
#define RADAR_TX_PIN  5    // ESP32-C6 GPIO5  → Sensor RX
#define RADAR_BAUD    256000

// ─── Globale Objekte ──────────────────────────────────────────────────────────
HardwareSerial radarSerial(1);   // UART1
WebServer      httpServer(80);
WebSocketsServer wsServer(81);

// ─── Target-Struktur ──────────────────────────────────────────────────────────
struct Target {
  int16_t  x;       // mm  (negativ = links, positiv = rechts)
  int16_t  y;       // mm  (immer positiv = vor dem Sensor)
  int16_t  speed;   // cm/s (negativ = weg, positiv = nah)
  uint16_t res;     // Distanz-Auflösung mm
  bool     active;
};

Target targets[3];
uint32_t lastFrameMs = 0;

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

    // X: Bit15=1 → negativ (links), Bit15=0 → positiv (rechts)
    targets[i].x = (xRaw & 0x8000) ? -(int16_t)(xRaw & 0x7FFF)
                                    :  (int16_t)(xRaw & 0x7FFF);
    targets[i].y = (int16_t)(yRaw & 0x7FFF);

    // Speed: Bit15=1 → positiv (nähert sich), Bit15=0 → negativ (entfernt)
    targets[i].speed = (spdRaw & 0x8000) ?  (int16_t)(spdRaw & 0x7FFF)
                                          : -(int16_t)(spdRaw & 0x7FFF);
    targets[i].res    = resRaw;
    targets[i].active = (xRaw != 0 || yRaw != 0);
  }
  lastFrameMs = millis();

  // Alle 50 Frames (ca. 5 Sekunden) eine Zeile ausgeben
  static uint32_t frameCount = 0;
  frameCount++;
  if (frameCount % 50 == 1) {
    Serial.printf("[Frame %5lu] ", frameCount);
    bool anyActive = false;
    for (int i = 0; i < 3; i++) {
      if (targets[i].active) {
        anyActive = true;
        float dist = sqrt((float)targets[i].x * targets[i].x +
                          (float)targets[i].y * targets[i].y);
        Serial.printf("T%d: X=%5dmm Y=%5dmm Spd=%4dcm/s Dist=%5.0fmm  ",
                      i+1, targets[i].x, targets[i].y, targets[i].speed, dist);
      }
    }
    if (!anyActive) Serial.print("(kein Target erkannt)");
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

// ─── WebSocket: JSON senden ────────────────────────────────────────────────────
void pushWebSocket() {
  char json[256];
  snprintf(json, sizeof(json),
    "{\"t\":["
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"active\":%d},"
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"active\":%d},"
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"active\":%d}"
    "]}",
    targets[0].x, targets[0].y, targets[0].speed, targets[0].active ? 1 : 0,
    targets[1].x, targets[1].y, targets[1].speed, targets[1].active ? 1 : 0,
    targets[2].x, targets[2].y, targets[2].speed, targets[2].active ? 1 : 0
  );
  wsServer.broadcastTXT(json);
}

// ─── WebSocket-Ereignisse ─────────────────────────────────────────────────────
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = wsServer.remoteIP(num);
    Serial.printf("[WS] Client #%d verbunden von %s\n", num, ip.toString().c_str());
    pushWebSocket();
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Client #%d getrennt\n", num);
  }
}

// ─── HTML Dashboard (inline) ─────────────────────────────────────────────────
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
    --bg:      #050d12;
    --panel:   #0a1a24;
    --border:  #0e3347;
    --grid:    #0a2233;
    --t1:      #00ffe7;
    --t2:      #ff6b35;
    --t3:      #a855f7;
    --text:    #7ecfea;
    --dim:     #2a5570;
    --glow1:   0 0 8px #00ffe7aa;
    --glow2:   0 0 8px #ff6b35aa;
    --glow3:   0 0 8px #a855f7aa;
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
    padding: 16px;
  }
  h1 {
    font-family: 'Orbitron', sans-serif;
    font-size: 1.1rem;
    letter-spacing: 0.2em;
    color: var(--t1);
    text-shadow: var(--glow1);
    margin-bottom: 4px;
  }
  #status {
    font-size: 0.7rem;
    color: var(--dim);
    margin-bottom: 14px;
    letter-spacing: 0.1em;
  }
  #status.ok  { color: #00ff88; }
  #status.err { color: #ff4444; }

  .layout {
    display: flex;
    flex-wrap: wrap;
    gap: 16px;
    justify-content: center;
    width: 100%;
    max-width: 900px;
  }

  /* ── Radar-Canvas ── */
  .radar-wrap {
    position: relative;
    width: 420px;
    height: 420px;
    flex-shrink: 0;
  }
  #radarCanvas {
    width: 100%;
    height: 100%;
    border: 1px solid var(--border);
    background: var(--panel);
    border-radius: 4px;
  }

  /* ── Target-Karten ── */
  .cards {
    display: flex;
    flex-direction: column;
    gap: 10px;
    flex: 1;
    min-width: 200px;
  }
  .card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 12px 16px;
    transition: border-color 0.2s, box-shadow 0.2s;
  }
  .card.active-1 { border-color: var(--t1); box-shadow: var(--glow1); }
  .card.active-2 { border-color: var(--t2); box-shadow: var(--glow2); }
  .card.active-3 { border-color: var(--t3); box-shadow: var(--glow3); }
  .card-title {
    font-family: 'Orbitron', sans-serif;
    font-size: 0.65rem;
    letter-spacing: 0.15em;
    margin-bottom: 10px;
  }
  .card-title.c1 { color: var(--t1); }
  .card-title.c2 { color: var(--t2); }
  .card-title.c3 { color: var(--t3); }
  .row {
    display: flex;
    justify-content: space-between;
    font-size: 0.75rem;
    margin-bottom: 4px;
  }
  .label { color: var(--dim); }
  .val   { color: var(--text); }
  .val.big {
    font-size: 1.1rem;
    font-family: 'Orbitron', sans-serif;
  }
  .inactive .val { color: var(--dim); }

  #fps-bar {
    margin-top: 14px;
    font-size: 0.65rem;
    color: var(--dim);
    letter-spacing: 0.1em;
    text-align: center;
  }
</style>
</head>
<body>
<h1>HLK-LD2450 · RADAR TRACKER</h1>
<div id="status">VERBINDE...</div>

<div class="layout">
  <div class="radar-wrap">
    <canvas id="radarCanvas" width="420" height="420"></canvas>
  </div>
  <div class="cards" id="cards"></div>
</div>
<div id="fps-bar">FRAME — · · ·</div>

<script>
const COLORS = ['#00ffe7','#ff6b35','#a855f7'];
const RANGE  = 6000;   // mm  Sensorreichweite
const canvas = document.getElementById('radarCanvas');
const ctx    = canvas.getContext('2d');
const W = canvas.width, H = canvas.height;

// Sensor sitzt oben-mitte, Y nach unten = in den Raum
const CX = W / 2, CY = 30;
const SCALE = (H - 50) / RANGE;   // Pixel pro mm

let targets = [{},{},{}];
let lastFrameTime = 0, fps = 0, frameCount = 0;

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

  // Hintergrund
  ctx.fillStyle = '#0a1a24';
  ctx.fillRect(0, 0, W, H);

  // Sweep-Animation (dekorativ, nur mit Standard-Canvas-APIs)
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
  ctx.strokeStyle = '#0d2f45';
  ctx.lineWidth = 1;
  [1500,3000,4500,6000].forEach(r => {
    const pr = r * SCALE;
    ctx.beginPath();
    ctx.arc(CX, CY, pr, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = '#1a4a66';
    ctx.font = '9px Share Tech Mono';
    ctx.fillText(`${r/1000}m`, CX + pr - 22, CY + 11);
  });

  // Winkellinien ±60°
  ctx.strokeStyle = '#0d2f45';
  [-60,-30,0,30,60].forEach(deg => {
    const rad = (deg - 90) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(CX, CY);
    ctx.lineTo(CX + Math.cos(rad)*RANGE*SCALE, CY + Math.sin(rad)*RANGE*SCALE);
    ctx.stroke();
  });

  // Sensor-Markierung
  ctx.fillStyle = '#00ffe7';
  ctx.shadowBlur = 8;
  ctx.shadowColor = '#00ffe7';
  ctx.beginPath();
  ctx.arc(CX, CY, 4, 0, Math.PI*2);
  ctx.fill();
  ctx.shadowBlur = 0;

  // Sensor-Label
  ctx.fillStyle = '#2a8aaa';
  ctx.font = '9px Share Tech Mono';
  ctx.fillText('SENSOR', CX - 22, CY - 8);

  // Targets
  targets.forEach((t, i) => {
    if (!t.active) return;
    const px = CX + t.x * SCALE;
    const py = CY + t.y * SCALE;
    if (py < 0 || py > H || px < 0 || px > W) return;

    // Trail (Geschwindigkeit visualisieren)
    const trailLen = Math.min(Math.abs(t.speed) * 0.4, 40);
    const trailDir = t.speed >= 0 ? -1 : 1; // nähert: trail nach oben
    ctx.strokeStyle = COLORS[i] + '55';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px, py + trailLen * trailDir);
    ctx.stroke();

    // Punkt
    ctx.fillStyle = COLORS[i];
    ctx.shadowBlur = 12;
    ctx.shadowColor = COLORS[i];
    ctx.beginPath();
    ctx.arc(px, py, 6, 0, Math.PI*2);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Label
    ctx.fillStyle = COLORS[i];
    ctx.font = 'bold 10px Share Tech Mono';
    ctx.fillText(`T${i+1}`, px + 9, py - 4);
  });

  } catch(e) { console.error('drawRadar Fehler:', e); }
  requestAnimationFrame(drawRadar);
}
requestAnimationFrame(drawRadar);

// ── DOM-Update ────────────────────────────────────────────────────────────────
function updateCards(ts) {
  const now = Date.now();
  frameCount++;
  if (now - lastFrameTime >= 1000) {
    fps = frameCount;
    frameCount = 0;
    lastFrameTime = now;
    document.getElementById('fps-bar').textContent =
      `FRAME RATE ${fps} Hz · LETZTER FRAME ${((now - window._lastRadar)||0)} ms`;
  }

  ts.forEach((t, i) => {
    const dist = t.active ? Math.round(Math.sqrt(t.x*t.x + t.y*t.y)) : null;
    const ang  = t.active ? Math.round(Math.atan2(t.x, t.y) * 180 / Math.PI) : null;

    const card = document.getElementById(`card${i}`);
    card.className = 'card' + (t.active ? ` active-${i+1}` : ' inactive');

    document.getElementById(`t${i}_st`).textContent = t.active ? 'ERKANNT' : 'KEIN TARGET';
    document.getElementById(`t${i}_d`).textContent  = dist != null ? `${dist} mm` : '—';
    document.getElementById(`t${i}_x`).textContent  = t.active ? `${t.x} mm` : '—';
    document.getElementById(`t${i}_y`).textContent  = t.active ? `${t.y} mm` : '—';
    document.getElementById(`t${i}_s`).textContent  = t.active ? `${t.speed} cm/s` : '—';
    document.getElementById(`t${i}_a`).textContent  = ang != null ? `${ang}°` : '—';
  });
}

// ── WebSocket ─────────────────────────────────────────────────────────────────
const wsUrl = `ws://${location.hostname}:81/`;
let   ws, reconnectTimer;
const statusEl = document.getElementById('status');

function connect() {
  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    statusEl.textContent = 'VERBUNDEN · ' + location.hostname;
    statusEl.className   = 'ok';
  };
  ws.onclose = () => {
    statusEl.textContent = 'GETRENNT – VERBINDE NEU...';
    statusEl.className   = 'err';
    reconnectTimer = setTimeout(connect, 2000);
  };
  ws.onerror = () => ws.close();

  ws.onmessage = e => {
    window._lastRadar = Date.now();
    try {
      const data = JSON.parse(e.data);
      // active kommt als 0/1 vom ESP32, in Boolean umwandeln
      targets = data.t.map(t => ({ ...t, active: t.active === 1 }));
      updateCards(targets);
    } catch(ex) { console.error('JSON Fehler:', ex, e.data); }
  };
}
connect();
</script>
</body>
</html>
)rawhtml";

// ─── HTTP-Handler ──────────────────────────────────────────────────────────────
void handleRoot() {
  Serial.printf("[HTTP] GET /  von %s\n", httpServer.client().remoteIP().toString().c_str());
  httpServer.send_P(200, "text/html", INDEX_HTML);
}

void handleJson() {
  char buf[256];
  float dist[3];
  for (int i = 0; i < 3; i++) {
    dist[i] = sqrt((float)targets[i].x * targets[i].x +
                   (float)targets[i].y * targets[i].y);
  }
  snprintf(buf, sizeof(buf),
    "{\"targets\":["
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"dist\":%.0f,\"active\":%s},"
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"dist\":%.0f,\"active\":%s},"
    "{\"x\":%d,\"y\":%d,\"speed\":%d,\"dist\":%.0f,\"active\":%s}"
    "]}",
    targets[0].x, targets[0].y, targets[0].speed, dist[0], targets[0].active?"true":"false",
    targets[1].x, targets[1].y, targets[1].speed, dist[1], targets[1].active?"true":"false",
    targets[2].x, targets[2].y, targets[2].speed, dist[2], targets[2].active?"true":"false"
  );
  httpServer.sendHeader("Access-Control-Allow-Origin","*");
  httpServer.send(200, "application/json", buf);
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  // ESP32-C6 USB-CDC braucht kurz bis der Port im Monitor erscheint
  delay(2000);

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
    delay(500);
    Serial.print('.');
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("        WLAN verbunden!");
    Serial.printf ("        IP-Adresse : http://%s\n",   WiFi.localIP().toString().c_str());
    Serial.printf ("        Gateway    : %s\n",           WiFi.gatewayIP().toString().c_str());
    Serial.printf ("        RSSI       : %d dBm\n",       WiFi.RSSI());
    Serial.printf ("        MAC        : %s\n\n",         WiFi.macAddress().c_str());
  } else {
    Serial.println("        WLAN fehlgeschlagen - starte Access Point!");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("LD2450-Radar", "radar1234");
    delay(500);
    Serial.println("        AP-SSID    : LD2450-Radar");
    Serial.println("        AP-Passwort: radar1234");
    Serial.printf ("        AP-IP      : http://%s\n\n", WiFi.softAPIP().toString().c_str());
  }

  // -- HTTP --
  Serial.println("[ 3/4 ] HTTP-Server starten...");
  httpServer.on("/",     handleRoot);
  httpServer.on("/json", handleJson);
  httpServer.begin();
  Serial.println("        Port 80  OK\n");

  // -- WebSocket --
  Serial.println("[ 4/4 ] WebSocket-Server starten...");
  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.println("        Port 81  OK\n");

  Serial.println("==========================================");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf ("  Dashboard: http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.printf ("  Dashboard: http://%s\n", WiFi.softAPIP().toString().c_str());
  }
  Serial.println("  Radar-Frames werden unten geloggt...");
  Serial.println("==========================================\n");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  readRadar();
  httpServer.handleClient();
  wsServer.loop();

  // Warnung wenn seit 5 Sekunden kein Frame mehr kam
  static uint32_t lastWarnMs = 0;
  if (lastFrameMs > 0 && millis() - lastFrameMs > 5000 && millis() - lastWarnMs > 5000) {
    Serial.printf("[WARN] Kein Radar-Frame seit %lu Sekunden!\n",
                  (millis() - lastFrameMs) / 1000);
    lastWarnMs = millis();
  }
  // Hinweis solange noch gar kein Frame ankam
  static bool firstFrameSeen = false;
  if (!firstFrameSeen && lastFrameMs > 0) {
    firstFrameSeen = true;
    Serial.println("[INFO] Erster Radar-Frame empfangen - Sensor kommuniziert!");
  }
}
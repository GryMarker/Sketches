#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <vl53l7cx_class.h>

// ---------------- Direction helpers ----------------
enum Direction { DIR_LEFT, DIR_SLIGHTLY_LEFT, DIR_CENTER, DIR_SLIGHTLY_RIGHT, DIR_RIGHT, DIR_UNKNOWN };

static inline Direction dirFromColFW(int col){
  float x = col / 7.0f;
  if (x < 0.20f) return DIR_LEFT;
  if (x < 0.40f) return DIR_SLIGHTLY_LEFT;
  if (x < 0.60f) return DIR_CENTER;
  if (x < 0.80f) return DIR_SLIGHTLY_RIGHT;
  return DIR_RIGHT;
}

static inline uint8_t directionToTrackIndex(Direction d){
  // 1=center, 2=left, 3=right, 4=slightly
  switch (d){
    case DIR_CENTER: return 1;
    case DIR_LEFT:   return 2;
    case DIR_RIGHT:  return 3;
    case DIR_SLIGHTLY_LEFT:
    case DIR_SLIGHTLY_RIGHT: return 4;
    default: return 0;
  }
}

static inline const char* dirToText(Direction d){
  switch(d){
    case DIR_CENTER:          return "center";
    case DIR_LEFT:            return "left";
    case DIR_RIGHT:           return "right";
    case DIR_SLIGHTLY_LEFT:   return "slightly left";
    case DIR_SLIGHTLY_RIGHT:  return "slightly right";
    default:                  return "—";
  }
}

// ---------------- Pins & config ----------------
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
static const int LPN_PIN = -1;      

// JQ6500 on UART2
static const int JQ_RX = 16;        // ESP32 RX2  <- JQ6500 TX
static const int JQ_TX = 17;        // ESP32 TX2  -> JQ6500 RX

// Wi-Fi AP
const char* AP_SSID = "SmartCane_AP";
const char* AP_PASS = "12345678";
WebServer server(80);

// ---------------- User settings (persisted) ----------------
Preferences prefs;
struct Settings {
  uint16_t range_cm;  // speak/visualize up to this distance (cm)
  uint8_t  freq_hz;   // ranging frequency (Hz)
};
Settings g_set;

// derived runtime values
static uint16_t g_maxRangeMm = 200;    // default 20cm
static uint32_t g_frameIntervalMs = 500; // default 2 Hz

// bounds
static const uint16_t RANGE_CM_MIN = 5;    // 5 cm
static const uint16_t RANGE_CM_MAX = 200;  // 2 m 
static const uint8_t  FREQ_HZ_MIN  = 1;    // 1 Hz
static const uint8_t  FREQ_HZ_MAX  = 10;   // 10 Hz

// Audio debounce
static const uint32_t COOLDOWN_MS = 1500;
static const uint16_t DELTA_CM_FOR_REPEAT = 5;

// ---------------- Sensor & MP3 ----------------
static const uint16_t VL53L7CX_I2C_ADDR = 0x52;
VL53L7CX tof(&Wire, LPN_PIN, VL53L7CX_I2C_ADDR);

HardwareSerial MP3(2);
static inline void jqSend(uint8_t cmd, int nArgs=0, uint8_t a1=0, uint8_t a2=0){
  uint8_t len = 1 + nArgs + 1;
  uint8_t buf[6]; int i=0;
  buf[i++]=0x7E; buf[i++]=len; buf[i++]=cmd;
  if(nArgs>=1) buf[i++]=a1;
  if(nArgs>=2) buf[i++]=a2;
  buf[i++]=0xEF;
  MP3.write(buf,i);
  MP3.flush();
}
static inline void jqSetVolume(uint8_t v){ if(v>30) v=30; jqSend(0x06,1,v); }
static inline void jqPlayIndex(uint16_t idx){ jqSend(0x03,2,uint8_t(idx>>8),uint8_t(idx&0xFF)); }

// ---------------- Shared (cached) frame ----------------
struct Frame {
  uint16_t grid[64];      // clipped to 0..range_mm (0=invalid/far)
  uint16_t nearest_mm;    // 0 if none in range
  int      nearest_idx;   // 0..63 or -1
  Direction dir;
  uint32_t  timestamp_ms; // millis() of last update
  bool      valid;
} g_frame;

// ---------------- UI (heatmap + editable settings) ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Smart Cane — 8×8 Heatmap</title>
<style>
  :root{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}
  body{margin:0;padding:20px;background:#0b0d14;color:#e8ecf1}
  .wrap{max-width:820px;margin:0 auto}
  h1{margin:0 0 6px;font-size:1.25rem}
  .row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
  .badge{display:inline-block;padding:4px 10px;border-radius:999px;background:#1f2a44;color:#a9c1ff;font-size:.85rem}
  .grid{margin-top:14px;display:grid;grid-template-columns:repeat(8,1fr);gap:6px}
  .cell{aspect-ratio:1/1;background:#0e1422;border-radius:6px;display:flex;align-items:center;justify-content:center;font-size:11px;color:#c8d6f0}
  .metric{font-size:2.2rem;font-weight:700}
  .muted{color:#8ea2c4}
  .footer{margin-top:12px;font-size:.85rem;color:#7588a8}
  input{background:#0e1422;color:#e8ecf1;border:1px solid #2b3653;border-radius:8px;padding:8px}
  button{background:#2c7be5;color:#fff;border:none;padding:8px 12px;border-radius:8px;font-weight:600;cursor:pointer}
  .card{background:#121826;border-radius:14px;padding:12px;margin-top:12px}
  label{display:flex;align-items:center;gap:8px}
</style>
</head><body>
<div class="wrap">
  <h1>Smart Cane — 8×8 Heatmap</h1>
  <div class="row"><span class="badge" id="stat">Sensor: …</span><span class="badge">AP: SmartCane_AP</span></div>

  <div class="card">
    <div class="row">
      <label>Detection range (cm) <input id="range" type="number" min="5" max="200" step="1" value="20" style="width:100px"/></label>
      <label>Frequency (Hz) <input id="freq" type="number" min="1" max="10" step="1" value="2" style="width:90px"/></label>
      <button id="save">Save</button>
      <span class="muted" id="saveMsg"></span>
    </div>
  </div>

  <div class="row" style="margin-top:10px">
    <div class="metric"><span id="near">—</span> <small>cm</small></div>
    <div class="muted" id="dir">—</div>
    <div class="muted" id="when">Waiting for first frame…</div>
  </div>

  <div class="grid" id="grid"></div>
  <div class="footer">Audio plays on the JQ6500 when the nearest object is within the selected range.</div>
</div>
<script>
const gridEl=document.getElementById('grid'), statEl=document.getElementById('stat');
const nearEl=document.getElementById('near'), dirEl=document.getElementById('dir'), whenEl=document.getElementById('when');
const rangeEl=document.getElementById('range'), freqEl=document.getElementById('freq'), saveBtn=document.getElementById('save'), saveMsg=document.getElementById('saveMsg');

for(let i=0;i<64;i++){const c=document.createElement('div');c.className='cell';gridEl.appendChild(c)}
const cells=[...document.querySelectorAll('.cell')];

let pollMs = 500;

function loadSettings(){
  fetch('/settings').then(r=>r.json()).then(s=>{
    if(!s.ok) return;
    rangeEl.value = s.range_cm;
    freqEl.value  = s.freq_hz;
    // compute poll interval ~ frame period, cap 150..1000 ms
    const f = Math.max(1, Math.min(10, parseInt(freqEl.value||'2',10)));
    pollMs = Math.max(150, Math.min(1000, Math.round(1000/f)));
  });
}
loadSettings();

saveBtn.onclick = ()=>{
  const rc = parseInt(rangeEl.value||'20',10);
  const fh = parseInt(freqEl.value||'2',10);
  fetch(`/set?range_cm=${rc}&freq_hz=${fh}`).then(r=>r.json()).then(s=>{
    if(s.ok){
      saveMsg.textContent = 'Saved ✓';
      const f = Math.max(1, Math.min(10, s.freq_hz));
      pollMs = Math.max(150, Math.min(1000, Math.round(1000/f)));
      setTimeout(()=>saveMsg.textContent='', 1200);
    } else {
      saveMsg.textContent = 'Save failed';
    }
  }).catch(()=>saveMsg.textContent='Save failed');
};

function colorize(vals){
  const valid = vals.filter(v=>v>0);
  if(!valid.length){ cells.forEach(c=>{c.style.background='#0e1422'; c.textContent='';}); return; }
  const mn=Math.min(...valid), mx=Math.max(...valid), rg=Math.max(1,mx-mn);
  vals.forEach((v,i)=>{
    if(v>0){
      const t=(v-mn)/rg, shade=Math.round(20+(1-t)*220);
      cells[i].style.background=`rgb(${shade*0.4},${shade*0.55},${Math.min(255,shade+30)})`;
      cells[i].textContent=v;
    } else {
      cells[i].style.background='#0e1422';
      cells[i].textContent='';
    }
  });
}

function poll(){
  fetch('/frame',{cache:'no-store'}).then(r=>r.json()).then(d=>{
    if(!d.ok){ statEl.textContent='Sensor: not ready'; return; }
    statEl.textContent='Sensor: online';
    colorize(d.grid_mm);
    const cm = d.nearest_cm;
    nearEl.textContent = cm>0? cm : '—';
    dirEl.textContent = d.direction || '—';
    whenEl.textContent = 'Last: '+new Date(d.ts).toLocaleTimeString();
  }).catch(()=>{ statEl.textContent='Sensor: error'; })
    .finally(()=>setTimeout(poll, pollMs));
}
poll();
</script>
</body></html>
)HTML";

// ---------------- HTTP handlers ----------------
void sendSettingsJSON(){
  String j;
  j.reserve(128);
  j += F("{\"ok\":true,\"range_cm\":");
  j += g_set.range_cm;
  j += F(",\"freq_hz\":");
  j += g_set.freq_hz;
  j += F("}");
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json", j);
}

void handleSettings(){ sendSettingsJSON(); }

void applySettingsToSensor(uint8_t freq_hz){
  // Adjust VL53L7CX ranging frequency safely
  
  tof.vl53l7cx_stop_ranging();
  delay(5);
  tof.vl53l7cx_set_ranging_frequency_hz(freq_hz);
  tof.vl53l7cx_start_ranging();
  Serial.printf("[ToF] Frequency set to %u Hz\n", freq_hz);
}

void handleSet(){
  // parse ?range_cm=&freq_hz=
  uint16_t range_cm = g_set.range_cm;
  uint8_t  freq_hz  = g_set.freq_hz;

  if (server.hasArg("range_cm")) {
    int v = server.arg("range_cm").toInt();
    if (v < (int)RANGE_CM_MIN) v = RANGE_CM_MIN;
    if (v > (int)RANGE_CM_MAX) v = RANGE_CM_MAX;
    range_cm = (uint16_t)v;
  }
  if (server.hasArg("freq_hz")) {
    int v = server.arg("freq_hz").toInt();
    if (v < (int)FREQ_HZ_MIN) v = FREQ_HZ_MIN;
    if (v > (int)FREQ_HZ_MAX) v = FREQ_HZ_MAX;
    freq_hz = (uint8_t)v;
  }

  bool freqChanged = (freq_hz != g_set.freq_hz);
  bool rangeChanged= (range_cm != g_set.range_cm);

  // Apply
  g_set.range_cm = range_cm;
  g_set.freq_hz  = freq_hz;
  g_maxRangeMm   = (uint16_t)(g_set.range_cm * 10);
  g_frameIntervalMs = max<uint32_t>(100, (1000 / max<uint8_t>(1, g_set.freq_hz))); // keep sane

  // Persist
  prefs.putUShort("range_cm", g_set.range_cm);
  prefs.putUChar ("freq_hz",  g_set.freq_hz);

  if (freqChanged) applySettingsToSensor(g_set.freq_hz);

  // respond
  sendSettingsJSON();
}

void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }

void handleFrame(){
  Frame snap = g_frame; // simple copy
  String json;
  if(snap.valid){
    json.reserve(1200);
    json += F("{\"ok\":true,\"nearest_cm\":");
    json += (snap.nearest_mm>0? (snap.nearest_mm/10) : 0);
    json += F(",\"direction\":\""); json += dirToText(snap.dir); json += F("\",\"ts\":"); json += (uint32_t) (snap.timestamp_ms);
    json += F(",\"grid_mm\":[");
    for(int i=0;i<64;i++){ if(i) json+=','; json+= snap.grid[i]; }
    json += F("]}");
  } else {
    json = F("{\"ok\":false}");
  }
  server.sendHeader("Cache-Control","no-store");
  server.send(200, "application/json", json);
}

void notFound(){ server.send(404, "text/plain", "Not found"); }

// ---------------- Audio trigger (≤ range) ----------------
uint32_t lastPlayMs = 0;
uint16_t lastPlayCm = 9999;
Direction lastPlayDir = DIR_UNKNOWN;

void maybePlayVoice(uint16_t nearest_mm, int nearest_idx){
  if (nearest_idx < 0) return;
  if (nearest_mm == 0 || nearest_mm > g_maxRangeMm) return;

  uint16_t cm = nearest_mm / 10;
  Direction dir = dirFromColFW(nearest_idx % 8);

  uint32_t now = millis();
  bool crossed     = (lastPlayCm > g_set.range_cm && cm <= g_set.range_cm);
  bool dirChanged  = (dir != lastPlayDir);
  bool distChanged = (abs((int)cm - (int)lastPlayCm) >= (int)DELTA_CM_FOR_REPEAT);
  bool cooled      = (now - lastPlayMs >= COOLDOWN_MS);

  if (cm <= g_set.range_cm && (crossed || dirChanged || distChanged) && cooled) {
    uint8_t track = directionToTrackIndex(dir);
    if (track >= 1) {
      jqPlayIndex(track);
      lastPlayMs  = now;
      lastPlayCm  = cm;
      lastPlayDir = dir;
    }
  }
}

// ---------------- Init helpers ----------------
void tofInit(){
  if(LPN_PIN>=0){ pinMode(LPN_PIN,OUTPUT); digitalWrite(LPN_PIN,LOW); delay(5); digitalWrite(LPN_PIN,HIGH); delay(5); }
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(400000);

  if(tof.begin()!=0){
    Serial.println("[ToF] begin() failed");
    return;
  }
  tof.vl53l7cx_off(); delay(5);
  tof.vl53l7cx_init();
  tof.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
  tof.vl53l7cx_set_ranging_frequency_hz(max<uint8_t>(1, g_set.freq_hz));
  tof.vl53l7cx_start_ranging();
  Serial.printf("[ToF] Ranging @ %u Hz\n", g_set.freq_hz);
}

void mp3Init(){
  MP3.begin(9600, SERIAL_8N1, JQ_RX, JQ_TX);
  delay(200);
  jqSetVolume(22); // 0..30
  Serial.println("[JQ6500] UART ready");
}

void wifiInit(){
  WiFi.mode(WIFI_AP);
  WiFi.softAPsetHostname("smartcane");
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] AP %s, IP: %s\n", ok?"started":"FAILED", WiFi.softAPIP().toString().c_str());
  server.on("/", HTTP_GET, handleRoot);
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/set", HTTP_GET, handleSet);
  server.onNotFound(notFound);
  server.begin();
  Serial.println("[HTTP] Listening on :80");
}

void loadSettings(){
  prefs.begin("smartcane", false);
  g_set.range_cm = prefs.getUShort("range_cm", 20); // default 20 cm
  g_set.freq_hz  = prefs.getUChar ("freq_hz",  2);  // default 2 Hz
  // clamp
  if (g_set.range_cm < RANGE_CM_MIN) g_set.range_cm = RANGE_CM_MIN;
  if (g_set.range_cm > RANGE_CM_MAX) g_set.range_cm = RANGE_CM_MAX;
  if (g_set.freq_hz  < FREQ_HZ_MIN ) g_set.freq_hz  = FREQ_HZ_MIN;
  if (g_set.freq_hz  > FREQ_HZ_MAX ) g_set.freq_hz  = FREQ_HZ_MAX;

  g_maxRangeMm = (uint16_t)(g_set.range_cm * 10);
  g_frameIntervalMs = max<uint32_t>(100, (1000 / max<uint8_t>(1, g_set.freq_hz)));
  Serial.printf("[Settings] range=%ucm, freq=%uHz\n", g_set.range_cm, g_set.freq_hz);
}

// ---------------- Setup / Loop ----------------
void setup(){
  Serial.begin(115200); delay(200);
  Serial.println("\n[SmartCane — Editable Range & Frequency] Boot");

  // Init cache
  memset((void*)g_frame.grid, 0, sizeof(g_frame.grid));
  g_frame.nearest_mm = 0; g_frame.nearest_idx = -1; g_frame.dir = DIR_UNKNOWN; g_frame.timestamp_ms = 0; g_frame.valid=false;

  loadSettings();
  tofInit();
  mp3Init();
  wifiInit();
}

void loop(){
  // Always prioritize serving clients
  server.handleClient();

  // Sensor cadence (based on current settings)
  static uint32_t lastFrameTick = 0;
  if (millis() - lastFrameTick >= g_frameIntervalMs){
    lastFrameTick = millis();

    static VL53L7CX_ResultsData res;
    uint8_t ready = 0;
    if (tof.vl53l7cx_check_data_ready(&ready) == 0 && ready){
      if (tof.vl53l7cx_get_ranging_data(&res) == 0){
        // Build new snapshot
        Frame tmp; tmp.valid = true; tmp.nearest_mm = 0; tmp.nearest_idx = -1; tmp.dir = DIR_UNKNOWN;
        uint16_t min_mm = 0xFFFF; int min_idx=-1;
        for (int i=0;i<64;i++){
          uint16_t d = res.distance_mm[i];
          uint16_t clipped = (d>0 && d<=g_maxRangeMm) ? d : 0;
          tmp.grid[i] = clipped;
          if (clipped>0 && clipped < min_mm){ min_mm = clipped; min_idx = i; }
        }
        if (min_idx >= 0){
          tmp.nearest_mm = min_mm;
          tmp.nearest_idx = min_idx;
          tmp.dir = dirFromColFW(min_idx % 8);
        }
        tmp.timestamp_ms = millis();

        // Publish snapshot
        g_frame = tmp;

        // Drive audio from fresh frame
        if (min_idx >= 0) maybePlayVoice(min_mm, min_idx);
      }
    }
  }


  delay(1);
}

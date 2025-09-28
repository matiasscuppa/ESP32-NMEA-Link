/*  NMEA Link - ESP32 (AP + Monitor + Generator + UDP) - Dual Core
    ----------------------------------------------------------------
    • AP (portal cautivo): SSID "NMEA_Link", clave "12345678"
    • NMEA Monitor (UART RX=16) y NMEA Generator (UART TX=17 + UDP:10110)
    • Generator: 4 slots simultáneos editables (plantillas por sensor/sentencia)
      – editor sin *HH; checksum se calcula al vuelo y se guarda completo
      – intervalo independiente por slot (0.1 / 0.5 / 1 / 2 s)
    • Dual core: Core0 (web/dns), Core1 (NMEA + LED)
    • LED NeoPixel GPIO48: CIAN arranque, VERDE rx válida, ROJO rx inválida, AZUL tx
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Adafruit_NeoPixel.h>
#include <DNSServer.h>
#include "esp_log.h"

// ======================= AP / Captive Portal =======================
const char* AP_SSID     = "NMEA_Link";
const char* AP_PASSWORD = "12345678";
const byte  DNS_PORT    = 53;
DNSServer dnsServer;

// ======================= LED (NeoPixel) ============================
#define LED_PIN 48
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
unsigned long ledMillis = 0;
const int LED_DURATION = 50;
bool ledOn = false;

// ======================= UART ============================
HardwareSerial NMEA_Serial(1);
#define RX_PIN 16
#define TX_PIN 17
volatile int currentBaud = 4800;

// ======================= UDP =============================
WiFiUDP udp;
IPAddress udpAddress;
const int udpPort = 10110;

// ======================= Web =============================
WebServer server(80);

// ======================= Buffer Monitor ===================
#define BUFFER_LINES 50
String nmeaBuffer[BUFFER_LINES];
int bufferIndex = 0;
String currentLine = "";

// ======================= Buffer Generator =================
#define GEN_BUFFER_LINES 200
String genBuffer[GEN_BUFFER_LINES];
int genIndex = 0;

// ======================= Generator (multi-slot) ===========
enum AppMode { MODE_MONITOR = 0, MODE_GENERATOR = 1 };
volatile AppMode appMode = MODE_MONITOR;
volatile bool monitorRunning   = false;     // Monitor arranca en pausa
volatile bool generatorRunning = false;     // Generator arranca en pausa

// Catálogo de baudrates (UI)
const int baudRates[4] = {4800, 9600, 38400, 115200};

// --- Slots configurables ---
const int MAX_SLOTS = 4;
struct GenSlot {
  bool   enabled;
  String sensor;      // GPS / WEATHER / HEADING / SOUNDER / VELOCITY / RADAR / TRANSDUCER / AIS / CUSTOM
  String sentence;    // RMC / VTG / ...
  String text;        // línea NMEA completa (con checksum)
};
GenSlot slots[MAX_SLOTS] = {
  {true,  "GPS",       "RMC", ""},    // Slot 1
  {false, "GPS",       "VTG", ""},    // Slot 2
  {false, "VELOCITY",  "VHW", ""},    // Slot 3
  {false, "HEADING",   "HDT", ""},    // Slot 4
};

// Intervalo por slot (ms) y timestamp último envío
unsigned long slotInterval[MAX_SLOTS] = {500, 500, 500, 500};
unsigned long lastSentMs  [MAX_SLOTS] = {0,   0,   0,   0  };

// ======================= FreeRTOS Sync ====================
SemaphoreHandle_t nmeaBufMutex;
SemaphoreHandle_t genBufMutex;
SemaphoreHandle_t serialMutex;

// ======================= LED helpers ======================
void flashLed(uint32_t color) {
  pixels.setPixelColor(0, color);
  pixels.show();
  ledOn = true;
  ledMillis = millis();
}
void updateLed() {
  if (ledOn && millis() - ledMillis >= LED_DURATION) {
    pixels.setPixelColor(0, 0);
    pixels.show();
    ledOn = false;
  }
}

// ======================= NMEA helpers =====================
bool processNMEA(const String &line) {
  return (line.startsWith("$") || line.startsWith("!"));
}
String detectSentenceType(const String &line) {
  if (line.startsWith("!")) return "AIS";
  if (line.length() >= 6 && line[0] == '$') {
    String formatter = line.substring(3, 6);
    formatter.toUpperCase();
    if (formatter == "GLL" || formatter == "RMC" || formatter == "VTG" ||
        formatter == "GGA" || formatter == "GSA" || formatter == "GSV" ||
        formatter == "DTM" || formatter == "ZDA") return "GPS";
    if (formatter == "DBT" || formatter == "DPT" || formatter == "DBK" ||
        formatter == "DBS") return "SOUNDER";
    if (formatter == "MWD" || formatter == "MWV" || formatter == "VWR" ||
        formatter == "VWT" || formatter == "MTW") return "WEATHER";
    if (formatter == "HDG" || formatter == "HDT" || formatter == "HDM" ||
        formatter == "THS" || formatter == "ROT" || formatter == "RSA") return "HEADING";
    if (formatter == "VHW" || formatter == "VLW" || formatter == "VBW") return "SPEED";
    if (formatter == "TLL" || formatter == "TTM" || formatter == "TLB" || formatter == "OSD") return "RADAR";
    if (formatter == "XDR") return "TRANSDUCER";
  }
  return "OTROS";
}
void sendUDP(const String &line) {
  udp.beginPacket(udpAddress, udpPort);
  udp.print(line);
  udp.endPacket();
}

// ======================= Builders / checksum ============
String nmeaChecksum(const String &payload) {
  uint8_t cs = 0;
  for (size_t i = 0; i < payload.length(); i++) cs ^= (uint8_t)payload[i];
  char buf[3]; snprintf(buf, sizeof(buf), "%02X", cs);
  return String(buf);
}
String buildDollarSentence(const String& talker, const String& code, const String& fields) {
  String payload = talker + code + "," + fields;
  return "$" + payload + "*" + nmeaChecksum(payload);
}
String buildAISSentence_VDM() {
  String payload = "AIVDM,1,1,,A,13aG?P0P00PD;88MD5MT?wvl0<0,0";
  return "!" + payload + "*" + nmeaChecksum(payload);
}
String talkerForSensor(const String& sensor) {
  if (sensor == "GPS")        return "GP";
  if (sensor == "AIS")        return "AI";
  if (sensor == "SOUNDER")    return "SD";
  if (sensor == "HEADING")    return "HC";
  if (sensor == "CUSTOM")     return "";
  return "II"; // WEATHER / VELOCITY / RADAR / TRANSDUCER
}

// Genera plantilla por sensor/sentencia
String generateSentence(const String& sensor, const String& codeIn) {
  if (sensor.equalsIgnoreCase("CUSTOM") || codeIn.equalsIgnoreCase("CUSTOM")) {
    return ""; // custom: viene del editor
  }
  String t = talkerForSensor(sensor);
  String c = codeIn; c.toUpperCase();

  if (sensor == "AIS") return buildAISSentence_VDM();

  // GPS
  if (t == "GP" && c == "RMC") return buildDollarSentence(t, c, "123519,A,4807.038,N,01131.000,E,5.5,054.7,230394,003.1,W");
  if (t == "GP" && c == "GGA") return buildDollarSentence(t, c, "123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  if (t == "GP" && c == "GLL") return buildDollarSentence(t, c, "4916.45,N,12311.12,W,225444,A");
  if (t == "GP" && c == "VTG") return buildDollarSentence(t, c, "054.7,T,034.4,M,005.5,N,010.2,K");
  if (t == "GP" && c == "GSA") return buildDollarSentence(t, c, "A,3,04,05,09,12,24,25,29,31,,,,,2.5,1.3,2.1");
  if (t == "GP" && c == "GSV") return buildDollarSentence(t, c, "2,1,08,01,40,083,41,02,17,308,43,12,07,021,42,14,25,110,45");
  if (t == "GP" && c == "DTM") return buildDollarSentence(t, c, "W84,,0.0,N,0.0,E,0.0,W84");
  if (t == "GP" && c == "ZDA") return buildDollarSentence(t, c, "201530.00,04,07,2002,00,00");

  // WEATHER
  if (t == "II" && c == "MWD") return buildDollarSentence(t, c, "054.7,T,034.4,M,10.5,N,5.4,M");
  if (t == "II" && c == "MWV") return buildDollarSentence(t, c, "054.7,R,10.5,N,A");
  if (t == "II" && c == "VWR") return buildDollarSentence(t, c, "054.7,R,10.5,N,5.4,M,19.4,K");
  if (t == "II" && c == "VWT") return buildDollarSentence(t, c, "054.7,T,10.5,N,5.4,M,19.4,K");
  if (t == "II" && c == "MTW") return buildDollarSentence(t, c, "18.0,C");

  // HEADING
  if (t == "HC" && c == "HDG") return buildDollarSentence(t, c, "238.5,,E,0.5");
  if (t == "HC" && c == "HDT") return buildDollarSentence(t, c, "238.5,T");
  if (t == "HC" && c == "HDM") return buildDollarSentence(t, c, "236.9,M");
  if (t == "HC" && c == "THS") return buildDollarSentence(t, c, "238.5,A");
  if (t == "HC" && c == "ROT") return buildDollarSentence(t, c, "0.0,A");
  if (t == "HC" && c == "RSA") return buildDollarSentence(t, c, "0.0,A,0.0,A");

  // SOUNDER
  if (t == "SD" && c == "DBT") return buildDollarSentence(t, c, "036.4,f,011.1,M,006.0,F");
  if (t == "SD" && c == "DPT") return buildDollarSentence(t, c, "11.2,0.5");
  if (t == "SD" && c == "DBK") return buildDollarSentence(t, c, "036.4,f,011.1,M,006.0,F");
  if (t == "SD" && c == "DBS") return buildDollarSentence(t, c, "036.4,f,011.1,M,006.0,F");

  // VELOCITY
  if (t == "II" && c == "VHW") return buildDollarSentence(t, c, "054.7,T,034.4,M,5.5,N,10.2,K");
  if (t == "II" && c == "VLW") return buildDollarSentence(t, c, "12.4,N,0.5,N");
  if (t == "II" && c == "VBW") return buildDollarSentence(t, c, "5.5,0.1,0.0,5.3,0.1,0.0");

  // RADAR
  if (t == "II" && c == "TLL") return buildDollarSentence(t, c, "1,4916.45,N,12311.12,W,225444,TGT1");
  if (t == "II" && c == "TTM") return buildDollarSentence(t, c, "1,2.5,N,054.7,T,0.0,N,054.7,T,0.0,54.7,TGT1");
  if (t == "II" && c == "TLB") return buildDollarSentence(t, c, "1,LOCK,4916.45,N,12311.12,W,225444");
  if (t == "II" && c == "OSD") return buildDollarSentence(t, c, "054.7,A,5.5,N,10.2,K");

  // TRANSDUCER
  if (t == "II" && c == "XDR") return buildDollarSentence(t, c, "C,19.5,C,AirTemp");

  return buildDollarSentence(t, c, "");
}

// ======================= helpers HTML ====================
String htmlEscape(const String& s){
  String o; o.reserve(s.length()+8);
  for(size_t i=0;i<s.length();++i){
    char c=s[i];
    if(c=='&') o += "&amp;";
    else if(c=='<') o += "&lt;";
    else if(c=='>') o += "&gt;";
    else if(c=='\"') o += "&quot;";
    else if(c=='\'') o += "&#39;";
    else o += c;
  }
  return o;
}
String fullToEditable(const String& full){
  if(full.length()==0) return "";
  String s=full;
  char ch = (s[0]=='$' || s[0]=='!')? s[0] : 0;
  if(ch) s.remove(0,1);
  int star = s.indexOf('*');
  if(star>=0) s = s.substring(0,star);
  return (ch?String(ch):String("")) + s;
}

// ======================= Generator buffer =============
void pushGen(const String& line) {
  xSemaphoreTake(genBufMutex, portMAX_DELAY);
  genIndex = (genIndex + 1) % GEN_BUFFER_LINES;
  genBuffer[genIndex] = line;
  xSemaphoreGive(genBufMutex);
}

// ======================= UART control =================
void startSerial(int baud) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  NMEA_Serial.end();
  delay(5);
  NMEA_Serial.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);
  while (NMEA_Serial.available()) (void)NMEA_Serial.read();
  currentBaud = baud;
  xSemaphoreGive(serialMutex);
}

// ======================= Web helpers ==================
void noCache() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}
void handle204() { noCache(); server.send(204, "text/plain", ""); }
void handleCaptive() {
  noCache();
  server.send(200, "text/html",
    "<!DOCTYPE html><html><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>NMEA Link</title><body style='background:#000;color:#0f0;font-family:monospace;'>"
    "<p>Redirigiendo…</p><script>location.href='/'</script></body></html>");
}

// ======================= UI: MONITOR ==================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>NMEA Reader</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  html += "body{font-family:monospace;background:#000;color:#0f0;margin:0;padding:10px;}";
  html += "h2{text-align:center;color:#0ff;margin:8px 0;}";
  html += ".lang-selector{position:absolute;top:10px;right:10px;color:#0f0;background:#111;border:1px solid #0f0;border-radius:5px;}";
  html += "#console{width:100%;max-width:100%;box-sizing:border-box;height:40vh;overflow-y:auto;overflow-x:auto;border:1px solid #0f0;padding:5px;background:#000;font-size:14px;white-space:pre-wrap;word-wrap:break-word;overflow-wrap:anywhere;margin-top:12px;}";
  html += ".btn-container{display:flex;flex-wrap:wrap;gap:5px;margin:8px 0;}";
  html += ".btn{flex:1;padding:10px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:8px;font-size:16px;text-align:center;cursor:pointer;}";
  html += ".btn.active{background:#0f0;color:#000;font-weight:bold;}";
  html += ".filter-btn{flex:1 1 calc(33.33% - 6px);padding:5px 0;border-radius:5px;margin:2px;text-align:center;transition:all .2s ease;border:1px solid #333;}";
  html += ".filter-btn:not(.active){background:#111;color:#666;border-color:#444;}";
  html += ".filter-btn.active{font-weight:600;border:1px solid #222;text-shadow:none;}";
  html += ".filter-btn.active.GPS{background:#00ffff;color:#000;} .GPS{color:#00ffff;}";
  html += ".filter-btn.active.AIS{background:#ffff00;color:#000;} .AIS{color:#ffff00;}";
  html += ".filter-btn.active.SOUNDER{background:#00ff00;color:#000;} .SOUNDER{color:#00ff00;}";
  html += ".filter-btn.active.SPEED{background:#ff00ff;color:#000;} .SPEED{color:#ff00ff;}";
  html += ".filter-btn.active.HEADING{background:#1e90ff;color:#000;} .HEADING{color:#1e90ff;}";
  html += ".filter-btn.active.RADAR{background:#ff4500;color:#000;} .RADAR{color:#ff4500;}";
  html += ".filter-btn.active.WEATHER{background:#7fffd4;color:#000;} .WEATHER{color:#7fffd4;}";
  html += ".filter-btn.active.TRANSDUCER{background:#ffa500;color:#000;} .TRANSDUCER{color:#ffa500;}";
  html += ".filter-btn.active.OTROS{background:#aaaaaa;color:#000;} .OTROS{color:#aaaaaa;}";
  html += "footer{text-align:center;color:#666;font-size:12px;margin-top:10px;}";
  html += "</style></head><body>";

  html += "<select id='langSelect' class='lang-selector' onchange='setLang(this.value)'>";
  html += "<option value='en' selected>EN</option><option value='es'>ES</option><option value='fr'>FR</option></select>";

  html += "<h2 id='title'>NMEA Reader</h2>";
  html += "<div class='btn-container' id='filterContainer'></div>";
  html += "<div id='console'></div>";

  // Baudrates
  html += "<div class='btn-container'>";
  for (int i = 0; i < 4; i++) {
    html += "<button type='button' id='baud_" + String(baudRates[i]) + "' class='btn baud' onclick='setBaud(" + String(baudRates[i]) + ")'>" + String(baudRates[i]) + "</button>";
  }
  html += "</div>";

  // Start/Pause + Clear
  html += "<div class='btn-container'>";
  html += "<button type='button' class='btn' id='pauseBtn' onclick='togglePause()'>▶ Start</button>";
  html += "<button type='button' class='btn' id='clearBtn' onclick='clearConsole()'>🧹 Clear</button>";
  html += "</div>";

  // Velocidad UI
  html += "<div class='btn-container'>";
  html += "<button type='button' class='btn' onclick='setSpeed(0.25,this)'>25%</button>";
  html += "<button type='button' class='btn active' onclick='setSpeed(0.5,this)'>50%</button>";
  html += "<button type='button' class='btn' onclick='setSpeed(0.75,this)'>75%</button>";
  html += "<button type='button' class='btn' onclick='setSpeed(1,this)'>100%</button>";
  html += "</div>";

  // Ir al generador
  html += "<div class='btn-container'><button type='button' class='btn' onclick=\"gotoGenerator()\">➡ NMEA Generator</button></div>";

  html += "<footer>© 2025 Matías Scuppa — by Themys</footer>";

  // JS Monitor
  html += "<script>";
  html += "let lang='en';";
  html += "const labelsByLang={en:{pause:'⏸ Pause',resume:'▶ Start',clear:'🧹 Clear'},es:{pause:'⏸ Pausar',resume:'▶ Iniciar',clear:'🧹 Limpiar'},fr:{pause:'⏸ Pause',resume:'▶ Démarrer',clear:'🧹 Effacer'}};";
  html += "const catLabels={en:{GPS:'GPS',AIS:'AIS',SOUNDER:'SOUNDER',SPEED:'SPEED',HEADING:'HEADING',RADAR:'RADAR',WEATHER:'WEATHER',TRANSDUCER:'TRANSDUCER',OTROS:'OTHER'},es:{GPS:'GPS',AIS:'AIS',SOUNDER:'SOUNDER',SPEED:'SPEED',HEADING:'HEADING',RADAR:'RADAR',WEATHER:'WEATHER',TRANSDUCER:'TRANSDUCER',OTROS:'OTROS'},fr:{GPS:'GPS',AIS:'AIS',SOUNDER:'SOUNDER',SPEED:'SPEED',HEADING:'HEADING',RADAR:'RADAR',WEATHER:'WEATHER',TRANSDUCER:'TRANSDUCER',OTROS:'AUTRES'}};";
  html += "let filters=['GPS','AIS','SOUNDER','SPEED','HEADING','RADAR','WEATHER','TRANSDUCER','OTROS'];let filtersState={};filters.forEach(f=>filtersState[f]=true);";
  html += "let paused=true, intervalMs=1000, intervalId=null;";
  html += "function setLang(l){lang=l;localStorage.setItem('lang',l);applyLang();}";
  html += "function applyLang(){document.getElementById('pauseBtn').innerText=paused?labelsByLang[lang].resume:labelsByLang[lang].pause;document.getElementById('clearBtn').innerText=labelsByLang[lang].clear;drawFilters();}";
  html += "function drawFilters(){let c=document.getElementById('filterContainer');c.innerHTML='';filters.forEach(f=>{let b=document.createElement('button');b.setAttribute('type','button');b.className='filter-btn '+f;if(filtersState[f])b.classList.add('active');b.innerText=catLabels[lang][f];b.onclick=()=>toggleFilter(f,b);c.appendChild(b);});let all=document.createElement('button');all.setAttribute('type','button');all.className='filter-btn';all.innerText='ALL/NONE';all.onclick=toggleAll;c.appendChild(all);}";
  html += "function toggleFilter(f,btn){filtersState[f]=!filtersState[f];btn.classList.toggle('active',filtersState[f]);}";
  html += "function toggleAll(){let any=Object.values(filtersState).some(v=>v);Object.keys(filtersState).forEach(k=>filtersState[k]=!any);drawFilters();}";
  html += "function togglePause(){paused=!paused;applyLang();fetch('/setmonitor?state='+(paused?0:1),{cache:'no-store'}).catch(()=>{});}";
  html += "function clearConsole(){document.getElementById('console').innerHTML='';}";
  html += "async function setBaud(b){await fetch('/setbaud?baud='+b,{cache:'no-store'}).catch(()=>{});document.querySelectorAll('.baud').forEach(x=>x.classList.remove('active'));let el=document.getElementById('baud_'+b);if(el)el.classList.add('active');}";
  html += "function setSpeed(mult,btn){document.querySelectorAll('.btn').forEach(b=>{if(b.innerText.includes('%'))b.classList.remove('active');});btn.classList.add('active');intervalMs=Math.max(100,Math.round(1000/mult));if(intervalId)clearInterval(intervalId);intervalId=setInterval(poll,intervalMs);}";

  html += "function poll(){if(paused)return;fetch('/getnmea?ts='+Date.now(),{cache:'no-store'}).then(r=>r.text()).then(t=>{let c=document.getElementById('console');let lines=t.trim()?t.trim().split('\\n'):[];let visible=lines.filter(l=>{let lb=l.indexOf(']');let type=(lb>0&&l[0]=='[')?l.substring(1,lb):'OTROS';return filtersState[type];});c.innerHTML=visible.map(l=>{let type=l.substring(1,l.indexOf(']'));return '<span class=\\\"'+type+'\\\">'+l+'</span>';}).join('<br>');c.scrollTop=c.scrollHeight;}).catch(()=>{});}";

  html += "async function gotoGenerator(){paused=true;applyLang();try{await fetch('/setmonitor?state=0',{cache:'no-store'});await fetch('/setmode?m=generator',{cache:'no-store'});}catch(e){} window.location='/generator';}";
  html += "document.addEventListener('DOMContentLoaded',()=>{fetch('/setmode?m=monitor',{cache:'no-store'});fetch('/setmonitor?state=0',{cache:'no-store'});let saved=localStorage.getItem('lang');if(saved){lang=saved;let sel=document.getElementById('langSelect');if(sel)sel.value=saved;}applyLang();let b=document.getElementById('baud_" + String(currentBaud) + "');if(b)b.classList.add('active');intervalId=setInterval(poll,intervalMs);});";
  html += "window.addEventListener('beforeunload',()=>{if(intervalId)clearInterval(intervalId);});";
  html += "</script></body></html>";

  noCache();
  server.send(200, "text/html", html);
}

// -------- listas de opciones para Generator (lado servidor) --------
const char* SENSOR_LIST[] = {"GPS","WEATHER","HEADING","SOUNDER","VELOCITY","RADAR","TRANSDUCER","AIS","CUSTOM"};
const int   SENSOR_COUNT  = 9;

String optionsForSensorSelect(const String& current){
  String s;
  for(int i=0;i<SENSOR_COUNT;i++){
    String v = SENSOR_LIST[i];
    s += "<option value='"; s += v; s += "'";
    if(v == current) s += " selected";
    s += ">"; s += v; s += "</option>";
  }
  return s;
}
String optionsForSentence(const String& sensor, const String& selected){
  auto add=[&](const char* v, String& out){
    out += "<option value='"; out += v; out += "'";
    if(selected == v) out += " selected";
    out += ">"; out += v; out += "</option>";
  };
  String out;
  if(sensor == "GPS"){ const char* arr[]={"GLL","RMC","VTG","GGA","GSA","GSV","DTM","ZDA"};
    for(const char* v:arr) add(v,out);
  } else if(sensor=="WEATHER"){ const char* arr[]={"MWD","MWV","VWR","VWT","MTW"};
    for(const char* v:arr) add(v,out);
  } else if(sensor=="HEADING"){ const char* arr[]={"HDG","HDT","HDM","THS","ROT","RSA"};
    for(const char* v:arr) add(v,out);
  } else if(sensor=="SOUNDER"){ const char* arr[]={"DBT","DPT","DBK","DBS"};
    for(const char* v:arr) add(v,out);
  } else if(sensor=="VELOCITY"){ const char* arr[]={"VHW","VLW","VBW"};
    for(const char* v:arr) add(v,out);
  } else if(sensor=="RADAR"){ const char* arr[]={"TLL","TTM","TLB","OSD"};
    for(const char* v:arr) add(v,out);
  } else if(sensor=="TRANSDUCER"){ const char* arr[]={"XDR"};
    for(const char* v:arr) add(v,out);
  } else if(sensor=="AIS"){ const char* arr[]={"AIVDM","AIVDO"};
    for(const char* v:arr) add(v,out);
  } else { add("CUSTOM", out); }
  if(out.length()==0) add("CUSTOM", out);
  return out;
}
String initialEditableForSlot(int i){
  String full;
  if(slots[i].text.length()) full = slots[i].text;
  else {
    if(slots[i].sensor=="CUSTOM" || slots[i].sentence=="CUSTOM"){
      String payload = "GPCUS,FIELD1,FIELD2";
      full = "$" + payload + "*" + nmeaChecksum(payload);
    } else {
      full = generateSentence(slots[i].sensor, slots[i].sentence);
    }
  }
  return htmlEscape(fullToEditable(full));
}

// ======================= UI: GENERATOR ============================
void handleGenerator() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>NMEA Generator</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  html += "body{font-family:monospace;background:#000;color:#0f0;margin:0;padding:10px;}h2{text-align:center;color:#0ff;margin:8px 0;}";
  html += ".grid{display:grid;grid-template-columns:1fr;gap:10px;}";
  html += ".card{border:1px solid #0f0;border-radius:8px;padding:8px;background:#000;text-align:left;}";
  html += ".slotTitle{display:flex;align-items:center;justify-content:flex-start;margin-bottom:6px;}";
  html += ".slotTitle .left{display:flex;align-items:center;gap:8px;}";
  html += ".slotTitle input[type=checkbox]{margin:0 8px 0 0;transform:scale(1.1);accent-color:#0f0;}";
  html += "label{display:block;margin:6px 0 4px 0;font-weight:bold;text-align:left;}";
  html += "select,input{width:100%;box-sizing:border-box;padding:6px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:6px;}";
  html += ".row{display:flex;gap:6px;flex-wrap:wrap;align-items:flex-start;justify-content:flex-start;}";
  html += ".row>*{flex:1;min-width:160px;}";
  html += ".row.spaceTop{margin-top:8px;}";
  html += ".btn{padding:10px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:8px;font-size:16px;cursor:pointer;text-align:center;}";
  html += ".btn.small{padding:6px 8px;font-size:14px;border-radius:6px;}";
  html += ".btn.active{background:#0f0;color:#000;font-weight:bold;}";
  html += "#genconsole{width:100%;box-sizing:border-box;height:40vh;overflow:auto;border:1px solid #0f0;padding:5px;background:#000;margin-top:10px;}";
  html += ".btn-row{display:flex;gap:6px;margin-top:10px;align-items:stretch;}";
  html += ".btn-row .start{flex:2;}";   // Start más grande
  html += ".btn-row .clear{flex:1;}";   // Clear más chico
  html += ".btn-full{width:100%;display:block;}";
  html += "footer{text-align:center;color:#666;font-size:12px;margin-top:10px;}";
  html += "a.btn{text-decoration:none;display:inline-block}";
  html += "</style></head><body>";

  html += "<h2 id='genTitle'>NMEA Generator</h2>";
  html += "<div class='grid' id='slots'>";

  // --- Slots pre-render ---
  for (int i=0;i<MAX_SLOTS;i++) {
    unsigned long ms = slotInterval[i];
    bool a100 = (ms==100), a500=(ms==500), a1000=(ms==1000), a2000=(ms==2000);

    html += "<div class='card' id='slot_"+String(i)+"'>";
    html += "  <div class='slotTitle'><div class='left'>";
    html += "    <input type='checkbox' id='en_"+String(i)+"'"; if(slots[i].enabled) html += " checked"; html += "> ";
    html += "    <strong class='slotLabel' data-idx='"+String(i)+"'>Sentence</strong>";
    html += "  </div></div>";

    html += "  <div class='row'>";
    html += "    <div><label class='lblSensor'>Sensor</label><select id='sensor_"+String(i)+"'>";
    html += optionsForSensorSelect(slots[i].sensor);
    html += "</select></div>";
    html += "    <div><label class='lblSentence'>Sentence type</label><select id='sentence_"+String(i)+"'>";
    html += optionsForSentence(slots[i].sensor, slots[i].sentence);
    html += "</select></div>";
    html += "  </div>";

    html += "  <div class='row spaceTop'>";
    html += "    <div style='flex:1 1 100%;'><input id='text_"+String(i)+"' placeholder='$GPRMC,.....' autocomplete='off' value='";
    html += initialEditableForSlot(i);
    html += "'></div>";
    html += "  </div>";

    // Intervalo por slot
    html += "  <div class='row spaceTop'>";
    html += "    <div style='flex:1 1 100%;'><label class='lblIntervalSlot'>Interval</label>";
    html += "      <div id='intgrp_"+String(i)+"' class='row' style='gap:8px'>";
    html += "        <button type='button' class='btn small int-btn"; if(a100)  html+=" active"; html += "' onclick='setIntervalSlot("+String(i)+",100,this)'>0.1s</button>";
    html += "        <button type='button' class='btn small int-btn"; if(a500)  html+=" active"; html += "' onclick='setIntervalSlot("+String(i)+",500,this)'>0.5s</button>";
    html += "        <button type='button' class='btn small int-btn"; if(a1000) html+=" active"; html += "' onclick='setIntervalSlot("+String(i)+",1000,this)'>1s</button>";
    html += "        <button type='button' class='btn small int-btn"; if(a2000) html+=" active"; html += "' onclick='setIntervalSlot("+String(i)+",2000,this)'>2s</button>";
    html += "      </div>";
    html += "    </div>";
    html += "  </div>";

    html += "</div>";
  }
  html += "</div>"; // grid

  // Baudrate (Generator)
  html += "<label id='lblBaud'>Baudrate</label><div class='row'>";
  for (int i = 0; i < 4; i++) {
    html += "<button type='button' id='gen_baud_" + String(baudRates[i]) + "' class='btn gen-baud' onclick='setGenBaud(" + String(baudRates[i]) + ",this)'>" + String(baudRates[i]) + "</button>";
  }
  html += "</div>";

  // Visor salida
  html += "<div id='genconsole'></div>";

  // Botones principales
  html += "<div class='btn-row'>";
  html += "<button type='button' id='startBtn' class='btn start' onclick='toggleGen(event)'>▶ Iniciar</button>";
  html += "<button type='button' id='clearBtn' class='btn clear' onclick='clearGen(event)'>🧹 Limpiar</button>";
  html += "</div>";

  // Botón volver: ancho completo
  html += "<div class='btn-row'>";
  html += "<a id='btnBack' class='btn btn-full' href='/' onclick='return backToMonitor(event)'>⬅ NMEA Monitor</a>";
  html += "</div>";

  // --- JS Generator ---
  html += "<script>";
  html += "const sentencesBySensor={GPS:['GLL','RMC','VTG','GGA','GSA','GSV','DTM','ZDA'],WEATHER:['MWD','MWV','VWR','VWT','MTW'],HEADING:['HDG','HDT','HDM','THS','ROT','RSA'],SOUNDER:['DBT','DPT','DBK','DBS'],VELOCITY:['VHW','VLW','VBW'],RADAR:['TLL','TTM','TLB','OSD'],TRANSDUCER:['XDR'],AIS:['AIVDM','AIVDO'],CUSTOM:[]};";
  html += "let lang=localStorage.getItem('lang')||'en';";
  html += "const L={en:{title:'NMEA Generator', sensor:'Sensor', sentenceSel:'Sentence type', sentenceInline:'Sentence', interval:'Interval', start:'▶ Start', pause:'⏸ Pause', clear:'🧹 Clear', back:'⬅ NMEA Monitor', baud:'Baudrate'},es:{title:'NMEA Generator', sensor:'Sensor', sentenceSel:'Tipo de sentencia', sentenceInline:'Sentencia', interval:'Intervalo', start:'▶ Iniciar', pause:'⏸ Pausar', clear:'🧹 Limpiar', back:'⬅ NMEA Monitor', baud:'Baudrate'},fr:{title:'NMEA Generator', sensor:'Capteur', sentenceSel:'Type de trame', sentenceInline:'Trame', interval:'Intervalle', start:'▶ Démarrer', pause:'⏸ Pause', clear:'🧹 Effacer', back:'⬅ NMEA Monitor', baud:'Baudrate'}};";

  // ---- checksum en vivo ----
  html += "function hex2(n){return n.toString(16).toUpperCase().padStart(2,'0');}";
  html += "function csPayload(s){let cs=0;for(let i=0;i<s.length;i++){cs^=s.charCodeAt(i);}return hex2(cs);}";

  // Construye línea completa desde editor (oculta HH en input)
  html += "function buildFullFromEditor(str){ if(!str) return ''; str=str.trim(); let ch=null; if(str[0]==='$'||str[0]==='!'){ ch=str[0]; str=str.slice(1);} let up=str.toUpperCase(); if(!ch) ch=(up.startsWith('AIVDM')||up.startsWith('AIVDO'))?'!':'$'; let payload=str; let hh=csPayload(payload); return ch+payload+'*'+hh; }";

  // Rellena sentences del sensor
  html += "function refillSent(sensorSel,sentSel){ sentSel.innerHTML=''; const arr=sentencesBySensor[sensorSel.value]||[]; if(arr.length===0){ let o=document.createElement('option'); o.value='CUSTOM'; o.text='CUSTOM'; sentSel.appendChild(o);} else { arr.forEach(c=>{ let o=document.createElement('option'); o.value=c; o.text=c; sentSel.appendChild(o); }); }}";

  // ---- Estado desde el servidor ----
  html += "async function getStatus(){try{const r=await fetch('/getstatus',{cache:'no-store'});return await r.json();}catch(e){return {baud:4800,genRunning:false};}}";

  // Wire por slot
  html += "function initSlot(i){";
  html += "  const en=document.getElementById('en_'+i);";
  html += "  const sensorSel=document.getElementById('sensor_'+i);";
  html += "  const sentSel=document.getElementById('sentence_'+i);";
  html += "  const txt=document.getElementById('text_'+i);";
  html += "  en.addEventListener('change',e=>{fetch('/gen_slot_enable?i='+i+'&en='+(e.target.checked?1:0),{cache:'no-store'}).catch(()=>{});});";

  // Cambiar SENSOR => setear primera SENTENCE y cargar plantilla
  html += "  sensorSel.addEventListener('change',async ()=>{";
  html += "    refillSent(sensorSel,sentSel);";
  html += "    const newSent = sentSel.value;";
  html += "    try{";
  html += "      await fetch('/gen_slot_sensor?i='+i+'&sensor='+sensorSel.value,{cache:'no-store'});";
  html += "      await fetch('/gen_slot_sentence?i='+i+'&sentence='+newSent,{cache:'no-store'});";
  html += "      const r=await fetch('/gen_slot_template?i='+i,{cache:'no-store'});";
  html += "      const t=await r.text();";
  html += "      const ch=(t && (t[0]==='$'||t[0]==='!'))?t[0]:'';";
  html += "      let s=t? t.slice(ch?1:0):'';";
  html += "      let star=s.indexOf('*'); if(star>=0) s=s.slice(0,star);";
  html += "      txt.value=(ch?s?ch+s:s:s);";
  html += "    }catch(e){}";
  html += "  });";

  // Cambiar SENTENCE => pedir plantilla y actualizar editor
  html += "  sentSel.addEventListener('change',async ()=>{";
  html += "    try{";
  html += "      await fetch('/gen_slot_sentence?i='+i+'&sentence='+sentSel.value,{cache:'no-store'});";
  html += "      const r=await fetch('/gen_slot_template?i='+i,{cache:'no-store'});";
  html += "      const t=await r.text();";
  html += "      const ch=(t && (t[0]==='$'||t[0]==='!'))?t[0]:'';";
  html += "      let s=t? t.slice(ch?1:0):'';";
  html += "      let star=s.indexOf('*'); if(star>=0) s=s.slice(0,star);";
  html += "      txt.value=(ch?s?ch+s:s:s);";
  html += "    }catch(e){}";
  html += "  });";

  // Guardado del editor con checksum recalculado (POST)
  html += "  txt.addEventListener('input',e=>{ if(e.target.value.indexOf('*')>=0){ e.target.value=e.target.value.replace(/\\*/g,''); } const full=buildFullFromEditor(e.target.value); fetch('/gen_slot_text',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'i='+i+'&text='+encodeURIComponent(full),cache:'no-store'}).catch(()=>{}); });";
  html += "}";

  // Helpers locales
  html += "function setActive(selector,scope,el){(scope?scope:document).querySelectorAll(selector).forEach(b=>b.classList.remove('active'));if(el)el.classList.add('active');}";
  html += "function setIntervalSlot(i,ms,btn){ fetch('/gen_slot_interval?i='+i+'&ms='+ms,{cache:'no-store'}).then(()=>{ const g=document.getElementById('intgrp_'+i); if(!g) return; setActive('.int-btn',g,btn); }).catch(()=>{}); }";
  html += "async function setGenBaud(b,btn){ try{ await fetch('/setbaud?baud='+b,{cache:'no-store'}); setActive('.gen-baud',document,btn);}catch(e){console.error(e);} }";

  // Start / Pause Generator
  html += "let running=false;";
  html += "async function toggleGen(e){ if(e)e.preventDefault(); try{ running=!running; const r=await fetch('/togglegen?state='+(running?'1':'0'),{cache:'no-store'}); const t=await r.text(); if(t==='RUNNING')running=true; else if(t==='STOPPED')running=false; document.getElementById('startBtn').innerText=running?L[lang].pause:L[lang].start; }catch(err){console.error(err);} }";

  // Clear visor
  html += "function clearGen(e){ if(e)e.preventDefault(); fetch('/cleargen',{cache:'no-store'}).catch(()=>{}); document.getElementById('genconsole').innerHTML='';}";

  // Volver al Monitor
  html += "function backToMonitor(e){try{fetch('/togglegen?state=0',{cache:'no-store'});fetch('/setmode?m=monitor',{cache:'no-store'});fetch('/setmonitor?state=0',{cache:'no-store'});}catch(err){} return true;}";

  // Poll visor salida
  html += "function pollGen(){fetch('/getgen?ts='+Date.now(),{cache:'no-store'}).then(r=>r.text()).then(t=>{let c=document.getElementById('genconsole');c.innerHTML=(t||'').split('\\n').join('<br>');c.scrollTop=c.scrollHeight;}).catch(()=>{});}";
  html += "setInterval(pollGen,300);";

  // Traducciones
  html += "function applyLangGen(){document.getElementById('genTitle').innerText=L[lang].title;document.getElementById('startBtn').innerText=running?L[lang].pause:L[lang].start;document.getElementById('clearBtn').innerText=L[lang].clear;document.getElementById('btnBack').innerText=L[lang].back;document.querySelectorAll('.lblSensor').forEach(e=>e.innerText=L[lang].sensor);document.querySelectorAll('.lblSentence').forEach(e=>e.innerText=L[lang].sentenceSel);document.querySelectorAll('.slotLabel').forEach(e=>e.innerText=L[lang].sentenceInline);document.querySelectorAll('.lblIntervalSlot').forEach(e=>e.innerText=L[lang].interval);document.getElementById('lblBaud').innerText=L[lang].baud;}";

  // DOM ready
  html += "document.addEventListener('DOMContentLoaded',async function(){";
  html += "  fetch('/setmode?m=generator',{cache:'no-store'});";
  html += "  lang=localStorage.getItem('lang')||'en';";
  html += "  for(let i=0;i<"+String(MAX_SLOTS)+";i++){ initSlot(i); }";
  html += "  const st=await getStatus();";
  html += "  running=!!st.genRunning;";
  html += "  applyLangGen();";
  html += "  var b=document.getElementById('gen_baud_'+(st.baud||4800)); if(b) b.classList.add('active');";
  html += "});";
  html += "</script><footer>© 2025 Matías Scuppa — by Themys</footer></body></html>";

  noCache();
  server.send(200, "text/html", html);
}

// ======================= API: Monitor/Generator ========
void handleToggleGen() {
  if (server.hasArg("state")) {
    bool newState = (server.arg("state") == "1");
    generatorRunning = newState;
    Serial.println(newState ? "▶ Generator: START" : "⏸ Generator: PAUSE");
  }
  noCache();
  server.send(200, "text/plain", generatorRunning ? "RUNNING" : "STOPPED");
}
void handleGetGen() {
  String output;
  xSemaphoreTake(genBufMutex, portMAX_DELAY);
  for (int i = 0; i < GEN_BUFFER_LINES; i++) {
    int idx = (genIndex + i) % GEN_BUFFER_LINES;
    if (genBuffer[idx].length() > 0) output += genBuffer[idx] + "\n";
  }
  xSemaphoreGive(genBufMutex);
  noCache();
  server.send(200, "text/plain", output);
}
void handleClearGen() {
  xSemaphoreTake(genBufMutex, portMAX_DELAY);
  for (int i = 0; i < GEN_BUFFER_LINES; i++) genBuffer[i] = "";
  genIndex = 0;
  xSemaphoreGive(genBufMutex);
  noCache();
  server.send(200, "text/plain", "OK");
  Serial.println("🧹 Generator visor limpiado");
}
void handleSetMode() {
  String m = server.hasArg("m") ? server.arg("m") : "monitor";
  appMode = (m == "generator") ? MODE_GENERATOR : MODE_MONITOR;
  generatorRunning = false;
  monitorRunning = false;
  Serial.printf("🔀 Modo => %s\n", (appMode == MODE_GENERATOR) ? "GENERATOR" : "MONITOR");
  noCache();
  server.send(200, "text/plain", (appMode == MODE_GENERATOR) ? "GENERATOR" : "MONITOR");
}
void handleSetMonitor() {
  if (server.hasArg("state")) {
    monitorRunning = (server.arg("state") == "1");
    Serial.println(monitorRunning ? "▶ Monitor: START" : "⏸ Monitor: PAUSE");
  }
  noCache();
  server.send(200, "text/plain", monitorRunning ? "RUNNING" : "PAUSED");
}
void handleGetNMEA() {
  String output;
  xSemaphoreTake(nmeaBufMutex, portMAX_DELAY);
  for (int i = 0; i < BUFFER_LINES; i++) {
    int idx = (bufferIndex + i) % BUFFER_LINES;
    if (nmeaBuffer[idx].length() > 0) output += nmeaBuffer[idx] + "\n";
  }
  xSemaphoreGive(nmeaBufMutex);
  noCache();
  server.send(200, "text/plain", output);
}
void handleSetBaud() {
  noCache();
  if (server.hasArg("baud")) {
    int b = server.arg("baud").toInt();
    if (b == 4800 || b == 9600 || b == 38400 || b == 115200) startSerial(b);
    server.send(200, "text/plain", "OK");
  } else server.send(400, "text/plain", "Error");
}

// ====== API SLOTS (Generator) ======
int argIndex() {
  if (!server.hasArg("i")) return -1;
  int i = server.arg("i").toInt();
  if (i < 0 || i >= MAX_SLOTS) return -1;
  return i;
}
void handleGenSlotEnable() {
  int i = argIndex(); if (i<0) { server.send(400,"text/plain","Bad slot"); return; }
  bool en = server.hasArg("en") && (server.arg("en").toInt()==1);
  slots[i].enabled = en;
  server.send(200,"text/plain", en?"1":"0");
  Serial.printf("⚙️ Slot %d: %s\n", i+1, en ? "ENABLED" : "DISABLED");
}
void handleGenSlotSensor() {
  int i = argIndex(); if (i<0) { server.send(400,"text/plain","Bad slot"); return; }
  if (server.hasArg("sensor")) {
    slots[i].sensor = server.arg("sensor");
    if (slots[i].sensor == "CUSTOM") slots[i].sentence = "CUSTOM";
    Serial.printf("⚙️ Slot %d: sensor = %s\n", i+1, slots[i].sensor.c_str());
  }
  server.send(200,"text/plain", slots[i].sensor);
}
void handleGenSlotSentence() {
  int i = argIndex(); if (i<0) { server.send(400,"text/plain","Bad slot"); return; }
  if (server.hasArg("sentence")) {
    slots[i].sentence = server.arg("sentence");
    Serial.printf("⚙️ Slot %d: sentence = %s\n", i+1, slots[i].sentence.c_str());
  }
  server.send(200,"text/plain", slots[i].sentence);
}
void handleGenSlotText_POST() {
  int i = -1;
  if (server.hasArg("i")) i = server.arg("i").toInt();
  if (i < 0 || i >= MAX_SLOTS) { server.send(400,"text/plain","Bad slot"); return; }
  String incoming = server.hasArg("text") ? server.arg("text") : "";
  slots[i].text = incoming; // llega con *HH desde el navegador
  server.send(200,"text/plain", incoming);
  Serial.printf("✏️  Slot %d: texto guardado (%u chars)\n", i+1, (unsigned)incoming.length());
}
void handleGenSlotText_GET() {
  int i = argIndex(); if (i<0) { server.send(400,"text/plain","Bad slot"); return; }
  String incoming = server.hasArg("text") ? server.arg("text") : "";
  slots[i].text = incoming;
  server.send(200,"text/plain", incoming);
  Serial.printf("✏️  Slot %d: texto guardado (%u chars)\n", i+1, (unsigned)incoming.length());
}
void handleGenSlotTemplate() {
  int i = argIndex(); if (i<0) { server.send(400,"text/plain","Bad slot"); return; }
  String t;
  if (slots[i].sensor == "CUSTOM" || slots[i].sentence == "CUSTOM") {
    t = slots[i].text.length()? slots[i].text : "$GPCUS,FIELD1,FIELD2*00";
    // normalizar checksum
    if (t.startsWith("$") || t.startsWith("!")) {
      int star = t.indexOf('*');
      String payload = (star>=0)? t.substring(1, star) : t.substring(1);
      t = String(t[0]) + payload + "*" + nmeaChecksum(payload);
    } else {
      String up=t; up.toUpperCase();
      char ch = (up.startsWith("AIVDM")||up.startsWith("AIVDO"))?'!':'$';
      String payload=t;
      t = String(ch)+payload+"*"+nmeaChecksum(payload);
    }
  } else {
    t = generateSentence(slots[i].sensor, slots[i].sentence);
  }
  slots[i].text = t;
  server.send(200,"text/plain", t);
  Serial.printf("🧩 Slot %d: plantilla (%s/%s)\n", i+1, slots[i].sensor.c_str(), slots[i].sentence.c_str());
}
void handleGenSlotInterval() {
  int i = argIndex(); if (i<0) { server.send(400,"text/plain","Bad slot"); return; }
  if (!server.hasArg("ms")) { server.send(400,"text/plain","Missing ms"); return; }
  long ms = server.arg("ms").toInt();
  if (ms < 50) ms = 50;
  slotInterval[i] = (unsigned long)ms;
  server.send(200,"text/plain", String(slotInterval[i]));
  Serial.printf("⏱️ Slot %d: intervalo = %lu ms\n", i+1, slotInterval[i]);
}

// ====== API de estado para sincronizar UI (Generator) ======
void handleGetStatus() {
  String json = "{";
  json += "\"mode\":\""; json += (appMode==MODE_GENERATOR?"generator":"monitor"); json += "\",";
  json += "\"baud\":"; json += String(currentBaud); json += ",";
  json += "\"genRunning\":"; json += (generatorRunning?"true":"false"); json += ",";
  json += "\"monRunning\":"; json += (monitorRunning?"true":"false");
  json += "}";
  noCache();
  server.send(200, "application/json", json);
}

// ======================= Tasks =========================
void TaskNet(void* pv) {
  for (;;) {
    dnsServer.processNextRequest();
    server.handleClient();
    vTaskDelay(1);
  }
}
void TaskNMEA(void* pv) {
  for (;;) {
    // MONITOR (leer UART)
    if (appMode == MODE_MONITOR && monitorRunning) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      while (NMEA_Serial.available()) {
        char c = (char)NMEA_Serial.read();
        if (c == '\n') {
          xSemaphoreGive(serialMutex);
          String line = currentLine; currentLine = "";
          bool valid = processNMEA(line);
          flashLed(valid ? pixels.Color(0,255,0) : pixels.Color(255,0,0));
          String type = detectSentenceType(line);
          String formatted = "[" + type + "] " + line;
          xSemaphoreTake(nmeaBufMutex, portMAX_DELAY);
          bufferIndex = (bufferIndex + 1) % BUFFER_LINES;
          nmeaBuffer[bufferIndex] = formatted;
          xSemaphoreGive(nmeaBufMutex);
          if (valid) sendUDP(line);
          xSemaphoreTake(serialMutex, portMAX_DELAY);
        } else if (c >= 32 && c <= 126) {
          currentLine += c;
        }
      }
      xSemaphoreGive(serialMutex);
    }

    // GENERATOR (TX por slot según intervalo individual)
    if (appMode == MODE_GENERATOR && generatorRunning) {
      unsigned long now = millis();
      for (int i=0;i<MAX_SLOTS;i++){
        if (!slots[i].enabled) continue;
        if (now - lastSentMs[i] >= slotInterval[i]) {
          lastSentMs[i] = now;
          String out = slots[i].text.length() ? slots[i].text : generateSentence(slots[i].sensor, slots[i].sentence);
          if (out.length()==0) continue;
          xSemaphoreTake(serialMutex, portMAX_DELAY);
          NMEA_Serial.println(out);
          xSemaphoreGive(serialMutex);
          sendUDP(out);
          pushGen(out);
          flashLed(pixels.Color(0,0,255)); // azul por envío
          Serial.printf("TX[%d]: %s\n", i+1, out.c_str());
        }
      }
    }

    updateLed();
    vTaskDelay(1);
  }
}

// ======================= Setup / Loop ===================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  esp_log_level_set("*", ESP_LOG_NONE);

  pixels.begin(); pixels.show();

  nmeaBufMutex = xSemaphoreCreateMutex();
  genBufMutex  = xSemaphoreCreateMutex();
  serialMutex  = xSemaphoreCreateMutex();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();

  dnsServer.start(DNS_PORT, "*", apIP);
  MDNS.begin("nmeareader"); MDNS.addService("http", "tcp", 80);

  flashLed(pixels.Color(0,255,255));
  startSerial(currentBaud);

  // UDP broadcast (AP típico .255)
  udpAddress = apIP; udpAddress[3] = 255;

  // Captive / probes
  server.on("/generate_204", handleCaptive);
  server.on("/gen_204", handleCaptive);
  server.on("/hotspot-detect.html", handleCaptive);
  server.on("/ncsi.txt", handle204);
  server.on("/favicon.ico", handle204);
  server.on("/robots.txt", handle204);
  server.on("/wpad.dat", handle204);

  // Monitor
  server.on("/", handleRoot);
  server.on("/getnmea", handleGetNMEA);
  server.on("/setbaud", handleSetBaud);
  server.on("/setmode", handleSetMode);
  server.on("/setmonitor", handleSetMonitor);

  // Generator
  server.on("/generator", handleGenerator);
  server.on("/togglegen", handleToggleGen);
  server.on("/getgen", handleGetGen);
  server.on("/cleargen", handleClearGen);
  server.on("/getstatus", handleGetStatus);

  // Slots
  server.on("/gen_slot_enable",    handleGenSlotEnable);
  server.on("/gen_slot_sensor",    handleGenSlotSensor);
  server.on("/gen_slot_sentence",  handleGenSlotSentence);
  server.on("/gen_slot_template",  handleGenSlotTemplate);
  server.on("/gen_slot_text", HTTP_POST, handleGenSlotText_POST);
  server.on("/gen_slot_text", HTTP_GET,  handleGenSlotText_GET);
  server.on("/gen_slot_interval",  handleGenSlotInterval);

  // Fallback: redirige todo lo desconocido a "/"
  server.onNotFound([](){
    noCache();
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();

  // -------- LOGS de arranque --------
  Serial.println("\n🚀 NMEA Link - boot");
  Serial.printf("📶 AP SSID: %s\n", AP_SSID);
  Serial.print( "📄 IP (AP): " ); Serial.println(apIP.toString());
  Serial.print( "🌐 UDP broadcast: " ); Serial.print(udpAddress.toString()); Serial.print(":"); Serial.println(udpPort);
  Serial.printf("🔧 UART RX=%d  TX=%d  baud=%d\n", RX_PIN, TX_PIN, currentBaud);
  Serial.println("✅ HTTP server + DNS (captive) listos");
  Serial.println("🧵 Tasks: Net(core0) + NMEA(core1) iniciadas");

  xTaskCreatePinnedToCore(TaskNet,  "TaskNet",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskNMEA, "TaskNMEA", 6144, NULL, 2, NULL, 1);
}

void loop() {
  // vacío: todo corre en tasks
}

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <INA226_WE.h>

const char* ssid     = "WIFI-NAME";
const char* password = "WIFI-PASS";

WebServer server(80);

#define I2C_ADDRESS  0x40
#define PANEL_MAX_W  0.9f
#define LOG_SIZE     3600  // 1 entry/sec = 1 hour max
#define GRAPH_SIZE   60    // Last 60 seconds shown on graph

INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

float totalEnergy_Wh = 0.0;
unsigned long lastTime = 0;

struct LogEntry {
  unsigned long t_sec;
  float voltage;
  float current;
  float power;
  float energy;
};

LogEntry logBuf[LOG_SIZE];
int  logCount  = 0;
bool recording = false;

// Circular buffer for graph data (last 60 power readings in mW)
float graphBuf[GRAPH_SIZE];
int   graphHead  = 0;
int   graphCount = 0;

// ─────────────────────────────────────────────────────────────
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SolarDuel Dashboard</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #fff; text-align: center; margin: 0; padding: 20px; }
    h2 { color: #f39c12; margin-bottom: 30px; letter-spacing: 1px; }
    .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; max-width: 600px; margin: 0 auto; }
    .card { background-color: #1e1e1e; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.4); border: 1px solid #333; }
    .card.full { grid-column: span 2; }
    .val { font-size: 28px; font-weight: bold; color: #4CAF50; margin-top: 10px; }
    .val.blue   { color: #38bdf8; }
    .val.yellow { color: #facc15; }
    .label { font-size: 14px; color: #aaaaaa; text-transform: uppercase; letter-spacing: 1px; }

    /* Graph */
    .graph-card { background-color: #1e1e1e; border-radius: 10px; border: 1px solid #333; padding: 16px 16px 8px; max-width: 600px; margin: 15px auto 0; }
    .graph-title { font-size: 13px; color: #aaaaaa; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 8px; }
    canvas { width: 100% !important; height: 140px !important; display: block; }

    /* Buttons */
    .btn-row { display: flex; justify-content: center; gap: 12px; flex-wrap: wrap; margin-top: 20px; }
    .btn {
      padding: 13px 28px; border-radius: 8px;
      font-size: 14px; font-weight: bold;
      border: none; cursor: pointer;
      letter-spacing: 0.5px; transition: background 0.2s; color: #fff;
    }
    .btn-start { background: #16a34a; }
    .btn-start:hover { background: #15803d; }
    .btn-stop  { background: #dc2626; }
    .btn-stop:hover  { background: #b91c1c; }
    .btn-reset { background: #475569; }
    .btn-reset:hover { background: #334155; }

    #rec-status { margin-top: 12px; font-size: 13px; color: #555; }
    #rec-dot { display:inline-block; width:9px; height:9px; border-radius:50%; background:#555; margin-right:5px; vertical-align:middle; }
    #rec-dot.active { background:#ef4444; animation: blink 1s infinite; }
    @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.2} }
    .footer { margin-top: 16px; font-size: 12px; color: #555; }
    @media (max-width: 400px) { .grid-container { grid-template-columns: 1fr; } .card.full { grid-column: span 1; } }
  </style>
</head>
<body>
  <h2>🌟 SolarDuel Pro Dashboard</h2>

  <div class="grid-container">
    <div class="card">
      <div class="label">Panel Voltage</div>
      <div class="val"><span id="voltage">--</span> V</div>
    </div>
    <div class="card">
      <div class="label">Current Draw</div>
      <div class="val"><span id="current">--</span> mA</div>
    </div>
    <div class="card">
      <div class="label">Instant Power</div>
      <div class="val yellow"><span id="power">--</span> mW</div>
    </div>
    <div class="card">
      <div class="label">Total Energy</div>
      <div class="val blue"><span id="energy">--</span> Wh</div>
    </div>
    <div class="card full">
      <div class="label">Panel Efficiency</div>
      <div class="val"><span id="efficiency">--</span> %</div>
    </div>
  </div>

  <!-- Power Graph -->
  <div class="graph-card">
    <div class="graph-title">⚡ Power — Last 60 Seconds (mW)</div>
    <canvas id="graph"></canvas>
  </div>

  <!-- Buttons -->
  <div class="btn-row">
    <button id="recBtn" class="btn btn-start" onclick="toggleRecording()">⏺ Start Recording</button>
    <button class="btn btn-reset" onclick="resetAll()">Reset All</button>
  </div>

  <div id="rec-status">
    <span id="rec-dot"></span>
    <span id="rec-text">Not recording</span>
  </div>

  <div class="footer" id="status">Connected. Updating every second.</div>

  <script>
    let isRecording = false;

    // ── Graph Setup ───────────────────────────────────────────
    const canvas  = document.getElementById('graph');
    const ctx     = canvas.getContext('2d');
    let graphData = [];

    function drawGraph(data) {
      const W = canvas.offsetWidth;
      const H = canvas.offsetHeight;
      canvas.width  = W;
      canvas.height = H;

      const pad = { top: 10, bottom: 24, left: 40, right: 10 };
      const w = W - pad.left - pad.right;
      const h = H - pad.top  - pad.bottom;

      ctx.clearRect(0, 0, W, H);

      const max = Math.max(...data, 0.01);

      // Grid lines
      ctx.strokeStyle = '#2d3748';
      ctx.lineWidth   = 1;
      for (let i = 0; i <= 4; i++) {
        const y = pad.top + (h / 4) * i;
        ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(pad.left + w, y); ctx.stroke();
        ctx.fillStyle = '#4a5568';
        ctx.font = '10px Segoe UI';
        ctx.textAlign = 'right';
        ctx.fillText((max * (1 - i / 4)).toFixed(1), pad.left - 4, y + 4);
      }

      if (data.length < 2) return;

      // Fill area
      const grad = ctx.createLinearGradient(0, pad.top, 0, pad.top + h);
      grad.addColorStop(0,   'rgba(250, 204, 21, 0.4)');
      grad.addColorStop(1,   'rgba(250, 204, 21, 0.02)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      data.forEach((v, i) => {
        const x = pad.left + (i / (data.length - 1)) * w;
        const y = pad.top  + h - (v / max) * h;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });
      ctx.lineTo(pad.left + w, pad.top + h);
      ctx.lineTo(pad.left,     pad.top + h);
      ctx.closePath();
      ctx.fill();

      // Line
      ctx.strokeStyle = '#facc15';
      ctx.lineWidth   = 2;
      ctx.lineJoin    = 'round';
      ctx.beginPath();
      data.forEach((v, i) => {
        const x = pad.left + (i / (data.length - 1)) * w;
        const y = pad.top  + h - (v / max) * h;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });
      ctx.stroke();

      // X-axis labels
      ctx.fillStyle  = '#4a5568';
      ctx.font       = '10px Segoe UI';
      ctx.textAlign  = 'center';
      const labels   = ['-60s', '-45s', '-30s', '-15s', 'now'];
      labels.forEach((l, i) => {
        const x = pad.left + (i / (labels.length - 1)) * w;
        ctx.fillText(l, x, H - 4);
      });
    }

    // ── Recording ─────────────────────────────────────────────
    function toggleRecording() {
      if (!isRecording) {
        fetch('/start').then(r => r.json()).then(d => {
          if (d.ok) {
            isRecording = true;
            document.getElementById('recBtn').className   = 'btn btn-stop';
            document.getElementById('recBtn').textContent = '⏹ Stop & Download CSV';
            document.getElementById('rec-dot').className  = 'active';
            document.getElementById('rec-text').textContent = 'Recording...';
          }
        });
      } else {
        fetch('/stop').then(r => r.json()).then(d => {
          isRecording = false;
          document.getElementById('recBtn').className   = 'btn btn-start';
          document.getElementById('recBtn').textContent = '⏺ Start Recording';
          document.getElementById('rec-dot').className  = '';
          document.getElementById('rec-text').textContent = d.count + ' entries recorded — downloading...';

          const a = document.createElement('a');
          a.href     = '/csv';
          a.download = 'solarduel_' + new Date().toISOString().slice(0,19).replace(/[:T]/g,'-') + '.csv';
          a.click();
        });
      }
    }

    // ── Reset ─────────────────────────────────────────────────
    function resetAll() {
      if (!confirm('Reset all measurements and graph data?')) return;
      fetch('/reset').then(r => r.json()).then(d => {
        if (d.ok) {
          graphData = [];
          drawGraph([]);
          document.getElementById('energy').innerText     = '0.0000';
          document.getElementById('rec-text').textContent = 'Not recording';
          document.getElementById('rec-dot').className    = '';
          isRecording = false;
          document.getElementById('recBtn').className   = 'btn btn-start';
          document.getElementById('recBtn').textContent = '⏺ Start Recording';
        }
      });
    }

    // ── Dashboard Refresh ─────────────────────────────────────
    setInterval(function() {
      fetch('/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('voltage').innerText    = d.voltage.toFixed(2);
          document.getElementById('current').innerText    = d.current.toFixed(2);
          document.getElementById('power').innerText      = d.power.toFixed(2);
          document.getElementById('energy').innerText     = d.energy.toFixed(4);
          document.getElementById('efficiency').innerText = d.efficiency.toFixed(1);
          document.getElementById('status').innerText     = 'Last update: ' + new Date().toLocaleTimeString();

          if (isRecording && d.log_count !== undefined)
            document.getElementById('rec-text').textContent = 'Recording... (' + d.log_count + ' entries)';

          // Update graph
          graphData = d.graph;
          drawGraph(graphData);
        })
        .catch(() => {
          document.getElementById('status').innerText = '⚠️ Connection lost...';
        });
    }, 1000);
  </script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleData() {
  ina226.readAndClearFlags();

  float voltage = ina226.getBusVoltage_V();
  float current = ina226.getCurrent_mA();
  float power   = voltage * (current / 1000.0f);

  if (voltage < 0) voltage = 0;
  if (current < 0) current = 0;
  if (power   < 0) power   = 0;

  // Cumulative energy integration (Wh)
  unsigned long now = millis();
  float dt_hours = (now - lastTime) / 3600000.0f;
  if (lastTime > 0) totalEnergy_Wh += power * dt_hours;
  lastTime = now;

  // Append to CSV log buffer when recording
  if (recording && logCount < LOG_SIZE) {
    LogEntry& e = logBuf[logCount++];
    e.t_sec   = now / 1000;
    e.voltage = voltage;
    e.current = current;
    e.power   = power * 1000.0f;
    e.energy  = totalEnergy_Wh;
  }

  // Append to circular graph buffer
  graphBuf[graphHead] = power * 1000.0f; // store as mW
  graphHead = (graphHead + 1) % GRAPH_SIZE;
  if (graphCount < GRAPH_SIZE) graphCount++;

  float efficiency = (power / PANEL_MAX_W) * 100.0f;
  if (efficiency > 100.0f) efficiency = 100.0f;

  // Build JSON — include graph array in chronological order
  String json = "{";
  json += "\"voltage\":"    + String(voltage, 2)         + ",";
  json += "\"current\":"    + String(current, 2)         + ",";
  json += "\"power\":"      + String(power * 1000.0f, 2) + ",";
  json += "\"energy\":"     + String(totalEnergy_Wh, 4)  + ",";
  json += "\"efficiency\":" + String(efficiency, 1)      + ",";
  json += "\"log_count\":"  + String(logCount)           + ",";
  json += "\"graph\":[";

  int start = (graphCount < GRAPH_SIZE) ? 0 : graphHead;
  for (int i = 0; i < graphCount; i++) {
    int idx = (start + i) % GRAPH_SIZE;
    json += String(graphBuf[idx], 2);
    if (i < graphCount - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

void handleStart() {
  recording = true;
  logCount  = 0;
  server.send(200, "application/json", "{\"ok\":true}");
  Serial.println(">> Recording STARTED");
}

void handleStop() {
  recording = false;
  String json = "{\"ok\":true,\"count\":" + String(logCount) + "}";
  server.send(200, "application/json", json);
  Serial.printf(">> Recording STOPPED — %d entries ready\n", logCount);
}

void handleReset() {
  // Clear all runtime state
  totalEnergy_Wh = 0.0f;
  lastTime       = millis();
  recording      = false;
  logCount       = 0;
  graphHead      = 0;
  graphCount     = 0;
  memset(graphBuf, 0, sizeof(graphBuf));
  server.send(200, "application/json", "{\"ok\":true}");
  Serial.println(">> RESET — all data cleared");
}

void handleCSV() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("Time_s,Voltage_V,Current_mA,Power_mW,Energy_Wh\n");

  for (int i = 0; i < logCount; i++) {
    char row[80];
    snprintf(row, sizeof(row), "%lu,%.2f,%.2f,%.2f,%.4f\n",
             logBuf[i].t_sec, logBuf[i].voltage,
             logBuf[i].current, logBuf[i].power, logBuf[i].energy);
    server.sendContent(row);
  }
  server.sendContent("");
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // I2C pins: SDA → GPIO21, SCL → GPIO22
  Wire.begin(21, 22);

  if (!ina226.init()) {
    Serial.println("INA226 not found on I2C bus!");
    while (1) delay(1000);
  }

  // Shunt resistor: 0.1Ω, max expected current: 1.5A
  ina226.setResistorRange(0.1f, 1.5f);
  ina226.setAverage(INA226_AVERAGE_16);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! Open in browser: http://");
  Serial.println(WiFi.localIP());

  server.on("/",      handleRoot);
  server.on("/data",  handleData);
  server.on("/start", handleStart);
  server.on("/stop",  handleStop);
  server.on("/reset", handleReset); // ← Clear all runtime data
  server.on("/csv",   handleCSV);
  server.begin();

  lastTime = millis();
}

void loop() {
  server.handleClient();

  // Serial telemetry + INA226 flag clear every second
  // Flag clear prevents sensor freeze after panel reconnect
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 1000) {
    lastLog = millis();
    ina226.readAndClearFlags();
    Serial.printf("V: %.2f V  |  I: %.2f mA  |  E: %.4f Wh  |  Recording: %s (%d entries)\n",
                  ina226.getBusVoltage_V(),
                  ina226.getCurrent_mA(),
                  totalEnergy_Wh,
                  recording ? "ACTIVE" : "stopped",
                  logCount);
  }
}
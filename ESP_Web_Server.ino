#include <WiFi.h>
#include <WebServer.h>

// ================= PIN DEFINITIONS =================
#define PDB       4
#define LE_PIN    10
#define CLK_PIN   2
#define DATA_PIN  3
#define CE_PIN    18

#define ADC_PIN   0   // GPIO0

// ================= USER SETTINGS =================
double centerFreq = 2870.0;   // MHz
double span       = 75.0;     // MHz total span
double stepSize   = 0.8;      // MHz step size
int sweepDelay    = 10;       // ms delay after frequency step
int numOfPoints   = 150;      // number of averaged voltage points per frequency

// ================= PLL CONSTANTS =================
double fPFD = 25.0;
unsigned long MOD = 1000;

#define DELAY_US 1

#define REGISTER_5 0x00580005U
#define REGISTER_3 0x008004B3U
#define REGISTER_2 0x18006E42U
#define REG4_BASE  0x008C803CU
#define REG4_MASK  0xFF8FFFFFU

// ================= WIFI SETTINGS =================
const char* apSSID = "NV_Sensor";
const char* apPASS = "12345678";

// ================= SERVER =================
WebServer server(80);

// ================= SWEEP STORAGE =================
const int MAX_POINTS = 128;
double freqData[MAX_POINTS];
double voltData[MAX_POINTS];
int pointCount = 0;

// ================= RESULTS =================
double leftDipFreq    = 0.0;
double rightDipFreq   = 0.0;
double leftDipVolt    = 0.0;
double rightDipVolt   = 0.0;
double magneticFieldG = 0.0;
unsigned long sweepNumber = 0;

// ================= CONTROL FLAGS =================
bool continuousSweepEnabled = false;
bool requestSingleSweep     = false;
bool sweepInProgress        = false;

// ============================================================
// HTML PAGE
// ============================================================
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>NV Sensor Dashboard</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background: #111;
      color: #eee;
      margin: 0;
      padding: 16px;
    }
    h1 {
      margin-top: 0;
      font-size: 28px;
    }
    .wrap {
      display: flex;
      flex-wrap: wrap;
      gap: 16px;
    }
    .card {
      background: #1b1b1b;
      border-radius: 12px;
      padding: 16px;
      box-shadow: 0 0 10px rgba(0,0,0,0.35);
    }
    .stats {
      min-width: 300px;
      flex: 1;
    }
    .graph {
      flex: 2;
      min-width: 320px;
    }
    .value {
      font-size: 28px;
      font-weight: bold;
      margin: 8px 0 16px 0;
      color: #4fc3f7;
    }
    .small {
      font-size: 16px;
      margin: 6px 0;
    }
    .mode {
      margin-top: 14px;
      padding: 10px 12px;
      border-radius: 8px;
      background: #252525;
      font-size: 15px;
    }
    .btnRow {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 18px;
    }
    button {
      border: none;
      border-radius: 10px;
      padding: 12px 16px;
      font-size: 15px;
      font-weight: bold;
      cursor: pointer;
      color: white;
    }
    #singleBtn {
      background: #1976d2;
    }
    #contBtn {
      background: #2e7d32;
    }
    button:hover {
      opacity: 0.9;
    }
    canvas {
      width: 100%;
      max-width: 100%;
      height: 420px;
      background: #000;
      border-radius: 8px;
      border: 1px solid #444;
    }
    .footer {
      margin-top: 10px;
      color: #aaa;
      font-size: 14px;
    }
  </style>
</head>
<body>
  <h1>NV Sensor Dashboard</h1>

  <div class="wrap">
    <div class="card stats">
      <div class="small">Magnetic Field</div>
      <div class="value" id="bfield">Loading...</div>

      <div class="small">Left Dip: <span id="leftDip">--</span> MHz</div>
      <div class="small">Right Dip: <span id="rightDip">--</span> MHz</div>
      <div class="small">Sweep #: <span id="sweepNum">--</span></div>
      <div class="small">Points: <span id="pointCount">--</span></div>

      <div class="mode">
        Mode: <span id="modeText">Idle</span><br>
        Sweep In Progress: <span id="busyText">No</span>
      </div>

      <div class="btnRow">
        <button id="singleBtn" onclick="runSingleSweep()">Single Sweep</button>
        <button id="contBtn" onclick="toggleContinuousSweep()">Continuous Sweep</button>
      </div>
    </div>

    <div class="card graph">
      <canvas id="plot" width="900" height="420"></canvas>
      <div class="footer">Latest microwave sweep</div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('plot');
    const ctx = canvas.getContext('2d');

    async function runSingleSweep() {
      try {
        await fetch('/single', { method: 'POST' });
      } catch (err) {
        console.log('Single sweep error:', err);
      }
    }

    async function toggleContinuousSweep() {
      try {
        await fetch('/continuous', { method: 'POST' });
      } catch (err) {
        console.log('Continuous toggle error:', err);
      }
    }

    function drawPlot(freq, volt, leftDip, rightDip) {
      const w = canvas.width;
      const h = canvas.height;
      const padL = 70;
      const padR = 20;
      const padT = 20;
      const padB = 45;

      ctx.clearRect(0, 0, w, h);

      // Background
      ctx.fillStyle = "#000";
      ctx.fillRect(0, 0, w, h);

      if (!freq || freq.length < 2 || !volt || volt.length < 2) {
        ctx.fillStyle = "#fff";
        ctx.font = "20px Arial";
        ctx.fillText("No sweep data yet...", 40, 50);
        return;
      }

      let minX = Math.min(...freq);
      let maxX = Math.max(...freq);
      let minY = Math.min(...volt);
      let maxY = Math.max(...volt);

      if (maxY === minY) {
        maxY += 1;
        minY -= 1;
      }

      const plotW = w - padL - padR;
      const plotH = h - padT - padB;

      function xMap(x) {
        return padL + ((x - minX) / (maxX - minX)) * plotW;
      }

      function yMap(y) {
        return padT + (1 - (y - minY) / (maxY - minY)) * plotH;
      }

      // Grid
      ctx.strokeStyle = "#333";
      ctx.lineWidth = 1;
      const gridLines = 5;
      for (let i = 0; i <= gridLines; i++) {
        let y = padT + (i / gridLines) * plotH;
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(w - padR, y);
        ctx.stroke();
      }
      for (let i = 0; i <= gridLines; i++) {
        let x = padL + (i / gridLines) * plotW;
        ctx.beginPath();
        ctx.moveTo(x, padT);
        ctx.lineTo(x, h - padB);
        ctx.stroke();
      }

      // Axes
      ctx.strokeStyle = "#aaa";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(padL, padT);
      ctx.lineTo(padL, h - padB);
      ctx.lineTo(w - padR, h - padB);
      ctx.stroke();

      // Plot line
      ctx.strokeStyle = "#33aaff";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(xMap(freq[0]), yMap(volt[0]));
      for (let i = 1; i < freq.length; i++) {
        ctx.lineTo(xMap(freq[i]), yMap(volt[i]));
      }
      ctx.stroke();

      // Dip markers
      function drawMarker(xVal, color) {
        if (!isFinite(xVal)) return;
        const x = xMap(xVal);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x, padT);
        ctx.lineTo(x, h - padB);
        ctx.stroke();
      }

      drawMarker(leftDip, "#ff5252");
      drawMarker(rightDip, "#00e676");

      // Labels
      ctx.fillStyle = "#ddd";
      ctx.font = "16px Arial";
      ctx.fillText("Voltage (V)", 10, 22);
      ctx.fillText("Frequency (MHz)", w / 2 - 55, h - 8);

      // X axis numbers
      ctx.fillStyle = "#bbb";
      ctx.font = "13px Arial";
      for (let i = 0; i <= gridLines; i++) {
        let xVal = minX + (i / gridLines) * (maxX - minX);
        let x = padL + (i / gridLines) * plotW;
        ctx.fillText(xVal.toFixed(1), x - 14, h - padB + 20);
      }

      // Y axis numbers
      for (let i = 0; i <= gridLines; i++) {
        let yVal = maxY - (i / gridLines) * (maxY - minY);
        let y = padT + (i / gridLines) * plotH;
        ctx.fillText(yVal.toFixed(4), 8, y + 4);
      }
    }

    async function fetchData() {
      try {
        const response = await fetch('/data');
        const data = await response.json();

        document.getElementById('bfield').textContent = data.magneticFieldG.toFixed(4) + " G";
        document.getElementById('leftDip').textContent = data.leftDipFreq.toFixed(3);
        document.getElementById('rightDip').textContent = data.rightDipFreq.toFixed(3);
        document.getElementById('sweepNum').textContent = data.sweepNumber;
        document.getElementById('pointCount').textContent = data.pointCount;
        document.getElementById('modeText').textContent = data.mode;
        document.getElementById('busyText').textContent = data.sweepInProgress ? "Yes" : "No";

        drawPlot(data.freq, data.volt, data.leftDipFreq, data.rightDipFreq);

      } catch (err) {
        console.log("Fetch error:", err);
      }
    }

    fetchData();
    setInterval(fetchData, 1000);
  </script>
</body>
</html>
)rawliteral";

// ============================================================
// PLL FUNCTIONS
// ============================================================
void write_register(unsigned long value) {
  digitalWrite(LE_PIN, LOW);
  delayMicroseconds(DELAY_US);

  for (int i = 31; i >= 0; i--) {
    digitalWrite(DATA_PIN, (value >> i) & 1);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(DELAY_US);
    digitalWrite(CLK_PIN, LOW);
  }

  digitalWrite(LE_PIN, HIGH);
  delayMicroseconds(DELAY_US);
}

void setFrequency(double freqMHz) {
  uint8_t rfDiv = 0;
  uint32_t divider = 1;

  if (freqMHz >= 2200.0)      { rfDiv = 0; divider = 1; }
  else if (freqMHz >= 1100.0) { rfDiv = 1; divider = 2; }
  else if (freqMHz >= 550.0)  { rfDiv = 2; divider = 4; }
  else if (freqMHz >= 275.0)  { rfDiv = 3; divider = 8; }
  else if (freqMHz >= 137.5)  { rfDiv = 4; divider = 16; }
  else if (freqMHz >= 68.75)  { rfDiv = 5; divider = 32; }
  else                        { rfDiv = 6; divider = 64; }

  double vcoFreq = freqMHz * divider;
  double N = vcoFreq / fPFD;

  unsigned long INT  = (unsigned long)N;
  unsigned long FRAC = (unsigned long)((N - INT) * MOD + 0.5);

  if (FRAC >= MOD) {
    FRAC = 0;
    INT++;
  }

  unsigned long reg4 = (REG4_BASE & REG4_MASK) | ((unsigned long)rfDiv << 20);
  unsigned long reg1 = (MOD << 3) | 0x01;
  unsigned long reg0 = (INT << 15) | (FRAC << 3) | 0x00;

  write_register(REGISTER_5);
  write_register(reg4);
  write_register(REGISTER_3);
  write_register(REGISTER_2);
  write_register(reg1);
  write_register(reg0);

  delay(15);
}

// ============================================================
// DATA ACQUISITION
// ============================================================
double readAveragedVoltage() {
  double avgVolt = 0.0;

  for (int j = 0; j < numOfPoints; j++) {
    int sum = 0;
    for (int i = 0; i < 32; i++) {
      sum += analogRead(ADC_PIN);
    }

    int adcValue = sum / 32;
    double voltage = (adcValue / 4095.0) * 3.3;
    avgVolt += voltage;
  }

  avgVolt /= numOfPoints;
  return avgVolt;
}

void findDipsAndField() {
  if (pointCount < 3) {
    leftDipFreq = 0;
    rightDipFreq = 0;
    leftDipVolt = 0;
    rightDipVolt = 0;
    magneticFieldG = 0;
    return;
  }

  int leftIndex = -1;
  int rightIndex = -1;

  // Find minimum on left side of center
  for (int i = 0; i < pointCount; i++) {
    if (freqData[i] < centerFreq) {
      if (leftIndex == -1 || voltData[i] < voltData[leftIndex]) {
        leftIndex = i;
      }
    }
  }

  // Find minimum on right side of center
  for (int i = 0; i < pointCount; i++) {
    if (freqData[i] > centerFreq) {
      if (rightIndex == -1 || voltData[i] < voltData[rightIndex]) {
        rightIndex = i;
      }
    }
  }

  if (leftIndex != -1) {
    leftDipFreq = freqData[leftIndex];
    leftDipVolt = voltData[leftIndex];
  }

  if (rightIndex != -1) {
    rightDipFreq = freqData[rightIndex];
    rightDipVolt = voltData[rightIndex];
  }

  if (leftIndex != -1 && rightIndex != -1) {
    double deltaF = rightDipFreq - leftDipFreq;   // MHz
    magneticFieldG = deltaF / (2.0 * 2.8);        // G
  } else {
    magneticFieldG = 0.0;
  }
}

void performSweep() {
  sweepInProgress = true;

  double startFreq = centerFreq - (span / 2.0);
  double endFreq   = centerFreq + (span / 2.0);

  pointCount = 0;

  for (double f = startFreq; f <= endFreq && pointCount < MAX_POINTS; f += stepSize) {
    setFrequency(f);
    delay(sweepDelay);

    double avgVolt = readAveragedVoltage();

    freqData[pointCount] = f;
    voltData[pointCount] = avgVolt;
    pointCount++;

    // Keep webpage responsive during sweep
    server.handleClient();
    delay(1);
  }

  setFrequency(centerFreq);
  findDipsAndField();
  sweepNumber++;

  sweepInProgress = false;
}

// ============================================================
// WEB SERVER HANDLERS
// ============================================================
String getModeString() {
  if (sweepInProgress) return "Sweeping";
  if (continuousSweepEnabled) return "Continuous";
  if (requestSingleSweep) return "Single Requested";
  return "Idle";
}

void handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void handleData() {
  String json = "{";
  json += "\"magneticFieldG\":" + String(magneticFieldG, 6) + ",";
  json += "\"leftDipFreq\":" + String(leftDipFreq, 6) + ",";
  json += "\"rightDipFreq\":" + String(rightDipFreq, 6) + ",";
  json += "\"leftDipVolt\":" + String(leftDipVolt, 6) + ",";
  json += "\"rightDipVolt\":" + String(rightDipVolt, 6) + ",";
  json += "\"sweepNumber\":" + String(sweepNumber) + ",";
  json += "\"pointCount\":" + String(pointCount) + ",";
  json += "\"sweepInProgress\":" + String(sweepInProgress ? "true" : "false") + ",";
  json += "\"mode\":\"" + getModeString() + "\",";

  json += "\"freq\":[";
  for (int i = 0; i < pointCount; i++) {
    json += String(freqData[i], 3);
    if (i < pointCount - 1) json += ",";
  }
  json += "],";

  json += "\"volt\":[";
  for (int i = 0; i < pointCount; i++) {
    json += String(voltData[i], 6);
    if (i < pointCount - 1) json += ",";
  }
  json += "]";

  json += "}";

  server.send(200, "application/json", json);
}

void handleSingleSweep() {
  continuousSweepEnabled = false;
  requestSingleSweep = true;
  server.send(200, "text/plain", "Single sweep requested");
}

void handleContinuousSweep() {
  continuousSweepEnabled = !continuousSweepEnabled;
  if (continuousSweepEnabled) {
    requestSingleSweep = false;
    server.send(200, "text/plain", "Continuous sweep enabled");
  } else {
    server.send(200, "text/plain", "Continuous sweep disabled");
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PDB, OUTPUT);
  pinMode(LE_PIN, OUTPUT);
  pinMode(CE_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  digitalWrite(PDB, HIGH);
  digitalWrite(LE_PIN, HIGH);
  digitalWrite(CE_PIN, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Start WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID, apPASS);

  Serial.println();
  Serial.println("==================================");
  Serial.println("ESP32 AP started");
  Serial.print("SSID: ");
  Serial.println(apSSID);
  Serial.print("Password: ");
  Serial.println(apPASS);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Open this in your browser: http://192.168.4.1");
  Serial.println("==================================");

  server.on("/", handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/single", HTTP_POST, handleSingleSweep);
  server.on("/continuous", HTTP_POST, handleContinuousSweep);
  server.begin();

  // One startup sweep so the page isn't empty on first load.
  // Delete this line if you want it to start with no data at all.
  performSweep();
}

void loop() {
  server.handleClient();

  if (!sweepInProgress) {
    if (requestSingleSweep) {
      requestSingleSweep = false;
      performSweep();
    } 
    else if (continuousSweepEnabled) {
      performSweep();
    }
  }

  delay(2);
}
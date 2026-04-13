#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ================= PIN DEFINITIONS =================
#define PDB       4
#define LE_PIN    10
#define CLK_PIN   2
#define DATA_PIN  3
#define CE_PIN    18

#define ADC_PIN   0   // GPIO0

// ================= USER SETTINGS =================
double centerFreq = 2870.0;   // MHz
double span       = 100.0;    // MHz total span
double stepSize   = 0.8;      // MHz step size
int sweepDelay    = 10;       // ms delay after frequency step
int numOfPoints   = 150;      // number of averaged voltage points per frequency

// ===== PDB modulation settings =====
// Set to 0 for steady HIGH, or set to a frequency in Hz for square-wave modulation.
uint32_t pdbModFreq = 11000;  // Hz, default = 11 kHz
const uint8_t pdbPwmResolution = 8;
const uint32_t pdbPwmDuty = 128; // ~50% duty for 8-bit PWM

// ===== Parked-mode settings =====
int parkedSettleDelay = 10;                  // ms after each parked/reference frequency set
unsigned long parkedUpdateIntervalMs = 500; // how often parked mode updates
unsigned long autoResweepIntervalMs   = 600000UL; // 10 minutes
bool autoResweepEnabled = true;

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
const int MAX_POINTS = 512;
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

// ===== Parked mode/calibration results =====
double parkedFreqMHz          = 0.0;
double refFreqMHz             = 0.0;
double parkedSlopeNormPerMHz  = 0.0;
double baselineNormSignal     = 0.0;
double baselineParkVolt       = 0.0;
double baselineRefVolt        = 0.0;
double baselineField_uT       = 0.0;
double currentParkVolt        = 0.0;
double currentRefVolt         = 0.0;
double currentNormSignal      = 0.0;
double deltaField_uT          = 0.0;
double currentField_uT        = 0.0;
bool parkedCalibrationValid   = false;

// ================= CONTROL FLAGS =================
bool continuousSweepEnabled = false;
bool requestSingleSweep     = false;
bool sweepInProgress        = false;

bool parkedModeEnabled      = false;
bool requestParkedCal       = false;

// ================= TIMERS =================
unsigned long lastParkedUpdateMs = 0;
unsigned long lastResweepMs      = 0;

// ============================================================
// PWM / PDB MODULATION HELPERS
// ============================================================

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Arduino ESP32 Core 3.x style
  void startPdbModulation(uint32_t freqHz) {
    if (freqHz == 0) {
      pinMode(PDB, OUTPUT);
      digitalWrite(PDB, HIGH);
      return;
    }

    ledcDetach(PDB); // detach first in case we are changing frequency
    if (!ledcAttach(PDB, freqHz, pdbPwmResolution)) {
      pinMode(PDB, OUTPUT);
      digitalWrite(PDB, HIGH);
      return;
    }
    ledcWrite(PDB, pdbPwmDuty);
  }

  void stopPdbModulationAndHoldHigh() {
    ledcDetach(PDB);
    pinMode(PDB, OUTPUT);
    digitalWrite(PDB, HIGH);
  }

#else
  // Arduino ESP32 Core 2.x style
  const uint8_t PDB_PWM_CHANNEL = 0;

  void startPdbModulation(uint32_t freqHz) {
    if (freqHz == 0) {
      ledcDetachPin(PDB);
      pinMode(PDB, OUTPUT);
      digitalWrite(PDB, HIGH);
      return;
    }

    ledcSetup(PDB_PWM_CHANNEL, freqHz, pdbPwmResolution);
    ledcAttachPin(PDB, PDB_PWM_CHANNEL);
    ledcWrite(PDB_PWM_CHANNEL, pdbPwmDuty);
  }

  void stopPdbModulationAndHoldHigh() {
    ledcDetachPin(PDB);
    pinMode(PDB, OUTPUT);
    digitalWrite(PDB, HIGH);
  }
#endif

void applyPdbMode() {
  if (pdbModFreq == 0) {
    stopPdbModulationAndHoldHigh();
  } else {
    startPdbModulation(pdbModFreq);
  }
}

// ============================================================
// HELPER FUNCTIONS
// ============================================================
double clampNonZero(double x, double minVal = 1e-9) {
  if (x >= 0.0 && x < minVal) return minVal;
  if (x < 0.0 && x > -minVal) return -minVal;
  return x;
}

double avgRange(double* arr, int startIdx, int endIdx) {
  if (pointCount <= 0) return 0.0;
  if (startIdx < 0) startIdx = 0;
  if (endIdx >= pointCount) endIdx = pointCount - 1;
  if (endIdx < startIdx) return 0.0;

  double sum = 0.0;
  int count = 0;
  for (int i = startIdx; i <= endIdx; i++) {
    sum += arr[i];
    count++;
  }
  if (count == 0) return 0.0;
  return sum / count;
}

double smoothedVoltAt(int idx) {
  int startIdx = idx - 2;
  int endIdx   = idx + 2;
  return avgRange(voltData, startIdx, endIdx);
}

int getExpectedSweepPoints() {
  if (stepSize <= 0.0) return 0;
  return (int)(span / stepSize) + 1;
}

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
      min-width: 340px;
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
      font-size: 15px;
      margin: 6px 0;
      line-height: 1.45;
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
    #singleBtn { background: #1976d2; }
    #contBtn   { background: #2e7d32; }
    #parkBtn   { background: #8e24aa; }
    #saveBtn   { background: #ef6c00; }
    button:hover { opacity: 0.9; }

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

    .sectionTitle {
      margin-top: 16px;
      margin-bottom: 8px;
      font-size: 17px;
      color: #ffd54f;
    }

    .settingsGrid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 10px;
    }

    .settingsGrid label {
      font-size: 13px;
      color: #ccc;
      display: block;
      margin-bottom: 4px;
    }

    .settingsGrid input {
      width: 100%;
      box-sizing: border-box;
      padding: 8px;
      border-radius: 8px;
      border: 1px solid #444;
      background: #0f0f0f;
      color: white;
    }

    #settingsMsg {
      margin-top: 10px;
      font-size: 14px;
      color: #90caf9;
    }

    .hint {
      color: #999;
      font-size: 12px;
      margin-top: 2px;
    }
  </style>
</head>
<body>
  <h1>NV Sensor Dashboard</h1>

  <div class="wrap">
    <div class="card stats">
      <div class="small">Current Magnetic Field Estimate</div>
      <div class="value" id="bfielduT">Loading...</div>

      <div class="small">ΔB from parked baseline: <span id="deltaB">--</span> µT</div>
      <div class="small">Baseline field from sweep: <span id="baselineB">--</span> µT</div>

      <div class="sectionTitle">Sweep Results</div>
      <div class="small">Left Dip: <span id="leftDip">--</span> MHz</div>
      <div class="small">Right Dip: <span id="rightDip">--</span> MHz</div>
      <div class="small">Sweep #: <span id="sweepNum">--</span></div>
      <div class="small">Actual Points: <span id="pointCount">--</span></div>
      <div class="small">Expected Points: <span id="expectedPoints">--</span></div>

      <div class="sectionTitle">PDB Modulation</div>
      <div class="small">PDB Mode: <span id="pdbModeText">--</span></div>
      <div class="small">PDB Frequency: <span id="pdbFreqText">--</span> Hz</div>

      <div class="sectionTitle">Parked Measurement</div>
      <div class="small">Parked Frequency: <span id="parkFreq">--</span> MHz</div>
      <div class="small">Reference Frequency: <span id="refFreq">--</span> MHz</div>
      <div class="small">Parked Voltage: <span id="parkVolt">--</span> V</div>
      <div class="small">Reference Voltage: <span id="refVolt">--</span> V</div>
      <div class="small">Normalized Signal: <span id="normSig">--</span></div>
      <div class="small">Baseline Normalized Signal: <span id="baseNormSig">--</span></div>
      <div class="small">Slope d(Norm)/df: <span id="slopeNorm">--</span> /MHz</div>
      <div class="small">Calibration Valid: <span id="calValid">--</span></div>

      <div class="mode">
        Mode: <span id="modeText">Idle</span><br>
        Sweep In Progress: <span id="busyText">No</span><br>
        Auto-Resweep: <span id="autoResweepText">--</span>
      </div>

      <div class="btnRow">
        <button id="singleBtn" onclick="runSingleSweep()">Single Sweep</button>
        <button id="contBtn" onclick="toggleContinuousSweep()">Continuous Sweep</button>
        <button id="parkBtn" onclick="toggleParkedMode()">Parked Measurement</button>
      </div>

      <div class="sectionTitle">Sweep Settings</div>
      <div class="settingsGrid">
        <div>
          <label>Center Frequency (MHz)</label>
          <input id="centerFreqInput" type="number" step="0.1">
        </div>
        <div>
          <label>Span (MHz)</label>
          <input id="spanInput" type="number" step="0.1">
        </div>
        <div>
          <label>Step Size (MHz)</label>
          <input id="stepSizeInput" type="number" step="0.1">
        </div>
        <div>
          <label>Sweep Delay (ms)</label>
          <input id="sweepDelayInput" type="number" step="1">
        </div>
        <div>
          <label>Avg Points per Freq</label>
          <input id="numOfPointsInput" type="number" step="1">
        </div>
        <div>
          <label>PDB Mod Frequency (Hz)</label>
          <input id="pdbModFreqInput" type="number" step="1" min="0">
          <div class="hint">Set 0 for steady HIGH. Set 11000 for 11 kHz.</div>
        </div>
      </div>

      <div class="btnRow">
        <button id="saveBtn" onclick="saveSettings()">Save Settings</button>
      </div>
      <div id="settingsMsg">No changes yet.</div>
    </div>

    <div class="card graph">
      <canvas id="plot" width="900" height="420"></canvas>
      <div class="footer">Latest microwave sweep with auto-selected parked/reference frequencies</div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('plot');
    const ctx = canvas.getContext('2d');

    const settingsInputIds = [
      'centerFreqInput',
      'spanInput',
      'stepSizeInput',
      'sweepDelayInput',
      'numOfPointsInput',
      'pdbModFreqInput'
    ];

    let settingsInitialized = false;
    let settingsDirty = false;

    function setupSettingsInputs() {
      settingsInputIds.forEach((id) => {
        const el = document.getElementById(id);
        el.addEventListener('input', () => {
          settingsDirty = true;
        });
      });
    }

    function isEditingSettings() {
      const active = document.activeElement;
      return active && settingsInputIds.includes(active.id);
    }

    function updateSettingsInputs(data, force = false) {
      if (!force) {
        if (!settingsInitialized && !settingsDirty) {
          // first load is allowed
        } else if (settingsDirty || isEditingSettings()) {
          return;
        }
      }

      document.getElementById('centerFreqInput').value = data.centerFreq;
      document.getElementById('spanInput').value = data.span;
      document.getElementById('stepSizeInput').value = data.stepSize;
      document.getElementById('sweepDelayInput').value = data.sweepDelay;
      document.getElementById('numOfPointsInput').value = data.numOfPoints;
      document.getElementById('pdbModFreqInput').value = data.pdbModFreq;

      settingsInitialized = true;
    }

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

    async function toggleParkedMode() {
      try {
        await fetch('/parked', { method: 'POST' });
      } catch (err) {
        console.log('Parked toggle error:', err);
      }
    }

    async function saveSettings() {
      const body =
        "centerFreq=" + encodeURIComponent(document.getElementById('centerFreqInput').value) +
        "&span=" + encodeURIComponent(document.getElementById('spanInput').value) +
        "&stepSize=" + encodeURIComponent(document.getElementById('stepSizeInput').value) +
        "&sweepDelay=" + encodeURIComponent(document.getElementById('sweepDelayInput').value) +
        "&numOfPoints=" + encodeURIComponent(document.getElementById('numOfPointsInput').value) +
        "&pdbModFreq=" + encodeURIComponent(document.getElementById('pdbModFreqInput').value);

      try {
        const response = await fetch('/settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: body
        });

        const text = await response.text();
        document.getElementById('settingsMsg').textContent = text;

        if (response.ok) {
          settingsDirty = false;
          settingsInitialized = false;
          setTimeout(fetchData, 150);
        }
      } catch (err) {
        document.getElementById('settingsMsg').textContent = "Failed to save settings.";
        console.log('Settings save error:', err);
      }
    }

    function drawVerticalMarker(xVal, color, minX, maxX, padL, padR, padT, padB, w, h) {
      if (!isFinite(xVal)) return;
      const plotW = w - padL - padR;
      const x = padL + ((xVal - minX) / (maxX - minX)) * plotW;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, h - padB);
      ctx.stroke();
    }

    function drawPlot(freq, volt, leftDip, rightDip, parkedFreq, refFreq) {
      const w = canvas.width;
      const h = canvas.height;
      const padL = 70;
      const padR = 20;
      const padT = 20;
      const padB = 45;

      ctx.clearRect(0, 0, w, h);
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

      ctx.strokeStyle = "#aaa";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(padL, padT);
      ctx.lineTo(padL, h - padB);
      ctx.lineTo(w - padR, h - padB);
      ctx.stroke();

      ctx.strokeStyle = "#33aaff";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(xMap(freq[0]), yMap(volt[0]));
      for (let i = 1; i < freq.length; i++) {
        ctx.lineTo(xMap(freq[i]), yMap(volt[i]));
      }
      ctx.stroke();

      drawVerticalMarker(leftDip,   "#ff5252", minX, maxX, padL, padR, padT, padB, w, h);
      drawVerticalMarker(rightDip,  "#00e676", minX, maxX, padL, padR, padT, padB, w, h);
      drawVerticalMarker(parkedFreq,"#ffd54f", minX, maxX, padL, padR, padT, padB, w, h);
      drawVerticalMarker(refFreq,   "#ab47bc", minX, maxX, padL, padR, padT, padB, w, h);

      ctx.fillStyle = "#ddd";
      ctx.font = "16px Arial";
      ctx.fillText("Voltage (V)", 10, 22);
      ctx.fillText("Frequency (MHz)", w / 2 - 55, h - 8);

      ctx.fillStyle = "#bbb";
      ctx.font = "13px Arial";
      for (let i = 0; i <= gridLines; i++) {
        let xVal = minX + (i / gridLines) * (maxX - minX);
        let x = padL + (i / gridLines) * plotW;
        ctx.fillText(xVal.toFixed(1), x - 14, h - padB + 20);
      }

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

        document.getElementById('bfielduT').textContent = data.currentField_uT.toFixed(3) + " µT";
        document.getElementById('deltaB').textContent = data.deltaField_uT.toFixed(3);
        document.getElementById('baselineB').textContent = data.baselineField_uT.toFixed(3);

        document.getElementById('leftDip').textContent = data.leftDipFreq.toFixed(3);
        document.getElementById('rightDip').textContent = data.rightDipFreq.toFixed(3);
        document.getElementById('sweepNum').textContent = data.sweepNumber;
        document.getElementById('pointCount').textContent = data.pointCount;
        document.getElementById('expectedPoints').textContent = data.expectedSweepPoints;

        document.getElementById('pdbModeText').textContent = data.pdbModFreq > 0 ? "Square Wave" : "Steady HIGH";
        document.getElementById('pdbFreqText').textContent = data.pdbModFreq.toFixed(0);

        document.getElementById('parkFreq').textContent = data.parkedFreqMHz.toFixed(3);
        document.getElementById('refFreq').textContent = data.refFreqMHz.toFixed(3);
        document.getElementById('parkVolt').textContent = data.currentParkVolt.toFixed(6);
        document.getElementById('refVolt').textContent = data.currentRefVolt.toFixed(6);
        document.getElementById('normSig').textContent = data.currentNormSignal.toFixed(6);
        document.getElementById('baseNormSig').textContent = data.baselineNormSignal.toFixed(6);
        document.getElementById('slopeNorm').textContent = data.parkedSlopeNormPerMHz.toFixed(6);
        document.getElementById('calValid').textContent = data.parkedCalibrationValid ? "Yes" : "No";

        document.getElementById('modeText').textContent = data.mode;
        document.getElementById('busyText').textContent = data.sweepInProgress ? "Yes" : "No";
        document.getElementById('autoResweepText').textContent = data.autoResweepEnabled ? "Enabled" : "Disabled";

        updateSettingsInputs(data);

        drawPlot(
          data.freq,
          data.volt,
          data.leftDipFreq,
          data.rightDipFreq,
          data.parkedFreqMHz,
          data.refFreqMHz
        );

      } catch (err) {
        console.log("Fetch error:", err);
      }
    }

    setupSettingsInputs();
    fetchData();
    setInterval(fetchData, 500);
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

double measureVoltageAtFrequency(double freqMHz) {
  setFrequency(freqMHz);
  delay(parkedSettleDelay);
  return readAveragedVoltage();
}

// ============================================================
// DIP FINDING / FIELD ESTIMATION
// ============================================================
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

  for (int i = 0; i < pointCount; i++) {
    if (freqData[i] < centerFreq) {
      if (leftIndex == -1 || voltData[i] < voltData[leftIndex]) {
        leftIndex = i;
      }
    }
  }

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

// ============================================================
// PARKED MODE CALIBRATION
// ============================================================
void updateParkedCalibrationFromSweep() {
  parkedCalibrationValid = false;

  if (pointCount < 7) return;

  int edgeWindow = min(8, pointCount / 4);
  if (edgeWindow < 3) edgeWindow = 3;

  double leftEdgeAvg  = avgRange(voltData, 0, edgeWindow - 1);
  double rightEdgeAvg = avgRange(voltData, pointCount - edgeWindow, pointCount - 1);

  int refIndex = 0;
  if (rightEdgeAvg >= leftEdgeAvg) {
    refIndex = pointCount - edgeWindow / 2 - 1;
  } else {
    refIndex = edgeWindow / 2;
  }

  int bestSlopeIndex = -1;
  double bestAbsSlope = 0.0;

  for (int i = 2; i < pointCount - 2; i++) {
    double vPrev = smoothedVoltAt(i - 1);
    double vNext = smoothedVoltAt(i + 1);
    double fPrev = freqData[i - 1];
    double fNext = freqData[i + 1];
    double df = fNext - fPrev;
    if (fabs(df) < 1e-9) continue;

    double slope = (vNext - vPrev) / df;
    double absSlope = fabs(slope);

    if (abs(i - refIndex) < 4) continue;

    if (absSlope > bestAbsSlope) {
      bestAbsSlope = absSlope;
      bestSlopeIndex = i;
    }
  }

  if (bestSlopeIndex < 2) return;

  parkedFreqMHz = freqData[bestSlopeIndex];
  refFreqMHz    = freqData[refIndex];

  if (fabs(parkedFreqMHz - refFreqMHz) < 8.0) {
    if (refIndex < pointCount / 2) {
      refIndex = pointCount - 2;
    } else {
      refIndex = 1;
    }
    refFreqMHz = freqData[refIndex];
  }

  double refVoltSweep = smoothedVoltAt(refIndex);
  refVoltSweep = clampNonZero(refVoltSweep);

  double vPrev = smoothedVoltAt(bestSlopeIndex - 1);
  double vNext = smoothedVoltAt(bestSlopeIndex + 1);
  double fPrev = freqData[bestSlopeIndex - 1];
  double fNext = freqData[bestSlopeIndex + 1];
  double df = fNext - fPrev;

  if (fabs(df) < 1e-9) return;

  double slopeRaw = (vNext - vPrev) / df;
  parkedSlopeNormPerMHz = slopeRaw / refVoltSweep;

  if (fabs(parkedSlopeNormPerMHz) < 1e-9) return;

  baselineParkVolt = measureVoltageAtFrequency(parkedFreqMHz);
  baselineRefVolt  = measureVoltageAtFrequency(refFreqMHz);
  baselineRefVolt  = clampNonZero(baselineRefVolt);
  baselineNormSignal = baselineParkVolt / baselineRefVolt;

  currentParkVolt   = baselineParkVolt;
  currentRefVolt    = baselineRefVolt;
  currentNormSignal = baselineNormSignal;

  baselineField_uT = magneticFieldG * 100.0;
  deltaField_uT    = 0.0;
  currentField_uT  = baselineField_uT;

  parkedCalibrationValid = true;
}

void updateParkedMeasurement() {
  if (!parkedCalibrationValid) return;

  currentParkVolt = measureVoltageAtFrequency(parkedFreqMHz);
  currentRefVolt  = measureVoltageAtFrequency(refFreqMHz);
  currentRefVolt  = clampNonZero(currentRefVolt);

  currentNormSignal = currentParkVolt / currentRefVolt;

  double deltaNorm = currentNormSignal - baselineNormSignal;
  double deltaFreqMHz = deltaNorm / parkedSlopeNormPerMHz;

  deltaField_uT = deltaFreqMHz * 35.7142857;
  currentField_uT = baselineField_uT + deltaField_uT;
}

// ============================================================
// SWEEP
// ============================================================
void performSweep() {
  sweepInProgress = true;

  int expectedPoints = getExpectedSweepPoints();
  if (expectedPoints > MAX_POINTS) {
    Serial.println("ERROR: Sweep settings exceed MAX_POINTS. Sweep aborted.");
    sweepInProgress = false;
    return;
  }

  double startFreq = centerFreq - (span / 2.0);
  double endFreq   = centerFreq + (span / 2.0);

  pointCount = 0;

  for (double f = startFreq; f <= endFreq + 1e-9; f += stepSize) {
    if (pointCount >= MAX_POINTS) break;

    setFrequency(f);
    delay(sweepDelay);

    double avgVolt = readAveragedVoltage();

    freqData[pointCount] = f;
    voltData[pointCount] = avgVolt;
    pointCount++;

    server.handleClient();
    delay(1);
  }

  setFrequency(centerFreq);
  findDipsAndField();
  updateParkedCalibrationFromSweep();

  sweepNumber++;
  lastResweepMs = millis();
  sweepInProgress = false;
}

// ============================================================
// WEB SERVER HANDLERS
// ============================================================
String getModeString() {
  if (sweepInProgress) return "Sweeping";
  if (continuousSweepEnabled) return "Continuous Sweep";
  if (parkedModeEnabled) return "Parked Measurement";
  if (requestSingleSweep) return "Single Sweep Requested";
  if (requestParkedCal) return "Parked Calibration Requested";
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
  json += "\"expectedSweepPoints\":" + String(getExpectedSweepPoints()) + ",";
  json += "\"sweepInProgress\":" + String(sweepInProgress ? "true" : "false") + ",";
  json += "\"mode\":\"" + getModeString() + "\",";
  json += "\"parkedModeEnabled\":" + String(parkedModeEnabled ? "true" : "false") + ",";
  json += "\"parkedCalibrationValid\":" + String(parkedCalibrationValid ? "true" : "false") + ",";
  json += "\"autoResweepEnabled\":" + String(autoResweepEnabled ? "true" : "false") + ",";

  json += "\"centerFreq\":" + String(centerFreq, 3) + ",";
  json += "\"span\":" + String(span, 3) + ",";
  json += "\"stepSize\":" + String(stepSize, 3) + ",";
  json += "\"sweepDelay\":" + String(sweepDelay) + ",";
  json += "\"numOfPoints\":" + String(numOfPoints) + ",";
  json += "\"pdbModFreq\":" + String(pdbModFreq) + ",";

  json += "\"parkedFreqMHz\":" + String(parkedFreqMHz, 6) + ",";
  json += "\"refFreqMHz\":" + String(refFreqMHz, 6) + ",";
  json += "\"parkedSlopeNormPerMHz\":" + String(parkedSlopeNormPerMHz, 9) + ",";
  json += "\"baselineNormSignal\":" + String(baselineNormSignal, 9) + ",";
  json += "\"baselineParkVolt\":" + String(baselineParkVolt, 6) + ",";
  json += "\"baselineRefVolt\":" + String(baselineRefVolt, 6) + ",";
  json += "\"baselineField_uT\":" + String(baselineField_uT, 6) + ",";
  json += "\"currentParkVolt\":" + String(currentParkVolt, 6) + ",";
  json += "\"currentRefVolt\":" + String(currentRefVolt, 6) + ",";
  json += "\"currentNormSignal\":" + String(currentNormSignal, 9) + ",";
  json += "\"deltaField_uT\":" + String(deltaField_uT, 6) + ",";
  json += "\"currentField_uT\":" + String(currentField_uT, 6) + ",";

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
  parkedModeEnabled = false;
  requestSingleSweep = true;
  requestParkedCal = false;
  server.send(200, "text/plain", "Single sweep requested");
}

void handleContinuousSweep() {
  parkedModeEnabled = false;
  requestParkedCal = false;
  continuousSweepEnabled = !continuousSweepEnabled;

  if (continuousSweepEnabled) {
    requestSingleSweep = false;
    server.send(200, "text/plain", "Continuous sweep enabled");
  } else {
    server.send(200, "text/plain", "Continuous sweep disabled");
  }
}

void handleParkedMode() {
  continuousSweepEnabled = false;
  requestSingleSweep = false;

  parkedModeEnabled = !parkedModeEnabled;

  if (parkedModeEnabled) {
    requestParkedCal = true;
    server.send(200, "text/plain", "Parked measurement enabled");
  } else {
    requestParkedCal = false;
    server.send(200, "text/plain", "Parked measurement disabled");
  }
}

void handleSettings() {
  if (!server.hasArg("centerFreq") || !server.hasArg("span") || !server.hasArg("stepSize") ||
      !server.hasArg("sweepDelay") || !server.hasArg("numOfPoints") || !server.hasArg("pdbModFreq")) {
    server.send(400, "text/plain", "Missing settings arguments.");
    return;
  }

  double newCenterFreq = server.arg("centerFreq").toDouble();
  double newSpan       = server.arg("span").toDouble();
  double newStepSize   = server.arg("stepSize").toDouble();
  int newSweepDelay    = server.arg("sweepDelay").toInt();
  int newNumOfPoints   = server.arg("numOfPoints").toInt();
  long newPdbModFreq   = server.arg("pdbModFreq").toInt();

  if (newCenterFreq <= 0.0 || newSpan <= 0.0 || newStepSize <= 0.0 || newSweepDelay < 0 || newNumOfPoints <= 0) {
    server.send(400, "text/plain", "Invalid sweep settings. Stop trying to break physics.");
    return;
  }

  if (newPdbModFreq < 0) {
    server.send(400, "text/plain", "PDB modulation frequency cannot be negative. Nice try.");
    return;
  }

  int newExpectedPoints = (int)(newSpan / newStepSize) + 1;
  if (newExpectedPoints > MAX_POINTS) {
    String msg = "Too many sweep points (" + String(newExpectedPoints) + "). "
                 "Increase step size or reduce span. MAX_POINTS = " + String(MAX_POINTS);
    server.send(400, "text/plain", msg);
    return;
  }

  centerFreq  = newCenterFreq;
  span        = newSpan;
  stepSize    = newStepSize;
  sweepDelay  = newSweepDelay;
  numOfPoints = newNumOfPoints;
  pdbModFreq  = (uint32_t)newPdbModFreq;

  applyPdbMode();

  continuousSweepEnabled = false;
  parkedModeEnabled = false;
  requestSingleSweep = true;
  requestParkedCal = false;

  String pdbMsg = (pdbModFreq == 0)
    ? "PDB steady HIGH"
    : ("PDB modulating at " + String(pdbModFreq) + " Hz");

  String msg = "Settings saved. Expected sweep points = " + String(newExpectedPoints) +
               ". " + pdbMsg + ". Running new sweep.";
  server.send(200, "text/plain", msg);
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

  digitalWrite(LE_PIN, HIGH);
  digitalWrite(CE_PIN, HIGH);

  applyPdbMode();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

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
  Serial.println("PDB modulation mode: ");
  if (pdbModFreq == 0) {
    Serial.println("Steady HIGH");
  } else {
    Serial.print("Square wave at ");
    Serial.print(pdbModFreq);
    Serial.println(" Hz");
  }
  Serial.println("==================================");

  server.on("/", handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/single", HTTP_POST, handleSingleSweep);
  server.on("/continuous", HTTP_POST, handleContinuousSweep);
  server.on("/parked", HTTP_POST, handleParkedMode);
  server.on("/settings", HTTP_POST, handleSettings);
  server.begin();

  performSweep();
  lastParkedUpdateMs = millis();
}

void loop() {
  server.handleClient();

  if (!sweepInProgress) {
    if (requestSingleSweep) {
      requestSingleSweep = false;
      performSweep();
    }
    else if (requestParkedCal) {
      requestParkedCal = false;
      performSweep();
    }
    else if (continuousSweepEnabled) {
      performSweep();
    }
    else if (parkedModeEnabled) {
      unsigned long now = millis();

      if (autoResweepEnabled && (now - lastResweepMs >= autoResweepIntervalMs)) {
        performSweep();
      }
      else if (now - lastParkedUpdateMs >= parkedUpdateIntervalMs) {
        updateParkedMeasurement();
        lastParkedUpdateMs = now;
      }
    }
  }

  delay(2);
}

// ================= PIN DEFINITIONS =================
#define PDB       4
#define LE_PIN    10
#define CLK_PIN   2
#define DATA_PIN  3
#define CE_PIN    18

#define ADC_PIN   0

// ================= USER SETTINGS =================
double centerFreq = 2870.0;   // Center frequency in MHz
double span       = 70.0;    // Total sweep span in MHz
double stepSize   = 0.8;      // Sweep step size in MHz
int sweepDelay    = 10;       // Delay between sweep points in ms

// ================= LOCK-IN SETTINGS =================
// This code does two-point frequency dither:
// high = f + A
// low  = f - A
//
// A = modulation depth in MHz
// fm = effective modulation frequency in Hz
//
// Since this is a two-state hop system, we approximate the modulation
// timing using the half-period of fm.

double fmDevMHz      = 11.0;   // A = modulation depth in MHz
double fmHz          = 640.0;  // modulation frequency in Hz
int settleTimeUs     = 781;    // half-period of 640 Hz ≈ 781 us
int lockinAverages   = 100;    // number of lock-in cycles to average
int adcSamples       = 32;     // ADC samples per point

// ================= PLL CONSTANTS =================
// Output frequency = (fPFD × (INT + FRAC/MOD)) / divider
double fPFD = 25.0;
unsigned long MOD = 1000;

#define DELAY 1

#define REGISTER_5 0x00580005U
#define REGISTER_3 0x008004B3U
#define REGISTER_2 0x18006E42U
#define REG4_BASE  0x008C803CU
#define REG4_MASK  0xFF8FFFFFU

uint8_t currentRfDiv = 0;

// Function to write a 32-bit register to the PLL
void write_register(unsigned long value)
{
  digitalWrite(LE_PIN, LOW);
  delayMicroseconds(DELAY);

  for (int i = 31; i >= 0; i--) {
    digitalWrite(DATA_PIN, (value >> i) & 1);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(DELAY);
    digitalWrite(CLK_PIN, LOW);
  }

  digitalWrite(LE_PIN, HIGH);
  delayMicroseconds(DELAY);
}

// Function to determine RF divider settings
void getDividerInfo(double freqMHz, uint8_t &rfDiv, uint32_t &divider)
{
  if (freqMHz >= 2200.0)      { rfDiv = 0; divider = 1; }
  else if (freqMHz >= 1100.0) { rfDiv = 1; divider = 2; }
  else if (freqMHz >= 550.0)  { rfDiv = 2; divider = 4; }
  else if (freqMHz >= 275.0)  { rfDiv = 3; divider = 8; }
  else if (freqMHz >= 137.5)  { rfDiv = 4; divider = 16; }
  else if (freqMHz >= 68.75)  { rfDiv = 5; divider = 32; }
  else                        { rfDiv = 6; divider = 64; }
}

// Function to fully program PLL registers
void programFrequencyFull(double freqMHz)
{
  uint8_t rfDiv;
  uint32_t divider;
  getDividerInfo(freqMHz, rfDiv, divider);

  double vcoFreq = freqMHz * divider;
  double N = vcoFreq / fPFD;

  unsigned long INT  = (unsigned long)N;
  unsigned long FRAC = (unsigned long)((N - INT) * MOD + 0.5);

  if (FRAC >= MOD) {
    FRAC = 0;
    INT++;
  }

  currentRfDiv = rfDiv;

  unsigned long reg4 = (REG4_BASE & REG4_MASK) | ((unsigned long)rfDiv << 20);
  unsigned long reg1 = (MOD << 3) | 0x01;
  unsigned long reg0 = (INT << 15) | (FRAC << 3);

  write_register(REGISTER_5);
  write_register(reg4);
  write_register(REGISTER_3);
  write_register(REGISTER_2);
  write_register(reg1);
  write_register(reg0);
}

// Faster frequency update using only register 0 when possible
void setFrequencyFast(double freqMHz)
{
  uint8_t rfDiv;
  uint32_t divider;
  getDividerInfo(freqMHz, rfDiv, divider);

  if (rfDiv != currentRfDiv) {
    programFrequencyFull(freqMHz);
    return;
  }

  double vcoFreq = freqMHz * divider;
  double N = vcoFreq / fPFD;

  unsigned long INT  = (unsigned long)N;
  unsigned long FRAC = (unsigned long)((N - INT) * MOD + 0.5);

  if (FRAC >= MOD) {
    FRAC = 0;
    INT++;
  }

  unsigned long reg0 = (INT << 15) | (FRAC << 3);
  write_register(reg0);
}

// Function to read averaged ADC voltage
double readAveragedVoltage(int samples)
{
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(ADC_PIN);
  }

  double adc = (double)sum / samples;
  return (adc / 4095.0) * 3.3;
}

// Two-point lock-in style measurement
double measureLockin(double f)
{
  double accumulator = 0;

  for (int i = 0; i < lockinAverages; i++)
  {
    // High side: f + A
    setFrequencyFast(f + fmDevMHz);
    delayMicroseconds(settleTimeUs);
    double high = readAveragedVoltage(adcSamples);

    // Low side: f - A
    setFrequencyFast(f - fmDevMHz);
    delayMicroseconds(settleTimeUs);
    double low = readAveragedVoltage(adcSamples);

    // Difference gives derivative-like signal
    accumulator += (high - low);
  }

  return accumulator / lockinAverages;
}

void setup()
{
  Serial.begin(115200);

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

  delay(1000);

  // Program initial frequency
  programFrequencyFull(centerFreq);

  // CSV header for MATLAB
  Serial.println("Frequency_MHz,Voltage_V");

  double startFreq = centerFreq - span / 2.0;
  double endFreq   = centerFreq + span / 2.0;

  for (double f = startFreq; f <= endFreq; f += stepSize)
  {
    setFrequencyFast(f);
    delay(sweepDelay);

    double lockin = measureLockin(f);

    Serial.print(f, 4);
    Serial.print(",");
    Serial.println(lockin, 6);
  }

  // Return to center frequency
  setFrequencyFast(centerFreq);
}

void loop() {}



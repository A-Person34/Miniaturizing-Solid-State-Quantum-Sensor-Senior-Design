// ================= PIN DEFINITIONS =================
#define PDB       4
#define LE_PIN    10
#define CLK_PIN   2
#define DATA_PIN  3
#define CE_PIN    18

#define ADC_PIN   0   // USING GPIO0

// ================= USER SETTINGS =================
double centerFreq = 2870.0;
double span       = 300.0;
double stepSize   = 0.5;
int sweepDelay    = 500;

// ================= PLL CONSTANTS =================
double fPFD = 25.0;
unsigned long MOD = 1000;

#define DELAY 1

#define REGISTER_5 0x00580005U
#define REGISTER_3 0x008004B3U
#define REGISTER_2 0x18006E42U
#define REG4_BASE  0x008C803CU
#define REG4_MASK  0xFF8FFFFFU

// =================================================

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

void setup() {

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

  Serial.println("Frequency_MHz,Voltage_V");

  double startFreq = centerFreq - (span / 2.0);
  double endFreq   = centerFreq + (span / 2.0);

  for (double f = startFreq; f <= endFreq; f += stepSize) {

    setFrequency(f);

    int sum = 0;
    for (int i = 0; i < 32; i++) {
      sum += analogRead(ADC_PIN);
    }

    int adcValue = sum / 32;
    double voltage = (adcValue / 4095.0) * 3.3;

    Serial.print(f);
    Serial.print(",");
    Serial.println(voltage, 6);

    delay(sweepDelay);
  }

  // Return to center frequency
  setFrequency(centerFreq);
}

void loop() {
  // Nothing here
}
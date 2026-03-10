// ================= PIN DEFINITIONS =================
#define PDB       4
#define LE_PIN    10
#define CLK_PIN   2
#define DATA_PIN  3
#define CE_PIN    18

#define ADC_PIN   0

// ================= USER SETTINGS =================
double centerFreq = 2870.0;   // Center frequency for where the middle should be in MHz
double span       = 200.0;    // How far we will sweep in total (span/2 on each side of the center frequency)
double stepSize   = 1;        // How far we step each time in MHz
int sweepDelay    = 10;

// ================= LOCK-IN SETTINGS =================
// Equation is sin[2π(f0 + Asin(fmt))t]
// f0 is the carrier frequency
// A is the amplitude of the modulated signal
// fm is the modulation frequency
// settleTimeUs is the modulation freqeuncy. We find it using this for loop as an example
/*
    for(double fm = 100; fm <= 1000; fm += 10)
    {
        settleTimeUs = 500.0 / fm;
    }
*/
double fmDevMHz      = 26.8;  // frequency hop (A in above equation)
int settleTimeUs     = 3000;  // PLL settling time
int lockinAverages   = 100;   // number of lock-in cycles
int adcSamples       = 32;

// ================= PLL CONSTANTS =================
// Output frequency = (fPFD × (INT + FRAC/MOD)) / divider 
double fPFD = 25.0;         // fPFD = phase detector frequency
unsigned long MOD = 1000;

#define DELAY 1

#define REGISTER_5 0x00580005U
#define REGISTER_3 0x008004B3U
#define REGISTER_2 0x18006E42U
#define REG4_BASE  0x008C803CU
#define REG4_MASK  0xFF8FFFFFU

uint8_t currentRfDiv = 0;

// Function to get the right values in the registers for each step
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

// Function to get the dividers for the registers to get proper output signals
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

// Function to calculate the PLL parameters 
void programFrequencyFull(double freqMHz)
{
  uint8_t rfDiv;
  uint32_t divider;
  getDividerInfo(freqMHz, rfDiv, divider);

  // Converts the output frequency to VCO frequency
  double vcoFreq = freqMHz * divider;

  // Calculates the divider frequency 
  double N = vcoFreq / fPFD;

  // Splits into integer and fractinoal parts
  unsigned long INT  = (unsigned long)N;
  unsigned long FRAC = (unsigned long)((N - INT) * MOD + 0.5);

  if (FRAC >= MOD) {
    FRAC = 0;
    INT++;
  }

  currentRfDiv = rfDiv;

  // Builds the register values
  unsigned long reg4 = (REG4_BASE & REG4_MASK) | ((unsigned long)rfDiv << 20);
  unsigned long reg1 = (MOD << 3) | 0x01;
  unsigned long reg0 = (INT << 15) | (FRAC << 3);

  // Writes our register values to the right registers
  write_register(REGISTER_5);
  write_register(reg4);
  write_register(REGISTER_3);
  write_register(REGISTER_2);
  write_register(reg1);
  write_register(reg0);
}

// Function to only update register 0 when sweeping to make it faster
// Similar function to the previous one, but only updates register 0 for quicker sweeping
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

// Function to read our voltage properly
double readAveragedVoltage(int samples)
{
  long sum = 0;

  // Read value from ADC pin and sum them
  for (int i = 0; i < samples; i++)
    sum += analogRead(ADC_PIN);

  // Averaging for the values read in and put into voltage scaled for 3.3V
  double adc = (double)sum / samples;
  return (adc / 4095.0) * 3.3;
}

// Function for doing Lock-In
double measureLockin(double f)
{
  double accumulator = 0;

  for(int i = 0; i < lockinAverages; i++)
  {
    // First, measure at high frequency
    setFrequencyFast(f + fmDevMHz);
    delayMicroseconds(settleTimeUs);
    double high = readAveragedVoltage(adcSamples);

    // Next, measure at low frequency 
    setFrequencyFast(f - fmDevMHz);
    delayMicroseconds(settleTimeUs);
    double low = readAveragedVoltage(adcSamples);

    // Next, take the difference between them and take a bunch of samples to average 
    // This will give the derivative plot
    accumulator += (high - low);
  }

  return accumulator / lockinAverages;
}

void setup()
{
  // For MATLAB communication
  Serial.begin(115200);

  // Configuring all PLL pins
  pinMode(PDB, OUTPUT);
  pinMode(LE_PIN, OUTPUT);
  pinMode(CE_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  // Enabling the pins
  digitalWrite(PDB, HIGH);
  digitalWrite(LE_PIN, HIGH);
  digitalWrite(CE_PIN, HIGH);

  // Configuring the ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  delay(1000);

  // Sets the initial frequncy
  programFrequencyFull(centerFreq);

  // Sets the csv header
  Serial.println("Frequency_MHz,Voltage_V");

  // Gets our starting and ending frequencies
  double startFreq = centerFreq - span/2;
  double endFreq   = centerFreq + span/2;

  // Loop for the sweep
  for(double f = startFreq; f <= endFreq; f += stepSize)
  {
    // Set the frequency at each step and delay
    setFrequencyFast(f);
    delay(sweepDelay);

    // Measure the lock-in value at that point 
    double lockin = measureLockin(f);

    // Send the data to read in MATLAB
    Serial.print(f,4);
    Serial.print(",");
    Serial.println(lockin,6);
  }

  // Return to center frequency when done
  setFrequencyFast(centerFreq);
}

void loop(){} 

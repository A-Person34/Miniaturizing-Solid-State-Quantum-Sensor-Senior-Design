// Assume long integers are at least 32 bits.
// Park clock and data low when not in use.

// begin customizing here
#define PDB 4
#define LE_PIN 10
#define CLK_PIN 2
#define DATA_PIN 3
#define CE_PIN 18

// Register 5 contains the LD pin mode
#define REGISTER_5 0x00580005U

// Register 4 is output divider
#define REGISTER_4 0x00B64274U

// Register 3 has a weird clock divider
#define REGISTER_3 0x00000003U

// Register 2 is the R bit counter
// Divides down input frequency (cystal oscillator)
#define REGISTER_2 0x18005E42U

// Register 1 contains the MOD for the calcultion below
#define REGISTER_1 0x00008011U

// Register 0 is the INT value
// INT value = (desired output frequency)/(crystal ocsillator frequecny)
// INT value also depends on FRAC and MOD but for now we are not worrying about it
// RFout = [INT + (FRAC/MOD)] (fpfd/RF Divider)
// Rgister 0 also contains FRAC to change through for other frequencies
#define REGISTER_0 0x000B8000U

// end customizing here

#define DELAY 1

void setup_ports() {

   pinMode(PDB, OUTPUT);
    digitalWrite(PDB, HIGH);
    
  pinMode(LE_PIN, OUTPUT);
  digitalWrite(LE_PIN, HIGH);
  
 pinMode(CE_PIN, OUTPUT);
 digitalWrite(CE_PIN, HIGH);
 
  pinMode(CLK_PIN, OUTPUT);
  
  pinMode(DATA_PIN, OUTPUT);

  digitalWrite(CLK_PIN, LOW);
  digitalWrite(DATA_PIN, LOW);
  Serial.begin(9600);
}

void write_register(unsigned long value) {
  digitalWrite(LE_PIN, LOW);
  delayMicroseconds(DELAY);
  for (int i=31; i >= 0; i--) {
    if ((value >> i) & 1)
      digitalWrite(DATA_PIN, HIGH);
    else
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(DELAY);
    
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(DELAY);
    
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(DELAY);
  }
  digitalWrite(DATA_PIN, LOW);
  digitalWrite(LE_PIN, HIGH);
  delayMicroseconds(DELAY);
  Serial.print(DATA_PIN);
}

void setup() {
  setup_ports();
  delay(1000);
  write_register(REGISTER_5);
  write_register(REGISTER_4);
  write_register(REGISTER_3);
  write_register(REGISTER_2);
  write_register(REGISTER_1);
  write_register(REGISTER_0);
}

void loop() {
}

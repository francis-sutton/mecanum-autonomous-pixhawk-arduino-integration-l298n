// ===== Phase 2: Pixhawk mixing → Arduino → L298N (Mecanum) =====
// Pixhawk outputs wheel commands on OUT1..OUT4 (1500=stop, 1000..2000us).
// Arduino reads 4 channels (3 via interrupts, 1 via pulseIn) and drives L298Ns.

// ---------------- Includes & tuning ----------------
#include <Arduino.h>
//#include <math.h>

// Deadband around neutral
const int  DEADZONE_US_LOW  = 1450;
const int  DEADZONE_US_HIGH = 1550;

// Overall output cap
const float MAX_SCALE = 1.00f;   // 0..1

// Per-wheel inversion (keep your left pair inverted as before; tweak if needed)
#define INV_LF 1
#define INV_RF 0
#define INV_LR 1
#define INV_RR 0

// Failsafe & soft start
const unsigned long STALE_US      = 100000UL; // >100ms since last pulse -> stale
const uint8_t       RAMP_STEP_PWM = 6;        // smaller = gentler

// ---------------- Pixhawk → Arduino input pin map ----------------
// OUT1 (LF) -> D0, OUT2 (RF) -> D1, OUT3 (LR) -> D7, OUT4 (RR) -> D8
const uint8_t PIN_LF = 0;  // D0 (interrupt capable)
const uint8_t PIN_RF = 1;  // D1 (interrupt capable)
const uint8_t PIN_LR = 7;  // D7 (interrupt capable)
const uint8_t PIN_RR = 8;  // D8 (polled)

// ---------------- L298N pin map (unchanged from Phase 1) ----------------
// Left Front
const uint8_t LF_IN1 = 2,  LF_IN2 = 4,  LF_EN = 5;    // PWM
// Left Rear  (LR_IN2 moved earlier to D11)
const uint8_t LR_IN1 = 6,  LR_IN2 = 11, LR_EN = 3;    // PWM
// Right Front
const uint8_t RF_IN1 = 12, RF_IN2 = 13, RF_EN = 9;    // PWM
// Right Rear
const uint8_t RR_IN1 = A0, RR_IN2 = A1, RR_EN = 10;   // PWM

// ---------------- Motor struct & array ----------------
struct Motor {
  uint8_t in1, in2, en, invert;
  int curr; // current signed PWM for soft-start
};
Motor M[4] = {
  {LF_IN1, LF_IN2, LF_EN, INV_LF, 0}, // 0 LF
  {RF_IN1, RF_IN2, RF_EN, INV_RF, 0}, // 1 RF
  {LR_IN1, LR_IN2, LR_EN, INV_LR, 0}, // 2 LR
  {RR_IN1, RR_IN2, RR_EN, INV_RR, 0}  // 3 RR
};

// ---------------- RC capture (D0, D1, D7 via interrupts) ----------------
volatile unsigned long lfRise=0, lfWidth=1500, lfStamp=0;
volatile unsigned long rfRise=0, rfWidth=1500, rfStamp=0;
volatile unsigned long lrRise=0, lrWidth=1500, lrStamp=0;

void isrLF(){ // D0
  if (digitalRead(PIN_LF)) lfRise = micros();
  else { unsigned long now=micros(); lfWidth = now - lfRise; lfStamp = now; }
}
void isrRF(){ // D1
  if (digitalRead(PIN_RF)) rfRise = micros();
  else { unsigned long now=micros(); rfWidth = now - rfRise; rfStamp = now; }
}
void isrLR(){ // D7
  if (digitalRead(PIN_LR)) lrRise = micros();
  else { unsigned long now=micros(); lrWidth = now - lrRise; lrStamp = now; }
}

void rcInputBegin(){
  pinMode(PIN_LF, INPUT);
  pinMode(PIN_RF, INPUT);
  pinMode(PIN_LR, INPUT);
  pinMode(PIN_RR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_LF), isrLF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RF), isrRF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LR), isrLR, CHANGE);
}

// Read latest width for interrupt-backed channels
unsigned long rcReadUs_int(char which){
  noInterrupts();
  unsigned long w=1500, t=0;
  switch (which){
    case 'F': w=lfWidth; t=lfStamp; break; // LF
    case 'R': w=rfWidth; t=rfStamp; break; // RF
    case 'L': w=lrWidth; t=lrStamp; break; // LR
  }
  interrupts();
  if ((long)(micros() - t) > (long)STALE_US) return 0; // stale
  if (w < 800)  w = 800;
  if (w > 2200) w = 2200;
  return w;
}

// ---------------- Helpers ----------------
static inline bool inDeadzone(unsigned long us){
  return (us > (unsigned long)DEADZONE_US_LOW && us < (unsigned long)DEADZONE_US_HIGH);
}
static inline float usToNorm(unsigned long us){
  if (us == 0) return 0.0f;             // stale -> stop
  if (inDeadzone(us)) return 0.0f;      // deadband
  if (us < 1000UL) us = 1000UL;
  if (us > 2000UL) us = 2000UL;
  long span = (long)us - 1500L;         // -500..+500
  float v = (float)span / 500.0f;       // -1..+1
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}
static inline int normToPWM(float v){
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return (int)roundf(v * 255.0f); // signed
}
static inline int rampTo(int current, int target){
  if (current < target){ current += RAMP_STEP_PWM; if (current > target) current = target; }
  else if (current > target){ current -= RAMP_STEP_PWM; if (current < target) current = target; }
  return current;
}
static inline void driveMotor(Motor &m, int target){
  if (m.invert) target = -target;
  m.curr = rampTo(m.curr, target);
  if (m.curr > 0){
    digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);  analogWrite(m.en, m.curr);
  } else if (m.curr < 0){
    digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); analogWrite(m.en, -m.curr);
  } else {
    digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW);  analogWrite(m.en, 0);
  }
}
static inline void stopAll(){
  for (int i=0;i<4;i++) M[i].curr=0;
  digitalWrite(LF_IN1,LOW); digitalWrite(LF_IN2,LOW); analogWrite(LF_EN,0);
  digitalWrite(RF_IN1,LOW); digitalWrite(RF_IN2,LOW); analogWrite(RF_EN,0);
  digitalWrite(LR_IN1,LOW); digitalWrite(LR_IN2,LOW); analogWrite(LR_EN,0);
  digitalWrite(RR_IN1,LOW); digitalWrite(RR_IN2,LOW); analogWrite(RR_EN,0);
}

// ---------------- Setup ----------------
void setup(){
  Serial.begin(115200);
  rcInputBegin();

  pinMode(LF_IN1,OUTPUT); pinMode(LF_IN2,OUTPUT); pinMode(LF_EN,OUTPUT);
  pinMode(LR_IN1,OUTPUT); pinMode(LR_IN2,OUTPUT); pinMode(LR_EN,OUTPUT);
  pinMode(RF_IN1,OUTPUT); pinMode(RF_IN2,OUTPUT); pinMode(RF_EN,OUTPUT);
  pinMode(RR_IN1,OUTPUT); pinMode(RR_IN2,OUTPUT); pinMode(RR_EN,OUTPUT);

  stopAll();
}

// ---------------- Loop ----------------
void loop(){
  // Read 3 channels via interrupts
  unsigned long usLF = rcReadUs_int('F'); // D0
  unsigned long usRF = rcReadUs_int('R'); // D1
  unsigned long usLR = rcReadUs_int('L'); // D7

  // Read RR (D8) with a full-frame timeout and hold-last-good
static unsigned long rr_last = 1500;
unsigned long usRR = pulseIn(PIN_RR, HIGH, 25000UL); // up to 25ms (one frame)
if (usRR >= 800 && usRR <= 2200) {
  rr_last = usRR;
  usRR = rr_last;
} else {
  // if we miss a pulse this loop, keep last good so RR doesn’t glitch to 0
  usRR = rr_last;
}

  // Failsafe: if ALL stale, stop and wait
  if (usLF==0 && usRF==0 && usLR==0 && usRR==0){
    stopAll();
    delay(15);
    return;
  }

  // Map to -1..+1 and clamp with overall scale
  float lf = usToNorm(usLF) * MAX_SCALE;
  float rf = usToNorm(usRF) * MAX_SCALE;
  float lr = usToNorm(usLR) * MAX_SCALE;
  float rr = usToNorm(usRR) * MAX_SCALE;

auto snap = [](float v){ return (fabs(v) < 0.15f) ? 0.0f : v; }; // 8% deadzone
lf = snap(lf); rf = snap(rf); lr = snap(lr); rr = snap(rr);

  // Drive each wheel
  driveMotor(M[0], normToPWM(lf)); // LF
  driveMotor(M[1], normToPWM(rf)); // RF
  driveMotor(M[2], normToPWM(lr)); // LR
  driveMotor(M[3], normToPWM(rr)); // RR

  // Debug (optional)
  Serial.print("LF:"); Serial.print(usLF);
  Serial.print(" RF:"); Serial.print(usRF);
  Serial.print(" LR:"); Serial.print(usLR);
  Serial.print(" RR:"); Serial.print(usRR);
  Serial.print(" | lf:"); Serial.print(lf,2);
  Serial.print(" rf:"); Serial.print(rf,2);
  Serial.print(" lr:"); Serial.print(lr,2);
  Serial.print(" rr:"); Serial.println(rr,2);

  delay(10); // ~100 Hz loop
}

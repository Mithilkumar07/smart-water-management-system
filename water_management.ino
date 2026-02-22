#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <arduinoFFT.h>

// ---------------- LCD & Bluetooth ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial BT(8, 7);   // RX, TX

// ---------------- Pins ----------------
const int trig = 9;        // Tank ultrasonic
const int echo = 10;
const int irLeakPin = 12;  // IR leak sensor digital output
const int redLED = 5;
const int buzzer = 4;
const int flowPin = 2;     // Flow sensor (interrupt)

// ---------------- Tank Calibration ----------------
float emptyDistance = 9.52;  // cm when tank empty
float fullDistance  = 2.47;  // cm when tank full

// ---------------- Flow Variables ----------------
volatile unsigned long pulseCount = 0;
unsigned long lastFlowTime = 0;
float flowRate = 0;   // L/min

// ---------------- FFT Variables ----------------
#define SAMPLES 64
#define SAMPLING_INTERVAL 200 // ms
ArduinoFFT<double> FFT;

double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long lastSampleTime = 0;
int sampleIndex = 0;

// ---------------- Flow Limits ----------------
float minFlow = 1.0;  
unsigned long lowFlowStartTime = 0;
bool lowFlowLeakActive = false;

// ---------------- Leak Detection ----------------
bool leakDetected = false;

// ---------------- Interrupt ----------------
void pulseCounter() {
  pulseCount++;
}

// ---------------- Setup ----------------
void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(irLeakPin, INPUT);  // IR sensor input
  pinMode(redLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(flowPin, INPUT);

  digitalWrite(redLED, LOW);
  digitalWrite(buzzer, LOW);

  attachInterrupt(digitalPinToInterrupt(flowPin), pulseCounter, RISING);

  lcd.init();
  lcd.backlight();
  BT.begin(9600);

  Serial.begin(9600);
}

// ---------------- Measure Distance ----------------
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2;
}

// ---------------- Loop ----------------
void loop() {

  // -------- Ultrasonic Tank Level --------
  float tankDistance = measureDistance(trig, echo);
  float tankPercentage = 0;
  if (tankDistance > 0) {
    tankPercentage = (emptyDistance - tankDistance) / (emptyDistance - fullDistance) * 100.0;
    tankPercentage = constrain(tankPercentage, 0, 100);
  }

  // -------- Flow Calculation (1 sec) --------
  if (millis() - lastFlowTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(flowPin));
    flowRate = pulseCount / 7.5;  // YF-S201
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(flowPin), pulseCounter, RISING);
    lastFlowTime = millis();

    // Low flow timer
    if (flowRate < minFlow && flowRate > 0) {
      if (lowFlowStartTime == 0) lowFlowStartTime = millis();
      else if (millis() - lowFlowStartTime >= 5000) lowFlowLeakActive = true;
    } else {
      lowFlowStartTime = 0;
      lowFlowLeakActive = false;
    }
  }

  // -------- FFT Sampling --------
  if (millis() - lastSampleTime >= SAMPLING_INTERVAL) {
    vReal[sampleIndex] = flowRate;
    vImag[sampleIndex] = 0;
    sampleIndex++;
    lastSampleTime = millis();

    if (sampleIndex >= SAMPLES) {
      analyzeFFT();
      sampleIndex = 0;
    }
  }

  // -------- Leak Detection (IR Sensor) --------
  // Assume: LOW = leak detected, HIGH = no leak (typical for IR water sensor)
  leakDetected = (digitalRead(irLeakPin) == LOW);

  // -------- Alert Logic --------
  bool anyAlert = leakDetected || lowFlowLeakActive || (tankPercentage >= 100);
  digitalWrite(redLED, anyAlert ? HIGH : LOW);

  // -------- Buzzer & Bluetooth --------
  if (leakDetected) {
    digitalWrite(buzzer, HIGH);
    BT.println("IR LEAK DETECTED!");
  } else if (lowFlowLeakActive) {
    digitalWrite(buzzer, HIGH);
    BT.println("LOW FLOW LEAK DETECTED!");
  } else if (tankPercentage >= 100) {
    digitalWrite(buzzer, HIGH);
    BT.println("TANK FULL ALERT!");
  } else {
    digitalWrite(buzzer, LOW);
  }

  // -------- LCD Display --------
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Water Left:");
  lcd.setCursor(0,1);
  lcd.print(tankPercentage,0);
  lcd.print("%");

  delay(500);
}

// ---------------- FFT Analysis ----------------
void analyzeFFT() {

  // ---- Remove DC Offset ----
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
  mean /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;

  // ---- Windowing & FFT ----
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // ---- DC Ratio ----
  double dcEnergy = vReal[0];
  double totalEnergy = 0;
  for (int i = 0; i < SAMPLES/2; i++) totalEnergy += vReal[i];
  double dcRatio = 0;
  if (totalEnergy != 0) dcRatio = dcEnergy / totalEnergy;

  Serial.print("Mean Flow: "); Serial.print(mean);
  Serial.print(" | DC Ratio: "); Serial.println(dcRatio);

  // ---- Leak Detection via FFT ----
  if (mean > 0.2 && dcRatio > 0.7) {
    BT.println("FFT LEAK DETECTED!");
    digitalWrite(redLED, HIGH);
    digitalWrite(buzzer, HIGH);
  }
}
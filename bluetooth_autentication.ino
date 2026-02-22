#include <Servo.h>
#include <SoftwareSerial.h>

Servo myServo;
SoftwareSerial bt(10, 11);   // RX, TX

const int statePin = 2;
const int trigPin = 4;
const int echoPin = 5;

String inputPassword = "";
String correctPassword = "1234";

bool previousState = LOW;
bool doorOpen = false;

void setup() {
  myServo.attach(9);
  myServo.write(0);

  pinMode(statePin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  bt.begin(9600);
  Serial.begin(9600);
}

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;

  return distance;
}

void loop() {

  bool currentState = digitalRead(statePin);

  // Ask password once when connected
  if (currentState == HIGH && previousState == LOW) {
    bt.println("=== Bluetooth Door Lock ===");
    bt.println("Enter Password:");
    inputPassword = "";
  }

  previousState = currentState;

  // Handle Password
  if (currentState == HIGH && bt.available()) {

    char c = bt.read();

    if (c == '\r' || c == '\n') {

      inputPassword.trim();

      if (inputPassword.equals(correctPassword)) {

        bt.println("Access Granted");

        // Smooth open
        for (int pos = 0; pos <= 180; pos++) {
          myServo.write(pos);
          delay(10);
        }

        doorOpen = true;
        bt.println("Door Open - Waiting for object within 7 cm");

      } else {
        bt.println("Wrong Password");
      }

      inputPassword = "";
      bt.println("Enter Password:");
    }
    else {
      inputPassword += c;
    }
  }

  // Ultrasonic Distance Monitoring
  if (doorOpen) {

    long distance = getDistance();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance > 0 && distance <= 7) {

      bt.println("Object within 7 cm - Closing Door");

      // Smooth close
      for (int pos = 180; pos >= 0; pos--) {
        myServo.write(pos);
        delay(10);
      }

      doorOpen = false;
    }

    delay(200);  // small stability delay
  }
}
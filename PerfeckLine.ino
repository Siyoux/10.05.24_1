#include <PID_v1.h>
#include <SoftwareSerial.h>

// Definisi Pin
#define btx 0 // TX 
#define brx 1 // RX 
// Pin Motor
#define Kiri1mtrPin 3
#define Kanan1mtrPin 4
#define Kiri2mtrPin 5
#define Kanan2mtrPin 6
// Pin Sensor
#define S1 A0    
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5

// Konfigurasi Bluetooth dan PID
SoftwareSerial bluetooth(btx, brx);
double Setpoint = 3;
double Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variabel S
bool manualMode = false;
char lastCommand = 0;
unsigned long lastCommandTime = 0;
unsigned long autoModeStartTime = 0;
const unsigned long autoModeTimeout = 30000; // 30 detik

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(Kiri1mtrPin, OUTPUT);
  pinMode(Kanan1mtrPin, OUTPUT);
  pinMode(Kiri2mtrPin, OUTPUT);
  pinMode(Kanan2mtrPin, OUTPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  myPID.SetMode(AUTOMATIC);
  calibrateSensors();
}

void loop() {
  if (bluetooth.available()) {
    char btt = bluetooth.read();
    if (btt != lastCommand || millis() - lastCommandTime > 200) { // Debounce logic
      lastCommand = btt;
      lastCommandTime = millis();
      if (btt >= '0' && btt <= '5') {
        processCommand(btt);
      } else {
        Serial.println("Invalid command received");
      }
    }
  }

  if (!manualMode && millis() - autoModeStartTime > autoModeTimeout) {
    stopMotors();
    manualMode = true; // Balik Ke Manual
  }

  if (!manualMode) {
    Input = getSensorReading();
    myPID.Compute();
    driveMotors(Output);
  }
}

double getSensorReading() {
  double total = 0;
  int count = 0;
  for (int i = S1; i <= S6; i++) {
    int reading = analogRead(i);
    if (reading > 0) {
      total += reading * (i - S1 + 1);
      count++;
    }
  }
  return count > 0 ? total / count : 0; // Avoid division by zero
}

void processCommand(char command) {
  switch (command) {
    case '0': moveLeft(); break;
    case '1': moveRight(); break;
    case '2': moveForward(); break;
    case '3': moveBackward(); break;
    case '4': stopMotors(); manualMode = true; break;
    case '5': manualMode = false; autoModeStartTime = millis(); break;
  }
  bluetooth.print("Executed command: ");
  bluetooth.println(command);
}

void driveMotors(double output) {
  int speed = constrain(abs(output), 0, 255);
  digitalWrite(Kiri1mtrPin, output > 0 ? HIGH : LOW);
  digitalWrite(Kanan1mtrPin, output < 0 ? HIGH : LOW);
  analogWrite(Kiri2mtrPin, speed);
  analogWrite(Kanan2mtrPin, 255 - speed);
}

void moveLeft() {
  digitalWrite(Kiri1mtrPin, HIGH);
  digitalWrite(Kanan1mtrPin, LOW);
  analogWrite(Kiri2mtrPin, 255);
  analogWrite(Kanan2mtrPin, 0);
}

void moveRight() {
  digitalWrite(Kiri1mtrPin, LOW);
  digitalWrite(Kanan1mtrPin, HIGH);
  analogWrite(Kiri2mtrPin, 255);
  analogWrite(Kanan2mtrPin, 0);
}

void moveForward() {
  analogWrite(Kiri1mtrPin, 0);
  analogWrite(Kanan1mtrPin, 0);
  analogWrite(Kiri2mtrPin, 255);
  analogWrite(Kanan2mtrPin, 255);
}

void moveBackward() {
  analogWrite(Kiri1mtrPin, 255);
  analogWrite(Kanan1mtrPin, 255);
  analogWrite(Kiri2mtrPin, 0);
  analogWrite(Kanan2mtrPin, 0);
}

void stopMotors() {
  analogWrite(Kiri1mtrPin, 0);
  analogWrite(Kanan1mtrPin, 0);
  analogWrite(Kiri2mtrPin, 0);
  analogWrite(Kanan2mtrPin, 0);
}

void calibrateSensors() {
  for (int i = 0; i < 10; i++) { // Take multiple readings to stabilize
    for (int pin = S1; pin <= S6; pin++) {
      analogRead(pin);
    }
    delay(10);
  }
}


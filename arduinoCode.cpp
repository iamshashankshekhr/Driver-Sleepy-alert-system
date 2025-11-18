// Sleepy Driver Alert System - Arduino side with 5 sec motor delay

const int buzzerPin = 8;
const int vibPin = 9;
const int motorIn1 = 10;
const int motorIn2 = 11;
const int enablePin = 6;

unsigned long alertStartTime = 0;
bool alertActive = false;       // True when ALERT received
bool motorStopped = false;      // To avoid stopping motor repeatedly

void setup() {
  Serial.begin(9600);

  pinMode(buzzerPin, OUTPUT);
  pinMode(vibPin, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Motor running normally
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  analogWrite(enablePin, 200);

  // Alerts off
  digitalWrite(buzzerPin, LOW);
  digitalWrite(vibPin, LOW);
}

void loop() {

  // Read serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // -------- ALERT RECEIVED --------
    if (command == "ALERT") {
      digitalWrite(buzzerPin, HIGH);
      digitalWrite(vibPin, HIGH);

      if (!alertActive) {
        alertStartTime = millis();  // start 5-sec countdown
        alertActive = true;
        motorStopped = false;
      }
    }

    // -------- NORMAL RECEIVED --------
    else if (command == "NORMAL") {
      digitalWrite(buzzerPin, LOW);
      digitalWrite(vibPin, LOW);

      // Motor ON again
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
      analogWrite(enablePin, 200);

      // Reset flags
      alertActive = false;
      motorStopped = false;
    }
  }

  // -------- DELAYED MOTOR STOP LOGIC --------
  if (alertActive && !motorStopped) {
    if (millis() - alertStartTime >= 5000) {  // 5 seconds
      // Stop motor after 5 seconds
      analogWrite(enablePin, 0);
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, LOW);

      motorStopped = true;
      Serial.println("Motor stopped after 5 seconds");
    }
  }
}

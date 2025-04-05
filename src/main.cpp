#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define PIN_CAPACITIVE 2
#define PIN_IR 3
#define CHANNEL_GET_BOTTLE 1
#define CHANNEL_RETURN_BOTTLE 0
#define CHANNEL_GET_BOTTLE_OPEN 80
#define CHANNEL_GET_BOTTLE_CLOSE 176
#define CHANNEL_RETURN_BOTTLE_OPEN 120
#define CHANNEL_RETURN_BOTTLE_CLOSE 0
#define DELAY_BETWEEN_ACTION 1500
#define DELAY_VALIDATE 3000
#define BTN_A 7
#define BTN_B 5
#define BTN_C 6
#define BTN_D 4
#define BIN_A 8
#define BIN_B 9
#define BIN_C 10
#define BIN_D 11
#define BIN_STOP_A 28
#define BIN_STOP_B 30
#define BIN_STOP_C 32
#define BIN_STOP_D 34

// 2 - 176

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long startMillis = 0;
bool buttonPressed = false;

void ready() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("   Welcome To");
  lcd.setCursor(0, 1);
  lcd.print("   Waste2Snack");
}

void invalid() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Invalid Bottle");
  delay(3000);
  ready();
}

void valid() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Choose your");
  lcd.setCursor(0, 1);
  lcd.print("snack");
}

void closeDoors() {
  int pulselen = map(CHANNEL_GET_BOTTLE_CLOSE, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(CHANNEL_GET_BOTTLE, 0, pulselen);
  pulselen = map(CHANNEL_RETURN_BOTTLE_CLOSE, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(CHANNEL_RETURN_BOTTLE, 0, pulselen);
}

void getBottle() {
  int pulselen = map(CHANNEL_GET_BOTTLE_OPEN, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(CHANNEL_GET_BOTTLE, 0, pulselen);
  delay(DELAY_BETWEEN_ACTION);
  pulselen = map(CHANNEL_GET_BOTTLE_CLOSE, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(CHANNEL_GET_BOTTLE, 0, pulselen);
  delay(DELAY_BETWEEN_ACTION);
}

void returnBottle() {
  int pulselen = map(CHANNEL_RETURN_BOTTLE_OPEN, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(CHANNEL_RETURN_BOTTLE, 0, pulselen);
  delay(DELAY_BETWEEN_ACTION);
  pulselen = map(CHANNEL_RETURN_BOTTLE_CLOSE, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(CHANNEL_RETURN_BOTTLE, 0, pulselen);
  delay(DELAY_BETWEEN_ACTION);
}

void runBin(int pin, int stopPin) {
  // Serial.print("Running bin: ");
  // Serial.println(pin);

  // digitalWrite(pin, LOW);
  
  // unsigned long startMillis = millis();
  // while (millis() - startMillis < 3000) {
  //   // Non-blocking delay for 3000ms
  // }

  // Serial.print("Stopping bin: ");
  // Serial.println(pin);
  
  // digitalWrite(pin, HIGH);
  
  // startMillis = millis();
  // while (millis() - startMillis < 1000) {
  //   // Non-blocking delay for 1000ms
  // }

  digitalWrite(pin, LOW);
  delay(1000);
  while (true) {
    if (!digitalRead(stopPin)) {
      break;
    }
    delay(1);
  }
  
  while (true) {
    if (digitalRead(stopPin)) {
      break;
    }
    delay(1);
  }
  digitalWrite(pin, HIGH);
}

void clickA() {
  buttonPressed = true;
  Serial.println("A");
  runBin(BIN_B, BIN_STOP_B);
}

void clickB() {
  buttonPressed = true;
  Serial.println("B");
  runBin(BIN_A, BIN_STOP_A);
}

void clickC() {
  buttonPressed = true;
  Serial.println("C");
  runBin(BIN_D, BIN_STOP_D);
}

void clickD() {
  buttonPressed = true;
  Serial.println("D");
  runBin(BIN_C, BIN_STOP_C);
}

void checkButtons() {
  while (true) {
    if (digitalRead(BTN_A) == LOW) {
      clickA();
      break;
    }
    if (digitalRead(BTN_B) == LOW) {
      clickB();
      break;
    }
    if (digitalRead(BTN_C) == LOW) {
      clickC();
      break;
    }
    if (digitalRead(BTN_D) == LOW) {
      clickD();
      break;
    }
  }
  ready();
}

void setup() {
  Serial.begin(9600);

  pinMode(PIN_CAPACITIVE, INPUT_PULLUP);
  pinMode(PIN_IR, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  pinMode(BTN_D, INPUT_PULLUP);
  pinMode(BIN_A, OUTPUT);
  pinMode(BIN_B, OUTPUT);
  pinMode(BIN_C, OUTPUT);
  pinMode(BIN_D, OUTPUT);
  digitalWrite(BIN_A, HIGH);
  digitalWrite(BIN_B, HIGH);
  digitalWrite(BIN_C, HIGH);
  digitalWrite(BIN_D, HIGH);
  pinMode(BIN_STOP_A, INPUT_PULLUP);
  pinMode(BIN_STOP_B, INPUT_PULLUP);
  pinMode(BIN_STOP_C, INPUT_PULLUP);
  pinMode(BIN_STOP_D, INPUT_PULLUP);

  pwm.begin();
  // pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  closeDoors();
  Serial.println("Setup complete. Doors closed.");

  ready();
}

void loop() {
  if (Serial.available()) {
    int pos = Serial.readStringUntil('\n').toInt();
    Serial.print("Setting servo position to: ");
    Serial.println(pos);
    if (pos >= 0 && pos <= 180) {
      int pulselen = map(pos, 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(CHANNEL_GET_BOTTLE, 0, pulselen);
    } else {
      Serial.println("Invalid position. Please enter a value between 0 and 180.");
    }
  }
  // Serial.print("Capacitive Sensor Value: ");
  // Serial.print(digitalRead(PIN_CAPACITIVE));
  // Serial.print(" | IR Sensor Value: ");
  // Serial.println(digitalRead(PIN_IR));

  if (digitalRead(PIN_CAPACITIVE) == LOW) {
    Serial.println("Capacitive sensor triggered.");
    startMillis = millis();
    while (true) {
      if (digitalRead(PIN_CAPACITIVE) == HIGH) {
        Serial.println("Capacitive sensor released.");
        break;
      }
      if (millis() - startMillis >= DELAY_VALIDATE) {
        Serial.println("Capacitive sensor validated. Executing return bottle.");
        returnBottle();
        invalid();
        break;
      }
    }
  }
  if (digitalRead(PIN_IR) == LOW) {
    Serial.println("IR sensor triggered.");
    startMillis = millis();
    while (true) {
      if (digitalRead(PIN_IR) == HIGH) {
        Serial.println("IR sensor released.");
        break;
      }
      if (millis() - startMillis >= DELAY_VALIDATE) {
        Serial.println("IR sensor validated. Executing get bottle.");
        if (digitalRead(PIN_CAPACITIVE) == LOW) {
          returnBottle();
          invalid();
        }
        else {
          getBottle();
          valid();
          checkButtons();
        }
        break;
      }
    }
  }
  delay(10);
}
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <U8g2lib.h>

#define MOTOR_INTERFACE_TYPE 4

LiquidCrystal_I2C lcd(0x27, 16, 2);

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, 30, 34, 32, U8X8_PIN_NONE);

const int trigPin = 9;
const int echoPin = 8;
long duration;
float distanceCm;
unsigned long previousDistMillis = 0;
const unsigned long distInterval = 50;

unsigned int shorterDistance = 30;
unsigned int longerDistance = 60;

const String numDA = "6291623";
const String laboName = "Labo 6";

#define IN_1 4
#define IN_2 5
#define IN_3 6
#define IN_4 7

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

const int stepsPerRevolution = 2048;
int motorAngle = 90;
int targetAngle = 90;
long targetStep = 0;
const int minAngle = 10;
const int maxAngle = 170;
const int stepsPerDegree = stepsPerRevolution / 360;

unsigned long previousSerialMillis = 0;
const unsigned long serialInterval = 100;

unsigned long previousLcdMillis = 0;
const unsigned long lcdInterval = 100;

//Alarm et LED RGB
const int buzzerPin = 10;
const int redPin = 11;
const int bluePin = 12;
bool alarmActive = false;
unsigned long lastDetectionTime = 0;
unsigned long gyroLastToggleTime = 0;
bool isRed = false;
const unsigned long gyroToggleInterval = 100;
const unsigned long gyroToggleTime = 3000;
const int minDistanceGyro = 15;
const int buzzerSound1 = 1000;
const int buzzerSound2 = 200;

bool limitsAreValid = true;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  lcd.clear();
  lcdStartup();

  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(100);

  pinMode(buzzerPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(redPin, LOW);
  digitalWrite(bluePin, LOW);

  u8g2.begin();
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
}

void lcdStartup() {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(numDA);
  lcd.setCursor(0, 1);
  lcd.print(laboName);
  delay(2000);
}

//Fonction pour la distance
void distTask() {
  if (millis() - previousDistMillis >= distInterval) {
    previousDistMillis = millis();

    digitalWrite(trigPin, LOW);
    digitalWrite(trigPin, HIGH);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distanceCm = duration * 0.034 / 2;
  }
}

//Fonction pour les états
void stateManager() {
  if (distanceCm < shorterDistance) {
    motorAngle = -1;
  } else if (distanceCm > longerDistance) {
    motorAngle = -2;
  } else {
    targetAngle = map(distanceCm, shorterDistance, longerDistance, minAngle, maxAngle);
    targetStep = targetAngle * stepsPerDegree;

    if (myStepper.currentPosition() != targetStep) {
      myStepper.moveTo(targetStep);
    }
  }
  myStepper.run();
}

//Fonction pour l'affichage lcd
void lcdTask() {
  if (millis() - previousLcdMillis >= lcdInterval) {
    previousLcdMillis = millis();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist : ");
    lcd.print((int)distanceCm);
    lcd.print(" cm");
    lcd.setCursor(0, 1);

    if (distanceCm <= shorterDistance) {
      lcd.print("Obj  : Trop pres");
    } else if (distanceCm >= longerDistance) {
      lcd.print("Obj  : Trop loin");
    } else {
      lcd.print("Obj  : ");
      lcd.print(targetAngle);
      lcd.print(" deg");
    }
  }
}

//Fonction pour buzzer et LED RGB
void alarmTask() {
  unsigned long currentTime = millis();

  if (distanceCm <= minDistanceGyro) {
    lastDetectionTime = currentTime;
    alarmActive = true;

  } else if (currentTime - lastDetectionTime >= gyroToggleTime) {
    alarmActive = false;
  }

  if (alarmActive) {
    tone(buzzerPin, buzzerSound1, buzzerSound2);

    if (currentTime - gyroLastToggleTime >= gyroToggleInterval) {
      gyroLastToggleTime = currentTime;
      isRed = !isRed;
      digitalWrite(redPin, isRed ? HIGH : LOW);
      digitalWrite(bluePin, isRed ? LOW : HIGH);
    }

  } else {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, LOW);
  }
}

//Fonction pour le serial monitor
void serialTask() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    String command, arg1 ,arg2;
    parseCommand(input, command, arg1, arg2);

    if (input.equalsIgnoreCase("gDist")) {
      Serial.print((int)distanceCm);
      return;
    }
     else if (command == "cfg" && arg1 == "alm") {
      shorterDistance = arg2.toInt();
      Serial.println("Configure la distance de détection de l'alarme à " + String(shorterDistance) + " cm");
      showConfirmationSymbol();
    }
    else if (command == "cfg" && (arg1 == "lim_inf" || arg1 == "lim_sup")) {
      int val = arg2.toInt();

      if (arg1 == "lim_inf") {
        if (val >= longerDistance) {
          Serial.println("Error - Limite inférieure plus gande que limite supérieure");
          showForbiddenSymbol();
          limitsAreValid = false;
        } else {
          shorterDistance = val;
          Serial.println("Il configure sa limite inférieure du moteur à " + String(val) + " cm");
          showConfirmationSymbol();
          limitsAreValid = true;
        }
      }
      else {
        if (val <= shorterDistance) {
          Serial.println("Error - Limite supérieure plus grande que limite inférieure");
          showForbiddenSymbol();
          limitsAreValid = false;
        } else {
          longerDistance = val;
          Serial.println("Il configure la limite supérieure du moteur à " + String(val) + " cm");
          showConfirmationSymbol();
          limitsAreValid = true;
        }
      }
    }
    else {
      showErrorSymbol();
    }
  }
}


void parseCommand(const String& input, String& command, String& arg1, String& arg2) {
  int i1 = input.indexOf(';');
  int i2 = input.indexOf(';', i1 + 1);

  if (i1 == -1) {
    command = input;
    return;
  }

  command = input.substring(0, i1);
  if (i2 != -1) {
    arg1 = input.substring(i1 + 1, i2);
    arg2 = input.substring(i2 + 1);
  } else {
    arg1 = input.substring(i1 + 1);
  }
}

//Fonction qui affiche le symbole de confirmation
void showConfirmationSymbol() {
  u8g2.clearBuffer();
  u8g2.drawLine(1, 4, 3, 6);
  u8g2.drawLine(3, 6, 6, 3);
  u8g2.sendBuffer();
  delay(3000);
  u8g2.clear();
}

//Fonction qui affiche le symbole d'erreur
void showErrorSymbol() {
  u8g2.clearBuffer();
  u8g2.drawLine(1, 1, 6, 6);
  u8g2.drawLine(6, 1, 1, 6);
  u8g2.sendBuffer();
  delay(3000);
  u8g2.clear();
}

//Fonction qui affiche le signe d'interdiction
void showForbiddenSymbol() {
  u8g2.clearBuffer();
  u8g2.drawCircle(3, 3, 3, U8G2_DRAW_ALL);
  u8g2.drawLine(1, 5, 5, 1);
  u8g2.sendBuffer();
  delay(3000);
  u8g2.clear();
}

void loop() {
  distTask();
  stateManager();
  lcdTask();
  alarmTask();
  serialTask();
}

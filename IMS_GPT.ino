#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <dht.h>

// Pin Definitions
#define DHTPIN 8
#define SERVO_PIN 6
#define MIC_PIN 9
#define BULB_PIN 13
#define FAN_PIN 12
#define LED_PIN 7
#define RAIN_SENSOR_PIN A0

#define DHT_DELAY 2000
#define SMS_DELAY 100
#define SERVO_SPEED 10  // reduce for fast servo movement and vice versa (i think 40 is max)
#define LCD_DELAY 1000
#define SERIAL_DELAY 1000
#define SIM_DELAY 10

// Object Initializations
LiquidCrystal_I2C lcd(0x27, 20, 4);
dht dht_;
Servo servo;
SoftwareSerial sim_serial(4, 5);  // Tx 4, Rx 5

// Global Variables
String user_no = "xxxxxxxxxx";
bool servo_active = false, over_ride = false, servo_state = false;
float humidity, temperature;
int wet_level = 0, turn_clk = 0, angle = 0;
uint32_t dht_last_time = 0, lcd_last_time = 0, sms_last_sent = 0, serial_last_time = 0, turn_last = 0, sim_last = 0;

void setup() {
  pinMode(BULB_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(MIC_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(FAN_PIN, HIGH);
  digitalWrite(BULB_PIN, HIGH);

  Serial.begin(9600);
  sim_serial.begin(9600);
  lcd.init();
  lcd.backlight();
  servo.attach(SERVO_PIN);
  servo.write(0);

  displayStartupScreen();
  delay(3000);
}

void loop() {
  handleSerialCommands();
  readSensors();
  controlEnvironment();
  controlServo();
  updateLCD();
  updateSerial();
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char recv = Serial.read();
    switch (recv) {
      case 'E':
        over_ride = true;
        digitalWrite(FAN_PIN, LOW);
        break;
      case 'R':
        over_ride = false;
        digitalWrite(FAN_PIN, HIGH);
        break;
      case 'T':
        over_ride = true;
        digitalWrite(BULB_PIN, LOW);
        break;
      case 'Y':
        over_ride = false;
        digitalWrite(BULB_PIN, HIGH);
        break;
      case 'U': digitalWrite(LED_PIN, HIGH); break;
      case 'I': digitalWrite(LED_PIN, LOW); break;
    }
  }

  if (sim_serial.available() > 0) {
    char recv_s = sim_serial.read();
    switch (recv_s) {
      case 'E': sendAlertSMS(); break;
    }
  }
}

void readSensors() {
  wet_level = analogRead(RAIN_SENSOR_PIN);

  if (millis() - dht_last_time >= DHT_DELAY) {
    if (dht_.read22(DHTPIN) == DHTLIB_OK) {
      humidity = dht_.humidity;
      temperature = dht_.temperature;
    }
    dht_last_time = millis();
  }

  if (digitalRead(MIC_PIN) == HIGH) {
    if (millis() - sms_last_sent > SMS_DELAY) {  // 1000ms is 1 sec increase or decrease at will (might not be up to 1sec in this context)
      sendAlertSMS();
    }
    servo_active = true;
    sms_last_sent = millis();
  } else {
    // servo_active = false;
  }
}

void controlEnvironment() {
  if (!over_ride) {
    if (temperature <= 20) {
      digitalWrite(FAN_PIN, HIGH);
      digitalWrite(BULB_PIN, LOW);
    } else if (temperature >= 30) {
      digitalWrite(FAN_PIN, LOW);
      digitalWrite(BULB_PIN, HIGH);
    } else if (temperature > 25 && temperature < 28) {
      digitalWrite(FAN_PIN, HIGH);
      digitalWrite(BULB_PIN, HIGH);
    }
  }

  if (wet_level <= 400) {  // 1000ms is 1 sec increase or decrease at will (might not be up to 1sec in this context)
    if (millis() - sms_last_sent > SMS_DELAY) sendAlertSMS();
    digitalWrite(BULB_PIN, LOW);
    sms_last_sent = millis();
  }
}

void controlServo() {

  //  Serial.print("Active: ");
  //  Serial.println(servo_active);
  //  Serial.print("Turn clk: ");
  //  Serial.println(turn_clk);
  //  Serial.print("Angle: ");
  //  Serial.println(angle);

  if (servo_active) {

    if (turn_clk == 0) {
      //      servo.write(angle);
      if (millis() - turn_last >= SERVO_SPEED) {
        servo.write(angle++);
        turn_last = millis();
      }
      if (angle == 90) {
        turn_clk = 1;
        turn_last = millis();
      }

    } else if (turn_clk == 1) {
      //      servo.write(angle);
      if (millis() - turn_last >= SERVO_SPEED) {  // reduce for fast servo movement and vice versa (i think 40 is max)
        servo.write(angle--);
        turn_last = millis();
      }

      if (angle == 0) {
        turn_clk = 2;
        turn_last = millis();
      }
    } else if (turn_clk == 2) {
      turn_clk = 0;
      servo_active = false;
      turn_last = millis();
    }
  }
}

void updateLCD() {
  if (millis() - lcd_last_time >= LCD_DELAY) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Temp: " + String(temperature));
    lcd.setCursor(1, 1);
    lcd.print("Humidity: " + String(humidity));
    lcd.setCursor(1, 2);
    lcd.print("Wet Level: " + String(wet_level < 400 ? "Wet" : "Not Wet"));
    lcd.setCursor(1, 3);
    lcd.print("Sound: " + String(digitalRead(MIC_PIN) == HIGH ? "Crying" : "Not Crying"));
    lcd_last_time = millis();
  }
}

void updateSerial() {
  if (millis() - serial_last_time >= SERIAL_DELAY) {
    Serial.print(" Temp: " + String(temperature) + "|");
    Serial.print("Humidity: " + String(humidity) + "|");
    Serial.print("Wet Level: " + String(wet_level < 400 ? "W" : "NW") + "|");
    Serial.println("Sound: " + String(digitalRead(MIC_PIN) == HIGH ? "C" : "NC"));
    serial_last_time = millis();
  }
}

void sendAlertSMS() {

  String message = "Alert:\nTemp: " + String(temperature) + "\nHumidity: " + String(humidity) + "\nWet level: " + (wet_level <= 400 ? "Wet" : "Not Wet") + "\nSound: " + (digitalRead(MIC_PIN) == HIGH ? "Crying" : "Not Crying");
  /*if (millis() - sim_last >= SIM_DELAY) {*/
  sim_serial.println("AT");
  /*sim_last = millis();*/
  /*}*/
  // _delay_ms(10);
  // while(!(millis() - sim_last >= SIM_DELAY));
  sim_serial.println("AT+CMGF=1");
  // sim_last = millis();
  // _delay_ms(10);
  // while (!(millis() - sim_last >= SIM_DELAY));
  sim_serial.print("AT+CMGS=\"");
  sim_serial.print(user_no);
  sim_serial.println("\"");
  // sim_last = millis();
  // _delay_ms(10);
  // while (!(millis() - sim_last >= SIM_DELAY));
  sim_serial.print(message);
  // sim_last = millis();
  // _delay_ms(10);
  // while (!(millis() - sim_last >= SIM_DELAY + 20));
  sim_serial.write(26);  // CTRL+Z
                         // Serial.println("Sent SMS");
                         // sim_last = millis();
}

void displayStartupScreen() {
  lcd.clear();
  lcd.setCursor(2, 1);
  lcd.print("INFANT MONITORING");
  lcd.setCursor(2, 2);
  lcd.print("& CONTROL SYSTEM");
}

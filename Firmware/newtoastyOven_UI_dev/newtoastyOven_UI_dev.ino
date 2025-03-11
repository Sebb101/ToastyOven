#include <Wire.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include "Adafruit_MAX31855.h"
#include <SimpleRotary.h>
#include <PID_v1.h>  // Standard PID library for Arduino

// Pin Definitions
#define SSRpin 9
#define statusLEDpin 7
#define alertLEDpin 12
#define buzzerPin 6
#define A_CLK 15
#define B_DT 14
#define encoder_switch 8
#define MAXDO 3
#define MAXCS 4
#define MAXCLK 5

// Rotary Encoder
SimpleRotary rotary(A_CLK, B_DT, encoder_switch);

// Thermocouple Sensor
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// PID Variables
double setTemp = 150.0;
double Input, Output;
double Kp = 3.5, Ki = 4.0, Kd = 1.2;
PID myPID(&Input, &Output, &setTemp, Kp, Ki, Kd, DIRECT);

// OLED Display (Optimized for Memory)
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

// Menu States
enum MenuState { MAIN_MENU, SET_TEMP, HEAT_MENU };
MenuState menu = MAIN_MENU;

// System Status
bool heating = false;
unsigned long startTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println(F("System Initialized..."));

    pinMode(buzzerPin, OUTPUT);
    pinMode(SSRpin, OUTPUT);
    pinMode(statusLEDpin, OUTPUT);
    pinMode(alertLEDpin, OUTPUT);
    digitalWrite(SSRpin, LOW);
    digitalWrite(statusLEDpin, LOW);
    digitalWrite(alertLEDpin, LOW);

    u8g2.begin();
    myPID.SetMode(AUTOMATIC);
}

void loop() {
    Input = thermocouple.readCelsius();
    myPID.Compute();

    if (heating) {
        digitalWrite(SSRpin, Output > 50 ? HIGH : LOW);
    } else {
        digitalWrite(SSRpin, LOW);
    }

    handleUserInput();
    displayUI();
}

// Handle Rotary Encoder Input
void handleUserInput() {
    int action = rotary.rotate();
    if (action == 1) {  
        if (menu == SET_TEMP) setTemp += 5;
    } else if (action == -1) {  
        if (menu == SET_TEMP) setTemp -= 5;
    }

    if (rotary.push()) {  
        if (menu == MAIN_MENU) {
            menu = SET_TEMP;
        } else if (menu == SET_TEMP) {
            menu = HEAT_MENU;
            heating = true;
            startTime = millis();
        } else if (menu == HEAT_MENU) {
            menu = MAIN_MENU;
            heating = false;
        }
    }
}

// Display UI
void displayUI() {
    u8g2.firstPage();
    do {
        if (menu == MAIN_MENU) {
            u8g2.setCursor(2, 10);
            u8g2.print("MAIN MENU");
            u8g2.setCursor(2, 30);
            u8g2.print("> Set Temp: "); u8g2.print(setTemp); u8g2.print("C");
        } else if (menu == SET_TEMP) {
            u8g2.setCursor(2, 10);
            u8g2.print("SET TEMP:");
            u8g2.setCursor(2, 30);
            u8g2.print(setTemp); u8g2.print(" C");
        } else if (menu == HEAT_MENU) {
            u8g2.setCursor(2, 10);
            u8g2.print("HEATING...");
            u8g2.setCursor(2, 30);
            u8g2.print("Target: "); u8g2.print(setTemp); u8g2.print("C");
            u8g2.setCursor(2, 40);
            u8g2.print("Current: "); u8g2.print(Input); u8g2.print("C");
            u8g2.setCursor(2, 50);
            u8g2.print("Time: "); u8g2.print((millis() - startTime) / 1000); u8g2.print("s");
        }
    } while (u8g2.nextPage());
}

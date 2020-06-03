#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Menu.h"
#include "Process.h"

//Buttons assignment
#define BTN_UP 4
#define BTN_DW 5
#define BTN_SEL 6

LCDMenu menu(PCF8574A_ADDR_A20_A10_A00, BTN_UP, BTN_DW, BTN_SEL);

void setup() {
  Serial.begin(115200);
  Process::process.setup();
  menu.setup();
}

void loop() {
  Process::process.loop();
  menu.loop();
}
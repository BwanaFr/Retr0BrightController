#include <Arduino.h>
#include <Wire.h>              //for ESP8266 use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver
#include <LiquidCrystal_I2C.h>
#include "Menu.h"
#include "Process.h"

//LCD configuration
#define LCD_COLUMNS  16
#define LCD_ROWS      2
#define LCD_SPACE_SYMBOL 0x20  //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet

//Buttons assignment
#define BTN_UP 4
#define BTN_DW 5
#define BTN_SEL 6

//Outputs
#define PUMP 3
#define HEATER LED_BUILTIN
//7

//Temperature probe
#define TH_PIN A7

LiquidCrystal_I2C lcd(PCF8574A_ADDR_A20_A10_A00);

Process p(TH_PIN, 0, PUMP, HEATER);
LCDMenu menu(lcd, p, BTN_UP, BTN_DW, BTN_SEL);

void setup() {
  Serial.begin(115200);
   //LCD configuration
  if(lcd.begin(LCD_COLUMNS, LCD_ROWS) != 1)
  {
    Serial.println(F("LCD not found"));
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Retr0bright!"));
  
  p.setup();
  menu.setup();
}

 void loop() {
  p.loop();
  menu.loop();
}
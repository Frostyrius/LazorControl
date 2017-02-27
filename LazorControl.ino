#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>

#define maxTemp 30
#define coolTemp 20
#define minTemp 10
#define maxCurrent 2.00

#define tempPin 2
#define coolingPin 3
#define cancelButton 4
#define laserPin 5
#define ledPin 13
#define potPin A0
#define voltagePin A1
#define currentPin A2
#define inputVoltagePin A3

unsigned char err = 0;

unsigned long currentTime = 0;

float laserVoltage = 0;
float laserCurrent = 0;
float laserTemp = 0;

unsigned long valueTime = 0;  
unsigned long valueDelay = 500; //write values to LCD
unsigned int potValue = 0;
unsigned char laserPower;

unsigned int cursorDelay = 500;
unsigned long cursorTime = 0;
byte cursorState = LOW;

unsigned int ledDelay = 500;
unsigned long ledTime = 0;
byte ledState = LOW;

unsigned long debugTime = 0;
unsigned int debugDelay = 1000;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
OneWire ds(tempPin);

void setup(){
  //----------------------- init
  Serial.begin(19200);
  Serial.println("Initiating, please wait...");
  Serial.println("-------------------------");
  analogWrite(laserPin, 0);
  pinMode(ledPin, OUTPUT);
  pinMode(cancelButton, INPUT);
  lcd.begin(20,4);
  lcd.backlight();
  lcd.clear();
  //----------------------- start screen
  lcd.setCursor(0,0);
  lcd.print("--------------------");
  lcd.setCursor(0,1);
  lcd.print("--- LazorControl ---");
  lcd.setCursor(0,2);
  lcd.print("---     v0.1     ---");
  lcd.setCursor(0,3);
  lcd.print("--------------------");
  delay(2000);
  lcd.clear();
  delay(500);
  lcd.setCursor(1,0);
  lcd.print("LazorControl v0.1");
  lcd.setCursor(0,1);
  lcd.print("--------------------");
  lcd.setCursor(0,2);
  lcd.print("PWR:");
  lcd.setCursor(0,3);
  lcd.print("Temp:");
}



void loop(){
//--------------------------------- loop begin
  laserTemp = getTemp();
  if( laserTemp > maxTemp ) err=1;    //thermal protection
  if( (analogRead(currentPin)/204.6) > maxCurrent) err=2;    //current protection
  
  potValue = analogRead(potPin);
  if(!err){
    laserPower = potValue/4;
  }else{
    laserPower = 0 ;
  }
  lcd.setCursor(0,1);
  switch(err){
    case 0: 
      lcd.print("--------------------");
      break;
    case 1: 
      lcd.print("- THERMAL OVERLOAD -"); 
      if(laserTemp<coolTemp) err=0;
      break;
    case 2:
      lcd.print("- CURRENT OVERLOAD -"); 
      if(analogRead(potValue)<10) err=0;
      laserPower = 0;
      break;
    default: break;
  }
  analogWrite(laserPin, laserPower);
  currentTime = millis();
  
  laserVoltage = analogRead(A1);
  laserCurrent = analogRead(A2);
  
//--------------------------------- cursor blink
  if (cursorTime < currentTime ){
    lcd.setCursor(19,0);
    if (cursorState) {
      lcd.print(" ");
      cursorState = LOW;
    } else {
      lcd.print("*");
      cursorState = HIGH;
    }
    cursorTime = currentTime + cursorDelay;
  }
    
//--------------------------------- write values on LCD
  if (valueTime < currentTime ){
    //-------------------- write power
    lcd.setCursor(5,2);
    float lcdTemp = laserPower/2.55; //temporary to percent
    if(lcdTemp<100) lcd.print(" ");
    if(lcdTemp<10) lcd.print(" ");
    lcd.print(lcdTemp);
    lcd.setCursor(10, 2);
    lcd.print("%");
    //-------------------- write temp
    lcd.setCursor(6,3);
    if(laserTemp == -1000){
      lcd.print("----- ");
    }else{
      if(laserTemp<10) lcd.print(" ");
      lcd.print(laserTemp);
    }
    //-------------------- write voltage
    lcd.setCursor(12,2);
    lcd.print("U: ");
    if( analogRead(voltagePin) > 1015 ){
      lcd.print("OL  ");
    }else{
      laserVoltage = analogRead(voltagePin)/204.6;
      lcd.print(laserVoltage);
    }
    lcd.print("V");
    //-------------------- write current
    lcd.setCursor(12,3);
    lcd.print("I: ");
    if( analogRead(currentPin) > 1015 ){
      lcd.print("OL  ");
    }else{
      laserCurrent = analogRead(currentPin)/204.6;
      lcd.print(laserCurrent);
    }
    lcd.print("A");
    
    valueTime = currentTime + valueDelay;
  }

//--------------------------------- LED blink  
  if(ledTime < currentTime){
    if(ledState){
      ledState = LOW;
    }else{
      ledState = HIGH;
    }
    digitalWrite(ledPin, ledState);
    ledTime = currentTime + ledDelay;
  }
  
//--------------------------------- debug - pritn values on serial

if(debugTime < currentTime){
    Serial.print("Input pot.:    ");
    Serial.println(potValue);
    Serial.print("Output power:  ");
    Serial.println(laserPower);
    Serial.print("Laser voltage: ");
    Serial.println((int)laserVoltage);
    Serial.print("Laser current: ");
    Serial.println((int)laserCurrent);
    Serial.print("LED temp.:     ");
    if(laserTemp == -1000){
      Serial.println("N/A");
    }else{
      Serial.println(laserTemp);
    }
    Serial.print("Current error: ");
    Serial.print(err);
    switch(err){
      case 0: Serial.println("  OK"); break;
      case 1: Serial.println("  Thermal overload"); break;
      case 2: Serial.println("  Current overload"); break;
      default: break;
    }
    Serial.print("Current time:  ");
    Serial.println(currentTime);
    Serial.println("-------------------------");
    
    debugTime = currentTime + debugDelay;
  }


//--------------------------------- end of loop
}

float getTemp(){
 
  byte data[12];
  byte addr[8];
 
  if ( !ds.search(addr)) {
    ds.reset_search();
    return -1000;
  }
 
  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }
 
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);
  for (int i = 0; i < 9; i++) { // potřebujeme 9 bytů
    data[i] = ds.read();
  }
 
  ds.reset_search();
   
  byte MSB = data[1];
  byte LSB = data[0];
 
  float tempRead = ((MSB << 8) | LSB);
  float TemperatureSum = tempRead / 16;
 
  return TemperatureSum; 
}

#include <Firmata.h>
#include <dht11.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <RFID.h>
#include <Stepper.h>

#define DS3231_I2C_ADDRESS 0x68
#define SS_PIN 10
#define RST_PIN 9

#define LOW_LIGHT 15
#define SECS_IN_MIN 60
#define SECS_IN_HOUR (SECS_IN_MIN * 60)
#define SECS_IN_DAY (SECS_IN_HOUR * 24)
#define TRUE 1
#define FALSE 0
#define DUSK_DELAY (SECS_IN_MIN / 4) 
#define DAWN_DELAY (SECS_IN_MIN / 4)
#define DUSK_LIGHT_DELAY (SECS_IN_MIN /2)

long lightLastLow = 0;
long lightLastHigh = 0;
boolean lightLowNow;
boolean henHouseLightOn = FALSE; //Make sure this is true in the world

RFID rfid(SS_PIN, RST_PIN); 
dht11 DHT11;

#define DHT11PIN 2
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

int BH1750_address = 0x23; // i2c Addresse
byte buff[2];

byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

// Setup variables:
    int serNum0;
    int serNum1;
    int serNum2;
    int serNum3;
    int serNum4;

unsigned long previousMillisLight = 0;        // will store last time Light was updated
long OnTimeLight = 5000;           // milliseconds of on-time
long OffTimeLight = 10000;          // milliseconds of off-time

unsigned long previousMillisTemp = 0;        // will store last time Temp was updated
long OnTimeTemp = 30000;           // milliseconds of on-time
long OffTimeTemp = 15000;          // milliseconds of off-time

unsigned long previousMillisCard = 0;        // will store last time Card was updated - prevents multiple readings on swipe
long OnTimeCard = 500;           // milliseconds of on-time
long OffTimeCard = 500;          // milliseconds of off-time


int in1Pin = 3;
int in2Pin = 4;
int in3Pin = 5;
int in4Pin = 6;

Stepper motor(512, in1Pin, in3Pin, in2Pin, in4Pin);

int ledPin = 13; // choose the pin for the LED
int switchPin = 7; // choose the input pin (for a pushbutton)
int egg = 0; // variable for reading the pin status
int counter = 0;
int currentState = 0;
int previousState = 0;

void setup(){
  pinMode(8, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  motor.setSpeed(25);
  
  pinMode(ledPin, OUTPUT); // declare LED as output
  pinMode(switchPin, INPUT); // declare pushbutton as input
   
  Wire.begin();
  BH1750_Init(BH1750_address);
  
  delay(200);
  Serial.begin(9600);
  SPI.begin(); 
  rfid.init();
  lcd.begin (20,4); //  <<----- My LCD was 16x2
  lcd.home (); // go home
  lcd.print("Automated Hens");
  delay(5000);  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("How many eggs?");
}

void loop(){

  lightSensor(); 
  cardReader();  
  tempSensor();
  eggCounter();
  doorCheck();
  henHouseLightCheck();
   
}/* --(end main loop )-- */

void eggCounter(){
egg = digitalRead(switchPin); // read input value
if (egg == HIGH) { // check if the input is HIGH (button released)
digitalWrite(ledPin, HIGH); // turn LED on
currentState = 1;
}
else {
digitalWrite(ledPin, LOW); // turn LED off
currentState = 0;
}
if(currentState != previousState){
if(currentState == 1){
counter = counter + 1;
Serial.println(counter);
lcd.setCursor(0, 0);
lcd.print("Eggs today: ");
lcd.print(counter);
lcd.print(" ");
}
}
previousState = currentState;
delay(250);
}

boolean doorOpen = TRUE; //need to worry about initialisation

void doorMotorOpen()
{
    int stepsForward = 2048;

    motor.step(stepsForward);
    
    delay(500);
}
void doorMotorClose() 
{
    int stepsBack = -2048;
    
    motor.step(stepsBack);
    
    delay(500);
}
void doorCheck() {

  //timing codes needed
  long timestampNow = readTimestamp();
  if(doorOpen && lightLowNow && ((timestampNow - lightLastHigh)>DUSK_DELAY)){
  doorMotorClose();
  doorOpen = FALSE;
  Serial.print("Door is closing");
  }
  if(!doorOpen && !lightLowNow && ((timestampNow - lightLastLow)>DAWN_DELAY)){
  doorMotorOpen();
  doorOpen = TRUE;
  Serial.print("Door is opening");
  }

}
long readTimestamp(){

  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  long timestamp=0;
  
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,  &year);
  
  timestamp = second + 
    SECS_IN_MIN * minute +
    SECS_IN_HOUR * hour;  //needs fixing to make proper timestamp with days
  
  
  
  return timestamp;
  
}
void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}
void displayTime()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,
  &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(" Day of week: ");
  switch(dayOfWeek){
  case 1:
    Serial.println("Sunday");
    break;
  case 2:
    Serial.println("Monday");
    break;
  case 3:
    Serial.println("Tuesday");
    break;
  case 4:
    Serial.println("Wednesday");
    break;
  case 5:
    Serial.println("Thursday");
    break;
  case 6:
    Serial.println("Friday");
    break;
  case 7:
    Serial.println("Saturday");
    break;
  }
}

void cardReader() {
  
    unsigned long currentMillis = millis();
  
   if(currentMillis - previousMillisCard >= OnTimeCard){
     previousMillisCard = currentMillis;  // Remember the time
     
if (rfid.isCard()) {
        if (rfid.readCardSerial()) {
            if (rfid.serNum[0] != serNum0
                && rfid.serNum[1] != serNum1
                && rfid.serNum[2] != serNum2
                && rfid.serNum[3] != serNum3
                && rfid.serNum[4] != serNum4
            ) {
                /* With a new cardnumber, show it. */
                //Serial.println(" ");
                //Serial.println("Card found");
                serNum0 = rfid.serNum[0];
                serNum1 = rfid.serNum[1];
                serNum2 = rfid.serNum[2];
                serNum3 = rfid.serNum[3];
                serNum4 = rfid.serNum[4];
               
                //Serial.println(" ");
                //Serial.println("Cardnumber:");
                //Serial.print("Dec: ");
		//Serial.print(rfid.serNum[0],DEC);
                //Serial.print(", ");
		//Serial.print(rfid.serNum[1],DEC);
                //Serial.print(", ");
		//Serial.print(rfid.serNum[2],DEC);
                //Serial.print(", ");
		//Serial.print(rfid.serNum[3],DEC);
                //Serial.print(", ");
		//Serial.print(rfid.serNum[4],DEC);
                //Serial.println(" ");
                        
               } else {
               /* If we have the same ID, just write a dot. */
               //Serial.print(".");
             }
          }
          
                        if(serNum0 == 244) {
                          Serial.println("Hen 1 is in");
                          Serial.println(" ");
                          lcd.setCursor(0, 1);
                          lcd.print("Hen 1 is inside ");
                          displayTime();
                        } else if(serNum0 == 133) {
                          Serial.println("Hen 2 is in");
                          Serial.println(" ");
                          lcd.setCursor(0, 1);
                          lcd.print("Hen 2 is inside");
                          displayTime();
                        }
                        
}
    rfid.halt();
}
}

void lightSensor(){

  unsigned long currentMillis = millis();
  
   if(currentMillis - previousMillisLight >= OnTimeLight){
     previousMillisLight = currentMillis;  // Remember the time
   
   float valf=0;

   if(BH1750_Read(BH1750_address)==2){
    
    valf=((buff[0]<<8)|buff[1])/1.2;
    
    if(valf<0)Serial.print("> 65535");
    else Serial.print("Light (lx): " ); 
    Serial.println((int)valf,DEC); 
    }
    
  long timestampNow = readTimestamp();
  
  if (valf<LOW_LIGHT){
    lightLastLow=timestampNow;
    lightLowNow = TRUE;
  }
  else{
    lightLastHigh = timestampNow;
    lightLowNow = FALSE;
  }
 lcd.setCursor(14, 2);
 lcd.print((int)valf,DEC) && lcd.print(" lx " );
   } //end timing 
} //end of light sensor

void henHouseLightCheck()

{
    //timing codes needed
  long timestampNow = readTimestamp();
  
  if(!henHouseLightOn && lightLowNow && ((timestampNow - lightLastHigh)< DUSK_LIGHT_DELAY)){
  digitalWrite(8, HIGH); 
  Serial.print("Light is on");
  henHouseLightOn = TRUE;
  lcd.setCursor(0, 2);
  lcd.print("Light is on. ");
  }
  if(henHouseLightOn && ((timestampNow - lightLastHigh)>DUSK_LIGHT_DELAY)){
  digitalWrite(8, LOW); 
  Serial.print("Light has been put off");
  henHouseLightOn = FALSE;
  lcd.setCursor(0, 2);
  lcd.print("Light is off. ");
  }
  if(henHouseLightOn && !lightLowNow && ((timestampNow - lightLastLow) > SECS_IN_MIN /4)){
  digitalWrite(8, LOW); 
  Serial.print("Light is off");
  henHouseLightOn = FALSE;
  lcd.setCursor(0, 2);
  lcd.print("Light is off. ");
  }
  
  
  //lcd.setCursor(0, 2);
  //if(valf<LOW_LIGHT) digitalWrite(8, HIGH);   // set the LED on
  //else digitalWrite(8, LOW);   // set the LED on
  //if(valf<LOW_LIGHT) lcd.print("Light is on. ") && lcd.print((int)valf,DEC) && lcd.print(" lx " );
  //else lcd.print("Light is off. ") && lcd.print((int)valf,DEC) && lcd.print(" lx" );
   

}

void tempSensor(){
  
  unsigned long currentMillis = millis();
  
   if(currentMillis - previousMillisTemp >= OnTimeTemp){
     previousMillisTemp = currentMillis;  // Remember the time

    
  int chk = DHT11.read(DHT11PIN);

  //Serial.print("Read sensor: ");
  //switch (chk)
  {
   // case 0: Serial.println("OK"); break;
   // case -1: Serial.println("Checksum error"); break;
   // case -2: Serial.println("Time out error"); break;
    //default: Serial.println("Unknown error"); break;
  }
  Serial.print("Humidity (%): ");
  Serial.println((float)DHT11.humidity, 0);

  Serial.print("Temp (oC): ");
  Serial.println((float)DHT11.temperature, 0);
  Serial.println(" " );
  
  lcd.setCursor(0, 3);
  lcd.print((float)DHT11.humidity, 2);
  lcd.print("% ");
  lcd.print((float)DHT11.temperature, 2);
  lcd.print("oC ");

}
}
void BH1750_Init(int address){
  
  Wire.beginTransmission(address);
  Wire.write(0x10); // 1 [lux] aufloesung
  Wire.endTransmission();
}

byte BH1750_Read(int address){
  
  byte i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();  
  return i;

}
/*-----( Declare User-written Functions )-----*/
//
//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
        return 1.8 * celsius + 32;
}

//Celsius to Kelvin conversion
double Kelvin(double celsius)
{
        return celsius + 273.15;
}

// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm 
double dewPoint(double celsius, double humidity)
{
        double A0= 373.15/(273.15 + celsius);
        double SUM = -7.90298 * (A0-1);
        SUM += 5.02808 * log10(A0);
        SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
        SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
        SUM += log10(1013.246);
        double VP = pow(10, SUM-3) * humidity;
        double T = log(VP/0.61078);   // temp var
        return (241.88 * T) / (17.558-T);
}  

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity)
{
        double a = 17.271;
        double b = 237.7;
        double temp = (a * celsius) / (b + celsius) + log(humidity/100);
        double Td = (b * temp) / (a - temp);
        return Td;
}

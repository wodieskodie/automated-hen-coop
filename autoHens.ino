#include <Firmata.h>
#include <dht11.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <RFID.h>


#define SS_PIN 10
#define RST_PIN 9

RFID rfid(SS_PIN, RST_PIN); 
dht11 DHT11;

#define DHT11PIN 2
#define I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
int n = 1;
LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

int BH1750_address = 0x23; // i2c Addresse
byte buff[2];

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

void setup(){
  pinMode(8, OUTPUT);
   
  Wire.begin();
  BH1750_Init(BH1750_address);
  
  delay(200);
  Serial.begin(9600);
  SPI.begin(); 
  rfid.init();
  lcd.begin (16,2); //  <<----- My LCD was 16x2
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // go home
  lcd.print("Automated Hens");
}

void loop(){

  cardReader();  
  lightSensor();  
  tempSensor();
   
}/* --(end main loop )-- */

void cardReader() {
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
                          Serial.print("Hen 1 is in");
                          Serial.println(" ");
                          lcd.setCursor(0, 0);
                          lcd.print("Hen 1 is in ");
                        } else if(serNum0 == 133) {
                          Serial.print("Hen 2 is in");
                          Serial.println(" ");
                          lcd.setCursor(0, 0);
                          lcd.print("Hen 2 is in ");
                        }
    }
    rfid.halt();
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
  lcd.setCursor(0, 0);
  if(valf<15) digitalWrite(8, HIGH);   // set the LED on
  else digitalWrite(8, LOW);   // set the LED on
  
  if(valf<15) lcd.print("Light is on. ");
  else lcd.print("Light is off. ");
   } 
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
  
  lcd.setCursor(0, 1);
  lcd.print((float)DHT11.humidity, 2);
  lcd.print("%");
  lcd.print(", ");
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

#ifdef __AVR__
  #include <avr/power.h>
#endif
#include <Adafruit_NeoPixel.h>  // NeoPixel LED
#include <LiquidCrystal.h>      // LCD
#include <OneWire.h>            // Temperature Sensor
#include <DallasTemperature.h>  // Temperature Sensor

#include <Wire.h>               // IMU
#include <L3G.h>                // IMU
#include <Servo.h>              // Servo


//******************************************
//   PIN Allocation
//******************************************

#define statusLEDPin 13
#define buttonPin01 6
#define buttonPin02 11
#define buttonPin03 5
#define servoPin 9

#define LEDPin 34
#define Pin_USS_Trig 25
#define Pin_USS_Echo 26
#define Pin_TempSensor_OneWireBus 35


#define Pin_LCD_1 29
#define Pin_LCD_2 30
#define Pin_LCD_3 31
#define Pin_LCD_4 39
#define Pin_LCD_5 40
#define Pin_LCD_6 41

#define Pin_Servo_Micro_1 32  
#define Pin_Servo_Micro_2 33
#define Pin_Servo_Camera_1 36
#define Pin_Servo_Camera_2 37

//******************************************
//   LCD Defintion and Variables
//******************************************

LiquidCrystal lcd(Pin_LCD_1,Pin_LCD_2,Pin_LCD_3,Pin_LCD_4,Pin_LCD_5,Pin_LCD_6);
int LCDcounter = 300;
char toPrint[16];


//******************************************
//   NeoPixel LED Defintion and Variables
//******************************************
#define numOfStrips 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numOfStrips, LEDPin, NEO_GRB + NEO_KHZ800);

int LEDcounter = 0;

long LEDTime;
String LEDmode = "";
int LEDsafeCycles = 0;
float brightness = 1.0;
//******************************************
//   Temperature Sensor Defintion and Variables
//******************************************
OneWire oneWire(Pin_TempSensor_OneWireBus); 
DallasTemperature sensors(&oneWire);

//******************************************
//   IMU Defintion and Variables
//******************************************
int timerIMU = 0;

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

float IMUpitch;
float IMUroll;
float IMUyaw;
float IMUheading;

//******************************************
//   Serial Port Defintion and Variables
//******************************************
int incomingByte = 0;  
bool AutoDisplay = false;
long inputTime = 0;

//******************************************
//   Servo Motors Defintion and Variables
//******************************************
//int rightArm = 0;
//int leftArm = 0;
bool isServoMicro1 = 0;
bool isServoMicro2 = 0;
bool isServoCamera1 = 0;
bool isServoCamera2 = 0;
Servo ServoMicro1;
Servo ServoMicro2;
Servo ServoCamera1;
Servo ServoCamera2;
int pos = 0;

//=======================================================
//             Button Pin Allocation
//=======================================================

int degree = 0;

bool switchLED;
bool switchStatusLED;
bool switchServoRight;
bool switchServoLeft;

long timerStatusLED = 0;


//=======================================================
//                  Setup Function
//=======================================================

void setup()
{
  
  Serial.println("=== LIVES - CubeSat Demo ==="); 
  Serial.println("Initilize System Variabls"); 
  Serial.begin(9600);

  pinMode(buttonPin01, INPUT);    // Pin 6
  pinMode(buttonPin02, INPUT);    // Pin 7
  pinMode(buttonPin03, INPUT);    // Pin 8
  
  pinMode (statusLEDPin, OUTPUT);   // Pin 13
  pinMode(Pin_Servo_Micro_1, OUTPUT);
  pinMode(Pin_Servo_Micro_2, OUTPUT);
  pinMode(Pin_Servo_Camera_1, OUTPUT);
  pinMode(Pin_Servo_Camera_2, OUTPUT);

  pinMode(2, OUTPUT);    
  pinMode(3, OUTPUT);    
  pinMode(4, OUTPUT);    
  
  digitalWrite(statusLEDPin,LOW);
  digitalWrite(servoPin, LOW);
  digitalWrite(Pin_Servo_Micro_1, LOW);
  digitalWrite(Pin_Servo_Micro_2, LOW);
  digitalWrite(Pin_Servo_Camera_1, LOW);
  digitalWrite(Pin_Servo_Camera_2, LOW);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  
  switchStatusLED = false;
  switchServoLeft = false;
  switchServoRight = false;
  switchLED = false;

  delay(1500);

  //degree = 0;
  
  //*****************************
  //    IMU initialization
  //*****************************
  Serial.println("Initialize IMU..."); 
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  
  //*****************************
  //    LCD initialization
  //*****************************
  Serial.println("Initialize LCD Display..."); 
  lcd.begin(16, 2);
  lcd.print("CubeSat System");

  //*****************************
  //    USS initialization
  //*****************************
  Serial.println("Initialize UltraSonic Sensor..."); 
  pinMode(Pin_USS_Trig, OUTPUT);
  pinMode(Pin_USS_Echo, INPUT);

  //*****************************
  // NeoPixel LED initialization
  //*****************************
  Serial.println("Initialize NeoPixel LED..."); 
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif  
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  //*****************************
  //  Temp Sensor initialization
  //*****************************  
  
  Serial.println("Initialize Temperature Sensor..."); 
  sensors.begin(); 

  //*****************************
  //  Servo Motor initialization
  //*****************************  
  ServoMicro1.attach(Pin_Servo_Micro_1);
  ServoMicro2.attach(Pin_Servo_Micro_2);
  ServoCamera1.attach(Pin_Servo_Camera_1);
  ServoCamera2.attach(Pin_Servo_Camera_2);
}


void threadIMU(){
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
  
  //if ((lastPrint + PRINT_SPEED) < millis())
  //{
    //printGyro();  // Print "G: gx, gy, gz"
    //printAccel(); // Print "A: ax, ay, az"
    //printMag();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    //printAttitude(imu.ax, imu.ay, imu.az, 
    //             -imu.my, -imu.mx, imu.mz);
    //Serial.println();
    
   // lastPrint = millis(); // Update lastPrint time
  //}

  timerIMU = millis();
}
/*
void threadButton(){
  if(digitalRead(buttonPin01) == HIGH || switchServoLeft){
    Serial.println("cL");
    Serial.flush();
    if (degree > 0){ 
      servo.write(degree);
      degree = degree - 1;
    }
    delay(20);
  }
   if(digitalRead(buttonPin02) == HIGH || switchServoRight){
    Serial.println("cR");
    Serial.flush();
    if (degree < 180){
      servo.write(degree);
      degree = degree + 1;
    }
    delay(20);
  }

}
*/
void threadLED(){
  if (switchStatusLED){
    digitalWrite(statusLEDPin, HIGH); 
    Serial.print("LED ON");
  }else{
    digitalWrite(statusLEDPin, LOW); 
    Serial.print("LED OFF");
  }
  switchStatusLED = !switchStatusLED;
  timerStatusLED = millis();
}

void loop(){
  
  //threadButton();

  threadSerial();

  threadNeoPixel();

  //if((millis()-timerStatusLED)>= 300) threadLED();
  

  //for (int thisPin = 2; thisPin < 5; thisPin++) {
  //  if (switchLED){
  //    digitalWrite(thisPin, HIGH);
  //  }else{
  //    digitalWrite(thisPin, LOW);
  //   }
  //}

  //=========================
  //     IMU Function
  //=========================
  if((millis()-timerIMU)>=20)  threadIMU();
  
}

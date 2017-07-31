#define is_IMU_enabled false
#define is_LCD_enabled true
#define is_camera_enabled true
#define is_cameraMode_motionDetect false
#define is_servo_enabled true
#define is_LCD_autoDisplay false

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

#include <Adafruit_VC0706.h>    // Camera - VC0706
#include <SPI.h>                // Camera - VC0706
#include <SD.h>                 // Camera - VC0706

//=======================================================
//   CubeSat PIN Allocations
//=======================================================

#define statusLEDPin 13
#define buttonPin01 6
#define buttonPin02 11
#define buttonPin03 5

#define Pin_NeoPixel_LED 34
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
#define Pin_Servo_Camera_1 8
#define Pin_Servo_Camera_2 9






int sysTime = 0;



//=======================================================
//   LCD Defintion and Variables
//=======================================================

LiquidCrystal lcd(Pin_LCD_1,Pin_LCD_2,Pin_LCD_3,Pin_LCD_4,Pin_LCD_5,Pin_LCD_6);
int LCDcounter = 300;
char toPrint[16];

//=======================================================
//   NeoPixel LED Defintion and Variables
//=======================================================

#define numOfStrips 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numOfStrips, Pin_NeoPixel_LED, NEO_GRB + NEO_KHZ800);

int LEDcounter = 0;

long LEDTime;
String LEDmode = "";
int LEDsafeCycles = 0;
float brightness = 1.0;

//=======================================================
//   Temperature Sensor Defintion and Variables
//=======================================================
OneWire oneWire(Pin_TempSensor_OneWireBus); 
DallasTemperature sensors(&oneWire);

float currentTemp;

//=======================================================
//   IMU Defintion and Variables
//=======================================================
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


//=======================================================
//   Camera VC0706 Defintion and Variables
//=======================================================

// SD card chip select line varies among boards/shields:
// Adafruit SD shields and modules: pin 10
// Arduino Ethernet shield: pin 4
// Sparkfun SD shield: pin 8
// Arduino Mega w/hardware SPI: pin 53
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
#define chipSelect 53

// Pins for camera connection are configurable.
// With the Arduino Uno, etc., most pins can be used, except for
// those already in use for the SD card (10 through 13 plus
// chipSelect, if other than pin 10).
// With the Arduino Mega, the choices are a bit more involved:
// 1) You can still use SoftwareSerial and connect the camera to
//    a variety of pins...BUT the selection is limited.  The TX
//    pin from the camera (RX on the Arduino, and the first
//    argument to SoftwareSerial()) MUST be one of: 62, 63, 64,
//    65, 66, 67, 68, or 69.  If MEGA_SOFT_SPI is set (and using
//    a conventional Arduino SD shield), pins 50, 51, 52 and 53
//    are also available.  The RX pin from the camera (TX on
//    Arduino, second argument to SoftwareSerial()) can be any
//    pin, again excepting those used by the SD card.
// 2) You can use any of the additional three hardware UARTs on
//    the Mega board (labeled as RX1/TX1, RX2/TX2, RX3,TX3),
//    but must specifically use the two pins defined by that
//    UART; they are not configurable.  In this case, pass the
//    desired Serial object (rather than a SoftwareSerial
//    object) to the VC0706 constructor.

// Using hardware serial on Mega: camera TX conn. to RX1,
// camera RX to TX1, no SoftwareSerial object is required:
//Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial3);

char filename[13];
int32_t elapsedTime;
uint16_t jpglen;
//=======================================================
//   Serial Port Defintion and Variables
//=======================================================

byte incomingByte;  
long inputTime = 0;

//=======================================================
//   Servo Motors Defintion and Variables
//=======================================================

bool isServoMicro1 = 0;
bool isServoMicro2 = 0;
bool isServoCamera1 = 0;
bool isServoCamera2 = 0;
Servo ServoMicro1;
Servo ServoMicro2;
Servo ServoCamera1;
Servo ServoCamera2;
int ServoCamera1pos = 0;
int ServoCamera2pos = 0;
int pos = 0;

//=======================================================
//             Button Pin Allocation
//=======================================================

bool switchServoRight;
bool switchServoLeft;

//=======================================================
//             USS Variables
//=======================================================
long USS_distance;
long USS_duration;
//=======================================================
//                  Setup Function
//=======================================================

void setup()
{
  Serial.println("=== LIVES - CubeSat Demo ==="); 
  Serial.println("Initilize System Variabls"); 
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(38400);
  
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
  //digitalWrite(Pin_Servo_Micro_1, LOW);
  //digitalWrite(Pin_Servo_Micro_2, LOW);
  //digitalWrite(Pin_Servo_Camera_1, LOW);
  //digitalWrite(Pin_Servo_Camera_2, LOW);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  
  switchServoLeft = false;
  switchServoRight = false;
  //switchLED = false;

  ServoCamera1pos = 90;
  ServoCamera2pos = 110;
  
  delay(1500);
  
  //*****************************
  //    IMU initialization
  //*****************************
  
  /*
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
  */
  
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
  //ServoMicro1.attach(Pin_Servo_Micro_1);
  //ServoMicro2.attach(Pin_Servo_Micro_2);
  //ServoCamera1.attach(Pin_Servo_Camera_1);
  //ServoCamera2.attach(Pin_Servo_Camera_2);

  //*****************************
  //  Camera VO0706 initialization
  //*****************************  

  // When using hardware SPI, the SS pin MUST be set to an
  // output (even if not connected or used).  If left as a
  // floating input w/SPI on, this can cause lockuppage.
//#if !defined(SOFTWARE_SPI)
//#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//  if(chipSelect != 53) pinMode(53, OUTPUT); // SS on Mega
//#else
//  if(chipSelect != 10) pinMode(10, OUTPUT); // SS on Uno, etc.
//#endif
//#endif
  Serial.println("VC0706 Camera test");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }  
  
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }

  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  
  //cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  cam.setImageSize(VC0706_160x120);          // small

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");


  //  Motion detection system can alert you when the camera 'sees' motion!
  cam.setMotionDetect(is_cameraMode_motionDetect);           // turn it on
  //cam.setMotionDetect(false);        // turn it off   (default)

  // You can also verify whether motion detection is active!
  Serial.print("Motion detection is ");
  if (cam.getMotionDetect()) 
    Serial.println("ON");
  else 
    Serial.println("OFF");
  
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

void loop(){
  
  threadSerial();

  threadNeoPixel();

  //=========================
  //     IMU Function
  //=========================
  //if((millis()-timerIMU)>=20)  threadIMU();
  
}

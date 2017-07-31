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

#define LEDPin 6
#define Pin_USS_Trig 25
#define Pin_USS_Echo 26
#define Pin_TempSensor_OneWireBus 2 


#define Pin_LCD_1 41
#define Pin_LCD_2 40
#define Pin_LCD_3 39
#define Pin_LCD_4 31
#define Pin_LCD_5 30
#define Pin_LCD_6 29




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
//******************************************
//   Temperature Sensor Defintion and Variables
//******************************************
OneWire oneWire(Pin_TempSensor_OneWireBus); 
DallasTemperature sensors(&oneWire);

//******************************************
//   IMU Defintion and Variables
//******************************************
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer


// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw



float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timerIMU=0;   //general purpuse timer
long timerIMU_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};



//******************************************
//   Serial Port Defintion and Variables
//******************************************
int incomingByte = 0;  
bool AutoDisplay = false;
long inputTime = 0;

//******************************************
//   Servo Motors Defintion and Variables
//******************************************
Servo servo;
int rightArm = 0;
int leftArm = 0;

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
  pinMode(servoPin, OUTPUT);

  pinMode(2, OUTPUT);    
  pinMode(3, OUTPUT);    
  pinMode(4, OUTPUT);    
  
  I2C_Init();  //INPUT & OUTPUT

  digitalWrite(statusLEDPin,LOW);
  digitalWrite(servoPin, LOW);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  
  switchStatusLED = false;
  switchServoLeft = false;
  switchServoRight = false;
  switchLED = false;

  delay(1500);

  servo.attach(servoPin);
  degree = 0;
  
  //*****************************
  //    IMU initialization
  //*****************************
  Serial.println("Initialize IMU..."); 
  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);

  timerStatusLED = millis();
  timerIMU = millis();
  delay(20);
  counter=0;

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
  
}


void threadIMU(){
  counter++;
  timerIMU_old = timerIMU;
  timerIMU=millis();
  if (timerIMU>timerIMU_old){
    G_Dt = (timerIMU-timerIMU_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    if (G_Dt > 0.2) G_Dt = 0; // ignore integration times over 200 ms
  }
  else G_Dt = 0;
  // *** DCM algorithm
  // Data adquisition
  Read_Gyro();   // This read gyro data
  Read_Accel();     // Read I2C accelerometer
  if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
  {
    counter=0;
    Read_Compass();    // Read I2C magnetometer
    Compass_Heading(); // Calculate magnetic heading
  }
  // Calculations...
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();
  
}

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

  if(digitalRead(buttonPin03) == HIGH){
    printdata();
  }
}



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
  
  threadButton();

  threadSerial();

  if((millis()-timerStatusLED)>= 300) threadLED();
  

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

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#include <LiquidCrystal.h>
int incomingByte = 0;  

int temperature = -200;
int rightArm = 0;
int leftArm = 0;
int LCDcounter = 300;
char toPrint[16];
long inputTime;
#define LEDPin 6
#define USStrigPin 40
#define USSechoPin 41

LiquidCrystal lcd(53,52,51,50,49,48);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, LEDPin, NEO_GRB + NEO_KHZ800);

void setup() {
  lcd.begin(16, 2);
  lcd.print("CubeSat System");
  Serial.begin(9600);
  pinMode(USStrigPin, OUTPUT);
  pinMode(USSechoPin, INPUT);
  
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif  
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);

  if (Serial.available()>0) {
    
    incomingByte = Serial.read();
    Serial.print("I received: ");
    Serial.print(incomingByte);
    Serial.print("-");
    Serial.println(incomingByte, DEC);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    
    if (incomingByte == 49){
        char toPrint[16] = "Hello World!";
        inputTime = millis();
        lcd.print(toPrint);
    }
    else if (incomingByte == 50){
        char toPrint[16] = "Right Arm ON";
        inputTime = millis();
        lcd.print(toPrint);
    }
    else if (incomingByte == 51){
        char toPrint[16] = "Left Arm ON";
        inputTime = millis();
        lcd.print(toPrint);
    }else if (incomingByte == 52){
        long duration, distance;
        digitalWrite(USStrigPin, LOW);  
        delayMicroseconds(2); 
        digitalWrite(USStrigPin, HIGH);
        delayMicroseconds(10); 
        digitalWrite(USStrigPin, LOW);
        duration = pulseIn(USSechoPin, HIGH);
        distance = (duration/2) / 29.1;
        if (distance >= 200 || distance <= 0){
          Serial.println("Out of range");
        }
        else {
          Serial.print(distance);
          Serial.println(" cm");
        }
        inputTime = millis();
        lcd.print(distance);
    }else{
        char toPrint[16] = "No such command";
        inputTime = millis();
        lcd.print(toPrint);
    }
    if (millis() - inputTime > 2000){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("CubeSat System");
    }
  }
  

  
  //colorWipe(strip.Color(255, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255), 50); // Blue
  //colorWipe(strip.Color(0, 0, 0, 255), 50); // White RGBW
  //theaterChase(strip.Color(127, 127, 127), 50); // White
  //theaterChase(strip.Color(127, 0, 0), 50); // Red
  //theaterChase(strip.Color(0, 0, 127), 50); // Blue

  rainbow(4);
  //rainbowCycle(20);
  //theaterChaseRainbow(50);
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

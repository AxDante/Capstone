typedef union {
  struct {
    byte func;
    unsigned long id;
  } addr;
  byte asBytes[5];
} SatID_type;
/*=====================================================================*/
/* Do not edit above this line! */
/*=====================================================================*/
// Set your unique satellite ID here
SatID_type SatID = { .addr = {0, 123456789} };

// Special functions available to you
void startPreCharge(void);
void stopPreCharge(void);
void deployAntenna(byte num);
bool checkAntenna(byte num);

// This function continuously executes before the mission starts
// Write code you would want to test here
void debug_loop(void) {

}

// This function continously executes during the mission launch
// Generally you do nothing here unless you want to log launch data
void launch_loop(void) {

}

// This function continuously executes after the satellite reaches orbit
// Write code for calibration, consistency check and antenna release here
void deploy_loop(void) {
  int i;
  static unsigned long start_charge_time = 0;
  static bool antenna_charging = 0;
  for (i = 1; i <= 2; i++) {
    if (!checkAntenna(i) && !antenna_charging) {
      startPreCharge();
      start_charge_time = millis();
      antenna_charging = 1;
      break;
    }
  }

  if (antenna_charging && (millis() - start_charge_time > 30000)) {
    stopPreCharge();
    for (i = 1; i <= 2; i++) {
      if (!checkAntenna(i)) {
        deployAntenna(i);
        break;
      }
    }
    antenna_charging = 0;
  }
}

// This function continuously executes after the antennas are released
// Write code that does most of the work that you wish the satellite to carry out
void idle_loop(void) {

}

/*=====================================================================*/
/* Do not edit below this line! */
/*=====================================================================*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Adafruit_VC0706.h>
#include "printf.h"
#include <TimerThree.h>

#define PB_DEPLOY 36
#define PB_USR1 35
#define PB_USR2 34
#define PB_USR3 33
#define PB_USR4 32

#define LED_BOOT 22
#define LED_LINK 23
#define LED_DEPL 24
#define LED_TX 25
#define LED_RX 26
#define LED_ERR 27
#define LED_USR1 28
#define LED_USR2 29
#define LED_USR3 30
#define LED_USR4 31

#define CHARGE_CTRL 13
#define ANT_ONE 4
#define ANT_TWO 5
#define ANT_ONE_SENSE 6
#define ANT_TWO_SENSE 7

#define RAD_CE A7
#define RAD_CSN A6

// Hardware configuration
RF24 radioSat(RAD_CE, RAD_CSN);
volatile unsigned long rflink_last_seen = 0;

byte GSID[5] = {0xDE, 0xED, 0xCA, 0xFE, 0x55};
byte GSBC[5] = {0xDE, 0xED, 0xCA, 0xFE, 0x55};

typedef union {
  struct {
    byte type;
    byte data[31];
    byte len;
  } dat;
  byte asBytes[32];
} radio_pkt;

#define PKT_TIME_SYNC     0
#define PKT_LAUNCH        1
#define PKT_DEPLOY        2
#define PKT_TELEREQUEST   3 // This command needs AOS
#define PKT_TELECOMMAND   4 // This command needs AOS
#define PKT_IMG_DATA      5 // This command needs AOS
#define PKT_STATE_RESTORE 6
#define PKT_PING          7

byte radio_pkt_baselen[] =
{
  5,
  5,
  5,
  7,
  6,
  8,
  9,
  6,
};

// Telecommands
#define TC_SNAP           0

byte tc_datalen[] =
{
  1,
};

// Telerequests
#define TR_SEND_IMG_DATA  0
#define TR_STATE_RESTORE  1

byte tr_datalen[] =
{
  2,
  0,
};

// Remember all offsets are with reference to mission time
typedef union {
  struct {
    byte period;
    byte deploy_offset;
    byte gs_offset;
    byte gs_window;
  } dat;
  byte asBytes[4];
} sat_orbit_type;
volatile sat_orbit_type orbit = { .dat = {0, 0, 0, 0} };

#define MISSION_TIME_INVALID 4294967295
#define DEPLOYMENT_TIME_INVALID 4294967295
volatile unsigned long mission_start_time = MISSION_TIME_INVALID;
volatile long time_offset = 0;

typedef enum {
  STATE_DEBUG,
  STATE_SLEEP,
  STATE_DEPLOYED,
  STATE_IDLE,
  STATE_MISSION,
  STATE_GROUNDSTATION,
} sat_state_type;

volatile sat_state_type sat_state = STATE_DEBUG;

#define IMG_160X120 0
#define IMG_320X240 1
#define IMG_640X480 2
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial3);
volatile unsigned int last_start_point = 65535;
volatile unsigned int img_size = 0;
volatile byte img_type = IMG_160X120;
volatile bool snapshot = false;
volatile bool senddata = false;
volatile unsigned int req_start_point = 0;
volatile byte buffered_img_dat[25] = { 0 };

#define MAX_TIMED_TASKS 10
typedef struct {
  unsigned long deadline;
  byte cmd;
  byte param[10];
  bool enabled;
} timed_task;
volatile timed_task timed_tasks[MAX_TIMED_TASKS] = { 0 };

#define NOMINAL_PING_TIME 13000L
#define NOMINAL_PING_RANGE 2000L
unsigned long next_ping = NOMINAL_PING_TIME;

/* Antenna Sim */
//bool antennas[3] = {false, false, false};
byte chargeLevel = 0;
unsigned long charge_start_time = 0;
bool charge_in_progress = false;

void startPreCharge(void) {
  if (sat_state != STATE_DEPLOYED || charge_in_progress) {
    charge_in_progress = false;
    return;
  }
  Serial.println("Charging capacitor");
  charge_in_progress = true;
  charge_start_time = millis();
}

void stopPreCharge(void) {
  if (sat_state != STATE_DEPLOYED || !charge_in_progress || !charge_in_progress) {
    charge_in_progress = false;
    return;
  }
  unsigned long charge_time_now = millis();
  charge_in_progress = false;
  if (charge_time_now > charge_start_time) {
    if ((((charge_time_now - charge_start_time) / 1000) + chargeLevel) <= 255) {
      chargeLevel += (charge_time_now - charge_start_time) / 1000;
    } else {
      chargeLevel = 255;
    }
  } else {
    chargeLevel = 0;
  }
  Serial.print("Stopped charge, cumulative charge at ");
  Serial.print(chargeLevel);
  Serial.println(" seconds");
}

void deployAntenna(byte num) {
  if (sat_state != STATE_DEPLOYED) {
    charge_in_progress = false;
    chargeLevel = 0;
    return;
  }
  if (charge_in_progress) {
    Serial.println("Short circuit detected, will not proceed");
    return;
  }
  Serial.println("Deploying... ");
  if (chargeLevel > 20) {
    //    antennas[num] = (random(21, 40) < chargeLevel) ? true : false;
    if (random(21, 40) < chargeLevel) {
      Serial.println("Success!");
      if (num == 1) {
        digitalWrite(ANT_ONE, HIGH);
        delay(500);
        digitalWrite(ANT_ONE, LOW);
        delay(1000);
      } else if (num == 2) {
        digitalWrite(ANT_TWO, HIGH);
        delay(500);
        digitalWrite(ANT_TWO, LOW);
        delay(1000);
      }
    } else {
      Serial.println("Failed!");
    }
  } else {
    Serial.println("Failed!");
  }
  chargeLevel = 0;
  return;
}

bool checkAntenna(byte num) {
  if (num == 1) {
    return digitalRead(ANT_ONE_SENSE);
  } else if (num == 2) {
    return digitalRead(ANT_TWO_SENSE);
  } else {
    return 1;
  }
}
/* End Antenna Sim */

/********************** Setup *********************/
void setup() {
  pinMode(53, OUTPUT);

  pinMode(A7, OUTPUT);
  pinMode(A6, OUTPUT);

  digitalWrite(A7, LOW);
  digitalWrite(A6, HIGH);

  pinMode(PB_DEPLOY, INPUT);
  digitalWrite(PB_DEPLOY, LOW);
  pinMode(PB_USR1, INPUT);
  digitalWrite(PB_USR1, LOW);
  pinMode(PB_USR2, INPUT);
  digitalWrite(PB_USR2, LOW);
  pinMode(PB_USR3, INPUT);
  digitalWrite(PB_USR3, LOW);
  pinMode(PB_USR4, INPUT);
  digitalWrite(PB_USR4, LOW);

  pinMode(LED_BOOT, OUTPUT);
  digitalWrite(LED_BOOT, LOW);
  pinMode(LED_LINK, OUTPUT);
  digitalWrite(LED_LINK, LOW);
  pinMode(LED_DEPL, OUTPUT);
  digitalWrite(LED_DEPL, LOW);
  pinMode(LED_TX, OUTPUT);
  digitalWrite(LED_TX, LOW);
  pinMode(LED_RX, OUTPUT);
  digitalWrite(LED_RX, LOW);
  pinMode(LED_ERR, OUTPUT);
  digitalWrite(LED_ERR, LOW);
  pinMode(LED_USR1, OUTPUT);
  digitalWrite(LED_USR1, LOW);
  pinMode(LED_USR2, OUTPUT);
  digitalWrite(LED_USR2, LOW);
  pinMode(LED_USR3, OUTPUT);
  digitalWrite(LED_USR3, LOW);
  pinMode(LED_USR4, OUTPUT);
  digitalWrite(LED_USR4, LOW);

  pinMode(CHARGE_CTRL, OUTPUT);
  digitalWrite(CHARGE_CTRL, HIGH);
  pinMode(ANT_ONE, OUTPUT);
  digitalWrite(ANT_ONE, LOW);
  pinMode(ANT_TWO, OUTPUT);
  digitalWrite(ANT_TWO, LOW);
  pinMode(ANT_ONE_SENSE, INPUT);
  digitalWrite(ANT_ONE_SENSE, LOW);
  pinMode(ANT_TWO_SENSE, INPUT);
  digitalWrite(ANT_TWO_SENSE, LOW);

  randomSeed(analogRead(0));

  Timer3.initialize(500000);

  Serial.begin(115200);
  printf_begin();
  Serial.print(F("Satellite: "));
  Serial.println(SatID.addr.id);

  if (cam.begin()) {
    Serial.println(F("Camera Found:"));
  } else {
    Serial.println(F("No camera found!"));
    return;
  }

  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print(F("Failed to get camera version"));
  } else {
    Serial.println(F("-----------------"));
    Serial.print(reply);
    Serial.println(F("-----------------"));
  }

  // Setup and configure rf radio
  radioSat.begin();
  radioSat.setPALevel(RF24_PA_MAX);
  radioSat.setDataRate(RF24_2MBPS);
  //radioSat.enableAckPayload();
  radioSat.enableDynamicPayloads();
  radioSat.enableDynamicAck();

  radioSat.maskIRQ(1, 0, 0);

  radioSat.openReadingPipe(1, SatID.asBytes);
  radioSat.openReadingPipe(0, GSBC);
  radioSat.openWritingPipe(GSID);

  radioSat.startListening();
  radioSat.printDetails();                             // Dump the configuration of the rf unit for debugging
  delay(50);
  attachInterrupt(0, check_radioSat, LOW);             // Attach interrupt handler to interrupt #0 (using pin 2) on BOTH the sender and receiver

  Timer3.attachInterrupt(timed_command_task);

  send_telerequest(TR_STATE_RESTORE, NULL);
}

/********************** Main Loop *********************/
void loop() {
  byte* imgPtr;
  byte len;
  byte i;
  radio_pkt pkt;
  unsigned long curr_time = millis();

  if (snapshot) {
    snapshot = false;

    cam.resumeVideo();

    switch (img_type)
    {
      case IMG_160X120:
        cam.setImageSize(VC0706_160x120);
        break;

      case IMG_320X240:
        cam.setImageSize(VC0706_320x240);
        break;

      case IMG_640X480:
        cam.setImageSize(VC0706_640x480);
        break;

      default:
        Serial.print(F("Error setting image size! Param is "));
        Serial.println(img_type);
        break;
    }

    delay(300);
    cam.reset();
    delay(300);

    if (!cam.takePicture())
      Serial.println(F("Failed to capture!"));
    else
    {
      // Get the size of the image (frame) taken
      img_size = cam.frameLength();

      Serial.print(F("Snapshot taken at "));
      Serial.print(get_mission_time());
      Serial.print(F(" with setting "));
      Serial.print(img_type);
      Serial.print(F(" file size "));
      Serial.println(img_size);
    }
  }

  if (senddata) {
    senddata = false;
    len = min(24, img_size - req_start_point);

    if (req_start_point != last_start_point)
    {
      imgPtr = cam.readPicture(len);

      for (i = 0; i < len; i++)
      {
        buffered_img_dat[i] = *imgPtr++;
      }
    }
    last_start_point = req_start_point;

    pkt.dat.type = PKT_IMG_DATA;

    pkt.dat.data[0] = SatID.asBytes[1];
    pkt.dat.data[1] = SatID.asBytes[2];
    pkt.dat.data[2] = SatID.asBytes[3];
    pkt.dat.data[3] = SatID.asBytes[4];

    pkt.dat.data[4] = highByte(req_start_point);
    pkt.dat.data[5] = lowByte(req_start_point);

    pkt.dat.data[6] = len;

    for (i = 0; i < len; i++)
    {
      //      if (buffered_img_dat[i] < 0x0F)Serial.print(F("0"));
      //      Serial.print(buffered_img_dat[i], HEX);
      pkt.dat.data[7 + i] = buffered_img_dat[i];
    }
    Serial.println();

    pkt.dat.len = radio_pkt_baselen[pkt.dat.type] + len;

    radioSat.stopListening();
    radioSat.write(pkt.asBytes, pkt.dat.len, 1);  // Was 0
    radioSat.startListening();
    Serial.print(F("Sending "));
    Serial.print(len);
    Serial.print(F(" bytes "));
    Serial.print(req_start_point);
    Serial.println(F(" offset"));
  }

  if (next_ping <= curr_time) {
    ping_gs();
    next_ping = (((((curr_time + (NOMINAL_PING_RANGE / 2)) /  NOMINAL_PING_TIME) + 1) * NOMINAL_PING_TIME) - (NOMINAL_PING_RANGE / 2)) + random(NOMINAL_PING_RANGE + 1);
    Serial.print(F("Next ping at "));
    Serial.println(next_ping);
  }

  switch (sat_state) {
    case STATE_DEBUG:
      debug_loop();
      break;
    case STATE_SLEEP:
      launch_loop();
      break;
    case STATE_DEPLOYED:
      deploy_loop();
      break;
    case STATE_MISSION:
    case STATE_GROUNDSTATION:
    case STATE_IDLE:
      idle_loop();
      break;
  }

  if (sat_state == STATE_DEPLOYED && checkAntenna(1) && checkAntenna(2)) {
    Serial.println("Antennas deployed!");
    stopPreCharge();
    sat_state = STATE_IDLE;
  }
}

/********************** Interrupt *********************/
void check_radioSat(void)
{
  bool tx, fail, rx;
  byte buf[32];
  byte* bufPtr = buf;
  byte temp[4];
  byte i;
  unsigned long temp_time;
  volatile timed_task* new_task;
  byte len = 0;

  radioSat.whatHappened(tx, fail, rx);                   // What happened?
  //radioSat.startListening();

  if ( tx ) {                                         // Have we successfully transmitted?
    radioSat.startListening();
    //Serial.println(F("Send:OK"));
  }

  if ( fail ) {                                       // Have we failed to transmit?
    radioSat.startListening();
    //Serial.println(F("Send:Failed"));
  }

  while ( rx || radioSat.available() ) {                     // Did we receive a message?
    rx = false;
    len = radioSat.getDynamicPayloadSize();
    if (!len) continue;
    radioSat.read(buf, len);
    switch (*bufPtr++)
    {
      case PKT_TIME_SYNC:
        rflink_last_seen = millis();

        temp[0] = *bufPtr++;
        temp[1] = *bufPtr++;
        temp[2] = *bufPtr++;
        temp[3] = *bufPtr++;
        temp_time = ((unsigned long)word(temp[0], temp[1]) << 16) | word(temp[2], temp[3]);

        sync_time(temp_time);

        Serial.print(F("Wall: "));
        Serial.print(get_time());
        Serial.print(F("\tMission: "));
        Serial.print(get_mission_time());
        Serial.print(F("\tOffset: "));
        Serial.println(time_offset);
        break;

      case PKT_LAUNCH:
        rflink_last_seen = millis();

        temp[0] = *bufPtr++;
        temp[1] = *bufPtr++;
        temp[2] = *bufPtr++;
        temp[3] = *bufPtr++;
        temp_time = ((unsigned long)word(temp[0], temp[1]) << 16) | word(temp[2], temp[3]);

        mission_start_time = temp_time;
        sat_state = STATE_SLEEP;
        Serial.println(F("Liftoff!"));
        break;

      case PKT_DEPLOY:
        rflink_last_seen = millis();

        orbit.asBytes[0] = *bufPtr++;
        orbit.asBytes[1] = *bufPtr++;
        orbit.asBytes[2] = *bufPtr++;
        orbit.asBytes[3] = *bufPtr++;
        sat_state = STATE_DEPLOYED;
        Serial.println(F("Ejected!"));
        break;

      case PKT_TELEREQUEST:
        rflink_last_seen = millis();
        bufPtr += 5;

        switch (*bufPtr++)
        {
          case TR_SEND_IMG_DATA:
            if (img_size)
            {
              temp[1] = *bufPtr++;
              temp[0] = *bufPtr++;
              req_start_point = word(temp[0], temp[1]);
              senddata = true;
            }
            break;
        }
        break;

      case PKT_TELECOMMAND:
        rflink_last_seen = millis();

        temp[0] = *bufPtr++;
        temp[1] = *bufPtr++;
        temp[2] = *bufPtr++;
        temp[3] = *bufPtr++;
        temp_time = ((unsigned long)word(temp[0], temp[1]) << 16) | word(temp[2], temp[3]);

        switch (*bufPtr++)
        {
          case TC_SNAP:
            len = *bufPtr++;  // Size of image

            new_task = NULL;
            for (i = 0; i < MAX_TIMED_TASKS; i++)
            {
              if (!timed_tasks[i].enabled)
              {
                new_task = &(timed_tasks[i]);
                break;
              }
            }

            if (new_task == NULL)
            {
              Serial.println(F("No slot for new timed task!"));
              break;
            }

            new_task->deadline = temp_time;
            new_task->cmd = TC_SNAP;
            new_task->param[0] = len;
            new_task->enabled = true;

            Serial.print(F("Scheduled snapshot at "));
            Serial.print(temp_time);
            Serial.print(F(" with setting "));
            Serial.println(len);
            break;
        }
        break;

      case PKT_STATE_RESTORE:
        rflink_last_seen = millis();

        temp[0] = *bufPtr++;
        temp[1] = *bufPtr++;
        temp[2] = *bufPtr++;
        temp[3] = *bufPtr++;
        temp_time = ((unsigned long)word(temp[0], temp[1]) << 16) | word(temp[2], temp[3]);

        mission_start_time = temp_time;

        if (mission_start_time != MISSION_TIME_INVALID)
        {
          sat_state = STATE_SLEEP;
          Serial.println(F("Liftoff!"));

          orbit.asBytes[0] = *bufPtr++;
          orbit.asBytes[1] = *bufPtr++;
          orbit.asBytes[2] = *bufPtr++;
          orbit.asBytes[3] = *bufPtr++;

          if (orbit.dat.gs_window)
          {
            sat_state = STATE_DEPLOYED;
            Serial.println(F("Ejected!"));
          }
        }
        break;

      case PKT_PING:
        rflink_last_seen = millis();
        Serial.println(F("Pinged!"));
        ping_gs();
        break;
    }
  }
}

unsigned long get_time(void)
{
  return millis() - time_offset;
}

unsigned long get_mission_time(void)
{
  if (mission_start_time != MISSION_TIME_INVALID)
    return get_time() - mission_start_time;
  else
    return MISSION_TIME_INVALID;
}

void sync_time(unsigned long curr_time)
{
  time_offset = (long long)millis() - curr_time;
}

byte nearest_min(unsigned long milliseconds)
{
  return (milliseconds + (60000 / 2)) / 60000;
}

bool isConnected(unsigned long deployment_time)
{
  unsigned long curr_mission_time = get_mission_time();
  //byte orbit_number = (curr_mission_time - deployment_time) / ((unsigned long)orbit.dat.period * 60000);

  unsigned long sat_time_pos = (curr_mission_time - deployment_time) % ((unsigned long)orbit.dat.period * 60000);

  unsigned long gs_connected_min = ((unsigned long)orbit.dat.gs_offset * 60000) % ((unsigned long)orbit.dat.period * 60000);
  unsigned long gs_connected_max = (gs_connected_min + (unsigned long)orbit.dat.gs_window * 60000) % ((unsigned long)orbit.dat.period * 60000);

  if (gs_connected_max < gs_connected_min)
  {
    //++------++
    //max    min
    if (sat_time_pos < gs_connected_max || sat_time_pos > gs_connected_min)
      return true;
  }
  else
  {
    //---++++---
    //min    max
    if (sat_time_pos < gs_connected_max && sat_time_pos > gs_connected_min)
      return true;
  }

  return false;
}

void timed_command_task(void)
{
  unsigned long curr_mission_time = get_mission_time();

  for (byte i = 0; i < MAX_TIMED_TASKS; i++)
  {
    if (timed_tasks[i].enabled)
    {
      if (timed_tasks[i].deadline <= curr_mission_time)
      {
        switch (timed_tasks[i].cmd)
        {
          case TC_SNAP:
            img_size = 0;
            last_start_point = 65535;

            // Workaround - Somehow Serial3 doesn't work in this interrupt, trigger actual capture in main loop
            img_type = timed_tasks[i].param[0];
            snapshot = true;

            Serial.println(F("Go!"));

            break;
        }
        timed_tasks[i].enabled = false;
      }
    }
  }
}

void send_telerequest(byte req, byte* param) {
  radio_pkt pkt;
  pkt.dat.type = PKT_TELEREQUEST;

  pkt.dat.data[0] = SatID.asBytes[0];
  pkt.dat.data[1] = SatID.asBytes[1];
  pkt.dat.data[2] = SatID.asBytes[2];
  pkt.dat.data[3] = SatID.asBytes[3];
  pkt.dat.data[4] = SatID.asBytes[4];

  pkt.dat.data[5] = req;

  for (byte i = 0; i < tr_datalen[req]; i++)
  {
    pkt.dat.data[6 + i] = param[i];
  }

  pkt.dat.len = radio_pkt_baselen[pkt.dat.type] + tr_datalen[req];

  radioSat.stopListening();
  radioSat.write(pkt.asBytes, pkt.dat.len, 1);  // Was 0
  radioSat.startListening();
}

void ping_gs()
{
  radio_pkt pkt;
  pkt.dat.type = PKT_PING;

  pkt.dat.data[4] = SatID.asBytes[0];
  pkt.dat.data[3] = SatID.asBytes[1];
  pkt.dat.data[2] = SatID.asBytes[2];
  pkt.dat.data[1] = SatID.asBytes[3];
  pkt.dat.data[0] = SatID.asBytes[4];

  pkt.dat.len = radio_pkt_baselen[pkt.dat.type];

  radioSat.stopListening();
  radioSat.write(pkt.asBytes, pkt.dat.len, 1);
  radioSat.startListening();
}

/*
  // Cut from here

  // This function continuously executes after the satellite reaches orbit
  // Write code for calibration, consistency check and antenna release here
void deploy_loop(void) {
  int i;
  static unsigned long start_charge_time = 0;
  static bool antenna_charging = 0;
  for (i = 1; i <= 2; i++) {
    if (!checkAntenna(i) && !antenna_charging) {
      startPreCharge();
      start_charge_time = millis();
      antenna_charging = 1;
      break;
    }
  }

  if (antenna_charging && (millis() - start_charge_time > 30000)) {
    stopPreCharge();
    for (i = 1; i <= 2; i++) {
      if (!checkAntenna(i)) {
        deployAntenna(i);
        break;
      }
    }
    antenna_charging = 0;
  }
}
  // Stop cut here
*/

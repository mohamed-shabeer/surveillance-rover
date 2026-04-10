{\rtf1\ansi\ansicpg1252\cocoartf2868
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\margl1440\margr1440\vieww23240\viewh17380\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 // rover_main.ino\
// main firmware for the surveillance retrieval rover\
// runs on mega 2560 \'97 dont try on an uno, ran out of pins halfway thru\
// wiring doc is in shared drive under /hardware/rover_v3/pinout.pdf\
// if you change ANY pin assignments update that doc or i will find you\
\
\
#include <Servo.h>\
#include <Wire.h>\
#include <SPI.h>\
#include <SD.h>\
#include <nRF24L01.h>\
#include <RF24.h>\
\
// \uc0\u9472 \u9472  pin assignments \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
// motors \'97 L298N dual H-bridge, running two of em for 4WD\
#define MOTOR_FL_EN   2    // front left enable (PWM)\
#define MOTOR_FL_IN1  22\
#define MOTOR_FL_IN2  23\
#define MOTOR_FR_EN   3\
#define MOTOR_FR_IN1  24\
#define MOTOR_FR_IN2  25\
#define MOTOR_RL_EN   4\
#define MOTOR_RL_IN1  26\
#define MOTOR_RL_IN2  27\
#define MOTOR_RR_EN   5\
#define MOTOR_RR_IN1  28\
#define MOTOR_RR_IN2  29\
\
// cam pan/tilt\
#define SERVO_PAN_PIN   6\
#define SERVO_TILT_PIN  7\
\
// nRF24L01 \'97 long range module w/ PA+LNA\
#define RF_CE_PIN   48\
#define RF_CSN_PIN  49\
\
// ultrasonics \'97 front, left, right\
// rear one broke during field test #2, been sitting on my desk for weeks\
#define ULTRA_F_TRIG  30\
#define ULTRA_F_ECHO  31\
#define ULTRA_L_TRIG  32\
#define ULTRA_L_ECHO  33\
#define ULTRA_R_TRIG  34\
#define ULTRA_R_ECHO  35\
\
// IR night vision array\
#define IR_LED_PIN    8\
\
// status LED \'97 lifesaver when you cant plug in serial\
#define STATUS_LED    13\
\
// batt voltage divider \'97 5200mAh LiPo\
// R1=30k R2=10k so we get roughly 1/4 of pack voltage into the ADC\
#define VBAT_PIN      A0\
#define VBAT_DIVIDER  4.0\
#define VBAT_LOW      10.8   // 3.6V/cell, time to wrap it up\
#define VBAT_CRIT     10.2   // had a lipo puff on us once. never again\
\
// SD for local footage backup\
#define SD_CS_PIN     53\
\
// \uc0\u9472 \u9472  constants \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
#define MAX_SPEED         255\
#define CRUISE_SPEED      180    // sweet spot btwn speed and not flipping in crawl spaces\
#define SLOW_SPEED        100    // for when youre threading a needle\
#define TURN_SPEED        150\
#define OBSTACLE_DIST_CM  25     // start paying attention\
#define CRIT_DIST_CM      10     // oh shit distance\
\
// radio\
const byte PIPE_ADDR[6] = "RVR01";\
#define RADIO_CHANNEL     108    // avoids most wifi interference, tested this in the field\
#define CMD_TIMEOUT_MS    1500   // no signal for this long = full stop\
\
// servo limits \'97 found these by breaking the first set of gears lol\
// seriously dont go past these\
#define PAN_MIN   20\
#define PAN_MAX   160\
#define PAN_CENTER 90\
#define TILT_MIN  30\
#define TILT_MAX  120\
#define TILT_CENTER 75   // slight downward angle, way better for crawl spaces\
\
// \uc0\u9472 \u9472  data structures \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
// cmd packet from controller \'97 12 bytes\
// keep it small, throughput on nrf isnt amazing\
struct ControlPacket \{\
  int8_t throttle;     // -100 to 100\
  int8_t steering;     // -100 to 100\
  int8_t panDelta;\
  int8_t tiltDelta;\
  uint8_t irToggle;    // 1 = flip IR on/off\
  uint8_t speedMode;   // 0=cruise 1=slow 2=send it\
  uint8_t camRecord;   // 1 = toggle recording\
  uint8_t reserved;    // might use for aux payload later idk\
\};\
\
// telemetry going back \'97 16 bytes\
struct TelemetryPacket \{\
  uint16_t distFront;  // cm\
  uint16_t distLeft;\
  uint16_t distRight;\
  uint16_t battMV;     // millivolts\
  int8_t   panAngle;\
  int8_t   tiltAngle;\
  uint8_t  irState;\
  uint8_t  isRecording;\
  uint8_t  errorFlags;  // bits: 0=low_batt 1=crit_batt 2=sd_fail 3=radio_weak\
  uint8_t  rssi;        // not real rssi, more of a vibe check on signal quality\
\};\
\
// \uc0\u9472 \u9472  globals \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
RF24 radio(RF_CE_PIN, RF_CSN_PIN);\
Servo panServo;\
Servo tiltServo;\
\
ControlPacket cmd;\
TelemetryPacket telem;\
\
int currentPan  = PAN_CENTER;\
int currentTilt = TILT_CENTER;\
bool irState    = false;\
bool isRecording = false;\
bool sdReady    = false;\
\
unsigned long lastCmdTime   = 0;\
unsigned long lastTelemTime = 0;\
unsigned long lastBattCheck = 0;\
unsigned long lastBlink     = 0;\
bool blinkState = false;\
\
// motor ramp vals \'97 cant just slam to full speed, treads skip on smooth floors\
int currentSpeedL = 0;\
int currentSpeedR = 0;\
#define RAMP_STEP  8\
\
// \uc0\u9472 \u9472  motor control \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void setMotor(int enPin, int in1, int in2, int speed) \{\
  if (speed > 0) \{\
    digitalWrite(in1, HIGH);\
    digitalWrite(in2, LOW);\
    analogWrite(enPin, constrain(speed, 0, 255));\
  \} else if (speed < 0) \{\
    digitalWrite(in1, LOW);\
    digitalWrite(in2, HIGH);\
    analogWrite(enPin, constrain(-speed, 0, 255));\
  \} else \{\
    digitalWrite(in1, LOW);\
    digitalWrite(in2, LOW);\
    analogWrite(enPin, 0);\
  \}\
\}\
\
void driveMotors(int leftSpeed, int rightSpeed) \{\
  // front and rear on each side wired parallel-ish\
  // drive em independently tho bc one side always has more friction\
  // on uneven ground and you gotta compensate\
  setMotor(MOTOR_FL_EN, MOTOR_FL_IN1, MOTOR_FL_IN2, leftSpeed);\
  setMotor(MOTOR_RL_EN, MOTOR_RL_IN1, MOTOR_RL_IN2, leftSpeed);\
  setMotor(MOTOR_FR_EN, MOTOR_FR_IN1, MOTOR_FR_IN2, rightSpeed);\
  setMotor(MOTOR_RR_EN, MOTOR_RR_IN1, MOTOR_RR_IN2, rightSpeed);\
\}\
\
void stopAll() \{\
  driveMotors(0, 0);\
  currentSpeedL = 0;\
  currentSpeedR = 0;\
\}\
\
int rampTo(int current, int target) \{\
  // ease into it so we dont jerk around and lose traction\
  if (current < target) return min(current + RAMP_STEP, target);\
  if (current > target) return max(current - RAMP_STEP, target);\
  return current;\
\}\
\
// \uc0\u9472 \u9472  ultrasonic \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
long readUltrasonic(int trigPin, int echoPin) \{\
  // standard HC-SR04 deal\
  // cap at 300cm bc past that its just making stuff up\
  digitalWrite(trigPin, LOW);\
  delayMicroseconds(2);\
  digitalWrite(trigPin, HIGH);\
  delayMicroseconds(10);\
  digitalWrite(trigPin, LOW);\
\
  long duration = pulseIn(echoPin, HIGH, 25000);  // 25ms timeout ~4.25m\
  if (duration == 0) return 300;  // nothing in range\
\
  long cm = duration * 0.034 / 2;\
  return constrain(cm, 0, 300);\
\}\
\
void updateDistances() \{\
  // stagger these readings a bit \'97 fire all 3 at once and they\
  // crosstalk and give you nonsense. spent 2 days debugging that\
  telem.distFront = readUltrasonic(ULTRA_F_TRIG, ULTRA_F_ECHO);\
  delay(5);\
  telem.distLeft  = readUltrasonic(ULTRA_L_TRIG, ULTRA_L_ECHO);\
  delay(5);\
  telem.distRight = readUltrasonic(ULTRA_R_TRIG, ULTRA_R_ECHO);\
\}\
\
// \uc0\u9472 \u9472  battery monitoring \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
float readBattVoltage() \{\
  // oversample bc the ADC on the mega is noisy as hell\
  long sum = 0;\
  for (int i = 0; i < 16; i++) \{\
    sum += analogRead(VBAT_PIN);\
    delayMicroseconds(100);\
  \}\
  float avg = sum / 16.0;\
  float voltage = (avg / 1023.0) * 5.0 * VBAT_DIVIDER;\
  return voltage;\
\}\
\
void checkBattery() \{\
  float v = readBattVoltage();\
  telem.battMV = (uint16_t)(v * 1000);\
\
  telem.errorFlags &= ~0x03;  // clear batt bits\
\
  if (v <= VBAT_CRIT) \{\
    // nope. full stop. not risking another puffed cell\
    telem.errorFlags |= 0x02;\
    stopAll();\
  \} else if (v <= VBAT_LOW) \{\
    telem.errorFlags |= 0x01;\
    // can still drive, controller gets a heads up tho\
  \}\
\}\
\
// \uc0\u9472 \u9472  camera pan/tilt \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void updateCamera() \{\
  // delta control \'97 each packet nudges position a bit\
  // way smoother than absolute when youre trying to track something\
  // while the whole chassis is bouncing around\
  currentPan  += cmd.panDelta;\
  currentTilt += cmd.tiltDelta;\
\
  currentPan  = constrain(currentPan, PAN_MIN, PAN_MAX);\
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);\
\
  panServo.write(currentPan);\
  tiltServo.write(currentTilt);\
\
  telem.panAngle  = (int8_t)(currentPan - PAN_CENTER);\
  telem.tiltAngle = (int8_t)(currentTilt - TILT_CENTER);\
\}\
\
// \uc0\u9472 \u9472  IR LEDs \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void handleIR() \{\
  if (cmd.irToggle) \{\
    irState = !irState;\
    digitalWrite(IR_LED_PIN, irState ? HIGH : LOW);\
    cmd.irToggle = 0;  // eat the toggle so it doesnt flip every loop\
  \}\
  telem.irState = irState ? 1 : 0;\
\}\
\
// \uc0\u9472 \u9472  SD card \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void initSD() \{\
  if (SD.begin(SD_CS_PIN)) \{\
    sdReady = true;\
    // session folder based on millis \'97 yeah i know its not ideal\
    // TODO: slap a DS3231 on there, been saying this for like 4 months\
  \} else \{\
    sdReady = false;\
    telem.errorFlags |= 0x04;\
  \}\
\}\
\
void logTelemetry() \{\
  // CSV line every 500ms when recording\
  // not great framerate but enough for post-mission stuff\
  if (!sdReady || !isRecording) return;\
\
  File f = SD.open("telem.csv", FILE_WRITE);\
  if (f) \{\
    f.print(millis()); f.print(",");\
    f.print(telem.distFront); f.print(",");\
    f.print(telem.distLeft); f.print(",");\
    f.print(telem.distRight); f.print(",");\
    f.print(telem.battMV); f.print(",");\
    f.print(telem.panAngle); f.print(",");\
    f.print(telem.tiltAngle); f.print(",");\
    f.print(telem.irState); f.print(",");\
    f.println(telem.errorFlags);\
    f.close();\
  \}\
\}\
\
void handleRecording() \{\
  if (cmd.camRecord) \{\
    isRecording = !isRecording;\
    cmd.camRecord = 0;\
  \}\
  telem.isRecording = isRecording ? 1 : 0;\
\}\
\
// \uc0\u9472 \u9472  radio \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void initRadio() \{\
  radio.begin();\
  radio.setChannel(RADIO_CHANNEL);\
  radio.setPALevel(RF24_PA_MAX);        // crank it, we need the range\
  radio.setDataRate(RF24_250KBPS);      // slower but way more reliable at distance\
  radio.setRetries(5, 15);              // 5x250us delay, 15 retries before giving up\
  radio.enableAckPayload();             // sneak telemetry into ACK packets\
  radio.openReadingPipe(1, PIPE_ADDR);\
  radio.startListening();\
\
  // preload first telem so the controller gets something right away\
  memset(&telem, 0, sizeof(telem));\
  radio.writeAckPayload(1, &telem, sizeof(telem));\
\}\
\
bool receiveCommand() \{\
  if (radio.available()) \{\
    radio.read(&cmd, sizeof(cmd));\
    lastCmdTime = millis();\
\
    // stuff next telem into ACK for the NEXT incoming packet\
    // yeah its always one cycle behind but honestly who cares\
    // alternative is a whole separate pipe and its not worth the hassle\
    radio.writeAckPayload(1, &telem, sizeof(telem));\
    return true;\
  \}\
  return false;\
\}\
\
bool isSignalLost() \{\
  return (millis() - lastCmdTime) > CMD_TIMEOUT_MS;\
\}\
\
// \uc0\u9472 \u9472  drive logic \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void processDrive() \{\
  // dead battery or no signal = dont move. end of discussion\
  if ((telem.errorFlags & 0x02) || isSignalLost()) \{\
    stopAll();\
    return;\
  \}\
\
  int baseSpeed;\
  switch (cmd.speedMode) \{\
    case 1:  baseSpeed = SLOW_SPEED;  break;\
    case 2:  baseSpeed = MAX_SPEED;   break;\
    default: baseSpeed = CRUISE_SPEED; break;\
  \}\
\
  // basic tank mixing \'97 throttle + steering\
  // tried a fancier exponential curve thing once, felt awful to drive\
  // simple wins\
  int throttle = map(cmd.throttle, -100, 100, -baseSpeed, baseSpeed);\
  int steering = map(cmd.steering, -100, 100, -baseSpeed, baseSpeed);\
\
  int targetL = constrain(throttle + steering, -MAX_SPEED, MAX_SPEED);\
  int targetR = constrain(throttle - steering, -MAX_SPEED, MAX_SPEED);\
\
  // obstacle override \'97 you can still steer but im not letting you\
  // drive straight into something\
  if (telem.distFront < CRIT_DIST_CM && throttle > 0) \{\
    // something is RIGHT there, hard stop\
    targetL = 0;\
    targetR = 0;\
  \} else if (telem.distFront < OBSTACLE_DIST_CM && throttle > 0) \{\
    // ease off proportionally \'97 closer = slower\
    float factor = (float)(telem.distFront - CRIT_DIST_CM) / (OBSTACLE_DIST_CM - CRIT_DIST_CM);\
    targetL = (int)(targetL * factor);\
    targetR = (int)(targetR * factor);\
  \}\
\
  // wall hugging \'97 nudges away from side walls automatically\
  // this is the thing that actually makes it usable in narrow corridors\
  // before we added this the operator had to micromanage every inch\
  if (telem.distLeft < 15 && telem.distRight > 15) \{\
    targetL += 30;  // push right\
    targetR -= 30;\
  \} else if (telem.distRight < 15 && telem.distLeft > 15) \{\
    targetL -= 30;  // push left\
    targetR += 30;\
  \}\
\
  currentSpeedL = rampTo(currentSpeedL, targetL);\
  currentSpeedR = rampTo(currentSpeedR, targetR);\
\
  driveMotors(currentSpeedL, currentSpeedR);\
\}\
\
// \uc0\u9472 \u9472  status LED \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void updateStatusLED() \{\
  unsigned long now = millis();\
\
  if (telem.errorFlags & 0x02) \{\
    // crit battery \'97 fast blink, get this thing back NOW\
    if (now - lastBlink > 100) \{\
      blinkState = !blinkState;\
      digitalWrite(STATUS_LED, blinkState);\
      lastBlink = now;\
    \}\
  \} else if (isSignalLost()) \{\
    // lost radio \'97 slow blink so you know its alive but confused\
    if (now - lastBlink > 500) \{\
      blinkState = !blinkState;\
      digitalWrite(STATUS_LED, blinkState);\
      lastBlink = now;\
    \}\
  \} else if (telem.errorFlags & 0x01) \{\
    // low batt \'97 double blink pattern\
    // kinda janky but the detectives can tell whats going on from\
    // across the room and thats all that matters\
    unsigned long phase = (now / 150) % 6;\
    digitalWrite(STATUS_LED, (phase == 0 || phase == 2) ? HIGH : LOW);\
  \} else \{\
    // were good, solid on\
    digitalWrite(STATUS_LED, HIGH);\
  \}\
\}\
\
// \uc0\u9472 \u9472  setup \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void setup() \{\
  Serial.begin(115200);  // debug only \'97 yank this cable before going in the field\
  Serial.println(F("rover_main v3.1 \'97 booting"));\
\
  // motor pins \'97 yeah theres a lot of em\
  int motorPins[] = \{\
    MOTOR_FL_EN, MOTOR_FL_IN1, MOTOR_FL_IN2,\
    MOTOR_FR_EN, MOTOR_FR_IN1, MOTOR_FR_IN2,\
    MOTOR_RL_EN, MOTOR_RL_IN1, MOTOR_RL_IN2,\
    MOTOR_RR_EN, MOTOR_RR_IN1, MOTOR_RR_IN2\
  \};\
  for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);\
\
  // ultrasonics\
  int trigPins[] = \{ULTRA_F_TRIG, ULTRA_L_TRIG, ULTRA_R_TRIG\};\
  int echoPins[] = \{ULTRA_F_ECHO, ULTRA_L_ECHO, ULTRA_R_ECHO\};\
  for (int i = 0; i < 3; i++) \{\
    pinMode(trigPins[i], OUTPUT);\
    pinMode(echoPins[i], INPUT);\
  \}\
\
  pinMode(IR_LED_PIN, OUTPUT);\
  pinMode(STATUS_LED, OUTPUT);\
  pinMode(VBAT_PIN, INPUT);\
  digitalWrite(IR_LED_PIN, LOW);\
\
  panServo.attach(SERVO_PAN_PIN);\
  tiltServo.attach(SERVO_TILT_PIN);\
  panServo.write(PAN_CENTER);\
  tiltServo.write(TILT_CENTER);\
\
  // let servos settle or they twitch on boot and its annoying\
  delay(500);\
\
  initSD();\
  initRadio();\
\
  // blink 3x = ready. the detectives watch for this before sending it in\
  for (int i = 0; i < 3; i++) \{\
    digitalWrite(STATUS_LED, HIGH); delay(150);\
    digitalWrite(STATUS_LED, LOW);  delay(150);\
  \}\
\
  memset(&cmd, 0, sizeof(cmd));\
  lastCmdTime = millis();\
\
  Serial.println(F("ready \'97 waiting for controller"));\
\}\
\
// \uc0\u9472 \u9472  main loop \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \u9472 \
\
void loop() \{\
  // radio always comes first, everything else can wait\
  receiveCommand();\
\
  // sensors every 50ms \'97 they aint that fast anyway\
  static unsigned long lastSensorRead = 0;\
  if (millis() - lastSensorRead > 50) \{\
    updateDistances();\
    lastSensorRead = millis();\
  \}\
\
  // batt check every 2s, no point hammering the ADC\
  if (millis() - lastBattCheck > 2000) \{\
    checkBattery();\
    lastBattCheck = millis();\
  \}\
\
  processDrive();\
  updateCamera();\
  handleIR();\
  handleRecording();\
\
  // log to SD every 500ms\
  static unsigned long lastLog = 0;\
  if (millis() - lastLog > 500) \{\
    logTelemetry();\
    lastLog = millis();\
  \}\
\
  updateStatusLED();\
\
  // tiny delay so the loop timing stays kinda consistent\
  // without this PWM gets weird at really high loop rates\
  delay(5);\
\}\
\
\
## Author\
Mohamed Shabeer Fnu}
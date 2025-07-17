#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>    // onboard IMU
#include "Arduino_BMI270_BMM150.h"

// ——— pin definitions ——— all pwm fans so analogwrite can be used. analogwrite is needed for correction
const uint8_t fanPin1 = 6;   
const uint8_t fanPin2 = 7;   
const uint8_t fanPin3 = 8;   
const uint8_t fanPin4 = 9;   
const uint8_t fanPin5 = 10;  
const uint8_t fanPin6 = 11;  

const int baseSpeed      = 150;   // hover PWM (0–255)
const float Kp           = 1.0;  // steering gain
const float Kd = 1.0; //constant damping
const float deadband     =   10.0; // ° within which we do nothing
const float alpha        =  0.90; // complementary filter weight

// ——— State variables ———
float angleGyro  = 0.0;    // integrated from gz
float initialRel = 0.0;    // heading offset at startup
unsigned long prevUs;

//sensor readings
float gx, gy, gz; //gyro axes
float mx, my, mz; // magnetometer axes

// BLE setup
#define SERVICE_UUID  "19B10000-E8F2-537E-4F6C-D104768A1214"
#define CHAR_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"

BLEService    moveService(SERVICE_UUID);
BLECharacteristic dirChar(CHAR_UUID, BLERead | BLEWrite, 1);

// last manual command from BLE (0 = none/off)
uint8_t lastCommand = 0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  // BLE peripheral init
  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1);
  }

  BLE.setLocalName("LEDPeripheral");
  BLE.setAdvertisedService(moveService);
  dirChar.writeValue((uint8_t)0);
  moveService.addCharacteristic(dirChar);
  BLE.addService(moveService);
  BLE.advertise();
  Serial.println("BLE up, advertising...");

  // IMU 
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }
  Serial.println("IMU ready");

  // initialize each pin and zero it
  pinMode(fanPin1, OUTPUT); 
  analogWrite(fanPin1, 0);
  pinMode(fanPin2, OUTPUT); 
  analogWrite(fanPin2, 0);
  pinMode(fanPin3, OUTPUT); 
  analogWrite(fanPin3, 0);
  pinMode(fanPin4, OUTPUT); 
  analogWrite(fanPin4, 0);
  pinMode(fanPin5, OUTPUT); 
  analogWrite(fanPin5, 0);
  pinMode(fanPin6, OUTPUT); 
  analogWrite(fanPin6, 0);

  // — measure your “zero” heading at startup —
  while (!IMU.magneticFieldAvailable());

  IMU.readMagneticField(mx, my, mz);
  float head0 = atan2(my, mx) * RAD_TO_DEG;
  if (head0 < 0) {
    head0 += 360.0;
  }
  initialRel = head0;     // store the absolute north reference
  angleGyro  = 0.0;       // define “0” as pointing toward initialRel
  prevUs     = micros();

  Serial.print("Calibrated zero-heading = ");
  Serial.println(initialRel, 1);

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    float heading = atan2(my, mx) * RAD_TO_DEG; //aten2 gets the tan(y/x) and tells quadrant

    if (heading < 0) 
    {
      heading += 360.0;
    }

    // compute error to north (0°)
    float error = 0.0f - heading; //initial error
    if (error >  180.0f)
    {
      error -= 360.0f;
    } 
    else if (error < -180.0f) 
    {
      error += 360.0f;
    } 

}

void loop() {
  // put your main code here, to run repeatedly:

  // 1) capture any new BLE command
  BLE.poll();
  if (dirChar.written()) {
    lastCommand = dirChar.value()[0]; //the value from the sender
    Serial.print("Manual cmd: ");
    Serial.println(lastCommand);
  }

  

  // if outside threshold, correct rotation
  while (abs(error) > deadband) {
    applyCorrection(error);
    // skip translation until oriented go back to 
  }

  applyTranslationCommand(lastCommand);
  delay(200);
  
}

// ——— translation: drive ONE fan at baseSpeed ———
void applyTranslationCommand(uint8_t code) {
  // zero all fans
  analogWrite(fanPin1, 0);
  analogWrite(fanPin2, 0);
  analogWrite(fanPin3, 0);
  analogWrite(fanPin4, 0);
  analogWrite(fanPin5, 0);
  analogWrite(fanPin6, 0);

  if (code == 0)
  {
    //all fans stay off
  }
  // now turn ON exactly the requested fans
  else if (code == 1) //+X direction
  {
    analogWrite(fanPin3, 255);
    analogWrite(fanPin4, 255);
  }
  else if (code == 2) //-X
  {
    analogWrite(fanPin1, 255);
    analogWrite(fanPin2, 255);
  }
  else if (code == 3) //+Y
  {
    analogWrite(fanPin2, 255);
    analogWrite(fanPin3, 255);
  }
  else if (code == 4) //-Y
  {
    analogWrite(fanPin1, 255);
    analogWrite(fanPin4, 255);
  }
  else if (code == 5) //+Z
  {
    analogWrite(fanPin6, 255);
  }
  else if (code == 6) //-Z
  {
    analogWrite(fanPin5, 255);
  }
  else 
  {
    // invalid codes: leave all OFF
    Serial.print("Invalid code: ");
    Serial.println(code);
    return;
  }

  Serial.print("→ blowing fans ");
  Serial.println(code);
}

void applyCorrection(float error) {

  // — compute dt —
  unsigned long nowUs = micros();
  float dt = (nowUs - prevUs) * 1e-6;
  prevUs = nowUs;

  // 2) integrate gyro rate around Z —
  if (IMU.gyroscopeAvailable()) {
    
    IMU.readGyroscope(gx, gy, gz);
    angleGyro += gz * dt;        // degrees
    // keep it in –180…+180 range to avoid runaway
    if      (angleGyro >  180) angleGyro -= 360;
    else if (angleGyro < -180) angleGyro += 360;
  }

  // 3) read absolute heading from mag —
  float relMag = 0;
  if (IMU.magneticFieldAvailable()) {
    float mx, my, mz;
    IMU.readMagneticField(mx, my, mz);
    float head = atan2(my, mx) * RAD_TO_DEG;
    if (head < 0) head += 360.0;
    // convert to “relative” around initialRel, in –180…+180
    relMag = head - initialRel;
    if      (relMag >  180.0) relMag -= 360.0;
    else if (relMag < -180.0) relMag += 360.0;
  }

  // 4) complementary filter —
  float angle = alpha * angleGyro + (1 - alpha) * relMag;

  
  // 5) compute error → steer fans —
  float error = -angle;  // want angle → 0

  float D     = gz;                    // °/s
  float rawP  = Kp * error;
  float rawD  = Kd * D;

  if (abs(error) < deadband) {
    // inside deadband: nothing
    //analogWrite(FAN_LEFT_PIN,  baseSpeed);
    //analogWrite(FAN_RIGHT_PIN, baseSpeed);
  } 
  else {
    float steer = constrain(int(rawP - rawD), -baseSpeed, +baseSpeed);
    int L = constrain(baseSpeed - steer, 0, 255);
    int R = constrain(baseSpeed + steer, 0, 255);
    analogWrite(fanPin1,  L);
    analogWrite(fanPin3,  L);
    analogWrite(fanPin2, R);
    analogWrite(fanPin4, R);
  }

}

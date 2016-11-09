#include <CurieIMU.h>
#include "MadgwickAHRS.h"
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float vx=0,vy=0,vz=0;
float sx=0,sy=0,sz=0;
float T;
float offSetNorm;
void setup() {
  Serial.begin(115200);
  while(!Serial);
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);
  delay(1000);
  CurieIMU.autoCalibrateGyroOffset();
  //Serial1.print("Starting Acceleration calibration and enabling offset compensation...");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  int axRaw,ayRaw,azRaw,gxRaw,gyRaw,gzRaw;
  for(int i=0;i<50;i++){
      CurieIMU.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);
      offSetNorm += sqrt(axRaw*axRaw + ayRaw*ayRaw + azRaw*azRaw);//单位化加速度计，
  }
  offSetNorm=offSetNorm/50;
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  T=microsPerReading*0.000001;
  microsPrevious = micros();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    float rAx,rAy,rAz;
    float norm = sqrt(aix*aix+aiy*aiy+aiz*aiz);
    ax=aix/offSetNorm;
    ay=aiy/offSetNorm;
    az=aiz/offSetNorm;
    filter.coordinateTransfromationB2R(ax,ay,az,rAx,rAy,rAz);
    norm = sqrt(aix*aix+aiy*aiy+aiz*aiz);
   // Serial.println(abs(norm-offSetNorm));
    if(abs(norm-offSetNorm)>300){//300
        vx = vx + rAx*9.8*T;
        sx = sx + vx*T + 0.5f*rAx*9.8*T*T;
        vy = vy + rAy*9.8*T;
        sy = sy + vy*T + 0.5f*rAy*9.8*T*T;
        vz = vz + (rAz-1)*9.8*T;
        sz = sz + vz*T + 0.5f*(rAz-1)*9.8*T*T;
    }else{
      vx=vy=vz=0;
    }
    // print the heading, pitch and roll
    /*roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();*/
    //Serial.print("Orientation: ");
    Serial.print(sx);
    Serial.print(",");
    Serial.print(sy);
    Serial.print(",");
    Serial.println(sz);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

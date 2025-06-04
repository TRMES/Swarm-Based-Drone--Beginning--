#include "MPU9250.h"
#include "Wire.h"

MPU9250 mpu;

// orientation/motion vars
float ypr[3];
float q[4];

// MPU control/status vars
bool dmpReady = false;
volatile bool mpuInterrupt = false;

void dmpDataReady(){
  mpuInterrupt = true;
}

struct Orientation previousOrientation;
unsigned long last_time = 0;

void initializeIMU(){
  Wire.begin();
  Wire.setClock(400000);

  if (!mpu.setup(0x68)) {  // MPU9250 bağlantı kontrolü
  Serial.println("MPU9250 bağlantı hatası!");
  while (1); // Sonsuz döngüye gir, bağlanamazsa ilerleme
  }

  mpu.setAccBias(ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z);
  mpu.setGyroBias(GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z);

  // Sensör Ayarları
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(4);


  dmpReady = true;
}

struct IMU_Values GetIMUvalues() {
  struct IMU_Values o;
  o.NewDataAvailable = false;

  if(!mpu.isConnected()){
    o.Error = true;
    return o;
  }

  if(!dmpReady){
    return o;
  }
  
  unsigned long current_time = millis();
  unsigned long delta_time_in_milliseconds = current_time - last_time;
  double delta_time_in_seconds = (double)delta_time_in_milliseconds / 1000.0;

  if(mpu.update()){
    q[0] = mpu.getQuaternionW();
    q[1] = mpu.getQuaternionX();
    q[2] = mpu.getQuaternionY();
    q[3] = mpu.getQuaternionZ();

    // Eular açıları
    o.CurrentOrientation.YawAngle = mpu.getYaw();
    o.CurrentOrientation.PitchAngle = mpu.getRoll(); // Donanım Ters Takıldığı için değiştirildi
    o.CurrentOrientation.RollAngle = -mpu.getPitch(); // Donanım Ters Takıldığı için değiştirildi

    /*
    Serial.print("yaw: ");
    Serial.print(o.CurrentOrientation.YawAngle);
    Serial.print(" | pitch: ");
    Serial.print(o.CurrentOrientation.PitchAngle);
    Serial.print(" | roll: ");
    Serial.println(o.CurrentOrientation.RollAngle);
    */

    o.PreviousOrientation = previousOrientation;
    o.NewDataAvailable = true;
    o.DeltaTimeInSeconds = delta_time_in_seconds;

    previousOrientation = o.CurrentOrientation;
    if(last_time == 0){
      last_time = current_time;
      o.Error = true;
      return o;
    }
    last_time = current_time;
  }
  
  if(delta_time_in_milliseconds > IMU_COMMUNICATION_TIMEOUT_IN_MILLISECONDS){
    o.Error = true;
  } else{
    o.Error = false;
  }
  
  return o;
}
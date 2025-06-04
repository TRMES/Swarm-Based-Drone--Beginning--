#include <avr/wdt.h>
void setup() {
  Serial.begin(115200);
  wdt_enable(PROGRAM_TIMEOUT_IN_MILLISECONDS);
  //Motor
  initializeMotors();
  // NRF24
  initializeReceiver();
  // IMU
  initializeIMU();
   
}

void loop() {
  wdt_reset();

  struct ReceiverCommands receiverCommands = GetReceiverCommands();
  struct IMU_Values imu_values = GetIMUvalues();

  if (receiverCommands.Error || receiverCommands.Throttle < THROTTLE_START_POINT  || imu_values.Error)
  {
    stopMotors();
    resetPidVariables();
    return;
  }

  if (imu_values.NewDataAvailable) {
    struct MotorPowers motorPowers = calculateMotorPowers(receiverCommands, imu_values);
    spinMotors(motorPowers);
  }

  
}

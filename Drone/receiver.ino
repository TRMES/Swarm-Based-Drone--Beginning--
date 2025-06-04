#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);
const byte address[6] = "77777";

struct ReceiverCommands prevReceiverCommands;
unsigned long receiver_last_communication_time = millis();

void initializeReceiver(){
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setChannel(110);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}

struct ReceiverCommands GetReceiverCommands(){
  struct ReceiverRawValues receiverRawValues = getReceiverRawValues();
  if(receiverRawValues.TransmitterCommunicationFailure){
    return getFailureReceiverCommand();
  }

  if(receiverRawValues.NewDataAvailable){
    receiver_last_communication_time = millis();

    struct ReceiverCommands cmd;
    cmd.RollAngle = map_double(receiverRawValues.ChannelValues[0], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, -QUADCOPTER_MAX_TILT_ANGLE, QUADCOPTER_MAX_TILT_ANGLE);
    cmd.PitchAngle = map_double(receiverRawValues.ChannelValues[1], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, -QUADCOPTER_MAX_TILT_ANGLE, QUADCOPTER_MAX_TILT_ANGLE);
    cmd.Throttle = map_double(receiverRawValues.ChannelValues[2], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, 0, THROTTLE_LIMIT_POINT);
    cmd.YawAngleChange = map_double(ignoreDeadBand(receiverRawValues.ChannelValues[3]), TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, -QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND, QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND);
    cmd.Error = false;  
    /*
    Serial.print("yaw: ");
    Serial.print(receiverRawValues.ChannelValues[3]);
    Serial.print(" | pitch: ");
    Serial.print(receiverRawValues.ChannelValues[1]);
    Serial.print(" | roll: ");
    Serial.print(receiverRawValues.ChannelValues[0]);
    Serial.print(" | thr: ");
    Serial.println(receiverRawValues.ChannelValues[2]);
    */
    prevReceiverCommands = cmd;
    return cmd;
  } else if(millis() - RECEIVER_COMMUNICATION_TIMEOUT_IN_MILLISECONDS > receiver_last_communication_time){
    return getFailureReceiverCommand();
  } else{
    return prevReceiverCommands;
  }
}


struct ReceiverRawValues getReceiverRawValues(){
  struct ReceiverRawValues v;
  v.NewDataAvailable = false;
  v.TransmitterCommunicationFailure = false;

  if (radio.available()) {
    radio.read(&v.ChannelValues, sizeof(v.ChannelValues)); // Veriyi al
    v.NewDataAvailable = true;
  } else if(millis() - RECEIVER_COMMUNICATION_TIMEOUT_IN_MILLISECONDS > receiver_last_communication_time){
    v.TransmitterCommunicationFailure = true;
  }

  return v;
}

struct ReceiverCommands getFailureReceiverCommand(){
  struct ReceiverCommands cmd;
  cmd.Error = true;
  return cmd;
}

int ignoreDeadBand(int val){
  int center = (TRANSMITTER_JOYSTICK_MIN_VALUE + TRANSMITTER_JOYSTICK_MAX_VALUE) / 2;
  if(abs(val - center) <= TRANSMITTER_JOYSTICK_DEAD_BAND){
    return center;
  }
  else{
    return val;
  }
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
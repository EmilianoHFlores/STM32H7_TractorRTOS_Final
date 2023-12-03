#include "Utils.h"

void Utils::handleEncoder(){
  encoder_tics++;
}

void Utils::updateSpeed(double& velocity , bool debug = false, bool RPMs = false){
  velocity = debug ? velocity * 1.05 + 1 : (encoder_tics / kEncoderTicsRev) * (RPMs ? kSamplesPerMinute : kSamplesPerSecond);
  velocity = velocity * PI * 2 * kMotorRadius;
  encoder_tics = 0;

  velocity = velocity > 150 ? 0 : velocity;
}
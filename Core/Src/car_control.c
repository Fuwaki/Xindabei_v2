#include "car_control.h"


static float target_velocity = 0.0;
static float target_angular_velocity=0.0;
void CarControlInit() {
  
}
void CarControlHandler() {
  
}
void SetTargetCarStatus(float velocity, float angular_velocity) {
  target_angular_velocity = velocity;
  target_velocity=velocity;
}
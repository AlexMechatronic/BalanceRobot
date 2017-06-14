#ifndef _DRIVERL293_
#define _DRIVERL293_

#include <Arduino.h>
#include <stdint.h>


typedef struct {
	uint8_t pin1;	//output digital pin, control of direction
	uint8_t pin2;	//output digital pin, control of direction
	uint8_t enable;	//output PWM pin to modify the velocity of the motor 
}MotorHandle_s;

void driverInit(MotorHandle_s* mtr_s)
{
	pinMode(mtr_s->pin1, OUTPUT);
	pinMode(mtr_s->pin2, OUTPUT);
	pinMode(mtr_s->enable, OUTPUT);
}

void driverMForward(MotorHandle_s* mtr_s, uint8_t velocity)
{
	digitalWrite(mtr_s->pin1, HIGH);
	digitalWrite(mtr_s->pin2, LOW);
	analogWrite(mtr_s->enable, velocity);
}

void driverMBackward(MotorHandle_s* mtr_s, uint8_t velocity)
{
	digitalWrite(mtr_s->pin1, LOW);
	digitalWrite(mtr_s->pin2, HIGH);
	analogWrite(mtr_s->enable, velocity);
}

void driverMStop(MotorHandle_s* mtr_s)
{
	digitalWrite(mtr_s->pin1, LOW);
	digitalWrite(mtr_s->pin2, LOW);
	analogWrite(mtr_s->enable, 0);
}


#endif // !_DRIVERL293_

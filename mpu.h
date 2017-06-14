#ifndef MPU_H
#define MPU_H
#include "MahonyAHRS.h"

enum OrientationMatrix_t{
/*	s-Signo H-Horizontal V-vertical
representacion en el plano 
	conforme al IMU
	_________
	|o		|
	|  IMU	|
	|_______|
	  
	  V
	  |
	Z o____H
	sHsVsZ 	*/
	PXPYPZ

};

struct s_mympu {
	EulerAnglesStruct ypr;
	EulerAnglesStruct gyro;
	Vector3F accel;
	long timeStamp;
};


extern struct s_mympu mympu;

int mympu_open(unsigned int rate);
int mympu_update();

#endif


/*Conceccion de arduino MEGA a componentes
Arduino		IMU		Driver
+5v-out		+5v		+5v logic
vin					12v in		es donde va conectado el + de la bateria ya que el arduino tiene un regulador de voltaje
								y el driver del voltaje de la bateria alimenta los motores

*/

#define MPU6500
#define TEST_CONTROL 0  // 1= triangle test pattern     0 = IMU slabilisation
#define _DEBUG_SERIAL 1
//#define VIEW_ONLY_IMU 1

#include "freeram.h"

#include "mpu.h"
#include "I2Cdev.h"
#include "PID.h"
#include "driverL293.h"
#include "MahonyAHRS.h"
#include "I2Cdev.h"

const uint8_t button = 37;
const uint8_t led13 = 13;

MotorHandle_s motorL = { .pin1 = 26/*IN1*/,.pin2 = 32/*IN2*/,.enable = 10/*A*/};
MotorHandle_s motorR = { .pin1 = 28/*IN3*/,.pin2 = 30/*IN4*/,.enable = 9/*B*/ };

//motores offset
uint8_t offsetMotors = 76; //desde este valor de pwm se empiezan a mover los motores

//banda proporcional Pb en porcentaje del SetPoint Pb = 10 //porciento

/*
 * banda = Pb * SP
 * Kp = 1/(Pb*SP) //Pb = 0.1
 * Kd = D / (Pb * SP)
 * Ki = I/(Pb*SP)/10000
*/

// 10% de error 20 es lo que se puede mover maximo
const float Pb		  = 0.10;
const float gradosNom = 40.0;
const float kpP1      = 1.0/(Pb * gradosNom);
const float kiP1      = 0.005/(Pb * gradosNom)/10000.0;
const float kdP1      = 0.005/(Pb * gradosNom);
const float treshold  = 1;//cuando este en un grado esta en ss

const float Pb2		  = 0.10;
const float Nom2      = 255;
const float kpP2      = 1.0/(Pb2 * Nom2);
const float kiP2      = 0.005/(Pb2 * Nom2)/10000.0;
const float kdP2      = 0.005/(Pb2 * Nom2);
const float treshold2  = 1;//cuando este en un grado esta en ss


PID pidMotor;
PID pidAngle;
float outPid;
float outPidMotor;


bool isCalibrated = false;

long timeLoop = 0;
const long timeLimit = 20; // es el tiempo que se tarda para que se calibre la imu

const uint8_t offsetMotor = 70; //desde este valor empiezan a moverse los motores para el pwm
bool direction;


int ret;
void setup() 
{
	Fastwire::setup(400,0);
	Serial.begin(115200);
	ret = mympu_open(200);
	pinMode(led13, OUTPUT);
	digitalWrite(led13, LOW);
	Serial.println("iniciando...");
	PID_init(&pidAngle, kpP1, kiP1, kdP1, treshold);
	PID_set_setpoint(&pidAngle, 0.0);//0� debe mantenerse en los 0 grados
  PID_init(&pidMotor, kpP2, kiP2, kdP2, treshold);
  PID_set_setpoint(&pidAngle, 0.0);//0� debe mantenerse en los 0 volts
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() 
{
	static float deltaTime, currentTime, previousTime;
	static byte  loop2;
	static int MotorDrive;
	static int Count;
	

	currentTime = millis()*0.001;//en milisegundos .001 es mas rapida una multiplicacion
	//deltaTime = (currentTime - previousTime);
	
	ret = mympu_update();

	switch (ret) {
	case 0: c++; break;
	case 1: np++; return;
	case 2: err_o++; return;
	case 3: err_c++; return;
	default:
		Serial.print("READ ERROR!  ");
		Serial.println(ret);
		return;
	}

	if (isCalibrated) //entra en esta seccion del codigo cuando las velocidades angulares convergan
	{
#if (!TEST_CONTROL)&& !defined(VIEW_ONLY_IMU)
		
			// Estimate platform velocity and distance 
		  outPid = PID_update(&pidAngle, (mympu.ypr.Angles.pitch), currentTime);
      if (outPid < 0.0)//backward
			{
				LimitFloat(&outPid, -gradosNom, 0);
        outPidMotor = PID_update(&pidMotor, abs(outPid), currentTime);
        outPidMotor = abs(PID_update(&pidMotor, outPid, currentTime));
        LimitFloat(&outPidMotor,0,1);
				//MotorDrive = //mapf(outPid, -gradosNom/*from low*/, 0.0/*from high*/, 255/*to low*/, 0/*to high*/);
				MotorDrive = mapf(outPidMotor, 0.0, 1.0, offsetMotor, 255);
				driverMBackward(&motorL, MotorDrive);
				driverMBackward(&motorR, MotorDrive);
				direction = true;
			}
			else if(outPid>0.0)//forward
			{
				LimitFloat(&outPid, 0, gradosNom);
        outPidMotor = abs(PID_update(&pidMotor, outPid, currentTime));
        LimitFloat(&outPidMotor,0,1);
				//MotorDrive = outPidMotor*255;//mapf(outPid, 0.0/*from low*/, gradosNom/*from high*/, offsetMotor/*to low*/, 255/*to high*/);
        MotorDrive = mapf(outPidMotor, 0.0, 1.0, offsetMotor, 255);
				driverMForward(&motorL, MotorDrive);
				driverMForward(&motorR, MotorDrive);
				direction = false;
			}
			else
			{
				//kill motors
				driverMStop(&motorL);
				driverMStop(&motorR);
			}
#endif//END Balance ctrl
	}
	else if(currentTime>timeLimit)
	{
		digitalWrite(led13, HIGH);
		if (digitalRead(button)) //despues de que se haya calibrado pulsar al boton para ajustar los angulos
		{
			isCalibrated = true;
		}
	}
	

								   // ================================================================
								   // 10Hz slow task loop
								   // Misc stuff
								   // ================================================================
#ifdef _DEBUG_SERIAL
	if (loop2 > 10)
	{
		loop2 = 0;
			//Serial.print("P");
			//Serial.print(PitchGyro + MTG_OFFSET);
			Serial.print("MD ");
			Serial.print(MotorDrive);
			Serial.print("  OutPID ");
			Serial.print(outPid);
     Serial.print("  OutPIDMotor ");
     Serial.print(outPidMotor);
			///Serial.println();
			//Serial.println();
				//Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
				Serial.print("\t Y: "); Serial.print(mympu.ypr.Angles.yaw);
				Serial.print(" P: "); Serial.print(mympu.ypr.Angles.pitch);
				Serial.print(" R: "); Serial.print(mympu.ypr.Angles.roll);
				Serial.print("\tgy: "); Serial.print(mympu.gyro.Angles.yaw);
				Serial.print(" gp: "); Serial.print(mympu.gyro.Angles.pitch);
				Serial.print(" gr: "); Serial.print(mympu.gyro.Angles.roll);
				Serial.print("\tax: "); Serial.print(mympu.accel.Vector.x);
				Serial.print(" ay: "); Serial.print(mympu.accel.Vector.y);
				Serial.print(" az: "); Serial.print(mympu.accel.Vector.z);
				Serial.println();
				Serial.print(" segundos: "); Serial.print(currentTime);
				if (direction)
					Serial.print(" back: ");
				else
					Serial.print(" forw: ");
	}//END 10HZ loop
  loop2++;
#endif
  previousTime = currentTime;
}

/*
 ===================================================================
 CSU:    LimitInt()
 ===================================================================
 Clamp an int between a min and max.

 ===================================================================
 */
void LimitInt(int *x, int Min, int Max)
{
	if (*x > Max)
		*x = Max;
	if (*x < Min)
		*x = Min;

}//END LimitInt


 /*
 ===================================================================
 CSU:    LimitFloat()
 ===================================================================
 Clamp a float between a min and max.  Note doubles are the same
 as floats on this platform.

 ===================================================================
 */
void LimitFloat(float *x, float Min, float Max)
{
	if (*x > Max)
		*x = Max;
	if (*x < Min)
		*x = Min;

}//END LimitInt

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

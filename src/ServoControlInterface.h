/*
 * ServoControlInterface.h
 *
 *  Created on: 2018年3月28日
 *      Author: root
 */

#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<pthread.h>

#ifndef SERVOCONTROLINTERFACE_H_
#define SERVOCONTROLINTERFACE_H_

#define PDO_CYCL_TIME 4000
#define MOTOR_CNT 6


#endif /* SERVOCONTROLINTERFACE_H_ */

int CreateThreads(pthread_t* cs,	pthread_t * cr,	pthread_t *sw);

int servoControllerInit();

int cyclicCommStart();

int servoOn();

int servoOff();

int servoReset();

int Hold();

int quickStop();

int getServoStatus(uint16_t AxisStatus[6],int axis_encoder[6],int torqueRatio[6]);

int getIOData();

int setIOData(unsigned int output);

int setAbsEncoder(int encoder[MOTOR_CNT]);

int setRelEncoder(int encoder[MOTOR_CNT]);

int setAxisSpeed(int AxisSpeed[MOTOR_CNT]);

//////////Security

int setAxisSpeedLimit(int AxisMaxSpeed[MOTOR_CNT]);

int setAxisEncoderLimit(int PosLimit[MOTOR_CNT],int NegLimit[MOTOR_CNT]);

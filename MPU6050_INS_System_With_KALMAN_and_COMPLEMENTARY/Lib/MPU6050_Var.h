#ifndef _MPU6050_Var_H
#define _MPU6050_Var_H

#include "arduino_variable_types.h"

#define YAW   0
#define PITCH 1
#define ROLL  2

//Flight Modes was defined here
#define SOFT 0
#define ACRO 1

//Movement type was defined here
#define STATIC 0
#define DYNAMIC 1

enum test_steps
{
	NOT_YET 	= 0 ,
	ALMOST_DONE		,
	DONE
};

typedef struct MPU_st_
{	
	FLOAT64 		current_ypr_f64		[3];   
	FLOAT64 		old_ypr_f64			[3];        
	FLOAT64 		beginning_ypr_f64	[3];
	UINT32			timer;
} MPU_st;

	
#endif /*_MPU6050_Var_H*/

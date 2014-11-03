// Standard C/C++ libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

//library for ros
#include <ros/ros.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"

//library for Android msg
#include <sensor_msgs/Imu.h>

//libraries for the sonar
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"

//libraries for the array
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//libraries for the control
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "state/state.h"
#include <sstream>

/***************************************************************************
********* PARAMETERS OF THE FUZZY CONTROLER*********************************
****************************************************************************/

//#define	FUZZY_TIME			100000		// microsec time between control loops !!! Has to be at least twice the sensor loop times
#define MAX_ERROR_FORCE_XY		.45 //0.45			// .25 Maximal force XY due to error [N]
#define MAX_ERROR_FORCE_Z		.3 //0.2			// .15 Maximal force Z due to error [N]
#define MAX_ERROR_FORCE_TZ		0.5//0.35			// .35 Maximal force Theta due to error [Nm]
#define MAX_ERROR_FORCE_TXY		0.4			// .35 Maximal force Theta due to error [Nm]
#define ERROR_RANGE_XY			4			// Maximal admittable errors in XY [m]
#define ERROR_RANGE_Z			2			// Maximal admittable errors in Z [m]
#define ERROR_RANGE_TZ			1			// Maximal admittable errors in T [rad]
#define ERROR_RANGE_TXY			0.7			// Maximal admittable errors in T [rad]

#define MAX_INTEGRAL_FORCE_XY		.25 //0.4			// .25 Maximal force XY for integral error [N]
#define MAX_INTEGRAL_FORCE_Z		.35 //0.25			// .1 Maximal force XY for integral error [N]
#define MAX_INTEGRAL_FORCE_TZ		0.4//0.3			// .5 Maximal force theta for integral error [N]
#define MAX_INTEGRAL_FORCE_TXY		0.5			// .5 Maximal force tXY for integral error [N]
#define ANTI_WINDUP_XY			20	//20		// Anti windup limits for integrals XY[m*s]
#define ANTI_WINDUP_Z			20	//20		// Anti windup limits for integrals Z[m*s]
#define ANTI_WINDUP_TZ			20	//20		// Anti windup limits for integrals theta [rad*s]
#define ANTI_WINDUP_TXY			20	//20		// Anti windup limits for integrals theta [rad*s]
#define INTEGRAL_RANGE_XY		2			// Distance under which integral error XY is linearly diminued [m]
#define INTEGRAL_RANGE_Z		2			// Distance under which integral error Z is linearly diminued [m]
#define INTEGRAL_RANGE_TZ		2			// Distance under which integral error theta is linearly diminued [rad]
#define INTEGRAL_RANGE_TXY		2			// Distance under which integral error theta is linearly diminued [rad]


#define MAX_INC_FORCE_XY		.65 //0.85			// .65 Maximal force XY for increasing error [N]
#define MAX_INC_FORCE_Z			.75 //0.55			// .35 Maximal force Z for increasing error [N]
#define MAX_INC_FORCE_TZ		0.85//0.35			// .5 Maximal force theta for increasing error [Nm]
#define MAX_INC_FORCE_TXY		0.6			// .5 Maximal force theta for increasing error [Nm]
#define INC_RANGE_XY			1			// Range for increasing error XY[m]
#define INC_RANGE_Z			0.5			// Range for increasing error Z [m]
#define INC_RANGE_TZ			0.65//0.5			// Range for increasing error Theta [rad]
#define INC_RANGE_TXY			0.5			// Range for increasing error Theta [rad]
#define INC_SLOPE_XY			0.2			// Slope for increasing error XY[m]
#define INC_SLOPE_Z			0.2			// Slope for increasing error Z	[m/s]
#define INC_SLOPE_TZ			0.2			// Slope for increasing error T [rad/sž]
#define INC_SLOPE_TXY			0.2			// Slope for increasing error T [rad/sž]

#define MAX_DEC_FORCE_XY		0.85 //0.95		// Maximal force XY for decreasing error [N]
#define MAX_DEC_FORCE_Z			0.8	//.7		// Maximal force Z for decreasing error [N]
#define MAX_DEC_FORCE_TZ		0.95	//0.65			// Maximal force T for decreasing error [N]
#define MAX_DEC_FORCE_TXY		0.6			// Maximal force T for decreasing error [N]
#define DEC_RANGE_XY			1			// Range for decreasing error XY[m]
#define DEC_RANGE_Z			0.5			// Range for decreasing error Z[m]
#define DEC_RANGE_TZ			0.65//0.5			// Range for decreasing error T[m]
#define DEC_RANGE_TXY			0.5			// Range for decreasing error T[m]
#define DEC_SLOPE_XY			0.2			// Slope for decreasing error XY[m]
#define DEC_SLOPE_Z			0.2			// Slope for decreasing error Z[m]
#define DEC_SLOPE_TZ			0.2			// Slope for decreasing error T[m]
#define DEC_SLOPE_TXY			0.2			// Slope for decreasing error T[m]

typedef struct {

	double time;//=((double)FUZZY_TIME)/1000000;
	double mef_xy;//=(double)MAX_ERROR_FORCE_XY;
	double mef_z;//=(double)MAX_ERROR_FORCE_Z;
	double mef_tz;//=(double)MAX_ERROR_FORCE_TZ;
	double mef_txy;//=(double)MAX_ERROR_FORCE_TXY;
	double exy;//=(double)ERROR_RANGE_XY;
	double ez;//=(double)ERROR_RANGE_Z;
	double etz;//=(double)ERROR_RANGE_TZ;
	double etxy;//=(double)ERROR_RANGE_TXY;

	double mixy;//=(double)MAX_INTEGRAL_FORCE_XY;
	double miz;//=(double)MAX_INTEGRAL_FORCE_Z;
	double mitz;//=(double)MAX_INTEGRAL_FORCE_TZ;
	double mitxy;//=(double)MAX_INTEGRAL_FORCE_TXY;
	double ixy;//=(double)INTEGRAL_RANGE_XY;
	double iz;//=(double)INTEGRAL_RANGE_Z;
	double itz;//=(double)INTEGRAL_RANGE_TZ;
	double itxy;//=(double)INTEGRAL_RANGE_TXY;

	double minc_xy;//=(double)MAX_INC_FORCE_XY;
	double minc_z;//=(double)MAX_INC_FORCE_Z;
	double minc_tz;//=(double)MAX_INC_FORCE_TZ;
	double minc_txy;//=(double)MAX_INC_FORCE_TXY;
	double incxy;//=(double)INC_RANGE_XY;
	double incz;//=(double)INC_RANGE_Z;
	double inctz;//=(double)INC_RANGE_TZ;
	double inctxy;//=(double)INC_RANGE_TXY;
	double incsxy;//=(double)INC_SLOPE_XY;
	double incsz;//=(double)INC_SLOPE_Z;
	double incstz;//=(double)INC_SLOPE_TZ;
	double incstxy;//=(double)INC_SLOPE_TXY;

	double mdec_xy;//=(double)MAX_DEC_FORCE_XY;
	double mdec_z;//=(double)MAX_DEC_FORCE_Z;
	double mdec_tz;//=(double)MAX_DEC_FORCE_TZ;
	double mdec_txy;//=(double)MAX_DEC_FORCE_TXY;
	double decxy;//=(double)DEC_RANGE_XY;
	double decz;//=(double)DEC_RANGE_Z;
	double dectz;//=(double)DEC_RANGE_TZ;
	double dectxy;//=(double)DEC_RANGE_TXY;
	double decsxy;//=(double)DEC_SLOPE_XY;
	double decsz;//=(double)DEC_SLOPE_Z;
	double decstz;//=(double)DEC_SLOPE_TZ;
	double decstxy;//=(double)DEC_SLOPE_TXY;

	double awinup_xy;//ANTI_WINDUP_XY		
	double awinup_z;//ANTI_WINDUP_Z			
	double awinup_tz;//ANTI_WINDUP_TZ			
	double awinup_txy;//ANTI_WINDUP_TXY

}
t_gainfuzzy;

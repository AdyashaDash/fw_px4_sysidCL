#ifndef CONSTS_H_
#define CONSTS_H_

//**********************************************************************
//*** DEFINES / CONSTS
//**********************************************************************

const float pi=3.141592653f;
const float g=9.806; 					// [m/s�]
const float DEG2RAD = pi/180.0f;		//Multiplication is assumed with these constants
const float RAD2DEG = 180.0f/pi;		//Multiplication is assumed with these constants

//**********************************************************************
//*** ENUMERATIONS
//**********************************************************************

enum StallStatusDescriptor {NO_STALL = 0, STALL_WARNING = 1, IN_STALL = 2 , AFTER_STALL = 3};
enum AltitudeStatusDescriptor {ALTITUDE_OK_LEVELING = 0, ALTITUDE_OK_CLIMBING = 1 , ALTITUDE_OK_DESCENDING = 2,ALTITUDE_WARNING_ALTLOW = 3};
enum AirspeedCtrlStatusDescriptor {NORMAL_VELCTRL = 0, AGGRESSIVE_VELCTRL = 1};

enum CtrlType {
	//PID Controllers
	PID_STD=0,					// PID controller (Standard)
	PID_GAINSDECOUPLED=1,		// PID controller, decoupled SAS&CAS gains
	//MPC Controllers
	MPC_STD=10,					// MPC controller (Standard)
	MPC_ROLLMPCONLY=11			// MPC controller on roll, simplified PID on pitch
};

enum CtrlMode {
	MODE_MANUAL=0,				// Manual control
	MODE_SAS=1,					// Stability augmented system (rate stabilization)
	MODE_CAS=2,					// Control augmented system (angle stabilization)
	MODE_ALT=3,					// Altitude control with roll controlled as in CAS
	MODE_HEAD=4,				// Heading Control
	MODE_AUTO=5,				// Waypoint following (with altitude control)
	//...
	MODE_RCLOSS_MANFAILSAFE=10,			// Control when RC-signal is lost. Different labels exist only
	MODE_RCLOSS_CASFAILSAFE=11,			// to signify different control-reactions on RC loss, e.g manual
	MODE_RCLOSS_RTHFAILSAFE=12,			// failsafe, CAS-failsafe, Return-To-Home (RTH) behaviour etc.
	MODE_RCLOSS_AUTOFAILSAFE=13,
	MODE_RCLOSS_ERR=14
};
#endif /*CONSTS_H_*/

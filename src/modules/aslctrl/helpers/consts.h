#ifndef CONSTS_H_
#define CONSTS_H_

//**********************************************************************
//*** DEFINES / CONSTS
//**********************************************************************

const float pi=3.141592653f;
const float g=9.806; 					// [m/s²]
const float DEG2RAD = pi/180.0f;		//Multiplication is assumed with these constants
const float RAD2DEG = 180.0f/pi;		//Multiplication is assumed with these constants

//**********************************************************************
//*** ENUMERATIONS
//**********************************************************************

enum CtrlType {
	//PID Controllers
	PID_STD=0,					// PID controller (Standard)
	PID_GAINSDECOUPLED=1,		// PID controller, decoupled SAS&CAS gains
	//MPC Controllers
	MPC_STD=10,					// MPC controller (Standard)
	MPC_ROLLMPCONLY=11,			// MPC controller on roll, simplified PID on pitch
	//CL SYS ID Controllers
	CLSYSID = 20				// Closed loop system ID maneuvers
};

enum CtrlMode {
	MODE_MANUAL=0,				// Manual control
	MODE_SAS=1,					// Stability augmented system (rate stabilization)
	MODE_CAS=2,					// Control augmented system (angle stabilization)
	MODE_ALT=3,					// Altitude control with roll controlled as in CAS
	MODE_HEAD=4,				// Heading Control
	MODE_AUTO=5,				// Waypoint following (with altitude control)
	//MODE_CLSYSID=6,
	//...
	MODE_RCLOSS_MANFAILSAFE=10,			// Control when RC-signal is lost. Different labels exist only
	MODE_RCLOSS_CASFAILSAFE=11,			// to signify different control-reactions on RC loss, e.g manual
	MODE_RCLOSS_RTHFAILSAFE=12,			// failsafe, CAS-failsafe, Return-To-Home (RTH) behaviour etc.
	MODE_RCLOSS_AUTOFAILSAFE=13,
	MODE_RCLOSS_ERR=14
};

static const char aslctrl_error_codes[13][45] =
{
	"ALL_OK",
	"ERROR: Home-alt wrong.",
	"ERROR: Param HL_AlthMax wrong",
	"WARNING: Ref-alt vs. home-alt bounds",
	"WARNING: Alt vs. Home-alt bounds",
	"WARNING: ",
	"WARNING: Ref-alt exceeds HL_AlthMax",
	"WARNING: HL_AlthMax below HL_AlthMin",
	"WARNING: Ref-alt below HL_AlthMin",
	"WARNING: Alt below HL_AlthMin",
	"WARNING: TECS_MODE_UNDERSPEED",
	"WARNING: TECS_MODE_BAD_DESCENT",
	"WARNING: TECS_MODE_CLIMBOUT"
};

#endif /*CONSTS_H_*/

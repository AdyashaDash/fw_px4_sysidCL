//ASL Autopilot main class

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/aslctrl_data.h>
#include <uORB/topics/parameter_update.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <poll.h>

#include "helpers/subs.h"
#include "SAS.h"
#include "CAS.h"
//#include "CAS_MPC.h"
#include "HL.h"


/**
 * ASL Autopilot
 */
class ASLAutopilot
{
public:
	ASLAutopilot();
	void update();
	virtual ~ASLAutopilot();

	int SetCtrlData(void);

	void ReloadParameters(void);
	//int UpdateFilters(const int & counter);

	int HandleRCLoss();
	bool OnGround(void);		//Check whether aircraft is on ground

private:
	//Control class members
	//SAS-Control
	SAS SAScontrol;
	//CAS-Control
	CAS CAScontrol;
	//CAS_MPC CAS_MPC_control;
	//High Level Control (Waypoint-Following including Loiter, RTL, Velocity Control)
	HL HLcontrol;
	bool MODE_CLSYSID = false;


private:
	//as per standard on PX4IO, correct ordering to spektrum standard is done in mixer.
	enum {CH_AIL_R, CH_ELV, CH_RDR, CH_THR_1, CH_FLAPS, CH_AIL_L, CH_AUX1, CH_AUX2}; //as per standard on PX4IO, correct ordering to

	bool initialized;
	bool bRunOnce;

	subscriptions subs; //UORB subscriptions from PX4
	aslctrl_parameters_s *params;	// Pointer to aslctrl parameters in the UORB subscriptions
	aslctrl_data_s * ctrldata; // Pointer to the current control variables

	int counter;
	int mavlink_fd;		//A mavlink file descriptor for debugging/user notification

	uint64_t lastTime_MotorOK;
};

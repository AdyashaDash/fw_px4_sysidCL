/*
 * HL.h
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include <mathlib/mathlib.h>
#include "helpers/subs.h"
#include "helpers/Filters.h"
#include "helpers/PI.h"
#include "helpers/helpfuncs.h"
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>

//*****************************************************************
//*** CONSTANT PARAMETERS (not changeable via QGroundControl)
//*****************************************************************

class HL
{
public:
	HL();
	HL(subscriptions *subs_arg);

	//WaypointControllers
	int WaypointControl_L1(float &RollAngleRef);

	//Combined Altitude&Airspeed Controllers
	int TECS_AltAirspeedControl(float &PitchAngleRef, float& uThrot, float& AirspeedRef, float &hRef, float const &h, float const h_home, float &hRef_t, bool& bEngageSpoilers, const bool bUseRamp, const bool bUseThermalHighEtaMode, const bool bModeChanged);
	int TECS_Update50Hz(void);

	//CLSYSID controller
	int CLSYSIDControl(float& id_step, bool bModeChanged);

	//Helper functions
	int CalcAltitudeRamp(float& hRef_t, const float& hRef, const float& h, const bool& bModeChanged);
	int CalcThermalModeModifications(const float& h, const float& hRef_t, bool& bEngageSpoilers, bool bUseThermalHighEtaMode);
	//float CalcAltitudeRamp(const float& hRef, const float& CurrentAlt, const bool& bModeChanged);

	void CopyUpdatedParams();

private:
	ECL_L1_Pos_Controller L1Ctrl;
	TECS tecs;

	LowPass LP_Airspeed; 		// Airspeed Low Pass Filter
	MovingAverage MA_Airspeed; 	// Airspeed Moving Average Filter
	LowPass LP_vZ;				// Vertical velocity Low Pass Filter
	LowPass LP_h;				// Altitude Low Pass Filter

	hrt_abstime t_old;
	math::Matrix<3,3> R_nb;			//TECS attitude rotation matrix

	hrt_abstime t_idstart;			//System ID maneuver start time
	uint8_t man_count = 0;	

	subscriptions *subs; 			//UORB subscriptions from PX4
	aslctrl_parameters_s *params;	// Pointer to aslctrl parameters in the UORB subscriptions

	bool bSpoilerAltExceeded;
	bool bhMinExceeded;
};




/*
 * HL.cpp
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include "ASLAutopilot.h"
#include <mavlink/mavlink_log.h>
#include <fcntl.h>
#include <stdio.h>
#include <mathlib/mathlib.h>
#include "HL.h"
#include "helpers/helpfuncs.h"
#include "helpers/consts.h"
#include "CAS.h"


HL::HL() : LP_Airspeed(0,0),MA_Airspeed(4,0.0f),LP_vZ(0,0),LP_h(0,0)
{
	bSpoilerAltExceeded=false;
	bhMinExceeded=false;
};

HL::HL(subscriptions *subs_arg) : LP_Airspeed(0,0),MA_Airspeed(4,0.0f),LP_vZ(0,0),LP_h(0,0)
{
	subs=subs_arg;
	params=&(subs->aslctrl_params);

	t_old=0;
	bSpoilerAltExceeded=false;
	bhMinExceeded=false;
};

void HL::CopyUpdatedParams(void)
{
	// When parameters have been updated, these can be accessed through the appropriate pointer and (if necessary, depending on the param)
	// are written to the objects requiring them.
	float HL_tSample=params->SAS_tSample*params->HL_fMult;

	L1Ctrl.set_l1_damping(params->HL_WPL1_Damping);
	L1Ctrl.set_l1_roll_limit(params->CAS_RollAngleLim);
	//L1Ctrl.set_l1_period(params->HL_WPL1_Period);
	LP_Airspeed.SetGains(HL_tSample,1.0f); //1.0 rad/s
	LP_vZ.SetGains(HL_tSample,1.0f); //1.0 rad/s
	LP_h.SetGains(HL_tSample,params->HL_AltLowPassOmega);

	tecs.enable_airspeed(true);
	tecs.set_time_const(params->time_const);
	tecs.set_time_const_throt(params->time_const_throt);
	tecs.set_min_sink_rate(params->min_sink_rate);
	tecs.set_max_sink_rate(params->max_sink_rate);
	tecs.set_throttle_damp(params->throttle_damp);
	tecs.set_integrator_gain(params->integrator_gain);
	tecs.set_throtILim(params->throttle_ILim);
	tecs.set_vertical_accel_limit(params->vertical_accel_limit);
	tecs.set_height_comp_filter_omega(params->height_comp_filter_omega);
	tecs.set_speed_comp_filter_omega(params->speed_comp_filter_omega);
	tecs.set_roll_throttle_compensation(math::radians(params->roll_throttle_compensation));
	tecs.set_speed_weight(params->speed_weight);
	tecs.set_pitch_damping(params->pitch_damping);
	tecs.set_indicated_airspeed_min(params->HL_Vel_vMin);
	tecs.set_indicated_airspeed_max(params->HL_Vel_vMax);
	tecs.set_max_climb_rate(params->max_climb_rate);
	tecs.set_heightrate_p(params->heightrate_p);
	tecs.set_heightrate_ff(params->heightrate_ff);
	tecs.set_speedrate_p(params->speedrate_p);
	tecs.set_throttle_slewrate(params->throttle_slewrate);
	tecs.set_detect_underspeed_enabled(true);
}

int HL::WaypointControl_L1(float &RollAngleRef)
{
	math::Vector<2> ground_speed = {subs->global_pos.vel_n, subs->global_pos.vel_e};
	math::Vector<2> cur_pos = {float(subs->global_pos.lat), float(subs->global_pos.lon)};

	// *********************************************************************
	// *** L1-PERIOD GAIN SCHEDULING
	// *********************************************************************
	float airspeed_lim = limit2(LP_Airspeed.update(subs->airspeed.true_airspeed_m_s),params->HL_Vel_vMax,params->HL_Vel_vMin);
	float L1_P_GS = params->HL_WPL1_P_vNom;

	if(airspeed_lim > 1.0f) {
		// Bi-linear interpolation when airspeed =[vMin...vMax].
		if(airspeed_lim < params->HL_Vel_vNom) {
			float dAirspeed_rel=(airspeed_lim-params->HL_Vel_vMin)/(params->HL_Vel_vNom-params->HL_Vel_vMin); //To speed up interpolation
			L1_P_GS = interp1_lin(dAirspeed_rel, params->HL_WPL1_P_vMin,params->HL_WPL1_P_vNom);
		}
		else { //bigger or equal vNom
			float dAirspeed_rel=(airspeed_lim-params->HL_Vel_vNom)/(params->HL_Vel_vMax-params->HL_Vel_vNom); //To speed up interpolation
			L1_P_GS = interp1_lin(dAirspeed_rel, params->HL_WPL1_P_vNom,params->HL_WPL1_P_vMax);
		}
	}
	L1Ctrl.set_l1_period(L1_P_GS);

	if(params->ASLC_DEBUG == 22) printf("L1_GS: vAir:%.2f L1_P_GS:%.2f\n",(double)LP_Airspeed.Get(),(double)L1_P_GS);

	// *********************************************************************
	// *** STD-WAYPOINT CONTROL
	// *********************************************************************
	if (subs->position_setpoint_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		/* waypoint is a plain navigation waypoint */
		if(params->ASLC_DEBUG==10) printf("M5.1: ");

		math::Vector<2> next_wp = {float(subs->position_setpoint_triplet.current.lat),float(subs->position_setpoint_triplet.current.lon)};
		math::Vector<2> prev_wp = {float(subs->position_setpoint_triplet.previous.lat),float(subs->position_setpoint_triplet.previous.lon)};
		if (!subs->position_setpoint_triplet.previous.valid) {
			// No valid next waypoint, go for heading hold. This is automatically handled by the L1 library.
			prev_wp(0)=(float)subs->position_setpoint_triplet.current.lat;
			prev_wp(1)=(float)subs->position_setpoint_triplet.current.lon;
		}
		L1Ctrl.navigate_waypoints(prev_wp, next_wp, cur_pos, ground_speed);

		if(params->ASLC_DEBUG==10) printf("Previous WP: (%7.5f,%7.5f) Next WP (x/y): (%7.5f,%7.5f) | ", (double)prev_wp(0), (double)prev_wp(1), (double)next_wp(0), (double)next_wp(1));
	}
	else if (subs->position_setpoint_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
		/* waypoint is a loiter waypoint */
		math::Vector<2> next_wp = {float(subs->position_setpoint_triplet.current.lat),float(subs->position_setpoint_triplet.current.lon)};
		if(params->ASLC_DEBUG==10) printf("M5.2: Next WP (x/y): (%7.5f,%7.5f) | ", (double)next_wp(0), (double)next_wp(1));

		if(params->ASLC_DEBUG <27) {
			L1Ctrl.navigate_loiter(next_wp, cur_pos, subs->position_setpoint_triplet.current.loiter_radius,
					subs->position_setpoint_triplet.current.loiter_direction, ground_speed);
		}
		else {
			//This is a call to ASL/Kostas' modified L1 loitering logic
			//L1Ctrl.navigate_loiter_adapt(next_wp, cur_pos, subs->position_setpoint_triplet.current.loiter_radius,
			//					subs->position_setpoint_triplet.current.loiter_direction, ground_speed,subs->att.yaw,subs->position_setpoint_triplet.current.altitude, subs->global_pos.alt);
		}

	}
	else {
		/* RETURN TO LAUNCH (RTL) */
		if(params->ASLC_DEBUG==10) printf("M5.3: Home=(%7.5f,%7.5f,%.3f)\n",subs->home_pos.lat,subs->home_pos.lon,double(subs->home_pos.alt));
		math::Vector<2> rtl_pos = {float(subs->home_pos.lat), float(subs->home_pos.lon)};

		L1Ctrl.navigate_waypoints(rtl_pos, rtl_pos, cur_pos, ground_speed);
	}
	RollAngleRef = L1Ctrl.nav_roll();

	if(params->ASLC_DEBUG==10) printf("Roll Angle Ref(unlim):%7.4f \n",(double)L1Ctrl.nav_roll());

	return 0;
}

int HL::CLSYSIDControl(float& PitchAngleRef, float& RollAngleRef, bool bModeChanged)
{
	int RET = 1;
	uint64_t current_time = hrt_absolute_time();
/*
	For reference (parameters):
	--------------
	float CLSYSID_tExcite; 				// excitation time (sec) corresponding to apprx. mode natural frequency
	float CLSYSID_step; 				// excitation unit step size eg: 5, -5 etc. (in degrees), NOTE: sign implies the direction of the first step in a given maneuver
	float CLSYSID_nom_pitch				// nominal pitch-reference value
	float CLSYSID_nom_roll				// nominal roll-reference value 
	uint8_t CLSYSID_maneuver; 			// maneuver selection: 2-1-1 (0), chirp (1)
	uint8_t CLSYSID_ctrlinput; 			// control selection: pitch (0), roll (1)
	uint8_t CLSYSID_f_start				// start frequency for chirp
	uint8_t CLSYSID_f_end				// end frequency for chirp
	float CLSYSID_Fs 					// sampling rate for chirp
	uint8_t CLSYSID_repeat; 			// number of times to repeat sys id maneuver
*/

	//If first time in loop
	if (bModeChanged) {

		//Set ID start time
		t_idstart = current_time;

		//Freeze angle settings
		ctrldata->PitchAngleRef = params->CLSYSID_nom_pitch
		ctrldata->RollAngleRef = params->CLSYSID_nom_roll
	}

	//ID time required
	float t_maneuver_mult = 0.0f;
	switch (params->CLSYSID_maneuver)
	{
	case 0: // 2-1-1
		t_maneuver_mult = 4.0f;
		break;
	case 1: // chirp
		t_maneuver_mult = 18.0f;
		break;
	default: // error
	RET = 0;
		break;
	}

	float t_req = params->CLSYSID_tExcite*(t_maneuver_mult+6.0f);

	//Check if time elapsed is over sys. id. requirement
	if (float(current_time-t_idstart)/1.0E6f>t_req) {
		man_count = man_count+1;
		t_idstart = current_time;
	}

	if (man_count>=params->CLSYSID_repeat) {

		//Check maneuver count
		RET = 0;

	} else {

		//cl sys id maneuver control
		float id_step = 0.0f;
		switch(params->CLSYSID_maneuver)
		{
		case 0: // 2-1-1
			if (float(current_time-t_idstart)/1.0E6f>params->CLSYSID_tExcite*4.0f) {
				id_step = 0.0f;
			} else if (float(current_time-t_idstart)/1.0E6f>params->CLSYSID_tExcite*3.0f) {
				id_step = params->CLSYSID_step;
			} else if (float(current_time-t_idstart)/1.0E6f>params->CLSYSID_tExcite*2.0f) {
				id_step = -params->CLSYSID_step;
			} else {
				id_step = params->CLSYSID_step;
			}
			break;
		case 1:// chirp
			if (float(current_time-t_idstart)/1.0E6f>params->CLSYSID_tExcite*18.0f) {
				id_step = 0.0f;
			} else {
				double delta = ((current_time-t_idstart)/1.0E6f) / params->CLSYSID_Fs;
    			double t = CLSYSID_tExcite * delta;
    			double phase = 2 * PI * t * (params->CLSYSID_f_start + (params->CLSYSID_f_end - params->CLSYSID_f_start) * delta / 2);
    			if (phase > 2 * PI) phase -= 2 * PI;
    			id_step = sin(phase);
			}
			break;	
		default: // error
			id_step = 0.0f;
			RET = 0;
			break;
		}
	}
		//Set control input 			// control selection: pitch (0), roll (1)
		switch (params->CLSYSID_ctrlinput)
		{
		case 0: // pitch
			ctrldata->PitchAngleRef = params->CLSYSID_nom_pitch + id_step;
			break;
		case 1: // roll
			ctrldata->RollAngleRef = params->CLSYSID_nom_roll + id_step;
			break;
		default: // error
			RET = 0;
			break;
		}

	return RET;
}

int HL::TECS_AltAirspeedControl(float &PitchAngleRef, float& uThrot, float& AirspeedRef, float &hRef, float const &h, float const h_home, float &hRef_t, bool& bEngageSpoilers, const bool bUseRamp, const bool bUseThermalHighEtaMode, const bool bModeChanged)
{
	//Absolute altitude controller
	//printf("hRef: %7.4f h:%7.4f h_home: %7.4f velw: %7.4f\n", hRef, h, h_home,velw);

	if(params->ASLC_DEBUG==11) printf("AltCtrl: bModeChanged:%u, bUseRamp:%u, h:%.2f, hRef:%.2f, hRef_t:%.2f, hHome:%.2f\n",bModeChanged,bUseRamp,(double)h,(double)hRef,(double)hRef_t,(double)h_home);

	int RET = 0;
	//Safety checks. Either
	//a) Errors (return error code directly) or
	if(h_home<0.0f || h_home>5000.0f) 				{return -1;}						// First, check whether home altitude is OK
	if(params->HL_AlthMax < h_home+50.0f)			{return -2;}						// To catch a wrong user input
	//b) Warnings (return RET after execution)
	if(hRef<h_home+50.0f || hRef>h_home+5000.0f) 	{RET=-3; hRef=h_home+150.0f;} 		// Relative altitude setpoint can be between these values
	if(h<h_home-100.0f || h>h_home+1000.0f) 		{RET=-4;} 							// Relative altitude can be between these values
	//if(fabs(hRef-h)>300.0f)						{return -5;}  						// Maximum altitude change
	if(hRef>params->HL_AlthMax)					{RET=-6; hRef=params->HL_AlthMax;} // Altitude-ref may not exceed max alt limit
	if(params->HL_AlthMax<params->HL_AlthMin)	{RET=-7; params->HL_AlthMin=params->HL_AlthMax;}
	if(hRef<params->HL_AlthMin)					{RET=-8; hRef=params->HL_AlthMin;}

	//Using Ramp for altitude? Then calculate new altitude reference (ramp-function)
	if(bUseRamp) CalcAltitudeRamp(hRef_t, hRef, h, bModeChanged);
	else hRef_t = hRef;

	//If using the relaxed-altitude-tracking (thermal compliance) mode, the following function makes the necessary parameter mods
	CalcThermalModeModifications(subs->global_pos.alt, hRef_t, bEngageSpoilers, bUseThermalHighEtaMode);

	// Try to protect against bad descents caused by a) thermal downdrafts or b) unreachable velocity references (with simple hysteresis)
	if(subs->global_pos.alt < params->HL_AlthMin) bhMinExceeded=true;
	if(subs->global_pos.alt > params->HL_AlthMin+20.0f) bhMinExceeded=false;
	float maxThrot=params->throttle_max;
	if(bhMinExceeded)
	{
		if(maxThrot<1.0f) maxThrot=1.0f;	// This protects mainly against a) thermal downdrafts. Compromise maxThrottle constraint here for max safety, i.e. allow temporary uThrot>uThrotMax
		AirspeedRef=params->HL_Vel_vNom;	// This protects mainly against b) bad descents caused by an unreachable (i.e. too high) airspeed reference.
		RET=-9;								// This warns the user to take further measures in case the above does not suffice as a protection
	}

	//TECS altitude & airspeed controller
	tecs.update_pitch_throttle(R_nb, subs->att.pitch, subs->global_pos.alt, hRef_t,
						AirspeedRef,subs->airspeed.true_airspeed_m_s, 1.0f, false, math::radians(params->pitch_limit_min),
						params->throttle_min, maxThrot, params->throttle_cruise, math::radians(params->pitch_limit_min),
						math::radians(params->pitch_limit_max));

	PitchAngleRef = tecs.get_pitch_demand();
	uThrot = tecs.get_throttle_demand();

	struct TECS::tecs_state TECS_State;
	tecs.get_tecs_state(TECS_State);
	if(TECS_State.mode != TECS::ECL_TECS_MODE_NORMAL)
	{
		if(TECS_State.mode == TECS::ECL_TECS_MODE_UNDERSPEED) RET=-10;
		if(TECS_State.mode == TECS::ECL_TECS_MODE_BAD_DESCENT) RET=-11;
		if(TECS_State.mode == TECS::ECL_TECS_MODE_CLIMBOUT) RET=-12;
	}

	//Limiters. TODO: Add additional checks here.
	PitchAngleRef=limit1(PitchAngleRef,params->CAS_PitchAngleLim);

	return RET;
}

int HL::TECS_Update50Hz(void)
{
	//This function updates the TECS data fields. It should be run at >50Hz.
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
		R_nb(i, j) = subs->att.R[3*i+j];  // TODO::PX4Merge: Check correctness of indexes!!!!!
	math::Vector<3> accel_body(subs->sensors.accelerometer_m_s2);
	math::Vector<3> accel_earth = R_nb * accel_body;
	bool in_air_alt_control = ((subs->vstatus.main_state==vehicle_status_s::MAIN_STATE_AUTO_MISSION));// TODO::PX4Merge:is it only running in AUTO_MISSION mode
	tecs.update_state(subs->global_pos.alt, subs->airspeed.true_airspeed_m_s, R_nb, accel_body, accel_earth, (subs->global_pos.timestamp>0),in_air_alt_control);

	return 0;
}

int HL::CalcAltitudeRamp(float& hRef_t, const float& hRef, const float& h, const bool& bModeChanged)
{
	// This function Calculates an altitude ramp (hRef_t)

	float dt = (hrt_absolute_time()-t_old)/1.0E6;

	if(dt > 1.0f /*|| bModeChanged*/) {
		//Reinit (because of ModeChange or first init), but don't run for now!
		hRef_t = h;
		t_old = hrt_absolute_time();
		return -1;
	}

	if((hRef_t < hRef+dt*params->HL_vZSink) && (hRef_t > hRef-dt*params->HL_vZClimb)) {
		// hRef_t is so close to hRef that it would exceed it in the next step -> Set it to hRef
		// to avoid this small overshoot.
		hRef_t=hRef;
	}
	else {
		//Do the standard ramp
		if(hRef_t < hRef) hRef_t += dt*params->HL_vZClimb;
		else if(hRef_t > hRef) hRef_t -= dt*params->HL_vZSink;
	}

	t_old = hrt_absolute_time();

	//DEBUG
	if(params->ASLC_DEBUG==11) printf("AltRAMP: bModeChanged:%u, h:%.2f, hRef:%.2f, hRef_t:%.2f\n",bModeChanged,(double)h,(double)hRef,(double)hRef_t);

	return 0;
}

const float dh_offset_spoilers = 0.0f;
const float dh_offset_spoilers_hysteresis = 10.0f;
const float dh_offset_spdweight_hmax = 20.0f;
const float dh_offset_spdweight_href = 5.0f;

int HL::CalcThermalModeModifications(const float& h, const float& hRef_t, bool& bEngageSpoilers, bool bUseThermalHighEtaMode)
{
	// This function calculates modifications to the altitude control parameters that may
	// be required when using the thermal high eta (thermal compliance) mode.

	if(bUseThermalHighEtaMode && params->HL_AlthMax>0.1f) {
		// Set adapted values for thermal high eta mode, i.e. speed-weight ratio and other
		// parameters for efficient/low-disturbance throttle actuation
		float dh_rel = limit2((h-params->HL_AlthMax)/dh_offset_spdweight_hmax,1.0f,0.0f);
		tecs.set_speed_weight(interp1_lin(dh_rel, 2.0f,params->speed_weight));
		tecs.set_roll_throttle_compensation(0.0f);
		tecs.set_heightrate_p(0.0f);
	}
	else {
		// Set default values if not using thermal mode or if wrong values for AtlhMax were set.
		tecs.set_speed_weight(params->speed_weight);
		tecs.set_roll_throttle_compensation(params->roll_throttle_compensation);
		tecs.set_heightrate_p(params->heightrate_p);
	}

	// Spoiler activation (With some simple hysteresis)
	if(h>params->HL_AlthMax+dh_offset_spoilers && bSpoilerAltExceeded==false) bSpoilerAltExceeded=true;
	if(h<params->HL_AlthMax+dh_offset_spoilers-dh_offset_spoilers_hysteresis && bSpoilerAltExceeded==true) bSpoilerAltExceeded=false;
	bEngageSpoilers=bSpoilerAltExceeded;

	if(params->ASLC_DEBUG == 29) printf("h:%.1f hMax:%.1f sw:%.2f bSpoil:%d\n",(double)h,(double)params->HL_AlthMax,(double)tecs.get_speed_weight(),/*(double)tecs.get_roll_throttle_compensation(), (double)tecs.get_heightrate_p(),*/bEngageSpoilers);

	return 0;
}

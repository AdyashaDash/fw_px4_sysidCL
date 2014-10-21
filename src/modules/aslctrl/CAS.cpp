/*
 * CAS.cpp
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include "CAS.h"
//#include "helpers/PI.h"
#include <math.h>
#include <stdio.h>
#include <drivers/drv_hrt.h>

//General paradigms
// - Function which sets a reference value imposes the limits on it

//*****************************************************************************************
//*** CONSTRUCTORS / DESTRUCTORS
//*****************************************************************************************

CAS::CAS() :
	LP_Airspeed(0,0), LP_AccZ(0,0), LP_Yaw(0,0), LP_vZ(0,0), StallStatus(NO_STALL),qmax(10.0f)
{
}

CAS::CAS(parameters *params_arg, subscriptions *subs_arg):
		LP_Airspeed(0,0), LP_AccZ(0,0), LP_Yaw(0,0), LP_vZ(0,0), qmax(10.0f)
{
	params=params_arg;
	subs=subs_arg;
}

void CAS::CopyUpdatedParams(void)
{
	// When parameters have been updated, these can be accessed through the appropriate pointer and (if necessary, depending on the param)
	// are written to the objects requiring them.

	float CAS_tSample=params->p.SAS_tSample*params->p.CAS_fMult;

	PI_PitchAngle.SetParams(params->p.CAS_PitchPGain, params->p.CAS_PitchIGain, params->p.CAS_PitchRateLim, -params->p.CAS_PitchRateLim,params->p.CAS_PitchRateILim,-params->p.CAS_PitchRateILim, CAS_tSample);
	PI_PitchTC.SetParams(params->p.CAS_uElevTurnFF, params->p.CAS_PitchTCkI, params->p.CAS_PitchAngleLim, -params->p.CAS_PitchAngleLim,params->p.CAS_PitchTCILim,-params->p.CAS_PitchTCILim, CAS_tSample);
	LP_Airspeed.SetGains(CAS_tSample,12.57f);
	LP_AccZ.SetGains(CAS_tSample,12.57f);
	LP_Yaw.SetGains(CAS_tSample,params->p.CAS_YawLowPassOmega);
	LP_vZ.SetGains(CAS_tSample,12.57f);
}

//*****************************************************************************************
//*** LOWER-LEVEL CAS CONTROL (BANK ANGLE, PITCH ANGLE)
//*****************************************************************************************

float CAS::PitchControl(float const& pitchRef, float& pitchRefCT, float const& pitch, float const& rollRef, float const& roll, float& PGain, float& uThrot, const int aslctrl_mode)
{
	// PI-Controller with integrated elevator compensation for coordinated turns. P-Gain was scheduled before.
	// [1]: A minimalist control strategy for Small UAVs, Severin Leven et al., EPFL

	// Limit the roll angle at some reasonable value
	float roll_lim = limit1(roll, params->p.CAS_RollAngleLim*1.3);
	// First order Taylor-series expansion (cp. [1]) of 1/cos(roll) to avoid steep increase/sensitivity to noise for 1/cos(roll) as roll -> 90�
	float inv_cosroll = 0.5*pow(roll_lim,2.0f) + 1.0;

	//Bank angle compensation -> apply feed-forward on pitch reference
	float pitchRef_ff=0.0f;
	if(params->p.ASLC_CoordTurn == 5) {
		//Compute and apply limit for maximum value of the integrator
		float inv_cosroll_max = 0.5*pow(params->p.CAS_RollAngleLim,2.0f) + 1.0;
		float cBankMax=limit2((inv_cosroll-1.0f)/SAFE_ZERO_DIV(inv_cosroll_max-1.0f),1.0f,0.0f);
		PI_PitchTC.SetParams(0.0f, params->p.CAS_PitchTCkI, cBankMax*params->p.CAS_PitchTCILim,-cBankMax*params->p.CAS_PitchTCILim);

		//Apply the pitch-angle-ref feedforward
		pitchRef_ff = PI_PitchTC.step(pitchRef-pitch);
		float temp=pitchRef_ff;
		pitchRef_ff += params->p.CAS_uElevTurnFF*(inv_cosroll-1);

		if(params->p.ASLC_DEBUG==25) {
			printf("roll: %.2f cBankMax: %.3f m_int:%.2f PRef_org:%.2f PRef_ff_PI %.2f Pref_ff_FF: %.2f PRef:%.2f P:%.2f \n",roll, cBankMax,PI_PitchTC.m_int,pitchRef, temp, pitchRef_ff-temp, pitchRef+pitchRef_ff,pitch);
		}
	}

	pitchRefCT=pitchRef+pitchRef_ff;

	//PI-Controller
	PI_PitchAngle.SetPGain(PGain);
	float qref = PI_PitchAngle.step(pitchRefCT-pitch); // This one is saturated within the PI controller

	// Elevator/Pitch turn compensation
	if((params->p.ASLC_CoordTurn == 1 || params->p.ASLC_CoordTurn == 4) && fabs(roll) < 85.0*DEG2RAD) {
		//LIMIT this w.r.t two things:
		// a) to avoid roll angles close to 90� to avoid the singularity
		// b) to avoid overwriting the normal PI-Roll-Ctrler command due to a too high turn-compensation feed-forward
		//TODO1: 1.0 is not correct.
		float remaining_ctrl_action = 1.0-fabs(qref);
		float roll_lim = limit1(roll, params->p.CAS_RollAngleLim);
		float qref_ff = limit1(params->p.CAS_uElevTurnFF*fabs(sin(roll_lim)*sin(roll_lim)*inv_cosroll), remaining_ctrl_action);
		if(params->p.ASLC_DEBUG==16) printf("roll: %.2f qref:%.2f qref_ff %.2f qref_rem:%.2f \n",roll, qref,qref_ff,remaining_ctrl_action);
		qref += qref_ff;
	}

	// Throttle feed forward
	if((params->p.ASLC_CoordTurn == 1 || params->p.ASLC_CoordTurn >= 4 && aslctrl_mode==MODE_CAS) && fabs(roll) < 85.0*DEG2RAD) {
		// Throttle feed forward (necessary due to 1) higher stall speed and 2) higher drag in curve)
		// Note: This is taken from TECS, and is just a "hack"to have the necessary throttle feed-forward - with the
		// same behaviour as in TECS - also in a pure CAS mode.
		// Note2: To make this fully correct, one would need to increase the airspeed_reference in TECS as a function of the bank angle
		// reference, in order to guarantee that we are above the stall speed even in the turn.
		float cosPhi = cos(roll_lim);
		//Note: This heavily changes throttle even for low bank angles, although it is .
		// -> Adapt this to only change e.g. for bank>10�
		// -> Make this dependant of the roll_reference, not the roll, to avoid induced noise from the roll angle.
		float STEdot_dem = params->p.roll_throttle_compensation * (1.0f / limit2(cosPhi , 1.0f, 0.1f) - 1.0f);
		float _STEdot_max = params->p.max_climb_rate*g;
		float ff_throttle = STEdot_dem / _STEdot_max * (1.0f - params->p.throttle_cruise); //TODO Why 1.0f here? Why not THR_MAX?
		uThrot += ff_throttle;

		if(params->p.ASLC_DEBUG==24) printf("roll: %.2f STEdot:%.2f ff_throttle %.2f uThrot_new:%.2f \n",roll_lim, STEdot_dem,ff_throttle,uThrot);
	}

	// General issue: When banked too hard, pitch ref angle is hard to track because elevator action is
	// influencing roll motion more than pitch (assumption of decoupled long&lat motion breaks down). Here,
	// we need to scale the output ctrl. action by the angle.This might help in general, and might make the
	// FF-term less necesssary.

	qref=limit1(qref,params->p.CAS_PitchRateLim);
	return qref;
}

float CAS::BankControl(float const &bankRef, float const &roll, float & PGain)
{
	//P-Controller, P-Gain was scheduled before
	float pRef(PGain*(bankRef-roll));
	pRef=limit1(pRef,params->p.CAS_RollRateLim);
	return pRef;
}

float CAS::SideslipControl(float const& rollRef, float const& roll, float const& pitch, aslctrl_data_s &ctrldata)
{
	// Very basic sideslip controller. Using rudder to control the error in the achieved yaw-rate.
	float rRef=0.0f;

	if(LP_Airspeed.Get()>1.0f /*bAirspeedValid*/) {

		// Yawrate reference. Calculated from coordinated turn constraint, assuming constant pitch (theta_dot=0)
		ctrldata.Yawdot_ref = g* tan(roll)/ LP_Airspeed.Get();
		// Yawrate. Calculated from body velocities
		ctrldata.Yawdot = sin(roll)/cos(pitch)*subs->att.pitchspeed + cos(roll)/cos(pitch)*subs->att.yawspeed;

		rRef = ctrldata.uRud*params->p.SAS_YawPDir; //Allow feed through from manual input for now
		rRef += params->p.SAS_YawCTkP * (ctrldata.Yawdot_ref-ctrldata.Yawdot);
		rRef = limit1(rRef,params->p.CAS_YawRateLim);

		return rRef;
	}

	//Just return zero
	return 0;
}

int CAS::CoordinatedTurnControl(const float& roll, const float& pitch, float& qRef, float& rRef, aslctrl_data_s &ctrldata)
{
	// Basic coordinated-turn controller. Uses two "additional" p-gains on the errors between the actual
	// q&r-rates and the desired q&r-rates, as defined through the desired yaw-rate in a coordinated
	// turn.

	if(LP_Airspeed.Get()>1.0f /*bAirspeedValid*/) {

		// Yawrate reference. Calculated from coordinated turn constraint, assuming constant pitch (theta_dot=0)
		ctrldata.Yawdot_ref = g * tan(roll)/ LP_Airspeed.Get();
		// Yawrate. NOT USED AS A CONTROL INPUT HERE, JUST FOR COMPLETENESS. Calculated from body velocities.
		ctrldata.Yawdot = sin(roll)/cos(pitch)*subs->att.pitchspeed + cos(roll)/cos(pitch)*subs->att.yawspeed;

		float q_coordturn_ref = sin(roll)*ctrldata.Yawdot_ref;
		float r_coordturn_ref = cos(roll)*ctrldata.Yawdot_ref;

		//Elevator compensation
		qRef += params->p.CAS_uElevTurnFF * (q_coordturn_ref-subs->att.pitchspeed);
		qRef = limit1(qRef,params->p.CAS_PitchRateLim);

		//Rudder compensation
		rRef = -ctrldata.uRud*params->p.SAS_YawPDir; //TODO: REMOVE LATER, but allow feed through from manual input for now
		rRef += params->p.SAS_YawCTkP * (r_coordturn_ref-subs->att.yawspeed);
		rRef = limit1(rRef,params->p.CAS_YawRateLim);

		if(params->p.ASLC_DEBUG==21) printf("roll[�]: %.3f Yawd_r/Yawd: (%.3f/%.3f) qRef/q/uE: (%.3f/%.3f/%.3f) rRef/r: (%.3f/%.3f%.3f)\n",
				roll, ctrldata.Yawdot_ref,ctrldata.Yawdot,q_coordturn_ref,subs->att.pitchspeed, qRef,r_coordturn_ref, subs->att.yawspeed, rRef);

		return 0;
	}

	//Just return zero
	return -1;
}


//*****************************************************************************************
//*** HIGHER LEVEL CAS CONTROL (HEADING)
//*****************************************************************************************

int CAS::CASRollPitchControl(float &pref, float &qref, float& rref, float const &RollAngleRef, float const &Roll, float const &PitchAngleRef, float const &Pitch, float const &accZ, aslctrl_data_s &ctrldata, bool bModeChanged)
{
	//Only control Roll and Pitch Angles to reference values. Leave rest untouched.

	//If mode was changed, reinitialise applicable filters and variables
	if(bModeChanged) {
		LP_Airspeed.Set(subs->sensors.dbaro_velo_ms);
		LP_AccZ.Set(accZ);
	}

	LP_Airspeed.update(subs->sensors.dbaro_velo_ms);

	// Gain Scheduling w.r.t angle error - if activated
	if(params->p.ASLC_GainSch_E == 0) {
		ctrldata.P_kP_GainSch_E=params->p.CAS_PitchPGain;
		ctrldata.R_kP_GainSch_E=params->p.CAS_RollPGain;
	}
	else if(params->p.ASLC_GainSch_E == 1) {
		ctrldata.P_kP_GainSch_E = GainScheduler_linear(PitchAngleRef-Pitch, 0.75f*params->p.CAS_PitchAngleLim,params->p.CAS_PitchPGain,params->p.CAS_PitchPGainM);
		ctrldata.R_kP_GainSch_E = GainScheduler_linear(RollAngleRef-Roll, 0.75f*params->p.CAS_RollAngleLim,params->p.CAS_RollPGain,params->p.CAS_RollPGainM);
		if(params->p.ASLC_DEBUG==3) printf("Pitch: PGain, error (%7.4f, %7.4f). Roll: PGain, error (%7.4f, %7.4f). \n", params->p.CAS_PitchPGain, PitchAngleRef-Pitch, params->p.CAS_RollPGain, RollAngleRef-Roll);
	}

	//Controllers
	pref = BankControl(RollAngleRef,Roll, ctrldata.R_kP_GainSch_E);
	qref = PitchControl(PitchAngleRef,ctrldata.PitchAngleRefCT,Pitch,RollAngleRef,Roll,ctrldata.P_kP_GainSch_E,ctrldata.uThrot,ctrldata.aslctrl_mode);
	if(params->p.ASLC_CoordTurn == 1 || params->p.ASLC_CoordTurn == 5) {
		//Option 1: Do coordinated turn control in SAS
	}
	else if(params->p.ASLC_CoordTurn == 2) {
		CoordinatedTurnControl(Roll,Pitch,qref,rref,ctrldata);
	}
	else if(params->p.ASLC_CoordTurn == 3) {
		//Option 3: Use integrator with adjustable limits as a function of roll angle here
	}
	else if(params->p.ASLC_CoordTurn == 4) {
		rref = SideslipControl(RollAngleRef,Roll,Pitch,ctrldata);
	}


	// Stall protection
	//STALL PROTECTION: Normal acceleration penalizer
	if(params->p.ASLC_StallProt > 0) {
		//Old Stuff. Don't do anything right now.
	}

	if(params->p.ASLC_DEBUG==8) {
		printf(" Pitch_ref: %7.4f: Pitch%7.4f qref: %7.5f gain:%7.4f\n",PitchAngleRef,Pitch,qref,ctrldata.P_kP_GainSch_E);
		printf(" Roll_ref: %7.4f: Roll%7.4f pref: %7.5f gain:%7.4f\n",RollAngleRef,Roll,pref,ctrldata.R_kP_GainSch_E);
	}
	if(params->p.ASLC_DEBUG==15) {
		printf("PI-Ctrl: P-Gain:%7.4f I-Gain:%7.4f tS:%7.4f m_int:%7.5f \n",PI_PitchAngle.m_PGain,PI_PitchAngle.m_IGain,PI_PitchAngle.m_tSample,PI_PitchAngle.m_int);
	}

	//printf("[CAS-Ctrl] AccZ: %7.5f AccZBias: %7.5f\n",subs.sensors.accelerometer_m_s2[2], subs.ekf.x_b_a[2]);

	return 0;
}

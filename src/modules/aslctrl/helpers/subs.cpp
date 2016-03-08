
#include "subs.h"
#include <string.h>
#include <systemlib/err.h> //Debug only
#include <math.h>
#include <drivers/drv_hrt.h>
#include <poll.h>
#include "consts.h"

//**********************************************************************
//*** Constructors / Destructors
//**********************************************************************
subscriptions::subscriptions(void)
{
	init();
}
subscriptions::~subscriptions(void)
{

}

//**********************************************************************
//*** Functions
//**********************************************************************

int subscriptions::init(void)
{
	memset(&vstatus, 0, sizeof(vstatus));
	memset(&att, 0, sizeof(att));
	//memset(&att_sp, 0, sizeof(att_sp));
	//memset(&rates_sp, 0, sizeof(rates_sp));
	memset(&global_pos, 0, sizeof(global_pos));
	memset(&position_setpoint_triplet, 0, sizeof(position_setpoint_triplet));
	memset(&manual_sp, 0, sizeof(manual_sp));
	memset(&actuators, 0, sizeof(actuators));
	memset(&actuators_zeros, 0, sizeof(actuators_zeros));
	memset(&param_update, 0, sizeof(param_update));
	memset(&sensors, 0, sizeof(sensors));
	memset(&ekf, 0, sizeof(ekf));
	memset(&home_pos, 0, sizeof(home_pos));
	memset(&airspeed, 0, sizeof(airspeed));
	memset(&sensor_power, 0, sizeof(sensor_power));

	//Inputs
	/* subscribe */
	att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	//att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	//vcontrol_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	ekf_sub = orb_subscribe(ORB_ID(state_estimator_EKF_parameters));
	home_pos_sub = orb_subscribe(ORB_ID(home_position));
	airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	sensor_power_sub = orb_subscribe(ORB_ID(sensor_power));

	//Outputs
	/* publish actuator controls */
	actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuators_zeros);
	actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &actuators);
	aslctrl_params_pub = orb_advertise(ORB_ID(aslctrl_parameters), &aslctrl_params);
	aslctrl_data_pub = orb_advertise(ORB_ID(aslctrl_data), &aslctrl_data);

	//Set some default values
	fds.fd=att_sub;
	fds.events=POLLIN;

	bParamSanityChecksEnabled=true;
	bInitialized=true;

	return 0;
}

int subscriptions::get_inputs(void)
{
	// Wait for attitude updates from PX4.
	// ATTENTION: this limits the ASLCTRL execution frequency to this attitude-update-frequency!
	// TODO IMPORTANT: Recheck this, because this is highly uneven! Have to use orb_set_interval before for sure!!
	if(poll(&(fds), 1, 5000) <= 0) {
		return -1;
	}

	orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
	//orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
	orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
	orb_copy(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_sub, &position_setpoint_triplet);
	orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
	//orb_copy(ORB_ID(vehicle_control_mode), vcontrol_sub, &vcontrol);
	orb_copy(ORB_ID(sensor_combined), sensors_sub, &sensors);
	orb_copy(ORB_ID(state_estimator_EKF_parameters), ekf_sub, &ekf);
	orb_copy(ORB_ID(home_position), home_pos_sub, &home_pos);
	orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);
	orb_copy(ORB_ID(sensor_power), sensor_power_sub, &sensor_power);

	vehicle_status_s temp=vstatus;
	orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);
	if((vstatus.main_state != temp.main_state) && !vstatus.rc_signal_lost)
	{
		warnx("State and/or Mode changed!");
		warnx("MainState/NavState: (%d/%d). Before:(%d/%d)",vstatus.main_state, vstatus.nav_state,temp.main_state, temp.nav_state);
		//warnx("Enabled control flags : %d/%d/%d/%d (pos/att/rate/man)\n",vcontrol.flag_control_position_enabled,
		//		vcontrol.flag_control_attitude_enabled,vcontrol.flag_control_rates_enabled,vcontrol.flag_control_manual_enabled);
	}

	return 0;
}

bool subscriptions::check_aslctrl_params_changed(void)
{
	// Check whether aslctrl parameters have been changed, and update then if they changed.
	// Note: This actually checks whether ANY of the params has been updated, not only aslctrl parameters
	// However, this is still better than stupidly updating all parameters every loop.

	bool ParamsUpdated=false;
	orb_check(param_update_sub, &ParamsUpdated);

	if(ParamsUpdated>0) {
		// Copy params update itentifier topic
		orb_copy(ORB_ID(parameter_update), param_update_sub, &param_update);
		return true;
	}
	else return false;
}

int subscriptions::update_aslctrl_params(void)
{
	// Reload parameters now
	param_t handle;

	//ASLCTRL-GENERAL
	handle=param_find("ASLC_DEBUG");
	param_get(handle, &(aslctrl_params.ASLC_DEBUG));
	handle=param_find("ASLC_CtrlType");
	param_get(handle, &(aslctrl_params.ASLC_CtrlType));
	handle=param_find("ASLC_GainSch_E");
	param_get(handle, &(aslctrl_params.ASLC_GainSch_E));
	handle=param_find("ASLC_GainSch_Q");
	param_get(handle, &(aslctrl_params.ASLC_GainSch_Q));
	handle=param_find("ASLC_StallProt");
	param_get(handle, &(aslctrl_params.ASLC_StallProt));
	handle=param_find("ASLC_VelCtrl");
	param_get(handle, &(aslctrl_params.ASLC_VelCtrl));
	handle=param_find("ASLC_OnRCLoss");
	param_get(handle, &(aslctrl_params.ASLC_OnRCLoss));
	handle=param_find("ASLC_OvSpdProt");
	param_get(handle, &(aslctrl_params.ASLC_OvSpdProt));
	handle=param_find("ASLC_CoordTurn");
	param_get(handle, &(aslctrl_params.ASLC_CoordTurn));

	//DEMIX
	handle=param_find("DEMIX_Enabled");
	param_get(handle, &(aslctrl_params.DEMIX_Enabled));
	handle=param_find("DEMIX_F_CH2t4");
	param_get(handle, &(aslctrl_params.DEMIX_F_CH2t4));
	handle=param_find("DEMIX_F_CH6t1");
	param_get(handle, &(aslctrl_params.DEMIX_F_CH6t1));
	handle=param_find("DEMIX_CH2t4");
	param_get(handle, &(aslctrl_params.DEMIX_CH2t4));
	handle=param_find("DEMIX_CH6t1");
	param_get(handle, &(aslctrl_params.DEMIX_CH6t1));

	//SAS
	handle=param_find("SAS_tSample");
	param_get(handle, &(aslctrl_params.SAS_tSample));
	handle=param_find("SAS_RollPGain");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_RollPGain));
	handle=param_find("SAS_PitchPGain");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_PitchPGain));
	handle=param_find("SAS_YawPGain");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_YawPGain));
	handle=param_find("SAS_RollPDir");
	param_get(handle, &(aslctrl_params.SAS_RollPDir));
	handle=param_find("SAS_PitchPDir");
	param_get(handle, &(aslctrl_params.SAS_PitchPDir));
	handle=param_find("SAS_YawPDir");
	param_get(handle, &(aslctrl_params.SAS_YawPDir));
	handle=param_find("SAS_RYDecK");
	param_get(handle, &(aslctrl_params.SAS_RollYawDecoupleKari));
	handle=param_find("SAS_YawCTkP");
	param_get(handle, &(aslctrl_params.SAS_YawCTkP));
	handle=param_find("SAS_YawCTFF");
	param_get(handle, &(aslctrl_params.SAS_YawCTFF));
	handle=param_find("SAS_RCtrlLim");
	param_get(handle, &(aslctrl_params.SAS_RCtrlLim));
	handle=param_find("SAS_PCtrlLim");
	param_get(handle, &(aslctrl_params.SAS_PCtrlLim));
	handle=param_find("SAS_YCtrlLim");
	param_get(handle, &(aslctrl_params.SAS_YCtrlLim));
	handle=param_find("SAS_vScaleLimF");
	param_get(handle, &(aslctrl_params.SAS_vScaleLimF));
	handle=param_find("SAS_vScaleExp");
	param_get(handle, &(aslctrl_params.SAS_vScaleExp));
	handle=param_find("SAS_YawHPw");
	param_get(handle, &(aslctrl_params.SAS_YawHighPassOmega));
	handle=param_find("SAS_PitchLPw");
	param_get(handle, &(aslctrl_params.SAS_PitchLowPassOmega));
	handle=param_find("SAS_RollLPw");
	param_get(handle, &(aslctrl_params.SAS_RollLowPassOmega));
	handle=param_find("SAS_TrimAilvNom");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimAilvNom));
	handle=param_find("SAS_TrimAilvMin");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimAilvMin));
	handle=param_find("SAS_TrimAilvMax");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimAilvMax));
	handle=param_find("SAS_TrimElevNom");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.5f, 0.5f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimElevNom));
	handle=param_find("SAS_TrimElevMin");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.5f, 0.5f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimElevMin));
	handle=param_find("SAS_TrimElevMax");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.5f, 0.5f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimElevMax));
	handle=param_find("SAS_TrimRudvNom");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimRudvNom));
	handle=param_find("SAS_TrimRudvMin");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimRudvMin));
	handle=param_find("SAS_TrimRudvMax");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(aslctrl_params.SAS_TrimRudvMax));

	//CAS
	handle=param_find("CAS_fMult");
	param_get(handle, &(aslctrl_params.CAS_fMult));
	handle=param_find("CAS_PitchPGain");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.CAS_PitchPGain));
	handle=param_find("CAS_PitchPGainM");
	param_get(handle, &(aslctrl_params.CAS_PitchPGainM));
	handle=param_find("CAS_PitchIGain");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 3.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.CAS_PitchIGain));
	handle=param_find("CAS_RollPGain");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.CAS_RollPGain));
	handle=param_find("CAS_RollPGainM");
	param_get(handle, &(aslctrl_params.CAS_RollPGainM));
	handle=param_find("CAS_PRateLim");
	param_get(handle, &(aslctrl_params.CAS_PitchRateLim));
	handle=param_find("CAS_PRateILim");
	param_get(handle, &(aslctrl_params.CAS_PitchRateILim));
	handle=param_find("CAS_PitchTCkI");
	param_get(handle, &(aslctrl_params.CAS_PitchTCkI));
	handle=param_find("CAS_PitchTCILim");
	param_get(handle, &(aslctrl_params.CAS_PitchTCILim));
	handle=param_find("CAS_RRateLim");
	param_get(handle, &(aslctrl_params.CAS_RollRateLim));
	handle=param_find("CAS_YRateLim");
	param_get(handle, &(aslctrl_params.CAS_YawRateLim));
	handle=param_find("CAS_PAngleLim");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 45.0f * DEG2RAD) != 0) return -1;
	param_get(handle, &(aslctrl_params.CAS_PitchAngleLim));
	handle=param_find("CAS_RAngleLim");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 60.0f * DEG2RAD) != 0) return -1;
	param_get(handle, &(aslctrl_params.CAS_RollAngleLim));
	handle=param_find("CAS_YawLPw");
	param_get(handle, &(aslctrl_params.CAS_YawLowPassOmega));
	handle= param_find("CAS_uElevTurnFF");
	param_get(handle, &(aslctrl_params.CAS_uElevTurnFF));

	//HL
	handle=param_find("HL_fMult");
	param_get(handle, &(aslctrl_params.HL_fMult));
	handle=param_find("HL_WPL1_Damping");
	param_get(handle, &(aslctrl_params.HL_WPL1_Damping));
	handle=param_find("HL_WPL1_P_vMin");
	param_get(handle, &(aslctrl_params.HL_WPL1_P_vMin));
	handle=param_find("HL_WPL1_P_vNom");
	param_get(handle, &(aslctrl_params.HL_WPL1_P_vNom));
	handle=param_find("HL_WPL1_P_vMax");
	param_get(handle, &(aslctrl_params.HL_WPL1_P_vMax));
	handle=param_find("HL_Vel_vNom");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, 5.0f, 20.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.HL_Vel_vNom));
	handle=param_find("HL_Vel_vMin");
	param_get(handle, &(aslctrl_params.HL_Vel_vMin));
	handle=param_find("HL_Vel_vMax");
	param_get(handle, &(aslctrl_params.HL_Vel_vMax));
	//Old altitude controller
	handle=param_find("HL_AltLPw");
	param_get(handle, &(aslctrl_params.HL_AltLowPassOmega));
	handle=param_find("HL_AlthMax");
	param_get(handle, &(aslctrl_params.HL_AlthMax));
	handle=param_find("HL_AlthMin");
	param_get(handle, &(aslctrl_params.HL_AlthMin));
	handle= param_find("HL_vZClimb");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 5.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.HL_vZClimb));
	handle= param_find("HL_vZSink");
	if(bParamSanityChecksEnabled && Sanitize_param(handle,0.0f, 5.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.HL_vZSink));

	//TECS & HL
	handle = param_find("FW_P_LIM_MIN");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, -60.0f, 0.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.pitch_limit_min));
	handle = param_find("FW_P_LIM_MAX");
	if(bParamSanityChecksEnabled && Sanitize_param(handle, 0.0f, 60.0f) != 0) return -1;
	param_get(handle, &(aslctrl_params.pitch_limit_max));
	handle = param_find("FW_THR_MIN");
	param_get(handle, &(aslctrl_params.throttle_min));
	handle = param_find("FW_THR_MAX");
	param_get(handle, &(aslctrl_params.throttle_max));
	handle = param_find("FW_THR_CRUISE");
	param_get(handle, &(aslctrl_params.throttle_cruise));
	handle = 	param_find("FW_T_TIME_CONST");
	param_get(handle, &(aslctrl_params.time_const));
	handle = param_find("FW_T_TC_THROT");
	param_get(handle, &(aslctrl_params.time_const_throt));
	handle = param_find("FW_T_SINK_MIN");
	param_get(handle, &(aslctrl_params.min_sink_rate));
	handle = param_find("FW_T_SINK_MAX");
	param_get(handle, &(aslctrl_params.max_sink_rate));
	handle = param_find("FW_T_CLMB_MAX");
	param_get(handle, &(aslctrl_params.max_climb_rate));
	handle = param_find("FW_T_THR_DAMP");
	param_get(handle, &(aslctrl_params.throttle_damp));
	handle =	param_find("FW_T_INTEG_GAIN");
	param_get(handle, &(aslctrl_params.integrator_gain));
	handle = param_find("FW_T_VERT_ACC");
	param_get(handle, &(aslctrl_params.vertical_accel_limit));
	handle = param_find("FW_T_HGT_OMEGA");
	param_get(handle, &(aslctrl_params.height_comp_filter_omega));
	handle =	param_find("FW_T_SPD_OMEGA");
	param_get(handle, &(aslctrl_params.speed_comp_filter_omega));
	handle = param_find("FW_T_RLL2THR");
	param_get(handle, &(aslctrl_params.roll_throttle_compensation));
	handle = param_find("FW_T_SPDWEIGHT");
	param_get(handle, &(aslctrl_params.speed_weight));
	handle = param_find("FW_T_PTCH_DAMP");
	param_get(handle, &(aslctrl_params.pitch_damping));
	handle = param_find("FW_T_HRATE_P");
	param_get(handle, &(aslctrl_params.heightrate_p));
	handle = param_find("FW_T_HRATE_FF");
	param_get(handle, &(aslctrl_params.heightrate_ff));
	handle =	param_find("FW_T_SRATE_P");
	param_get(handle, &(aslctrl_params.speedrate_p));
	handle = param_find("FW_T_THRSLEW");
	param_get(handle, &(aslctrl_params.throttle_slewrate));
	handle = param_find("FW_T_ThrILim");
	param_get(handle, &(aslctrl_params.throttle_ILim));

	// System Integrity checks
	handle = param_find("SAFE_IThrotWarn");
	param_get(handle, &(aslctrl_params.IThrotWarn));

	//CLSYSID
	handle = param_find("CLSYSID_tExcite");
	param_get(handle, &(aslctrl_params.CLSYSID_tExcite));
	handle = param_find("CLSYSID_step");
	param_get(handle, &(aslctrl_params.CLSYSID_step));
	handle = param_find("CLSYSID_nom_pitch");
	param_get(handle, &(aslctrl_params.CLSYSID_nom_pitch));
	handle = param_find("CLSYSID_nom_roll");
	param_get(handle, &(aslctrl_params.CLSYSID_nom_roll));
	handle = param_find("CLSYSID_maneuver");
	param_get(handle, &(aslctrl_params.CLSYSID_maneuver));
	handle = param_find("CLSYSID_ctrlinput");
	param_get(handle, &(aslctrl_params.CLSYSID_ctrlinput));
	handle = param_find("CLSYSID_repeat");
	param_get(handle, &(aslctrl_params.CLSYSID_repeat));

	return 0;
}

int subscriptions::Sanitize_param(param_t & parameter_handle, float minval, float maxval)
{
	float temp_param;
	param_get(parameter_handle, &(temp_param));

	if(temp_param > maxval) {
		param_set(parameter_handle,&maxval);
		return -1;
	}
	else if (temp_param < minval) {
		param_set(parameter_handle,&minval);
		return -1;
	}
	else return 0;
}

int subscriptions::publish_actuator_outputs(void)
{
	/* sanity check and publish actuator outputs */
	if (isfinite(actuators.control[0]) &&
		isfinite(actuators.control[1]) &&
		isfinite(actuators.control[2]) &&
		isfinite(actuators.control[3]) &&
		isfinite(actuators.control[4]) &&
		isfinite(actuators.control[5]))
	{
		orb_publish(ORB_ID(actuator_controls_1), actuators_1_pub, &actuators);	// publish the aslctr values
		orb_publish(ORB_ID(actuator_controls_0), actuators_0_pub, &actuators_zeros);	// publish the 0-value actuator group - triggers PX4IO update

		return 0;
	}
	else return -1;
}

int subscriptions::publish_aslctrl_params()
{
	orb_publish(ORB_ID(aslctrl_parameters), aslctrl_params_pub, &aslctrl_params);
	return 0;
}

int subscriptions::publish_aslctrl_data()
{
	orb_publish(ORB_ID(aslctrl_data), aslctrl_data_pub, &aslctrl_data);
	return 0;
}


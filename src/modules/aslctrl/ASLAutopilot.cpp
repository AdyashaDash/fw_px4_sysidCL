#include "ASLAutopilot.h"
#include <mavlink/mavlink_log.h>
#include <fcntl.h>
#include "helpers/helpfuncs.h"
#include "helpers/consts.h"
#include <mathlib/mathlib.h>

const int RET_OK = 0;

uint64_t t1_old, t2_old, t3_old; //DEBUG

//Small helper functions
bool MavlinkSendOK(int MsgID);

//**********************************************************************
//*** Constructors / Destructors
//**********************************************************************

ASLAutopilot::ASLAutopilot() :
	SAScontrol(&subs),
	CAScontrol(&subs),
	/*CAS_MPC_control(&params,&subs),*/
	HLcontrol(&subs),
	initialized(false), counter(0)
{
	//Open mavlink port for logging
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	if (mavlink_fd < 0) {warnx("ERROR: Failed to open MAVLink log stream again, start mavlink app first.");	}

	ReloadParameters();

	//Some initialization
	ctrldata=&subs.aslctrl_data;
	ctrldata->bEngageSpoilers=false;
	ctrldata->timestamp=hrt_absolute_time();
	params=&subs.aslctrl_params;

	lastTime_MotorOK = hrt_absolute_time();

	bRunOnce=false;
	initialized=true;
}
ASLAutopilot::~ASLAutopilot()
{
	// send one last publication when destroyed, setting all output to zero
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++)
		subs.actuators.control[i] = 0.0f;

	subs.publish_actuator_outputs();
	initialized=false;
	printf("flashforce3");
}

//**********************************************************************
//*** Functions
//**********************************************************************
void ASLAutopilot::update()
{
	int RET = 0;		//Return value variable

	//******************************************************************************************************************
	//*** INPUT UPDATE (SUBSCRIPTIONS AND PARAMETERS)
	//******************************************************************************************************************

	// Get new subscription data if available
	if(subs.get_inputs() < RET_OK) {
		if(bRunOnce) printf("[aslctrl] Error retrieving input subscriptions. Not executing control loop!\n");
		return;
	}

	// Get new parameters (if they have been updated)
	if(subs.check_aslctrl_params_changed())
		ReloadParameters();

	//******************************************************************************************************************
	//*** SETUP
	//******************************************************************************************************************

	//Reset control and logging variables where necessary
	SetCtrlData();

	//SetReferences();

	// check for sane values of dt to prevent large control responses
	ctrldata->dt = hrt_elapsed_time(&ctrldata->timestamp);
	ctrldata->timestamp = hrt_absolute_time();
	if (ctrldata->dt > 1.0E6) {
		if(bRunOnce) printf("[aslctrl] WARNING, time step (dt=%u[us]) not valid. Not executing control loop!\n",ctrldata->dt);
		return;
	}

	bRunOnce=true;

	//******************************************************************************************************************
	//*** RC LOSS HANDLING
	//******************************************************************************************************************

	//Save the last mode to signify any re-initializations necessary due to a mode switch
	ctrldata->aslctrl_last_mode=ctrldata->aslctrl_mode;

	//SetControlMode();

	//Check if we have RC reception.
	if(subs.vstatus.rc_signal_lost) {
		if(HandleRCLoss() != 0) return; // Either error or user wants manual PX4-failsafe -> exit this function.
	}
	if(ctrldata->aslctrl_mode >= MODE_RCLOSS_MANFAILSAFE && params->ASLC_DEBUG==11) printf("mode: %u, last_mode: %u\n",ctrldata->aslctrl_mode,ctrldata->aslctrl_last_mode);

	//******************************************************************************************************************
	//*** MAIN CONTROL LOOP
	//******************************************************************************************************************

	//Manual feed through of the following inputs in all cases. This might might be augmented by the SAS-loop.
	ctrldata->uRud=subs.manual_sp.r;

	// TEMP: Determine airspeed/throttle set point here:
	if(params->ASLC_VelCtrl==0 || ctrldata->aslctrl_mode < MODE_ALT) {		// Set new throttle reference if VELCTRL not activate at all (simple feed-through then))
		ctrldata->uThrot=subs.manual_sp.z;
		ctrldata->uThrot2=subs.manual_sp.aux2;
		ctrldata->AirspeedRef=0.0f;
	}
	else {
		// Scale Airspeed reference (from uThrot=[0...1] to Airspeed Ref=[vMin...vMax])
		// Note: Do not reset uThrot here. If velocity control is not executed in this control-loop-run (e.g. because it runs
		// 		 slower than SAS&CAS) then this would be falsly set to the manual throttle setpoint. Instead, we still want the
		//		 velocity controlled one!
		ctrldata->AirspeedRef=params->HL_Vel_vMin+subs.manual_sp.z*(params->HL_Vel_vMax-params->HL_Vel_vMin);
	}

	//******************************************************************************************************************
	//*** LOOP 1: HIGH-LEVEL AUTOPILOT (Waypoint/HDG & Altitude hold)
	//******************************************************************************************************************
	// Fully autonomous Autopilot (AUTO) control.
	//   Loop 1a: Lateral Autopilot (e.g. waypoint following)
	// 	 Loop 1b: Longitudinal / Altitude Autopilot

	// Run TECS update every time (should be called at >50Hz)
	HLcontrol.TECS_Update50Hz();

	if(subs.vstatus.main_state==vehicle_status_s::MAIN_STATE_AUTO_MISSION && (counter % params->HL_fMult == 0) && subs.position_setpoint_triplet.current.valid)
	{
		if((counter %20==0) && (params->ASLC_DEBUG==1)) printf("dt_wp:%8.6f\n", double(hrt_absolute_time()-t3_old)/1.0e6);
		t3_old=hrt_absolute_time();

		//******************************************************************************************************************
		//*** LOOP 1a: LATERAL AUTOPILOT
		//******************************************************************************************************************
		//LOOP 1a, CASE 1: Waypoint-following (WP-following AUTO via GCS, Altitude AUTO via GCS)
		if(subs.manual_sp.posctl_switch==manual_control_setpoint_s::SWITCH_POS_ON)	{
			if(!subs.vstatus.rc_signal_lost) ctrldata->aslctrl_mode = MODE_AUTO;

			RET = HLcontrol.WaypointControl_L1(ctrldata->RollAngleRef);
			if(RET != RET_OK && MavlinkSendOK(0)) {
				 mavlink_log_critical(mavlink_fd, "[aslctrl] L1 HL CTRL ERROR: CODE %d", RET);
			}
		}
		//LOOP 1a, CASE 2: Roll Angle feed-through (only Altitude hold)
		else if(subs.manual_sp.posctl_switch==manual_control_setpoint_s::SWITCH_POS_OFF) {
			//Roll angle CAS, altitude AUTO via GCS
			ctrldata->aslctrl_mode = MODE_ALT;
			ctrldata->RollAngleRef=-params->SAS_RollPDir*subs.manual_sp.y * params->CAS_RollAngleLim;;	//Scaling to reference angles
		} else {
				//Switch back to AUTO
				ctrldata->aslctrl_mode = MODE_RCLOSS_AUTOFAILSAFE;
		}

		//******************************************************************************************************************
		//*** LOOP 1b&c: ALTITUDE & AIRSPEED-CTRL
		//******************************************************************************************************************
		ctrldata->hRef=subs.position_setpoint_triplet.current.alt;
		bool bReinit=(ctrldata->aslctrl_mode-ctrldata->aslctrl_last_mode > 0 ? true : false);

		//Option 1b1 & 1c1: Classical, SISO alt&speed ctrl. based on the assumption of a decoupled pitch&throttle response
		if(params->ASLC_VelCtrl < 3) {
			//Old stuff. Don't do anything for now.
		}
		//Option 1b2 & 1c2: Total Energy Control System (TECS) approach, i.e. MIMO approach for pitch&throttle commands for altitude and speed control
		else if (params->ASLC_VelCtrl >= 3){
			//Use altitude ramp?
			bool bUseAltitudeRamp=false;
			bool bUseThermalHighEtaMode=false;
			if(params->ASLC_VelCtrl > 3) bUseAltitudeRamp=true;
			if(params->ASLC_VelCtrl ==5) bUseThermalHighEtaMode=true;

			//TECS Alt&Airspeed control
			RET = HLcontrol.TECS_AltAirspeedControl(ctrldata->PitchAngleRef, ctrldata->uThrot, ctrldata->AirspeedRef, ctrldata->hRef,
					subs.global_pos.alt, subs.home_pos.alt, ctrldata->hRef_t, ctrldata->bEngageSpoilers,bUseAltitudeRamp, bUseThermalHighEtaMode,bReinit);
			if(params->ASLC_DEBUG==10) printf("RET:%i\n",RET);
			if(RET != RET_OK && MavlinkSendOK(2)) {
				mavlink_log_critical(mavlink_fd, "[aslctrl] %s", aslctrl_error_codes[-RET]);
			}
		}
	}

	//******************************************************************************************************************
	//*** LOOP 2: CAS (ROLL&PITCH ANGLE HOLD)
	//******************************************************************************************************************
	// Control Augmented System (CAS) control.
	// Attitude (Angle) Control with reference inputs from RC-transmitter.
	if(subs.vstatus.main_state >= (main_state_t)MODE_CAS && (counter % params->CAS_fMult == 0))
	{
		//if((counter %20==0) && (params->ASLC_DEBUG==2)) printf("dt_cas:%8.6f\n", double(hrt_absolute_time()-t2_old)/1.0e6f);
		//t2_old=hrt_absolute_time();

		if (subs.vstatus.main_state == (main_state_t)MODE_CAS && !subs.vstatus.rc_signal_lost) 
		{
			//We are exactly in CAS mode, update references

			//Closed Loop System ID starts here:
			double id_switch = subs.manual_sp.aux3; //Check if in CLSYSID Mode
			if(id_switch > 0.5 || id_switch < -0.5){
				bool bModeChanged = false;
				if (ctrldata->aslctrl_mode != MODE_CLSYSID){
					//Change mode if first time in loop
					ctrldata->aslctrl_mode = MODE_CLSYSID;
					bModeChanged = true;
				}
				float rangleref = params->CLSYSID_nom_roll*DEG2RAD;
				float pangleref = params->CLSYSID_nom_pitch*DEG2RAD;
				RET = HLcontrol.CLSYSIDControl(pangleref, rangleref, bModeChanged);
				if(RET){
				ctrldata->aslctrl_mode = MODE_CAS;
				ctrldata->RollAngleRef = rangleref;
				ctrldata->PitchAngleRef = pangleref;
				}
			} else {
			ctrldata->aslctrl_mode = MODE_CAS;
			ctrldata->RollAngleRef = -params->SAS_RollPDir*subs.manual_sp.y * params->CAS_RollAngleLim; //Inputs scaled to reference angles
			ctrldata->PitchAngleRef = params->SAS_PitchPDir*subs.manual_sp.x * params->CAS_PitchAngleLim; //Inputs scaled to reference angles
			}
		}

		// Perform Pitch&Roll Angle control function. This includes gain scheduling & stall protection.
		bool bReinit=(ctrldata->aslctrl_mode-ctrldata->aslctrl_last_mode>0 ? true : false);
		// OPTION1: Do this using the MPC controller
		if(0){/*params->ASLC_CtrlType==MPC_STD || params->ASLC_CtrlType==MPC_ROLLMPCONLY) {
			if(RET_OK != CAS_MPC_control.CASRollPitchControl_MPC(ctrldata->uAil, ctrldata->uElev, ctrldata->RollAngleRef, subs.att.roll, ctrldata->p, ctrldata->PitchAngleRef, subs.att.pitch, ctrldata->q, bReinit)) {
				//Catch errors here
			}*/
		}
		// OPTION2 (DEFAULT): Do this using the standard PID controller
		else {
			if(RET_OK != CAScontrol.CASRollPitchControl(ctrldata->pRef,ctrldata->qRef,ctrldata->rRef,ctrldata->RollAngleRef,subs.att.roll,ctrldata->PitchAngleRef,subs.att.pitch, ctrldata, bReinit)) {
				//Catch errors here
			}
		}

	}


	//******************************************************************************************************************
	//*** LOOP 3: SAS (RATE CONTROL / STABILIZATION)
	//******************************************************************************************************************
	// Stability Augmented System (SAS) control
	// Damping of roll, pitch and yaw with reference inputs from RC-transmitter.
	if(subs.vstatus.main_state>=(main_state_t)MODE_SAS) {
		if((counter %20==0) && (params->ASLC_DEBUG==2)) {
			printf("dt_sas:%8.6f\n", double(hrt_absolute_time()-t1_old)/1.0e6);
		}
		//t1_old=hrt_absolute_time();

		// Check if the execution frequency of SAS (as the innermost loop) is ok, i.e. corresponds to the one given by the user.
		if(params->ASLC_DEBUG==2 && (ctrldata->dt<0.9f*params->SAS_tSample*1.0e6f || ctrldata->dt>1.3f*params->SAS_tSample*1.0e6f)) {
			printf("[aslctrl] WARNING, dt=%u[us] not matching parameter SAS_tSample=%8.5f[s]\n",ctrldata->dt,(double)params->SAS_tSample);
			if(MavlinkSendOK(6)) {mavlink_log_critical(mavlink_fd, "[aslctrl] WARNING: dt=%u [us] != SAS_tSample!",ctrldata->dt);}
		}

		if(subs.vstatus.main_state==(main_state_t)MODE_SAS) {
			//We are exactly in SAS mode, update references
			ctrldata->aslctrl_mode = MODE_SAS;
			ctrldata->uElev=-subs.manual_sp.x;
			ctrldata->uAil=subs.manual_sp.y;

			bool bReinit=(ctrldata->aslctrl_mode-ctrldata->aslctrl_last_mode>0 ? true : false);
			if(params->ASLC_CtrlType != MPC_STD && params->ASLC_CtrlType != MPC_ROLLMPCONLY) { //Only execute for standard PID controller, not for MPC controller (which does rate-stabilization/control internally)
				SAScontrol.StabilityAugmentation(ctrldata->uAil, ctrldata->uElev, ctrldata->uRud, ctrldata->f_GainSch_Q, subs.att.rollspeed,subs.att.pitchspeed,subs.att.yawspeed, bReinit);
			}
		}
		else {
			bool bReinit=(ctrldata->aslctrl_mode-ctrldata->aslctrl_last_mode>0 ? true : false);
			if(params->ASLC_CtrlType != MPC_STD && params->ASLC_CtrlType != MPC_ROLLMPCONLY) { //Only execute for standard PID controller, not for MPC controller (which does rate-stabilization/control internally)
				RET = SAScontrol.RateControl(ctrldata->pRef,ctrldata->qRef, ctrldata->rRef, ctrldata->uAil, ctrldata->uElev, ctrldata->uRud, ctrldata->f_GainSch_Q, subs.att.rollspeed,subs.att.pitchspeed,subs.att.yawspeed, ctrldata->RollAngle, ctrldata->RollAngleRef, bReinit);
				if(RET==-1)	{if(MavlinkSendOK(4)) {mavlink_log_critical(mavlink_fd, "[aslctrl] ERROR: SAS Rate Control. Params != Zero?");}}
				if(RET==-2) {if(MavlinkSendOK(7)) {mavlink_log_critical(mavlink_fd, "[aslctrl] WARNING: Overspeed detected!");}}
			}
		}

		//debug
		//printf("[CAS-Ctrl2] uAil: %7.5f uElev: %7.5f uRud: %7.5f\n", uAil, uElev, uRud);
		//printf("[CAS-Ctrl] Pitch Ref: %7.5f Pitch %7.5f qref: %7.5f q: %7.5f uelev: %7.5f\n", ctrldata->PitchAngleRef, subs.att.pitch, ctrldata->qRef,subs.att.pitchspeed,ctrldata->uElev);
		//printf("[CAS-Ctrl] Roll Ref: %7.5f Roll %7.5f pref: %7.5f uail: %7.5f\n", RollAngleRef, subs.att.roll, pref,uAil);
	}

	//******************************************************************************************************************
	//*** MANUAL MODE
	//******************************************************************************************************************
	if(subs.vstatus.main_state==vehicle_status_s::MAIN_STATE_MANUAL) {
		// Full manual (MANUAL) control

		// Note: In MANUAL mode, the actuator outputs are SOLELY driven through PX4IO, i.e. the RC commands follow the
		// PX4IO RC Input -> PX4IO mixer -> PX4IO Output path! This means that although the autopilot code here is executed,
		// it has NO EFFECT on the actual outputs!

		//Set these values, but only for proper logging of the rc-inputs, because they have no actual effect on the actuator outputs
		ctrldata->aslctrl_mode = MODE_MANUAL;
		ctrldata->uElev=-subs.manual_sp.x;
		ctrldata->uAil=subs.manual_sp.y;
	}

	//ctrldata->aZ=subs.sensors.accelerometer_m_s2[2]-subs.ekf.x_b_a[2];
	//float aS=sinf(ctrldata->RollAngle)*-ctrldata->aZ + cosf(ctrldata->RollAngle)*(subs.sensors.accelerometer_m_s2[1]-subs.ekf.x_b_a[1]);
	//if((counter % 20 == 0) && params->ASLC_DEBUG) printf("Yawref:%7.4f° RollRef:%7.4f° aScmd:%7.4f aS:%7.4f\n",ctrldata->YawAngleRef*RAD2DEG,ctrldata->RollAngleRef*RAD2DEG,HLcontrol.nav_lateral_acceleration_demand(),aS);

	//******************************************************************************************************************
	//*** UPDATE ACTUATORS
	//******************************************************************************************************************

	// Step1: Safety precaution: Check whether we need to disable the throttle commands.
	// This is either when still on-ground&rc lost or on-ground&any-auto-mode.
	if(OnGround() && subs.vstatus.arming_state>=vehicle_status_s::ARMING_STATE_ARMED &&
			(subs.vstatus.rc_signal_lost || ctrldata->aslctrl_mode>=MODE_ALT)) {
		ctrldata->uThrot=0.0f;
		ctrldata->uThrot2=0.0f;
		if(counter%20==0 && params->ASLC_DEBUG==30) printf("DEBUG: Disabling throttle. uThrot=%.3f\n",(double)ctrldata->uThrot);
		if(MavlinkSendOK(8)) {mavlink_log_critical(mavlink_fd, "[aslctrl] WARNING: Disabling throttle!(OnGround)\n");}
	}
	else {
		if(counter%20==0 && params->ASLC_DEBUG==30) printf("DEBUG: NOT Disabling throttle. uThrot=%.3f\n",(double)ctrldata->uThrot);
	}

	// Step2: Do the actual actuator updates
	subs.actuators.control[CH_AIL_L] = ctrldata->uAil;		//Aileron-Differential applied in PX4IO-mixer
	subs.actuators.control[CH_ELV] = ctrldata->uElev;
	subs.actuators.control[CH_RDR] = ctrldata->uRud;
	subs.actuators.control[CH_THR_1] = ctrldata->uThrot;

	//Flaps/Spoilers on CH_FLAPS
	if(subs.manual_sp.flaps > 0.5f || ctrldata->bEngageSpoilers) subs.actuators.control[CH_FLAPS] = 1.0f;
	else if(subs.manual_sp.flaps < -0.5f) subs.actuators.control[CH_FLAPS] = -1.0f;
	else subs.actuators.control[CH_FLAPS]=0.0f;

	subs.actuators.control[CH_AIL_R] = ctrldata->uAil;		//Aileron-Differential applied in PX4IO-mixer
	subs.actuators.control[CH_AUX1] = ctrldata->uThrot;

	//******************************************************************************************************************
	//*** SYSTEM INTEGRITY CHECKS
	//******************************************************************************************************************
	// Check 1: Motor functionality
	if(fabsf(params->IThrotWarn) > 0.01f) {
		if(fabsf(subs.sensor_power.adc121_cspb_amp) < fabsf(params->IThrotWarn) && ctrldata->uThrot > 0.95f * params->throttle_max) {
			if(hrt_elapsed_time(&lastTime_MotorOK) > 5000000) {
				mavlink_log_critical(mavlink_fd, "[aslctrl] Potential motor failure\n")
				lastTime_MotorOK = hrt_absolute_time();
			}
		}
		else lastTime_MotorOK = hrt_absolute_time();
	}

	//Debug
	if ((counter % 20 == 0) && (params->ASLC_DEBUG==1)) {
		printf("UMIXED actuators[1-6]: %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n",
				(double)subs.actuators.control[0],(double)subs.actuators.control[1],(double)subs.actuators.control[2],
				(double)subs.actuators.control[3],(double)subs.actuators.control[4],(double)subs.actuators.control[5]);
	}

	//Debug
	if ((counter % 20 == 0) && (params->ASLC_DEBUG==1)) {
		printf("MIXED actuators[1-6]: %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n",double(subs.actuators.control[0]),double(subs.actuators.control[1]),double(subs.actuators.control[2]),
				double(subs.actuators.control[3]),double(subs.actuators.control[4]),double(subs.actuators.control[5]));
	}

	// update all publications
	if(subs.publish_actuator_outputs()!=0 && MavlinkSendOK(5))
		mavlink_log_critical(mavlink_fd, "[aslctrl] ERROR setting actuator outputs! Sane?");
	subs.publish_aslctrl_data();

	counter++;
}

int ASLAutopilot::SetCtrlData(void)
{
	//Current values
	//Longitudinal loop
	ctrldata->h=subs.global_pos.alt;
	ctrldata->PitchAngle=subs.att.pitch;
	ctrldata->q=subs.att.pitchspeed;

	//Lateral loops
	ctrldata->YawAngle=subs.att.yaw;
	ctrldata->RollAngle=subs.att.roll;
	ctrldata->Yawdot=0.0f;
	ctrldata->Yawdot_ref=0.0f;
	ctrldata->rRef=0.0f;
	ctrldata->p=subs.att.rollspeed;
	ctrldata->r=subs.att.yawspeed;

	//Only reset final outputs (because these will be updated in any case)
	ctrldata->uAil=0.0f;
	ctrldata->uRud=0.0f;
	ctrldata->uElev=0.0f;

	ctrldata->bEngageSpoilers=false;

	//Do NOT RESET any control variables , e.g. none of the following variables:
	// - high-level references (these might not be re-updated because not all loops run at full frequency)
	// - ctrldata->aslctrl_mode=-1; //The control mode (manual, assisted, auto...)
	// - uThrot. uThrot2

	return 0;
}

void ASLAutopilot::ReloadParameters()
{
	int RET = subs.update_aslctrl_params();
	if (RET == 0) {
		//Params were updated successfully. Pass them to the specific controllers.
		SAScontrol.CopyUpdatedParams();
		CAScontrol.CopyUpdatedParams();
		HLcontrol.CopyUpdatedParams();
		//CAS_MPC_control.CopyUpdatedParams();

		subs.publish_aslctrl_params();

		//printf("aslctrl: Parameters updated successfully! \n");
	}
	else if(RET == -1) {
		warnx("ERROR updating parameters (Reason: limits exceeded)! Default values set. Verify your inputs!");
		mavlink_log_critical(mavlink_fd, "[aslctrl] ERROR updating parameters (Reason: limits exceeded)!");
		mavlink_log_critical(mavlink_fd, "[aslctrl] Default values set. Verify your inputs!");
	}
}

const int 		MaxID=8;
uint64_t 		MavlinkMsg_time[MaxID+1]={0};
const uint64_t 	delta_t[MaxID+1] = {5000000,5000000,5000000,5000000,5000000,5000000,5000000,5000000,10000000};	//in microseconds

bool MavlinkSendOK(int MsgID)
{
	if(MsgID<0 || MsgID>MaxID) { return false; }
	else if (hrt_absolute_time() < MavlinkMsg_time[MsgID] + delta_t[MsgID]) { return false; }

	//Checks ok, we can update & send
	MavlinkMsg_time[MsgID]=hrt_absolute_time();
	return true;
}

int ASLAutopilot::HandleRCLoss(void)
{
	// Notes:
	// 1) The new mode, MODE_RCLOSS_X will only be set IF the corresponding control routine will be called in the same run afterwards.
	//    This serves a) to guarantee proper reinitialization of the filters (e.g. LP-filters) after a change and b) to avoid
	//	  runs with uninitialized data (e.g., if we are in MODE_MANUAL and the rc-signal is lost, and we have ASLC_OnRCLoss=2,
	//	  if we'd directly set MODE_RCLOSS_RTHFAILSAFE then the CAS-loop might (given HL_fMult>1 and an appropriate counter value)
	//    be executed without initialization values).
	// 2) IFF we are in the correct mode already, we set all values EVERY TIME because they'd be overwritten otherwise.

	if(subs.aslctrl_params.ASLC_OnRCLoss == 0) {
		// MANUAL FAILSAFE: Don't do anything (causes PX4 to go into failsafe).

		ctrldata->aslctrl_mode = MODE_RCLOSS_MANFAILSAFE;
		return 1;
	}
	else if(subs.aslctrl_params.ASLC_OnRCLoss == 2) {
		// RTH FAILSAFE: Return-To-Home via auto/waypoint following mode

		if(counter % params->HL_fMult == 0) ctrldata->aslctrl_mode = MODE_RCLOSS_RTHFAILSAFE;

		if(ctrldata->aslctrl_mode == MODE_RCLOSS_RTHFAILSAFE) {
			// Overwrite switch positions locally here, in order to enter AUTO mode in control loop
			subs.vstatus.main_state = vehicle_status_s::MAIN_STATE_AUTO_MISSION;
			subs.manual_sp.posctl_switch = manual_control_setpoint_s::SWITCH_POS_ON;
			//Set RTL references
			//subs.position_setpoint_triplet.nav_state = NAV_CMD_RETURN_TO_LAUNCH; //TODO check whether this works
			subs.position_setpoint_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
			subs.position_setpoint_triplet.current.alt = subs.home_pos.alt + 100.0f;
			subs.position_setpoint_triplet.current.lat = subs.home_pos.lat;
			subs.position_setpoint_triplet.current.lon = subs.home_pos.lon;
			//printf("RTL alt:%.2f\n",subs.global_pos_set_triplet.current.altitude);
		}
		return 0;
	}
	else if(subs.aslctrl_params.ASLC_OnRCLoss == 3 &&
		(ctrldata->aslctrl_mode == MODE_AUTO || ctrldata->aslctrl_mode == MODE_RCLOSS_AUTOFAILSAFE)) {
		// IFF already in AUTO, continue in AUTO as usual despite of RC-loss. Don't change any references (or waypoints).
		// This might be because it is expected to fly out of range in scheduled BVLOS-operations.

		if(counter % params->HL_fMult == 0) ctrldata->aslctrl_mode = MODE_RCLOSS_AUTOFAILSAFE;

		if(ctrldata->aslctrl_mode == MODE_RCLOSS_AUTOFAILSAFE) {
			// Overwrite switch positions locally here, in order to enter AUTO mode in control loop
			subs.vstatus.main_state = vehicle_status_s::MAIN_STATE_AUTO_MISSION;
			subs.manual_sp.posctl_switch = manual_control_setpoint_s::SWITCH_POS_ON;
		}
		return 0;
	}
	else {
		// CAS FAILSAFE: Simple Loitering via CAS.
		// Note: This is also the default failsafe behaviour if an invalid failsafe setting is detected.
		if(counter % params->CAS_fMult == 0) {
			if(subs.aslctrl_params.ASLC_OnRCLoss == 1) ctrldata->aslctrl_mode = MODE_RCLOSS_CASFAILSAFE;
			else ctrldata->aslctrl_mode=MODE_RCLOSS_ERR;
		}

		if((ctrldata->aslctrl_mode == MODE_RCLOSS_CASFAILSAFE) || (ctrldata->aslctrl_mode == MODE_RCLOSS_ERR)) {
			//Overwrite switch positions locally here, in order to enter CAS mode in control loop
			subs.vstatus.main_state = (main_state_t)MODE_CAS;
			subs.manual_sp.posctl_switch = manual_control_setpoint_s::SWITCH_POS_ON;

			//Set RTL references
			ctrldata->RollAngleRef = 12.0f*DEG2RAD;		// Loiter in right-turn circle
			ctrldata->PitchAngleRef = 0.0f*DEG2RAD;		// Descend slowly
			subs.manual_sp.z = 0.0f;					// Deactivate throttle to descend
			subs.manual_sp.aux2 = 0.0f;					// TODO: This gives problems, does not put throttle to zero. See TF11 TODOs
			ctrldata->uThrot = 0.0f;
			ctrldata->uThrot2 = 0.0f;
		}
		return 0;
	}

	//None of the other cases was valid. Return error.
	return -1;
}

bool ASLAutopilot::OnGround(void)
{
	//Check conditions
	float vgnd=sqrt(pow(subs.global_pos.vel_n,2.0f)+pow(subs.global_pos.vel_e,2.0f)+pow(subs.global_pos.vel_d,2.0f));
	bool bCond1=subs.airspeed.true_airspeed_m_s < 4.5f;
	bool bCond2=sqrt(pow(subs.global_pos.vel_n,2.0f)+pow(subs.global_pos.vel_e,2.0f)+pow(subs.global_pos.vel_d,2.0f))<4.0;
	bool bCond3=subs.global_pos.alt < subs.home_pos.alt+30.0f;

	//Check validity of the data needed to assess the on-ground conditions
	if(subs.home_pos.alt<0.1f) {bCond3=true;bCond2=true;}
	// If home_alt==0, then this means that no valid home position was retrieved because the gps-signal has never been received.
	// This then also means that the global_position estimate is invalid. TODO: This should be handled differently, e.g. through
	// global_pos.valid or even through vehicle_status.XXX.

	if(params->ASLC_DEBUG == 30 && counter%20==0) printf("bCond1/2/3: %d/%d/%d. v_air/v_gnd/h/h_home:%.2f/%.2f/%.2f/%.2f\n",
			bCond1,bCond2,bCond3,(double)subs.airspeed.true_airspeed_m_s,(double)vgnd,(double)subs.global_pos.alt,(double)subs.home_pos.alt);

	if(bCond1 && bCond2 && bCond3){
		return true;
	}

	return false;
}


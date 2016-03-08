#include <systemlib/param/param.h>

//**********************************************************************
//*** PX4 Parameter defs
//**********************************************************************

/*PARAM_DEFINE_FLOAT(NAME,0.0f);*/

//***************************************************************************
//*** ASLCTRL GENERAL parameters
//***************************************************************************

PARAM_DEFINE_INT32(ASLC_DEBUG, 0);				// Enable Output Debugging Info (0=no debug information, >0 with debug information)
PARAM_DEFINE_INT32(ASLC_GainSch_E, 0);			// Enable Gain Scheduling on Error (1=yes, 0=no)
PARAM_DEFINE_INT32(ASLC_GainSch_Q, 0);			// Enable Gain Scheduling on Dynamic Pressure/Airspeed (1=yes, 0=no)
PARAM_DEFINE_INT32(ASLC_StallProt, 0);			// Enable stall protection (1=yes, 0=no)
PARAM_DEFINE_INT32(ASLC_VelCtrl, 0);			// Select velocity and altitude controller
PARAM_DEFINE_INT32(ASLC_OnRCLoss, 2);			// Sets action to take when RC signal is lost
PARAM_DEFINE_INT32(ASLC_OvSpdProt, 0);			// Activate over-speed protection (0=no, >=1 = yes)
PARAM_DEFINE_INT32(ASLC_CoordTurn, 0);			// Enable coordinated turns (1=yes, 0=no)

PARAM_DEFINE_INT32(ASLC_CtrlType, 0);
// ASLC_CtrlType: IF multiple controllers are implemented, this defines the controller type to use, i.e.
// enables switching between multiple implemented contorllers (e.g. various forms of PIDs or MPCs). See helpfuncs.h
// for a description of the implemented controllers

//***************************************************************************
//*** DEMIX parameters
//***************************************************************************
//DEBUG output enable/disable
PARAM_DEFINE_INT32(DEMIX_Enabled, 0);			//Enable demixing of inputs
//PARAM_DEFINE_FLOAT(DEMIX_F_CH2t4, 0.5f); 		//SAS_tSample, [s]
//PARAM_DEFINE_FLOAT(DEMIX_F_CH6t1, 0.5f); 		//SAS_tSample, [s]
//PARAM_DEFINE_FLOAT(DEMIX_CH2t4, 1.0f); 			//SAS_tSample, [s]
//PARAM_DEFINE_FLOAT(DEMIX_CH6t1, 1.0f); 			//SAS_tSample, [s]

//***************************************************************************
//*** SAS Gains and parameters
//***************************************************************************
//Setting default values such that they are very safe (i.e. non-active). Actual parameter values are read from PX4 parameter file.

//Sampling time
PARAM_DEFINE_FLOAT(SAS_tSample, 0.01f); 		//SAS_tSample, [s]

//Gains
PARAM_DEFINE_FLOAT(SAS_RollPGain, 0.0f);
PARAM_DEFINE_FLOAT(SAS_PitchPGain, 0.0f);
PARAM_DEFINE_FLOAT(SAS_YawPGain, 0.0f);
PARAM_DEFINE_FLOAT(SAS_RollPDir,1.0f);
PARAM_DEFINE_FLOAT(SAS_PitchPDir,1.0f);
PARAM_DEFINE_FLOAT(SAS_YawPDir,1.0f);
PARAM_DEFINE_FLOAT(SAS_RYDecK, 0.0f);			//SAS_RollYawDecoupleKari
PARAM_DEFINE_FLOAT(SAS_YawCTkP, 0.0f);			// Rudder Coordinated Turn kP
PARAM_DEFINE_FLOAT(SAS_YawCTFF, 0.0f);			// Rudder Coordinated FeedForward

//Filters
PARAM_DEFINE_FLOAT(SAS_YawHPw, 1.0f);  			//SAS_YawHighPassOmega, [rad/s]
PARAM_DEFINE_FLOAT(SAS_PitchLPw, 1.0f);  		//SAS_PitchLowPassOmega, Unit [rad/s]
PARAM_DEFINE_FLOAT(SAS_RollLPw, 1.0f);  		//SAS_RollLowPassOmega, Unit [rad/s]

//Ctrl Input limiters, applied to corrective ctrl-command, not to user inputs
PARAM_DEFINE_FLOAT(SAS_RCtrlLim, 0.0f);
PARAM_DEFINE_FLOAT(SAS_PCtrlLim, 0.0f);
PARAM_DEFINE_FLOAT(SAS_YCtrlLim, 0.0f);

//Dynamic Pressure Scaling
PARAM_DEFINE_FLOAT(SAS_vScaleLimF, 1.0f);		// Limiting factor on airspeeds for dynamic pressure scaling
PARAM_DEFINE_FLOAT(SAS_vScaleExp, 2.0f);		// Exponent of the dynamic pressure scaling

//Trims
PARAM_DEFINE_FLOAT(SAS_TrimAilvNom, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimAilvMin, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimAilvMax, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimElevNom, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimElevMin, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimElevMax, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimRudvNom, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimRudvMin, 0.0f);
PARAM_DEFINE_FLOAT(SAS_TrimRudvMax, 0.0f);

//***************************************************************************
//*** CAS Gains and parameters
//***************************************************************************
//Setting default values such that they are very safe (i.e. non-active). Actual parameter values are read from PX4 parameter file.

//Sampling time
PARAM_DEFINE_INT32(CAS_fMult, 1);			//Frequency multiple of the SAS vs. the CAS loop

//Gains
PARAM_DEFINE_FLOAT(CAS_PitchPGain, 0.0f);	// Pitch P-Gain, nominal
PARAM_DEFINE_FLOAT(CAS_PitchPGainM, 0.0f);	// Pitch P-Gain, minimal (for gain scheduling)
PARAM_DEFINE_FLOAT(CAS_PitchIGain, 0.0f);	// Pitch I-Gain
PARAM_DEFINE_FLOAT(CAS_PitchTCkI, 0.0f);	// Pitch Turn Compensation I-Gain
PARAM_DEFINE_FLOAT(CAS_PitchTCILim, 0.0f);	// Pitch Turn Compensation I-Limit
PARAM_DEFINE_FLOAT(CAS_RollPGain, 0.0f);	// Roll P-Gain, nominal
PARAM_DEFINE_FLOAT(CAS_RollPGainM, 0.0f);	// Roll P-Gain, minimal (for gain scheduling)

//Limits
PARAM_DEFINE_FLOAT(CAS_PRateLim, 0.0f);		// Maximum pitch rate / elevator command from CAS
PARAM_DEFINE_FLOAT(CAS_PRateILim, 0.0f);	// Maximum pitch rate due to integrator
PARAM_DEFINE_FLOAT(CAS_RRateLim, 0.0f);		// Maximum roll rate / aileron command from CAS
PARAM_DEFINE_FLOAT(CAS_YRateLim, 0.0f);		// Maximum yaw rate / rudder command from CAS
PARAM_DEFINE_FLOAT(CAS_PAngleLim, 0.0f);	// Maximum pitch angle
PARAM_DEFINE_FLOAT(CAS_RAngleLim, 0.0f);	// Maximum roll angle

//Other Protections

//Feed-forwards for coordinated turns
PARAM_DEFINE_FLOAT(CAS_uElevTurnFF, 0.0f);	// Elevator feed forward gain

//Filters
PARAM_DEFINE_FLOAT(CAS_YawLPw, 0.0f);

//***************************************************************************
//*** HighLevel Control (WP Following Gains and parameters, velocity control)
//***************************************************************************
//Setting default values such that they are very safe (i.e. non-active). Actual parameter values are read from PX4 parameter file.

PARAM_DEFINE_INT32(HL_fMult, 1);			// Frequency multiple of the SAS vs. the HL loop
PARAM_DEFINE_FLOAT(HL_WPL1_P_vMin, 25.0f);	// L1 Period Gain Scheduling - vMin
PARAM_DEFINE_FLOAT(HL_WPL1_P_vNom, 25.0f);	// L1 Period Gain Scheduling - vNom
PARAM_DEFINE_FLOAT(HL_WPL1_P_vMax, 25.0f);	// L1 Period Gain Scheduling - vMax
PARAM_DEFINE_FLOAT(HL_WPL1_Damping, 0.75f);
PARAM_DEFINE_FLOAT(HL_Vel_vNom, 9.0f);		// Nominal airspeed. This is the operating point for which the SAS DC-gains are tuned.
PARAM_DEFINE_FLOAT(HL_Vel_vMin, 7.0f);		// Minimum airspeed to be controlled
PARAM_DEFINE_FLOAT(HL_Vel_vMax, 16.0f);		// Maximum ...
//Old altitude controller
PARAM_DEFINE_FLOAT(HL_AltLPw, 0.0f);
PARAM_DEFINE_FLOAT(HL_AlthMax, -1.0f);		// Maximum allowable altitude; primarily used in thermal compliance alt ctrl mode
PARAM_DEFINE_FLOAT(HL_AlthMin, 0.0f);		// Minimum allowable altitude; Allow uThrot higher than uThrotMax if we are pushed below this
PARAM_DEFINE_FLOAT(HL_vZClimb, 0.0f);		// (Maximum) climb speed that the altitude controller shall prescribe
PARAM_DEFINE_FLOAT(HL_vZSink, 0.0f);		// (Maximum) climb speed that the altitude controller shall prescribe

// TECS
PARAM_DEFINE_FLOAT(FW_THR_CRUISE, 0.7f);
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);
PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);
PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -45.0f);
PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 45.0f);
PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX, 5.0f);
PARAM_DEFINE_FLOAT(FW_T_SINK_MIN, 2.0f);
PARAM_DEFINE_FLOAT(FW_T_TIME_CONST, 5.0f);
PARAM_DEFINE_FLOAT(FW_T_TC_THROT, 5.0f);	// (ASL/PhOe) Throttle time constant - used for P-gain calulcation
PARAM_DEFINE_FLOAT(FW_T_THR_DAMP, 0.5f);
PARAM_DEFINE_FLOAT(FW_T_INTEG_GAIN, 0.1f);
PARAM_DEFINE_FLOAT(FW_T_VERT_ACC, 7.0f);
PARAM_DEFINE_FLOAT(FW_T_HGT_OMEGA, 3.0f);
PARAM_DEFINE_FLOAT(FW_T_SPD_OMEGA, 2.0f);
PARAM_DEFINE_FLOAT(FW_T_RLL2THR, 10.0f);
PARAM_DEFINE_FLOAT(FW_T_SPDWEIGHT, 1.0f);
PARAM_DEFINE_FLOAT(FW_T_PTCH_DAMP, 0.0f);
PARAM_DEFINE_FLOAT(FW_T_SINK_MAX, 5.0f);
PARAM_DEFINE_FLOAT(FW_T_HRATE_P, 0.05f);
PARAM_DEFINE_FLOAT(FW_T_HRATE_FF, 0.0f);
PARAM_DEFINE_FLOAT(FW_T_SRATE_P, 0.05f);
PARAM_DEFINE_FLOAT(FW_T_THRSLEW, 0.00f);	//Added
PARAM_DEFINE_FLOAT(FW_T_ThrILim, 0.00f);	//Added

// SYSTEM INTEGRITY CHECKS
PARAM_DEFINE_FLOAT(SAFE_IThrotWarn, 0.0f);	// Motor failure warning based on an expected motor current. Default is warning off (param = 0.0)

//CL SYS ID
PARAM_DEFINE_FLOAT(CLSYSID_tExcite, 0.0f);
PARAM_DEFINE_FLOAT(CLSYSID_step, 0.0f);
PARAM_DEFINE_FLOAT(CLSYSID_nom_pitch, 0.0f);
PARAM_DEFINE_FLOAT(CLSYSID_nom_roll, 0.0f);
PARAM_DEFINE_INT32(CLSYSID_maneuver, 0);
PARAM_DEFINE_INT32(CLSYSID_ctrlinput, 0);
PARAM_DEFINE_INT32(CLSYSID_repeat, 3);

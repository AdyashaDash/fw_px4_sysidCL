/****************************************************************************
 *
 *   Copyright (C) 2013 Autonomous Systems Lab, ETH Zurich. All rights reserved.
 *   Author: @author
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file aslctrl_parameters.h
 * Definition of the aslctrl-parameters uORB topic.
 */

#ifndef ASLCTRL_PARAMETERS_H_
#define ASLCTRL_PARAMETERS_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 */

struct aslctrl_parameters_s {

	 /* Actual data, this is specific to the type of data which is stored in this struct
	 * A line containing L0GME will be added by the Python logging code generator to the
	 * logged dataset.
	 */

	uint64_t timestamp;      /**< in microseconds since system start          */

	//--- ASLCTRL GENERAL --------------------
	uint8_t ASLC_DEBUG;
	uint8_t ASLC_CtrlType;
	uint8_t ASLC_GainSch_E;
	uint8_t ASLC_GainSch_Q;
	uint8_t ASLC_StallProt;
	uint8_t ASLC_VelCtrl;
	uint8_t ASLC_OnRCLoss;
	uint8_t ASLC_OvSpdProt;
	uint8_t ASLC_CoordTurn;

	//--- DEMIXER --------------------
	uint8_t DEMIX_Enabled;
	float DEMIX_F_CH2t4;
	float DEMIX_F_CH6t1;
	float DEMIX_CH2t4;
	float DEMIX_CH6t1;

	//--- SAS --------------------
	//Sampling time
	float SAS_tSample;
	//Gains
	float SAS_RollPGain;
	float SAS_PitchPGain;
	float SAS_YawPGain;
	float SAS_RollPDir;
	float SAS_PitchPDir;
	float SAS_YawPDir;
	float SAS_RollYawDecoupleKari;
	float SAS_YawCTkP;
	float SAS_YawCTFF;
	//Limiters
	float SAS_RCtrlLim;
	float SAS_PCtrlLim;
	float SAS_YCtrlLim;
	//Filters
	float SAS_YawHighPassOmega;
	float SAS_PitchLowPassOmega;
	float SAS_RollLowPassOmega;
	//Dynamic Pressure Scaling
	float SAS_vScaleLimF;
	float SAS_vScaleExp;
	//Trim Values
	float SAS_TrimAilvNom;
	float SAS_TrimAilvMin;
	float SAS_TrimAilvMax;
	float SAS_TrimElevNom;
	float SAS_TrimElevMin;
	float SAS_TrimElevMax;
	float SAS_TrimRudvNom;
	float SAS_TrimRudvMin;
	float SAS_TrimRudvMax;

	//---CAS/AP ----------------------
	//Sampling time
	uint8_t CAS_fMult;
	//Gains
	float CAS_PitchPGain;
	float CAS_PitchPGainM;
	float CAS_PitchIGain;
	float CAS_RollPGain;
	float CAS_RollPGainM;
	//Limiters
	float CAS_PitchRateLim;
	float CAS_PitchRateILim;
	float CAS_PitchTCkI;
	float CAS_PitchTCILim;
	float CAS_RollRateLim;
	float CAS_YawRateLim;
	float CAS_PitchAngleLim;
	float CAS_RollAngleLim;
	//Coordinated Turn feed-forward gains
	float CAS_uElevTurnFF;
	//Filters
	float CAS_YawLowPassOmega;

	//---HighLevel (WP following & Velocity Control) ------------------
	uint8_t HL_fMult;
	float HL_WPL1_P_vMin;
	float HL_WPL1_P_vNom;
	float HL_WPL1_P_vMax;
	float HL_WPL1_Damping;
	float HL_Vel_vNom;
	float HL_Vel_vMin;
	float HL_Vel_vMax;
	//Old altitude controller
	float HL_AlthMax;
	float HL_AlthMin;
	float HL_vZClimb;
	float HL_vZSink;
	float HL_AltLowPassOmega;

	//---TECS--------------------
	float time_const;
	float time_const_throt;
	float min_sink_rate;
	float max_sink_rate;
	float max_climb_rate;
	float throttle_damp;
	float integrator_gain;
	float vertical_accel_limit;
	float height_comp_filter_omega;
	float speed_comp_filter_omega;
	float roll_throttle_compensation;
	float speed_weight;
	float pitch_damping;
	float pitch_limit_min;
	float pitch_limit_max;
	float throttle_min;
	float throttle_max;
	float throttle_cruise;
	float heightrate_p;
	float heightrate_ff;
	float speedrate_p;
	float throttle_slewrate;
	float throttle_ILim;

	//System Integrity Checks
	float IThrotWarn;

	//----CLSYSID--------------
	float CLSYSID_tExcite; 				// excitation time (sec) corresponding to apprx. mode natural frequency
	float CLSYSID_step; 				// excitation unit step size eg: 5, -5 etc. (in degrees), NOTE: sign implies the direction of the first step in a given maneuver
	uint8_t CLSYSID_maneuver; 			// maneuver selection: 2-1-1 (0), chirp (1)
	uint8_t CLSYSID_ctrlinput; 			// control selection: pitch (0), roll (1)
	uint8_t CLSYSID_repeat; 			// number of times to repeat sys id maneuver

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(aslctrl_parameters);

#endif

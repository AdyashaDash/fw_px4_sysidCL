/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file sdlog2_messages.h
 *
 * Log messages and structures definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef SDLOG2_MESSAGES_H_
#define SDLOG2_MESSAGES_H_

#include "sdlog2_format.h"

/* define message formats */

#pragma pack(push, 1)
/* --- ATT - ATTITUDE --- */
#define LOG_ATT_MSG 2
struct log_ATT_s {
	float q_w;
	float q_x;
	float q_y;
	float q_z;
	float roll;
	float pitch;
	float yaw;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float gx;
	float gy;
	float gz;
};

/* --- ATSP - ATTITUDE SET POINT --- */
#define LOG_ATSP_MSG 3
struct log_ATSP_s {
	float roll_sp;
	float pitch_sp;
	float yaw_sp;
	float thrust_sp;
	float q_w;
	float q_x;
	float q_y;
	float q_z;
};

/* --- IMU - IMU SENSORS --- */
#define LOG_IMU_MSG 4
#define LOG_IMU1_MSG 22
#define LOG_IMU2_MSG 23
struct log_IMU_s {
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float temp_acc;
	float temp_gyro;
	float temp_mag;
};

/* --- SENS - OTHER SENSORS --- */
#define LOG_SENS_MSG 5
struct log_SENS_s {
	float baro_pres;
	float baro_alt;
	float baro_temp;
	float diff_pres;
	float diff_pres_filtered;
};

/* --- LPOS - LOCAL POSITION --- */
#define LOG_LPOS_MSG 6
struct log_LPOS_s {
	float x;
	float y;
	float z;
	float ground_dist;
	float ground_dist_rate;
	float vx;
	float vy;
	float vz;
	int32_t ref_lat;
	int32_t ref_lon;
	float ref_alt;
	uint8_t pos_flags;
	uint8_t ground_dist_flags;
	float eph;
	float epv;
};

/* --- LPSP - LOCAL POSITION SETPOINT --- */
#define LOG_LPSP_MSG 7
struct log_LPSP_s {
	float x;
	float y;
	float z;
	float yaw;
	float vx;
	float vy;
	float vz;
	float acc_x;
	float acc_y;
	float acc_z;
};

/* --- GPS - GPS POSITION --- */
#define LOG_GPS_MSG 8
struct log_GPS_s {
	uint64_t gps_time;
	uint8_t fix_type;
	float eph;
	float epv;
	int32_t lat;
	int32_t lon;
	float alt;
	float vel_n;
	float vel_e;
	float vel_d;
	float cog;
	uint8_t sats;
	uint16_t snr_mean;
	uint16_t noise_per_ms;
	uint16_t jamming_indicator;
};

/* --- ATTC - ATTITUDE CONTROLS (ACTUATOR_0 CONTROLS)--- */
#define LOG_ATTC_MSG 9
#define LOG_ATC1_MSG 40
struct log_ATTC_s {
	float roll;
	float pitch;
	float yaw;
	float thrust;
};

/* --- STAT - VEHICLE STATE --- */
#define LOG_STAT_MSG 10
struct log_STAT_s {
	uint8_t main_state;
	uint8_t arming_state;
	uint8_t failsafe;
	float battery_remaining;
	uint8_t battery_warning;
	uint8_t landed;
	float load;
};

/* --- RC - RC INPUT CHANNELS --- */
#define LOG_RC_MSG 11
struct log_RC_s {
	float channel[8];
	uint8_t rssi;
	uint8_t channel_count;
	uint8_t signal_lost;
};

/* --- OUT0 - ACTUATOR_0 OUTPUT --- */
#define LOG_OUT0_MSG 12
struct log_OUT0_s {
	float output[8];
};

/* --- AIRS - AIRSPEED --- */
#define LOG_AIRS_MSG 13
struct log_AIRS_s {
	float indicated_airspeed;
	float true_airspeed;
	float air_temperature_celsius;
};

/* --- ARSP - ATTITUDE RATE SET POINT --- */
#define LOG_ARSP_MSG 14
struct log_ARSP_s {
	float roll_rate_sp;
	float pitch_rate_sp;
	float yaw_rate_sp;
};

/* --- FLOW - OPTICAL FLOW --- */
#define LOG_FLOW_MSG 15
struct log_FLOW_s {
	uint8_t sensor_id;
	float pixel_flow_x_integral;
	float pixel_flow_y_integral;
	float gyro_x_rate_integral;
	float gyro_y_rate_integral;
	float gyro_z_rate_integral;
	float ground_distance_m;
	uint32_t integration_timespan;
	uint32_t time_since_last_sonar_update;
	uint16_t frame_count_since_last_readout;
	int16_t gyro_temperature;
	uint8_t	quality;
};

/* --- GPOS - GLOBAL POSITION ESTIMATE --- */
#define LOG_GPOS_MSG 16
struct log_GPOS_s {
	int32_t lat;
	int32_t lon;
	float alt;
	float vel_n;
	float vel_e;
	float vel_d;
	float eph;
	float epv;
	float terrain_alt;
};

/* --- GPSP - GLOBAL POSITION SETPOINT --- */
#define LOG_GPSP_MSG 17
struct log_GPSP_s {
	uint8_t nav_state;
	int32_t lat;
	int32_t lon;
	float alt;
	float yaw;
	uint8_t type;
	float loiter_radius;
	int8_t loiter_direction;
	float pitch_min;
};

/* --- ESC - ESC STATE --- */
#define LOG_ESC_MSG 18
struct log_ESC_s {
	uint16_t counter;
	uint8_t  esc_count;
	uint8_t  esc_connectiontype;
	uint8_t  esc_num;
	uint16_t esc_address;
	uint16_t esc_version;
	float    esc_voltage;
	float    esc_current;
	int32_t  esc_rpm;
	float    esc_temperature;
	float    esc_setpoint;
	uint16_t esc_setpoint_raw;
};

/* --- GVSP - GLOBAL VELOCITY SETPOINT --- */
#define LOG_GVSP_MSG 19
struct log_GVSP_s {
	float vx;
	float vy;
	float vz;
};

/* --- BATT - BATTERY --- */
#define LOG_BATT_MSG 20
struct log_BATT_s {
	float voltage;
	float voltage_filtered;
	float current;
	float discharged;
};

/* --- DIST - DISTANCE TO SURFACE --- */
#define LOG_DIST_MSG 21
struct log_DIST_s {
	float bottom;
	float bottom_rate;
	uint8_t flags;
};

/* LOG IMU1 and IMU2 MSGs consume IDs 22 and 23 */


/* --- PWR - ONBOARD POWER SYSTEM --- */
#define LOG_PWR_MSG 24
struct log_PWR_s {
	float peripherals_5v;
	float servo_rail_5v;
	float servo_rssi;
	uint8_t usb_ok;
	uint8_t brick_ok;
	uint8_t servo_ok;
	uint8_t low_power_rail_overcurrent;
	uint8_t high_power_rail_overcurrent;
};

/* --- VICN - VICON POSITION --- */
#define LOG_VICN_MSG 25
struct log_VICN_s {
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
};

/* --- GS0A - GPS SNR #0, SAT GROUP A --- */
#define LOG_GS0A_MSG 26
struct log_GS0A_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS0B - GPS SNR #0, SAT GROUP B --- */
#define LOG_GS0B_MSG 27
struct log_GS0B_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS1A - GPS SNR #1, SAT GROUP A --- */
#define LOG_GS1A_MSG 28
struct log_GS1A_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS1B - GPS SNR #1, SAT GROUP B --- */
#define LOG_GS1B_MSG 29
struct log_GS1B_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- TECS - TECS STATUS --- */
#define LOG_TECS_MSG 30
struct log_TECS_s {
	float altitudeSp;
	float altitudeFiltered;
	float flightPathAngleSp;
	float flightPathAngle;
	float airspeedSp;
	float airspeedFiltered;
	float airspeedDerivativeSp;
	float airspeedDerivative;
	float totalEnergyError;
	float totalEnergyRateError;
	float energyDistributionError;
	float energyDistributionRateError;
	float pitch_integ;
	float throttle_integ;

	uint8_t mode;
};

/* --- WIND - WIND ESTIMATE --- */
#define LOG_WIND_MSG 31
struct log_WIND_s {
	float x;
	float y;
	float cov_x;
	float cov_y;
};

/* --- EST0 - ESTIMATOR STATUS --- */
#define LOG_EST0_MSG 32
struct log_EST0_s {
	float s[12];
	uint8_t n_states;
	uint8_t nan_flags;
	uint8_t health_flags;
	uint8_t timeout_flags;
};

/* --- EST1 - ESTIMATOR STATUS --- */
#define LOG_EST1_MSG 33
struct log_EST1_s {
	float s[16];
};

/* --- TEL0..3 - TELEMETRY STATUS --- */
#define LOG_TEL0_MSG 34
#define LOG_TEL1_MSG 35
#define LOG_TEL2_MSG 36
#define LOG_TEL3_MSG 37
struct log_TEL_s {
	uint8_t rssi;
	uint8_t remote_rssi;
	uint8_t noise;
	uint8_t remote_noise;
	uint16_t rxerrors;
	uint16_t fixed;
	uint8_t txbuf;
	uint64_t heartbeat_time;
};

/* --- VISN - VISION POSITION --- */
#define LOG_VISN_MSG 38
struct log_VISN_s {
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float qx;
	float qy;
	float qz;
	float qw;
};

/* --- ENCODERS - ENCODER DATA --- */
#define LOG_ENCD_MSG 39
struct log_ENCD_s {
	int64_t cnt0;
	float vel0;
	int64_t cnt1;
	float vel1;
};

/* --- AIR SPEED SENSORS - DIFF. PRESSURE --- */
#define LOG_AIR1_MSG 41

/* --- VTOL - VTOL VEHICLE STATUS */
#define LOG_VTOL_MSG 42
struct log_VTOL_s {
	float airspeed_tot;
};

/* --- TIMESYNC - TIME SYNCHRONISATION OFFSET */
#define LOG_TSYN_MSG 43
struct log_TSYN_s {
	uint64_t time_offset;
};

/* --- MACS - MULTIROTOR ATTITUDE CONTROLLER STATUS */
#define LOG_MACS_MSG 44
struct log_MACS_s {
	float roll_rate_integ;
	float pitch_rate_integ;
	float yaw_rate_integ;
};

/********** ASL-MESSAGES, ID > 100 **************/

/* --- ASLCTRL - ASLC parameter state --- (ASL/PhilippOe) */
// Abbreviations used to save labeling-space:
//    Category: A/ASLC=ASLC, S=SAS, C=CAS, HL=HL
//    Orientation: R=RollAngle/RRate=RollRate, P=Pitch, Y=Yaw
//    Type of data: kP = P-Gain, kI=I-Gain, L=Limit, IL=Integral-Limit, LP=Low Pass, HP=HighPass, T=Trim, R=Reference, FF=FeedForward
//    Specifier: N=Nominal, Ma=Max, Mi=Min
//    GS=GainScheduling, SP=StallProtection, tS=tSample, CT=CoordinatedTurn

#define LOG_ASLC_MSG 101
struct log_ASLC_s {

	uint64_t timestamp;      /**< in microseconds since system start          */

	//--- ASLCTRL GENERAL --------------------
	uint8_t ASLC_CtrlType;
	uint8_t ASLC_GainSch_E;
	uint8_t ASLC_GainSch_Q;
	uint8_t ASLC_StallProt;
	uint8_t ASLC_VelCtrl;
	uint8_t ASLC_OnRCLoss;
	uint8_t ASLC_OvSpdProt;
	uint8_t ASLC_CoordTurn;
};

#define LOG_ASAS_MSG 102
struct log_ASAS_s {
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
	float SAS_YawCTFF;
	float SAS_YawCTkP;

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
};

#define LOG_ACAS_MSG 103
struct log_ACAS_s {
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

	//Coordinated turn feed-forward gains
	float CAS_uElevTurnFF;

	//Filters
	float CAS_YawLowPassOmega;
};

#define LOG_AHL1_MSG 104
struct log_AHL1_s {
	//---WP following ------------------
	uint8_t HL_fMult;
	float HL_WPL1_Damping;
	float HL_WPL1_P_vMin;
	float HL_WPL1_P_vNom;
	float HL_WPL1_P_vMax;
	float HL_Vel_vNom;
	float HL_Vel_vMin;
	float HL_Vel_vMax;
	//Old altitude controller
	float HL_AlthMax;
	float HL_AlthMin;
	float HL_vZClimb;
	float HL_vZSink;
	float HL_AltLowPassOmega;
};

#define LOG_AHL2_MSG 105
struct log_AHL2_s {
	//---TECS--------------------
	float time_const;
	float time_const_throt;
	float min_sink_rate;
	float max_sink_rate;
	float max_climb_rate;
	float throttle_damp;
	float integrator_gain;
	float throttle_ILim;
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
	float speedrate_p;
	float throttle_slewrate;
};


#define LOG_ASLD_MSG 106
struct log_ASLD_s {
	/* --- ASLD - ASLD data state --- */
	uint64_t timestamp;      /**< in microseconds since system start          */
	uint32_t dt;
	uint8_t aslctrl_mode; //The control mode (manual, assisted, auto...)

	//Longitudinal loop
	float h;
	float hRef;
	float hRef_t;
	float PitchAngle;
	float PitchAngleRef;
	float PitchAngleRefCT;
	float q;
	float qRef;
	float uElev;
	float uThrot;
	float uThrot2;
	float AirspeedRef;
	uint8_t bEngageSpoilers;

	//Lateral loops
	float YawAngle;
	float YawAngleRef;
	float RollAngle;
	float RollAngleRef;
	float p;
	float pRef;
	float r;
	float rRef;
	float uAil;
	float uRud;
	float Yawdot_ref;
	float Yawdot;

	//General
	float f_GainSch_Q;
	float P_kP_GainSch_E;
	float R_kP_GainSch_E;
};

#define LOG_EKFS_MSG 107
struct log_EKFS_s {					// *** EKF-States log message ***
	uint64_t timestamp;      		// in microseconds since system start
	float state_p[3];				// position states for latitude, loglitude, altitude (above elipsoid)
	float state_q_NS[4]; 			// angular states for q1, q2, q3, q4
	float state_v_N[3];	 			// velocity states for vel_n, vel_e, vel_d
	float state_b_g[3];	 			// gyroscope offset states
	float state_b_a[3];				// accelerometer offset states
	float state_QFF;				// QFF states
	float state_w[3];				// wind states
	float state_K;					// dynamic states

	float alpha;					// Estimated angle of attack [rad]
	float beta;						// Estimated sideslip angle [rad]
	float airspeed;					// Filtered/estimated airspeed [m/s]. Safer than using raw airspeed-sensor data.
};


#define LOG_EKFV_MSG 108
struct log_EKFV_s {					// *** EKF-Variances log message ***
	uint64_t timestamp;      		// in microseconds since system start
	float state_P_var_vect[20];		// variance vector
};

#define LOG_MPPT_MSG 109
struct log_MPPT_s {					// *** MPPT log message ***
	uint64_t timestamp;      		// in microseconds since system start
	float mppt_amp[3];				// MPPT current readings in amps
	float mppt_volt[3];				// MPPT voltage readings in volts
	uint16_t mppt_pwm[3];			// MPPT pwm readings
	uint8_t mppt_status[3];			// MPPT status readings
};

#define LOG_POWS_MSG 110
struct log_POWS_s {					// *** Power sensors log message ***
	uint64_t timestamp;      		// in microseconds since system start
	float adc121_vspb_volt;			// power board voltage sensor readings in volts
	float adc121_cspb_amp;			// power board current sensor readings in amps
	float adc121_cs1_amp;			// board current sensor 1 readings in amps
	float adc121_cs2_amp;			// board current sensor 2 readings in amps
};

#define LOG_BAT0_MSG 111
#define LOG_BAT1_MSG 112
#define LOG_BAT2_MSG 113
struct log_BATM_s {						// *** Battery monitor sensors log message ***
	uint64_t timestamp;					/**< Timestamp in microseconds since boot, of bat monitor 	*/
	uint16_t temperature;				/**< battery monitor sensor temperature report in [0.1 K] 	*/
	uint16_t voltage;					/**< battery monitor sensor voltage report in [mV] 			*/
	int16_t  current;					/**< battery monitor sensor current report in [mA] 			*/
	uint8_t  stateofcharge;				/**< battery monitor sensor state of charge report in [%] 	*/
	uint16_t batterystatus;				/**< battery monitor sensor battery status report in Hex 	*/
	uint16_t serialnumber;				/**< battery monitor sensor serial number report in Hex 	*/
	uint16_t hostfetcontrol;			/**< battery monitor sensor host FET control report in Hex 	*/
	uint16_t cellvoltage1;				/**< battery monitor sensor cell 1 voltage  report in [mV] 	*/
	uint16_t cellvoltage2;				/**< battery monitor sensor cell 2 voltage report in [mV] 	*/
	uint16_t cellvoltage3;				/**< battery monitor sensor cell 3 voltage report in [mV] 	*/
	uint16_t cellvoltage4;				/**< battery monitor sensor cell 4 voltage report in [mV] 	*/
	uint16_t cellvoltage5;				/**< battery monitor sensor cell 5 voltage report in [mV] 	*/
	uint16_t cellvoltage6;				/**< battery monitor sensor cell 6 voltage report in [mV] 	*/

};

#define LOG_ATMO_MSG 114
struct log_ATMO_s {
	float amb_temp_celcius;
};

/********** SYSTEM MESSAGES, ID > 0x80 **********/

/* --- TIME - TIME STAMP --- */
#define LOG_TIME_MSG 129
struct log_TIME_s {
	uint64_t t;
};

/* --- VER - VERSION --- */
#define LOG_VER_MSG 130
struct log_VER_s {
	char arch[16];
	char fw_git[64];
};

/* --- PARM - PARAMETER --- */
#define LOG_PARM_MSG 131
struct log_PARM_s {
	char name[16];
	float value;
};

#pragma pack(pop)

/* construct list of all message formats */
static const struct log_format_s log_formats[] = {
	/* business-level messages, ID < 0x80 */
	LOG_FORMAT(ATT, "fffffffffffff",	"qw,qx,qy,qz,Roll,Pitch,Yaw,RollRate,PitchRate,YawRate,GX,GY,GZ"),
	LOG_FORMAT(ATSP, "ffffffff",		"RollSP,PitchSP,YawSP,ThrustSP,qw,qx,qy,qz"),
	LOG_FORMAT_S(IMU, IMU, "ffffffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_FORMAT_S(IMU1, IMU, "ffffffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_FORMAT_S(IMU2, IMU, "ffffffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_FORMAT_S(SENS, SENS, "fffff",		"BaroPres,BaroAlt,BaroTemp,DiffPres,DiffPresFilt"),
	LOG_FORMAT_S(AIR1, SENS, "fffff",	"BaroPa,BaroAlt,BaroTmp,DiffPres,DiffPresF"),
	LOG_FORMAT(LPOS, "ffffffffLLfBBff",	"X,Y,Z,Dist,DistR,VX,VY,VZ,RLat,RLon,RAlt,PFlg,GFlg,EPH,EPV"),
	LOG_FORMAT(LPSP, "ffffffffff",		"X,Y,Z,Yaw,VX,VY,VZ,AX,AY,AZ"),
	LOG_FORMAT(GPS, "QBffLLfffffBHHH",	"GPSTime,Fix,EPH,EPV,Lat,Lon,Alt,VelN,VelE,VelD,Cog,nSat,SNR,N,J"),
	LOG_FORMAT_S(ATTC, ATTC, "ffff",		"Roll,Pitch,Yaw,Thrust"),
	LOG_FORMAT_S(ATC1, ATTC, "ffff",		"Roll,Pitch,Yaw,Thrust"),
	LOG_FORMAT(STAT, "BBBfBBf",		"MainState,ArmS,Failsafe,BatRem,BatWarn,Landed,Load"),
	LOG_FORMAT(VTOL, "f",		"Arsp"),
	LOG_FORMAT(RC, "ffffffffBBB",		"Ch0,Ch1,Ch2,Ch3,Ch4,Ch5,Ch6,Ch7,RSSI,Count,SignalLost"),
	LOG_FORMAT(OUT0, "ffffffff",		"Out0,Out1,Out2,Out3,Out4,Out5,Out6,Out7"),
	LOG_FORMAT(AIRS, "fff",			"IndSpeed,TrueSpeed,AirTemp"),
	LOG_FORMAT(ARSP, "fff",			"RollRateSP,PitchRateSP,YawRateSP"),
	LOG_FORMAT(FLOW, "BffffffLLHhB",	"ID,RawX,RawY,RX,RY,RZ,Dist,TSpan,DtSonar,FrmCnt,GT,Qlty"),
	LOG_FORMAT(GPOS, "LLfffffff",		"Lat,Lon,Alt,VelN,VelE,VelD,EPH,EPV,TALT"),
	LOG_FORMAT(GPSP, "BLLffBfbf",		"NavState,Lat,Lon,Alt,Yaw,Type,LoitR,LoitDir,PitMin"),
	LOG_FORMAT(ESC, "HBBBHHffiffH",		"count,nESC,Conn,N,Ver,Adr,Volt,Amp,RPM,Temp,SetP,SetPRAW"),
	LOG_FORMAT(GVSP, "fff",			"VX,VY,VZ"),
	LOG_FORMAT(BATT, "ffff",		"V,VFilt,C,Discharged"),
	LOG_FORMAT(DIST, "ffB",			"Bottom,BottomRate,Flags"),
	LOG_FORMAT_S(TEL0, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL1, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL2, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL3, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT(EST0, "ffffffffffffBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,nStat,fNaN,fHealth,fTOut"),
	LOG_FORMAT(EST1, "ffffffffffffffff",	"s12,s13,s14,s15,s16,s17,s18,s19,s20,s21,s22,s23,s24,s25,s26,s27"),
	LOG_FORMAT(PWR, "fffBBBBB",		"Periph5V,Servo5V,RSSI,UsbOk,BrickOk,ServoOk,PeriphOC,HipwrOC"),
	LOG_FORMAT(VICN, "ffffff",		"X,Y,Z,Roll,Pitch,Yaw"),
	LOG_FORMAT(VISN, "ffffffffff",		"X,Y,Z,VX,VY,VZ,QuatX,QuatY,QuatZ,QuatW"),
	LOG_FORMAT(GS0A, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS0B, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS1A, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS1B, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(TECS, "ffffffffffffffB",	"ASP,AF,FSP,F,AsSP,AsF,AsDSP,AsD,EE,EDE,ERE,EDRE,PtchI,ThrI,M"),
	LOG_FORMAT(WIND, "ffff",	"X,Y,CovX,CovY"),
	LOG_FORMAT(ENCD, "qfqf",	"cnt0,vel0,cnt1,vel1"),
	LOG_FORMAT(TSYN, "Q", 		"TimeOffset"),
	LOG_FORMAT(MACS, "fff", "RRint,PRint,YRint"),

	/* ASL messages, ID >= 100(0x64) */
	// Note: Message labeling abbreviations are given above
	LOG_FORMAT(ASLC, "QBBBBBBBB","t,CtrlType,GainSch_E,GainSch_Q,StallProt,VelCtrl,OnRCLoss,OvSpdProt,CoordTurn"),
	LOG_FORMAT(ASAS, "fffffffffffffffffffffffffff","tS,R_kP,P_kP,Y_kP,RDi,PDi,YDi,RYDe,Y_CTFF,Y_CTkP,R_L,P_L,Y_L,Y_HPw,P_LPw,R_LPw,vSL,vSE,RTN,RTMi,RTMa,PTN,PTMi,PTMa,YTN,YTMi,YTMa"),
	LOG_FORMAT(ACAS, "Bfffffffffffffff","fM,P_kP,P_kP_M,P_kI,R_kP,R_kP_M,PRate_Lim,PRate_ILim,PTCkI,PTCILim,RRate_Lim,YRate_Lim,P_Lim,R_Lim,uElevTurnFF,Y_LPw"),
	LOG_FORMAT(AHL1,  "Bffffffffffff","fM,L1_D,L1_P_vMi,L1_P_vN,L1_P_vMa,v_N,v_Mi,v_Ma,hMa,hMi,h_vZcl,h_vZsi,h_LPw"),
	LOG_FORMAT(AHL2,  "ffffffffffffffffffffff","tc,tct,misr,masr,macr,td,ig,tIL,val,hcfo,scfo,rtc,sw,pd,plmi,plma,thrmi,thrma,thrc,hp,sp,ts"),
	LOG_FORMAT(ASLD,  "QIBffffffffffffBfffffffffffffff", "t,dt,mode,h,hR,hR_t,P,PR,PR_CT,q,qR,uE,uT,uT2,TAS_R,bSpoil,Y,YR,R,RR,p,pR,r,rR,uA,uR,Yd_R,Yd,GS_Q,P_kPE,R_kPE"),
	LOG_FORMAT(EKFS,  "Qffffffffffffffffffffffff", "t,p1,p2,p3,q1,q2,q3,q4,v1,v2,v3,bg1,bg2,bg3,ba1,ba2,ba3,qff,w1,w2,w3,k,alpha,beta,TAS"),
	LOG_FORMAT(EKFV,  "Qffffffffffffffffffff", "t,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15,v16,v17,v18,v19,v20"),
	LOG_FORMAT(MPPT,  "QffffffHHHBBB", "t,Cur1,Cur2,Cur3,Volt1,Volt2,Volt3,PWM1,PWM2,PWM3,Stat1,Stat2,Stat3"),
	LOG_FORMAT(POWS, "Qffff", "t,Main_Volt,Main_Cur,Cur1,Cur2"),
	LOG_FORMAT_S(BAT0, BATM, "QHHhBHHHHHHHHH", "t,Temp,V_bat,I_bat,SoC,status,SN,hostfetctrl,V_1,V_2,V_3,V_4,V_5,V_6"),
	LOG_FORMAT_S(BAT1, BATM, "QHHhBHHHHHHHHH", "t,Temp,V_bat,I_bat,SoC,status,SN,hostfetctrl,V_1,V_2,V_3,V_4,V_5,V_6"),
	LOG_FORMAT_S(BAT2, BATM, "QHHhBHHHHHHHHH", "t,Temp,V_bat,I_bat,SoC,status,SN,hostfetctrl,V_1,V_2,V_3,V_4,V_5,V_6"),
	LOG_FORMAT(ATMO, "f", "ambTemp"),

	/* system-level messages, ID >= 0x80 */
	/* FMT: don't write format of format message, it's useless */
	LOG_FORMAT(TIME, "Q", "StartTime"),
	LOG_FORMAT(VER, "NZ", "Arch,FwGit"),
	LOG_FORMAT(PARM, "Nf", "Name,Value")
};

static const unsigned log_formats_num = sizeof(log_formats) / sizeof(log_formats[0]);

#endif /* SDLOG2_MESSAGES_H_ */

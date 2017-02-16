// --COPYRIGHT--,BSD
// Copyright (c) 2015, Texas Instruments Incorporated
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// *  Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// *  Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// *  Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// --/COPYRIGHT
//! \file   solutions/instaspin_foc/src/proj_lab011d.c
//! \brief Dual motor implementation of InstaSPIN-FOC
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

//! \defgroup PROJ_LAB11d PROJ_LAB11d
//@{

//! \defgroup PROJ_LAB11d_OVERVIEW Project Overview
//!
//! Dual Motor Implementation of InstaSPIN-FOC based on the simplified proj_lab11 structure
//!
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"

#ifdef FLASH
#pragma CODE_SECTION(motor1_ISR,"ramfuncs");
#pragma CODE_SECTION(motor2_ISR,"ramfuncs");
#endif

#define USER_ENC_SAMPLE_PERIOD          (USER_ISR_FREQ_Hz_M2/USER_NUM_PWM_TICKS_PER_ISR_TICK_M2)/(USER_MOTOR_MAX_SPEED_KRPM_M2*1000/60 *2)



// Include header files used in the main function

// **************************************************************************
// the defines

// **************************************************************************
// the globals

CLARKE_Handle clarkeHandle_I[2];  //!< the handle for the current Clarke
//!< transform
CLARKE_Obj clarke_I[2];        //!< the current Clarke transform object

CLARKE_Handle clarkeHandle_V[2];        //!< the handle for the voltage Clarke
//!< transform
CLARKE_Obj clarke_V[2];     //!< the voltage Clarke transform object

EST_Handle estHandle[2];    //!< the handle for the estimator

PID_Obj pid[2][3];       //!< three objects for PID controllers
//!< 0 - Speed, 1 - Id, 2 - Iq
PID_Handle pidHandle[2][3]; //!< three handles for PID controllers
//!< 0 - Speed, 1 - Id, 2 - Iq
uint16_t pidCntSpeed[2];     //!< count variable to decimate the execution
//!< of the speed PID controller

PARK_Handle     parkHandle[2];      //!< the handle for the current Parke
                                    //!< transform
PARK_Obj        park[2];            //!< the current Parke transform object

IPARK_Handle iparkHandle[2];     //!< the handle for the inverse Park
//!< transform
IPARK_Obj ipark[2];           //!< the inverse Park transform object

SVGEN_Handle svgenHandle[2];     //!< the handle for the space vector generator
SVGEN_Obj svgen[2];           //!< the space vector generator object

TRAJ_Handle trajHandle_Id[2];    //!< the handle for the id reference trajectory
TRAJ_Obj traj_Id[2];                     //!< the id reference trajectory object

TRAJ_Handle trajHandle_spd[2]; //!< the handle for the speed reference trajectory
TRAJ_Obj traj_spd[2];                 //!< the speed reference trajectory object

#ifdef CSM_ENABLE
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif

HAL_Handle halHandle;       //!< the handle for the hardware abstraction
//!< layer for common CPU setup
HAL_Obj hal;             //!< the hardware abstraction layer object

HAL_Handle_mtr halHandleMtr[2]; 	//!< the handle for the hardware abstraction
//!< layer specific to the motor board.
HAL_Obj_mtr halMtr[2];       	//!< the hardware abstraction layer object
//!< specific to the motor board.

ANGLE_COMP_Handle angleCompHandle[2]; //!< the handle for the angle compensation
ANGLE_COMP_Obj angleComp[2];        //!< the angle compensation object

HAL_Handle_mtr halHandleMtr[2]; //!< the handle for the hardware abstraction
//!< layer specific to the motor board.
HAL_Obj_mtr halMtr[2];       //!< the hardware abstraction layer object
//!< specific to the motor board.
#ifdef QEP
ENC_Handle encHandle[2];
ENC_Obj enc[2];
#endif
HAL_PwmData_t gPwmData[2] = { { _IQ(0.0), _IQ(0.0), _IQ(0.0) }, //!< contains the
		{ _IQ(0.0), _IQ(0.0), _IQ(0.0) } };  //!< pwm values for each phase.
//!< -1.0 is 0%, 1.0 is 100%

HAL_AdcData_t gAdcData[2];       //!< contains three current values, three
//!< voltage values and one DC buss value

MATH_vec3 gOffsets_I_pu[2] = { { _IQ(0.0), _IQ(0.0), _IQ(0.0) },  //!< contains
		{ _IQ(0.0), _IQ(0.0), _IQ(0.0) } }; //!< the offsets for the current feedback

MATH_vec3 gOffsets_V_pu[2] = { { _IQ(0.0), _IQ(0.0), _IQ(0.0) },  //!< contains
		{ _IQ(0.0), _IQ(0.0), _IQ(0.0) } }; //!< the offsets for the voltage feedback

MATH_vec2 gIdq_ref_pu[2] = { { _IQ(0.0), _IQ(0.0) },  //!< contains the Id and
		{ _IQ(0.0), _IQ(0.0) } }; //!< Iq references

MATH_vec2 gVdq_out_pu[2] = { { _IQ(0.0), _IQ(0.0) },  //!< contains the output
		{ _IQ(0.0), _IQ(0.0) } }; //!< Vd and Vq from the current controllers

MATH_vec2 gIdq_pu[2] = { { _IQ(0.0), _IQ(0.0) },   //!< contains the Id and Iq
		{ _IQ(0.0), _IQ(0.0) } };  //!< measured values



FILTER_FO_Handle filterHandle[2][6]; //!< the handles for the 3-current and 3-voltage filters for offset calculation
FILTER_FO_Obj filter[2][6]; //!< the 3-current and 3-voltage filters for offset calculation
uint32_t gOffsetCalcCount[2] = { 0, 0 };
uint32_t gAlignCount[2] = {0,0};

// define CPU time
CPU_TIME_Handle cpu_timeHandle[2];
CPU_TIME_Obj cpu_time[2];

#ifdef CSM_ENABLE
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif

USER_Params gUserParams[2];
volatile _iq gAngle_pu = 0;
volatile MOTOR_Vars_t gMotorVars[2] = { MOTOR_Vars_INIT_Mtr1,
		MOTOR_Vars_INIT_Mtr2 };   //!< the global motor
//!< variables that are defined in main.h and
//!< used for display in the debugger's watch
//!< window

volatile SYSTEM_Vars_t gSystemVars = SYSTEM_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
		extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef CSM_ENABLE
		extern uint16_t *econst_start, *econst_end, *econst_ram_load;
		extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif

#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
		DRV_SPI_8301_Vars_t gDrvSpi8301Vars[2];
#endif

#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars[2];
#endif

uint16_t gCounter_updateGlobals[2] = { 0, 0 };

uint16_t gTrjCnt[2] = { 0, 0 };

_iq gFlux_pu_to_Wb_sf[2];

_iq gFlux_pu_to_VpHz_sf[2];

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf[2];

_iq gTorque_Flux_Iq_pu_to_Nm_sf[2];

_iq gSpeed_krpm_to_pu_sf[2];

_iq gSpeed_hz_to_krpm_sf[2];

_iq gId_MinValue[2];

_iq gId_MaxDelta[2];

_iq gId_RsEst[2];
// **************************************************************************
// the functions
void main(void) {
	// IMPORTANT NOTE: If you are not familiar with MotorWare coding guidelines
	// please refer to the following document:
	// C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf

	// Only used if running from FLASH
	// Note that the variable FLASH is defined by the project

#ifdef FLASH
	// Copy time critical code and Flash setup code to RAM
	// The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files.
	memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,
			(uint16_t *)&RamfuncsRunStart);

#ifdef CSM_ENABLE
	//copy .econst to unsecure RAM
	if(*econst_end - *econst_start)
	{
		memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
	}

	//copy .switch ot unsecure RAM
	if(*switch_end - *switch_start)
	{
		memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
	}
#endif
#endif

	// initialize the Hardware Abstraction Layer  (HAL)
	// halHandle will be used throughout the code to interface with the HAL
	// (set parameters, get and set functions, etc) halHandle is required since
	// this is how all objects are interfaced, and it allows interface with
	// multiple objects by simply passing a different handle. The use of
	// handles is explained in this document:
	// C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf
	halHandle = HAL_init(&hal, sizeof(hal));

	// initialize the individual motor hal files
	halHandleMtr[HAL_MTR1] = HAL_init_mtr(&halMtr[HAL_MTR1],
			sizeof(halMtr[HAL_MTR1]), (HAL_MtrSelect_e) HAL_MTR1);

	// initialize the individual motor hal files
	halHandleMtr[HAL_MTR2] = HAL_init_mtr(&halMtr[HAL_MTR2],
			sizeof(halMtr[HAL_MTR2]), (HAL_MtrSelect_e) HAL_MTR2);

	// initialize the user parameters
	// This function initializes all values of structure gUserParams with
	// values defined in user.h. The values in gUserParams will be then used by
	// the hardware abstraction layer (HAL) to configure peripherals such as
	// PWM, ADC, interrupts, etc.
	USER_setParamsMtr1(&gUserParams[HAL_MTR1]);
	USER_setParamsMtr2(&gUserParams[HAL_MTR2]);

	// set the hardware abstraction layer parameters
	// This function initializes all peripherals through a Hardware Abstraction
	// Layer (HAL). It uses all values stored in gUserParams.
	HAL_setParams(halHandle, &gUserParams[HAL_MTR1]);

	// initialize the estimator
	estHandle[HAL_MTR1] = EST_init((void *) USER_EST_HANDLE_ADDRESS, 0x200);
	estHandle[HAL_MTR2] = EST_init((void *) USER_EST_HANDLE_ADDRESS_1, 0x200);
#ifdef QEP
    // setup the ENC module
	encHandle[HAL_MTR1] = ENC_init(&enc[HAL_MTR1], sizeof(enc[HAL_MTR1]));
	encHandle[HAL_MTR2] = ENC_init(&enc[HAL_MTR2], sizeof(enc[HAL_MTR2]));
    //ENC_setup(encHandle[HAL_MTR1], 1, USER_MOTOR_NUM_POLE_PAIRS_M1, USER_MOTOR_ENCODER_LINES_M1, 0, USER_IQ_FULL_SCALE_FREQ_Hz_M1, USER_ISR_FREQ_Hz_M1, 8000.0);
    ENC_setup(encHandle[HAL_MTR2], USER_ENC_SAMPLE_PERIOD, USER_MOTOR_NUM_POLE_PAIRS_M2, USER_MOTOR_ENCODER_LINES_M2, 0, USER_IQ_FULL_SCALE_FREQ_Hz_M2, USER_ISR_FREQ_Hz_M2, 8000.0);
#endif


	{
		uint_least8_t mtrNum;

		for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {


			// initialize the individual motor hal files
			halHandleMtr[mtrNum] = HAL_init_mtr(&halMtr[mtrNum],
					sizeof(halMtr[mtrNum]), (HAL_MtrSelect_e) mtrNum);

			// Setup each motor board to its specific setting
			HAL_setParamsMtr(halHandleMtr[mtrNum], halHandle,
					&gUserParams[mtrNum]);

			{
				// These function calls are used to initialize the estimator with ROM
				// function calls. It needs the specific address where the controller
				// object is declared by the ROM code.
				CTRL_Handle ctrlHandle = CTRL_init(
						(void *) USER_CTRL_HANDLE_ADDRESS, 0x200);
				CTRL_Obj *obj = (CTRL_Obj *) ctrlHandle;

				// this sets the estimator handle (part of the controller object) to
				// the same value initialized above by the EST_init() function call.
				// This is done so the next function implemented in ROM, can
				// successfully initialize the estimator as part of the controller
				// object.
				obj->estHandle = estHandle[mtrNum];

				// initialize the estimator through the controller. These three
				// function calls are needed for the F2806xF/M implementation of
				// InstaSPIN.
				CTRL_setParams(ctrlHandle, &gUserParams[mtrNum]);
				CTRL_setUserMotorParams(ctrlHandle);
				CTRL_setupEstIdleState(ctrlHandle);

				// initialize the CPU usage module
				cpu_timeHandle[mtrNum] = CPU_TIME_init(&cpu_time[mtrNum],
						sizeof(cpu_time[mtrNum]));
				CPU_TIME_setParams(cpu_timeHandle[mtrNum],
						PWM_getPeriod(halHandleMtr[mtrNum]->pwmHandle[0]));
			}

			//Compensates for the delay introduced
			//from the time when the system inputs are sampled to when the PWM
			//voltages are applied to the motor windings.
			angleCompHandle[mtrNum] = ANGLE_COMP_init(&angleComp[mtrNum],
					sizeof(angleComp[mtrNum]));
			ANGLE_COMP_setParams(angleCompHandle[mtrNum],
					gUserParams[mtrNum].iqFullScaleFreq_Hz,
					gUserParams[mtrNum].pwmPeriod_usec,
					gUserParams[mtrNum].numPwmTicksPerIsrTick);

			// initialize the Clarke modules
			// Clarke handle initialization for current signals
			clarkeHandle_I[mtrNum] = CLARKE_init(&clarke_I[mtrNum],
					sizeof(clarke_I[mtrNum]));

			// Clarke handle initialization for voltage signals
			clarkeHandle_V[mtrNum] = CLARKE_init(&clarke_V[mtrNum],
					sizeof(clarke_V[mtrNum]));

			// compute scaling factors for flux and torque calculations
			gFlux_pu_to_Wb_sf[mtrNum] = USER_computeFlux_pu_to_Wb_sf(
					&gUserParams[mtrNum]);

			gFlux_pu_to_VpHz_sf[mtrNum] = USER_computeFlux_pu_to_VpHz_sf(
					&gUserParams[mtrNum]);

			gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum] =
					USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(
							&gUserParams[mtrNum]);

			gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum] =
					USER_computeTorque_Flux_Iq_pu_to_Nm_sf(
							&gUserParams[mtrNum]);

			gSpeed_krpm_to_pu_sf[mtrNum] = _IQ(
					(float_t )gUserParams[mtrNum].motor_numPolePairs * 1000.0
							/ (gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0));

			gSpeed_hz_to_krpm_sf[mtrNum] = _IQ(
					60.0 / (float_t )gUserParams[mtrNum].motor_numPolePairs
							/ 1000.0);

			// initialize the speed reference in kilo RPM where base speed is
			// USER_IQ_FULL_SCALE_FREQ_Hz.
			// Set 10 Hz electrical frequency as initial value, so the kRPM value would
			// be: 10 * 60 / motor pole pairs / 1000.
			gMotorVars[mtrNum].SpeedRef_krpm = _IQmpy(_IQ(10.0),
					gSpeed_hz_to_krpm_sf[mtrNum]);

			// disable Rs recalculation
			EST_setFlag_enableRsRecalc(estHandle[mtrNum], false);

			// set the number of current sensors
			setupClarke_I(clarkeHandle_I[mtrNum],
					gUserParams[mtrNum].numCurrentSensors);

			// set the number of voltage sensors
			setupClarke_V(clarkeHandle_V[mtrNum],
					gUserParams[mtrNum].numVoltageSensors);

			// initialize the PID controllers
			pidSetup((HAL_MtrSelect_e) mtrNum);

	        // Park handle initialization for current signals
	        parkHandle[mtrNum] = PARK_init(&park[mtrNum],sizeof(park[mtrNum]));

			// initialize the inverse Park module
			iparkHandle[mtrNum] = IPARK_init(&ipark[mtrNum],
					sizeof(ipark[mtrNum]));

			// initialize the space vector generator module
			svgenHandle[mtrNum] = SVGEN_init(&svgen[mtrNum],
					sizeof(svgen[mtrNum]));

			// initialize the speed reference trajectory
			trajHandle_spd[mtrNum] = TRAJ_init(&traj_spd[mtrNum],
					sizeof(traj_spd[mtrNum]));

			// initialize the Id reference trajectory
			trajHandle_Id[mtrNum] = TRAJ_init(&traj_Id[mtrNum],
					sizeof(traj_Id[mtrNum]));

			// initialize and configure offsets using filters
			{
				uint16_t cnt = 0;
				_iq b0 = _IQ(
						gUserParams[mtrNum].offsetPole_rps
								/ (float_t )gUserParams[mtrNum].ctrlFreq_Hz);
				_iq a1 = (b0 - _IQ(1.0));
				_iq b1 = _IQ(0.0);

				for (cnt = 0; cnt < 6; cnt++) {
					filterHandle[mtrNum][cnt] = FILTER_FO_init(
							&filter[mtrNum][cnt], sizeof(filter[mtrNum][0]));
					FILTER_FO_setDenCoeffs(filterHandle[mtrNum][cnt], a1);
					FILTER_FO_setNumCoeffs(filterHandle[mtrNum][cnt], b0, b1);
					FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt],
							_IQ(0.0), _IQ(0.0));
				}

				gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
			}

			// setup faults
			HAL_setupFaults(halHandleMtr[mtrNum]);

#ifdef DRV8301_SPI
			// turn on the DRV8301 if present
			HAL_enableDrv(halHandleMtr[mtrNum]);
			// initialize the DRV8301 interface
			HAL_setupDrvSpi(halHandleMtr[mtrNum],&gDrvSpi8301Vars[mtrNum]);
#endif

#ifdef DRV8305_SPI
			// turn on the DRV8305 if present
			HAL_enableDrv(halHandleMtr[mtrNum]);
			// initialize the DRV8305 interface
			HAL_setupDrvSpi(halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
#endif

			gCounter_updateGlobals[mtrNum] = 0;
		} // End of for loop
	}

	// configure the speed reference trajectory
	TRAJ_setTargetValue(trajHandle_spd[HAL_MTR1], _IQ(0.0));
	TRAJ_setIntValue(trajHandle_spd[HAL_MTR1], _IQ(0.0));
	TRAJ_setMinValue(trajHandle_spd[HAL_MTR1], _IQ(-1.0));
	TRAJ_setMaxValue(trajHandle_spd[HAL_MTR1], _IQ(1.0));
	TRAJ_setMaxDelta(trajHandle_spd[HAL_MTR1],
			_IQ(
					USER_MAX_ACCEL_Hzps_M1 / USER_IQ_FULL_SCALE_FREQ_Hz_M1 / USER_ISR_FREQ_Hz_M1));

	TRAJ_setTargetValue(trajHandle_spd[HAL_MTR2], _IQ(0.0));
	TRAJ_setIntValue(trajHandle_spd[HAL_MTR2], _IQ(0.0));
	TRAJ_setMinValue(trajHandle_spd[HAL_MTR2], _IQ(-1.0));
	TRAJ_setMaxValue(trajHandle_spd[HAL_MTR2], _IQ(1.0));
	TRAJ_setMaxDelta(trajHandle_spd[HAL_MTR2],
			_IQ(
					USER_MAX_ACCEL_Hzps_M2 / USER_IQ_FULL_SCALE_FREQ_Hz_M2 / USER_ISR_FREQ_Hz_M2));

	// configure the Id reference trajectory
	TRAJ_setTargetValue(trajHandle_Id[HAL_MTR1], _IQ(0.0));
	TRAJ_setIntValue(trajHandle_Id[HAL_MTR1], _IQ(0.0));
	TRAJ_setMinValue(trajHandle_Id[HAL_MTR1],
			_IQ(-USER_MOTOR_MAX_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1));
	TRAJ_setMaxValue(trajHandle_Id[HAL_MTR1],
			_IQ(USER_MOTOR_MAX_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1));
	TRAJ_setMaxDelta(trajHandle_Id[HAL_MTR1],
			_IQ(
					USER_MOTOR_RES_EST_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1 / USER_ISR_FREQ_Hz_M1));

	TRAJ_setTargetValue(trajHandle_Id[HAL_MTR2], _IQ(0.0));
	TRAJ_setIntValue(trajHandle_Id[HAL_MTR2], _IQ(0.0));
	TRAJ_setMinValue(trajHandle_Id[HAL_MTR2],
			_IQ(-USER_MOTOR_MAX_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2));
	TRAJ_setMaxValue(trajHandle_Id[HAL_MTR2],
			_IQ(USER_MOTOR_MAX_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2));
	TRAJ_setMaxDelta(trajHandle_Id[HAL_MTR2],
			_IQ(
					USER_MOTOR_RES_EST_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2 / USER_ISR_FREQ_Hz_M2));

	gId_MinValue[HAL_MTR1] = TRAJ_getMinValue(trajHandle_Id[HAL_MTR1]);
	gId_MaxDelta[HAL_MTR1] = TRAJ_getMaxDelta(trajHandle_Id[HAL_MTR1]);
	gId_MinValue[HAL_MTR2] = TRAJ_getMinValue(trajHandle_Id[HAL_MTR2]);
	gId_MaxDelta[HAL_MTR2] = TRAJ_getMaxDelta(trajHandle_Id[HAL_MTR2]);

	// set the pre-determined current and voltage feeback offset values
	gOffsets_I_pu[HAL_MTR1].value[0] = _IQ(I_A_offset_M1);
	gOffsets_I_pu[HAL_MTR1].value[1] = _IQ(I_B_offset_M1);
	gOffsets_I_pu[HAL_MTR1].value[2] = _IQ(I_C_offset_M1);
	gOffsets_V_pu[HAL_MTR1].value[0] = _IQ(V_A_offset_M1);
	gOffsets_V_pu[HAL_MTR1].value[1] = _IQ(V_B_offset_M1);
	gOffsets_V_pu[HAL_MTR1].value[2] = _IQ(V_C_offset_M1);

	gOffsets_I_pu[HAL_MTR2].value[0] = _IQ(I_A_offset_M2);
	gOffsets_I_pu[HAL_MTR2].value[1] = _IQ(I_B_offset_M2);
	gOffsets_I_pu[HAL_MTR2].value[2] = _IQ(I_C_offset_M2);
	gOffsets_V_pu[HAL_MTR2].value[0] = _IQ(V_A_offset_M2);
	gOffsets_V_pu[HAL_MTR2].value[1] = _IQ(V_B_offset_M2);
	gOffsets_V_pu[HAL_MTR2].value[2] = _IQ(V_C_offset_M2);

	gId_RsEst[HAL_MTR1] = _IQ(
			USER_MOTOR_RES_EST_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1);
	gId_RsEst[HAL_MTR2] = _IQ(
			USER_MOTOR_RES_EST_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2);

	// initialize the interrupt vector table
	HAL_initIntVectorTable(halHandle);

	// enable the ADC interrupts
	HAL_enableAdcInts(halHandle);

	// enable global interrupts
	HAL_enableGlobalInts(halHandle);

	// enable debug interrupts
	HAL_enableDebugInt(halHandle);

	// disable the PWM
	HAL_disablePwm(halHandleMtr[HAL_MTR1]);
	HAL_disablePwm(halHandleMtr[HAL_MTR2]);

	// enable the system by default
	gSystemVars.Flag_enableSystem = true;

	// Begin the background loop
	for (;;) {
		// Waiting for enable system flag to be set
		while (!(gSystemVars.Flag_enableSystem))
			;

		// loop while the enable system flag is true
		while (gSystemVars.Flag_enableSystem) {
			uint_least8_t mtrNum = HAL_MTR1;

			if (gSystemVars.Flag_enableSynControl == true) {
				gMotorVars[HAL_MTR1].Flag_Run_Identify =
						gSystemVars.Flag_enableRun;
				gMotorVars[HAL_MTR1].SpeedRef_krpm = gSystemVars.SpeedSet_krpm;
				gMotorVars[HAL_MTR1].MaxAccel_krpmps =
						gSystemVars.MaxAccelSet_krpmps;

				gMotorVars[HAL_MTR2].Flag_Run_Identify =
						gSystemVars.Flag_enableRun;
				gMotorVars[HAL_MTR2].SpeedRef_krpm = gSystemVars.SpeedSet_krpm;
				gMotorVars[HAL_MTR2].MaxAccel_krpmps =
						gSystemVars.MaxAccelSet_krpmps;
			}

			for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {

				// increment counters
				gCounter_updateGlobals[mtrNum]++;

				// If Flag_enableSys is set AND Flag_Run_Identify is set THEN
				// enable PWMs and set the speed reference
				if (gMotorVars[mtrNum].Flag_Run_Identify) {
					// disable Rs recalculation
					EST_setFlag_enableRsRecalc(estHandle[mtrNum], false);

					// update estimator state
					EST_updateState(estHandle[mtrNum], 0);

#ifdef FAST_ROM_V1p6
					// call this function to fix 1p6. This is only used for
					// F2806xF/M implementation of InstaSPIN (version 1.6 of
					// ROM), since the inductance calculation is not done
					// correctly in ROM, so this function fixes that ROM bug.
					softwareUpdate1p6(estHandle[mtrNum], &gUserParams[mtrNum]);
#endif

					// enable the PWM
					HAL_enablePwm(halHandleMtr[mtrNum]);

					// set trajectory target for speed reference
					TRAJ_setTargetValue(trajHandle_spd[mtrNum],
							_IQmpy(gMotorVars[mtrNum].SpeedRef_krpm,
									gSpeed_krpm_to_pu_sf[mtrNum]));

					// set trajectory target for Id reference
					TRAJ_setTargetValue(trajHandle_Id[mtrNum],
							gIdq_ref_pu[mtrNum].value[0]);

				} else if (gMotorVars[mtrNum].Flag_enableRsRecalc) {
					// set angle to zero
					EST_setAngle_pu(estHandle[mtrNum], _IQ(0.0));

					// enable or disable Rs recalculation
					EST_setFlag_enableRsRecalc(estHandle[mtrNum], true);

					// update estimator state
					EST_updateState(estHandle[mtrNum], 0);

#ifdef FAST_ROM_V1p6
					// call this function to fix 1p6
					softwareUpdate1p6(estHandle[mtrNum], &gUserParams[mtrNum]);
#endif

					// enable the PWM
					HAL_enablePwm(halHandleMtr[mtrNum]);

					// set trajectory target for speed reference
					TRAJ_setTargetValue(trajHandle_spd[mtrNum], _IQ(0.0));

					// set trajectory target for Id reference
					TRAJ_setTargetValue(trajHandle_Id[mtrNum],
							gId_RsEst[mtrNum]);

					// if done with Rs recalculation, disable flag
					if (EST_getState(estHandle[mtrNum]) == EST_State_OnLine)
						gMotorVars[mtrNum].Flag_enableRsRecalc = false;
				} else // Flag_enableSys is set AND Flag_Run_Identify is not set
				{
					// set estimator to Idle
					EST_setIdle(estHandle[mtrNum]);

					// disable the PWM
					if (!gMotorVars[mtrNum].Flag_enableOffsetcalc)
						HAL_disablePwm(halHandleMtr[mtrNum]);

					// clear the speed reference trajectory
					TRAJ_setTargetValue(trajHandle_spd[mtrNum], _IQ(0.0));
					TRAJ_setIntValue(trajHandle_spd[mtrNum], _IQ(0.0));

					// clear the Id reference trajectory
					TRAJ_setTargetValue(trajHandle_Id[mtrNum], _IQ(0.0));
					TRAJ_setIntValue(trajHandle_Id[mtrNum], _IQ(0.0));

					TRAJ_setMinValue(trajHandle_Id[mtrNum],
							gId_MinValue[mtrNum]);
					TRAJ_setMaxDelta(trajHandle_Id[mtrNum],
							gId_MaxDelta[mtrNum]);

					// clear integrator outputs
					PID_setUi(pidHandle[mtrNum][0], _IQ(0.0));
					PID_setUi(pidHandle[mtrNum][1], _IQ(0.0));
					PID_setUi(pidHandle[mtrNum][2], _IQ(0.0));

					// clear Id and Iq references
					gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
					gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);
				}

				// update the global variables
				// when appropriate, update the global variables
				if (gCounter_updateGlobals[mtrNum]
						>= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE) {
					// reset the counter
					gCounter_updateGlobals[mtrNum] = 0;

					updateGlobalVariables(estHandle[mtrNum], mtrNum);
				}

				// enable/disable the forced angle
				EST_setFlag_enableForceAngle(estHandle[mtrNum],
						gMotorVars[mtrNum].Flag_enableForceAngle);

				// set the speed acceleration
				TRAJ_setMaxDelta(trajHandle_spd[mtrNum],
						_IQmpy(MAX_ACCEL_KRPMPS_SF_M1,
								gMotorVars[mtrNum].MaxAccel_krpmps));

#ifdef DRV8301_SPI
				HAL_writeDrvData(halHandleMtr[mtrNum],&gDrvSpi8301Vars[mtrNum]);

				HAL_readDrvData(halHandleMtr[mtrNum],&gDrvSpi8301Vars[mtrNum]);
#endif
#ifdef DRV8305_SPI
				HAL_writeDrvData(halHandleMtr[mtrNum],
						&gDrvSpi8305Vars[mtrNum]);

				HAL_readDrvData(halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
#endif
			} // end of for loop
		} // end of while(gFlag_enableSys) loop

		// disable the PWM
		HAL_disablePwm(halHandleMtr[HAL_MTR1]);
		HAL_disablePwm(halHandleMtr[HAL_MTR2]);

		gMotorVars[HAL_MTR1].Flag_Run_Identify = false;
		gMotorVars[HAL_MTR2].Flag_Run_Identify = false;

	} // end of for(;;) loop
} // end of main() function



//! \brief     The main ISR that implements the motor control.
interrupt void motor1_ISR(void) {
	// Declaration of local variables
	_iq angle_pu = _IQ(0.0);
	_iq speed_pu = _IQ(0.0);
	_iq oneOverDcBus;
	MATH_vec2 Iab_pu;
	MATH_vec2 Vab_pu;
	MATH_vec2 phasor;

	// read the timer 2 value and update the CPU usage module
	uint32_t timer1Cnt = HAL_readTimerCnt(halHandle, 2);
	CPU_TIME_updateCnts(cpu_timeHandle[HAL_MTR1], timer1Cnt);

	HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED1);

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

	// convert the ADC data
	HAL_readAdcDataWithOffsets(halHandle, halHandleMtr[HAL_MTR1],
			&gAdcData[HAL_MTR1]);

	// remove offsets
	gAdcData[HAL_MTR1].I.value[0] = gAdcData[HAL_MTR1].I.value[0]
			- gOffsets_I_pu[HAL_MTR1].value[0];
	gAdcData[HAL_MTR1].I.value[1] = gAdcData[HAL_MTR1].I.value[1]
			- gOffsets_I_pu[HAL_MTR1].value[1];
	gAdcData[HAL_MTR1].I.value[2] = gAdcData[HAL_MTR1].I.value[2]
			- gOffsets_I_pu[HAL_MTR1].value[2];
	gAdcData[HAL_MTR1].V.value[0] = gAdcData[HAL_MTR1].V.value[0]
			- gOffsets_V_pu[HAL_MTR1].value[0];
	gAdcData[HAL_MTR1].V.value[1] = gAdcData[HAL_MTR1].V.value[1]
			- gOffsets_V_pu[HAL_MTR1].value[1];
	gAdcData[HAL_MTR1].V.value[2] = gAdcData[HAL_MTR1].V.value[2]
			- gOffsets_V_pu[HAL_MTR1].value[2];

	// run Clarke transform on current.  Three values are passed, two values
	// are returned.
	CLARKE_run(clarkeHandle_I[HAL_MTR1], &gAdcData[HAL_MTR1].I, &Iab_pu);

	// run Clarke transform on voltage.  Three values are passed, two values
	// are returned.
	CLARKE_run(clarkeHandle_V[HAL_MTR1], &gAdcData[HAL_MTR1].V, &Vab_pu);

	// run a trajectory for Id reference, so the reference changes with a ramp instead of a step
	TRAJ_run(trajHandle_Id[HAL_MTR1]);



	// generate the motor electrical angle
#ifdef QEP
    // compute the electrical angle
    //ENC_calcElecAngle(encHandle[HAL_MTR1], HAL_getQepPosnCounts(halHandleMtr[HAL_MTR1]));
	//angle_pu = ENC_getElecAngle(encHandle[HAL_MTR1]);
#else
	// run the estimator
	// The speed reference is needed so that the proper sign of the forced
	// angle is calculated. When the estimator does not do motor ID as in this
	// lab, only the sign of the speed reference is used
	EST_run(estHandle[HAL_MTR1], &Iab_pu, &Vab_pu, gAdcData[HAL_MTR1].dcBus,
			TRAJ_getIntValue(trajHandle_spd[HAL_MTR1]));
	angle_pu = EST_getAngle_pu(estHandle[HAL_MTR1]);
#endif


	//gAngle_pu = angle_pu;
	// get Idq from estimator to avoid sin and cos, and a Park transform,
	// which saves CPU cycles
	EST_getIdq_pu(estHandle[HAL_MTR1], &gIdq_pu[HAL_MTR1]);

	// run the appropriate controller
	if (gMotorVars[HAL_MTR1].Flag_Run_Identify
			|| (gMotorVars[HAL_MTR1].Flag_enableRsRecalc)) {
		// Declaration of local variables.
		_iq refValue;
		_iq fbackValue;
		_iq outMax_pu;


		if(gMotorVars[HAL_MTR1].Flag_enableAlignment == false) {
			// when appropriate, run the PID speed controller
			// This mechanism provides the decimation for the speed loop.
			if ((pidCntSpeed[HAL_MTR1]++
					>= gUserParams[HAL_MTR1].numCtrlTicksPerSpeedTick)
					&& (!gMotorVars[HAL_MTR1].Flag_enableRsRecalc)) {
				// Reset the Speed PID execution counter.
				pidCntSpeed[HAL_MTR1] = 0;

				// The next instruction executes the PI speed controller and places
				// its output in Idq_ref_pu.value[1], which is the input reference
				// value for the q-axis current controller.
				PID_run_spd(pidHandle[HAL_MTR1][0],
						TRAJ_getIntValue(trajHandle_spd[HAL_MTR1]), speed_pu,
						&(gIdq_ref_pu[HAL_MTR1].value[1]));
			}
		}
		else
		{
			// the alignment procedure is in effect

			// force motor angle and speed to 0
			angle_pu = _IQ(0.0);
			speed_pu = _IQ(0.0);

			// set D-axis current to Rs estimation current
			gIdq_ref_pu[HAL_MTR1].value[0] = _IQ(USER_MOTOR_RES_EST_CURRENT_M1/USER_IQ_FULL_SCALE_CURRENT_A_M1);
			// set Q-axis current to 0
			gIdq_ref_pu[HAL_MTR1].value[1] = _IQ(0.0);

			// save encoder reading when forcing motor into alignment
			if(gUserParams[HAL_MTR1].motor_type == MOTOR_Type_Pm)
			{
			  ENC_setZeroOffset(encHandle[HAL_MTR1], (uint32_t)(HAL_getQepPosnMaximum(halHandleMtr[HAL_MTR1]) - HAL_getQepPosnCounts(halHandleMtr[HAL_MTR1])));
			}

			// if alignment counter exceeds threshold, exit alignment
			if(gAlignCount[HAL_MTR1]++ >= gUserParams[HAL_MTR1].ctrlWaitTime[CTRL_State_OffLine])
			{
			  gMotorVars[HAL_MTR1].Flag_enableAlignment = false;
			  gAlignCount[HAL_MTR1] = 0;
			  gIdq_ref_pu[HAL_MTR1].value[0] = _IQ(0.0);
			}
		}
		// Get the reference value from the trajectory module
		refValue = TRAJ_getIntValue(trajHandle_Id[HAL_MTR1]);

		// Get the actual value of Id
		fbackValue = gIdq_pu[HAL_MTR1].value[0];

		// The next instruction executes the PI current controller for the
		// d axis and places its output in Vdq_pu.value[0], which is the
		// control voltage along the d-axis (Vd)
		PID_run(pidHandle[HAL_MTR1][1], refValue, fbackValue,
				&(gVdq_out_pu[HAL_MTR1].value[0]));

		// set Iq reference to zero when doing Rs recalculation
		if (gMotorVars[HAL_MTR1].Flag_enableRsRecalc) {
			gIdq_ref_pu[HAL_MTR1].value[1] = _IQ(0.0);
		}
		// get the Iq reference value
		refValue = gIdq_ref_pu[HAL_MTR1].value[1];

		// get the actual value of Iq
		fbackValue = gIdq_pu[HAL_MTR1].value[1];

		// The voltage limits on the output of the q-axis current controller
		// are dynamic, and are dependent on the output voltage from the d-axis
		// current controller.  In other words, the d-axis current controller
		// gets first dibs on the available voltage, and the q-axis current
		// controller gets what's left over.  That is why the d-axis current
		// controller executes first. The next instruction calculates the
		// maximum limits for this voltage as:
		// Vq_min_max = +/- sqrt(Vbus^2 - Vd^2)
		_iq max_vs = _IQmpy(_IQ(USER_MAX_VS_MAG_PU_M1),
				EST_getDcBus_pu(estHandle[HAL_MTR1]));
		outMax_pu =
				_IQsqrt(
						_IQmpy(max_vs,max_vs) - _IQmpy(gVdq_out_pu[HAL_MTR1].value[0],gVdq_out_pu[HAL_MTR1].value[0]));

		// Set the limits to +/- outMax_pu
		PID_setMinMax(pidHandle[HAL_MTR1][2], -outMax_pu, outMax_pu);

		// The next instruction executes the PI current controller for the
		// q axis and places its output in Vdq_pu.value[1], which is the
		// control voltage vector along the q-axis (Vq)
		PID_run(pidHandle[HAL_MTR1][2], refValue, fbackValue,
				&(gVdq_out_pu[HAL_MTR1].value[1]));

		// The voltage vector is now calculated and ready to be applied to the
		// motor in the form of three PWM signals.  However, even though the
		// voltages may be supplied to the PWM module now, they won't be
		// applied to the motor until the next PWM cycle. By this point, the
		// motor will have moved away from the angle that the voltage vector
		// was calculated for, by an amount which is proportional to the
		// sampling frequency and the speed of the motor.  For steady-state
		// speeds, we can calculate this angle delay and compensate for it.
		ANGLE_COMP_run(angleCompHandle[HAL_MTR1], speed_pu, angle_pu);
		angle_pu = ANGLE_COMP_getAngleComp_pu(angleCompHandle[HAL_MTR1]);

		// compute the sine and cosine phasor values which are part of the inverse
		// Park transform calculations. Once these values are computed,
		// they are copied into the IPARK module, which then uses them to
		// transform the voltages from DQ to Alpha/Beta reference frames.
		phasor.value[0] = _IQcosPU(angle_pu);
		phasor.value[1] = _IQsinPU(angle_pu);

		// set the phasor in the inverse Park transform
		IPARK_setPhasor(iparkHandle[HAL_MTR1], &phasor);

		// Run the inverse Park module.  This converts the voltage vector from
		// synchronous frame values to stationary frame values.
		IPARK_run(iparkHandle[HAL_MTR1], &gVdq_out_pu[HAL_MTR1], &Vab_pu);

		// These 3 statements compensate for variations in the DC bus by adjusting the
		// PWM duty cycle. The goal is to achieve the same volt-second product
		// regardless of the DC bus value.  To do this, we must divide the desired voltage
		// values by the DC bus value.  Or...it is easier to multiply by 1/(DC bus value).
		oneOverDcBus = EST_getOneOverDcBus_pu(estHandle[HAL_MTR1]);
		Vab_pu.value[0] = _IQmpy(Vab_pu.value[0], oneOverDcBus);
		Vab_pu.value[1] = _IQmpy(Vab_pu.value[1], oneOverDcBus);

		// Now run the space vector generator (SVGEN) module.
		// There is no need to do an inverse CLARKE transform, as this is
		// handled in the SVGEN_run function.
		SVGEN_run(svgenHandle[HAL_MTR1], &Vab_pu, &(gPwmData[HAL_MTR1].Tabc));

		gTrjCnt[HAL_MTR1]++;

	} else if (gMotorVars[HAL_MTR1].Flag_enableOffsetcalc == true) {
		runOffsetsCalculation(HAL_MTR1);
	}
    else  // gMotorVars.Flag_Run_Identify = 0
    {
	// disable the PWM
	HAL_disablePwm(halHandleMtr[HAL_MTR1]);

	// Set the PWMs to 50% duty cycle
	gPwmData[HAL_MTR1].Tabc.value[0] = _IQ(0.0);
	gPwmData[HAL_MTR1].Tabc.value[1] = _IQ(0.0);
	gPwmData[HAL_MTR1].Tabc.value[2] = _IQ(0.0);
    }
	// write to the PWM compare registers, and then we are done!
	HAL_writePwmData(halHandleMtr[HAL_MTR1], &gPwmData[HAL_MTR1]);

	if (gTrjCnt[HAL_MTR1] >= gUserParams[HAL_MTR1].numCtrlTicksPerTrajTick) {
		// clear counter
		gTrjCnt[HAL_MTR1] = 0;

		// run a trajectory for speed reference, so the reference changes with a ramp instead of a step
		TRAJ_run(trajHandle_spd[HAL_MTR1]);
	}


	// read the timer 2 value and update the CPU usage module
	timer1Cnt = HAL_readTimerCnt(halHandle, 2);
	CPU_TIME_run(cpu_timeHandle[HAL_MTR1], timer1Cnt);

	HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED1);

	return;
} // end of motor1_ISR() function

interrupt void motor2_ISR(void) {
	// Declaration of local variables
	static _iq angle_pu = _IQ(0.0);
	_iq speed_pu = _IQ(0.0);
	_iq oneOverDcBus;
	MATH_vec2 Iab_pu;
	MATH_vec2 Vab_pu;
	MATH_vec2 phasor;

	// read the timer 2 value and update the CPU usage module
	uint32_t timer1Cnt = HAL_readTimerCnt(halHandle, 2);
	CPU_TIME_updateCnts(cpu_timeHandle[HAL_MTR2], timer1Cnt);

	HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_2);

	// convert the ADC data
	HAL_readAdcDataWithOffsets(halHandle,halHandleMtr[HAL_MTR2],&gAdcData[HAL_MTR2]);

	// remove offsets
	gAdcData[HAL_MTR2].I.value[0] = gAdcData[HAL_MTR2].I.value[0] - gOffsets_I_pu[HAL_MTR2].value[0];
	gAdcData[HAL_MTR2].I.value[1] = gAdcData[HAL_MTR2].I.value[1] - gOffsets_I_pu[HAL_MTR2].value[1];
	gAdcData[HAL_MTR2].I.value[2] = gAdcData[HAL_MTR2].I.value[2] - gOffsets_I_pu[HAL_MTR2].value[2];
	gAdcData[HAL_MTR2].V.value[0] = gAdcData[HAL_MTR2].V.value[0] - gOffsets_V_pu[HAL_MTR2].value[0];
	gAdcData[HAL_MTR2].V.value[1] = gAdcData[HAL_MTR2].V.value[1] - gOffsets_V_pu[HAL_MTR2].value[1];
	gAdcData[HAL_MTR2].V.value[2] = gAdcData[HAL_MTR2].V.value[2] - gOffsets_V_pu[HAL_MTR2].value[2];

	// run Clarke transform on current.  Three values are passed, two values
	// are returned.
	CLARKE_run(clarkeHandle_I[HAL_MTR2], &gAdcData[HAL_MTR2].I, &Iab_pu);

	// run Clarke transform on voltage.  Three values are passed, two values
	// are returned.
	CLARKE_run(clarkeHandle_V[HAL_MTR2], &gAdcData[HAL_MTR2].V, &Vab_pu); // needed for estimator

	// compute the sine and cosine phasor values which are part of the
	// Park transform calculations. Once these values are computed,
	// they are copied into the PARK module, which then uses them to
	// transform the voltages from Alpha/Beta to DQ reference frames.
	phasor.value[0] = _IQcosPU(angle_pu);
	phasor.value[1] = _IQsinPU(angle_pu);

	// set the phasor in the Park transform
	PARK_setPhasor(parkHandle[HAL_MTR2],&phasor);

	// Run the Park module.  This converts the current vector from
	// stationary frame values to synchronous frame values.
	PARK_run(parkHandle[HAL_MTR2],&Iab_pu,&gIdq_pu[HAL_MTR2]);

	// run a trajectory for Id reference, so the reference changes with a ramp instead of a step
	TRAJ_run(trajHandle_Id[HAL_MTR2]);

	// run the estimator
	// The speed reference is needed so that the proper sign of the forced
	// angle is calculated. When the estimator does not do motor ID as in this
	// lab, only the sign of the speed reference is used
	EST_run(estHandle[HAL_MTR2], &Iab_pu, &Vab_pu, gAdcData[HAL_MTR2].dcBus,
			TRAJ_getIntValue(trajHandle_spd[HAL_MTR2]));
	ENC_run(encHandle[HAL_MTR2], HAL_getQepPosnCounts(halHandleMtr[HAL_MTR2]), HAL_getQepIndex(halHandleMtr[HAL_MTR2]), HAL_getQepDirection(halHandleMtr[HAL_MTR2]), true);
	// generate the motor electrical angle
#ifdef QEP
    // compute the electrical angle
    ENC_calcElecAngle(encHandle[HAL_MTR2], HAL_getQepPosnCounts(halHandleMtr[HAL_MTR2]));
	angle_pu = ENC_getElecAngle(encHandle[HAL_MTR2]);
#else
	angle_pu = EST_getAngle_pu(estHandle[HAL_MTR2]);
#endif



	gAngle_pu = angle_pu;
	// generate the motor electrical angle
	//angle_pu = EST_getAngle_pu(estHandle[HAL_MTR2]);
	//speed_pu = EST_getFm_pu(estHandle[HAL_MTR2]);
	speed_pu = ENC_getFilteredSpeed(encHandle[HAL_MTR2]);
	// get Idq from estimator to avoid sin and cos, and a Park transform,
	// which saves CPU cycles
	//EST_getIdq_pu(estHandle[HAL_MTR2], &gIdq_pu[HAL_MTR2]);

	// run the appropriate controller
	if (gMotorVars[HAL_MTR2].Flag_Run_Identify
			|| (gMotorVars[HAL_MTR2].Flag_enableRsRecalc)) {
		// Declaration of local variables.
		_iq refValue;
		_iq fbackValue;
		_iq outMax_pu;

		if(gMotorVars[HAL_MTR2].Flag_enableAlignment == false) {
			// when appropriate, run the PID speed controller
			// This mechanism provides the decimation for the speed loop.
			if ((pidCntSpeed[HAL_MTR2]++
					>= gUserParams[HAL_MTR2].numCtrlTicksPerSpeedTick)
					&& (!gMotorVars[HAL_MTR2].Flag_enableRsRecalc)) {
				// Reset the Speed PID execution counter.
				pidCntSpeed[HAL_MTR2] = 0;

				// The next instruction executes the PI speed controller and places
				// its output in Idq_ref_pu.value[1], which is the input reference
				// value for the q-axis current controller.
				PID_run_spd(pidHandle[HAL_MTR2][0],
						TRAJ_getIntValue(trajHandle_spd[HAL_MTR2]), speed_pu,
						&(gIdq_ref_pu[HAL_MTR2].value[1]));
			}
		} else {
			// the alignment procedure is in effect

			// force motor angle and speed to 0
			angle_pu = _IQ(0.0);
			speed_pu = _IQ(0.0);

			// set D-axis current to Rs estimation current
			gIdq_ref_pu[HAL_MTR2].value[0] = _IQ(USER_MOTOR_RES_EST_CURRENT_M2/USER_IQ_FULL_SCALE_CURRENT_A_M2);
			// set Q-axis current to 0
			gIdq_ref_pu[HAL_MTR2].value[1] = _IQ(0.0);

			// save encoder reading when forcing motor into alignment
			if(gUserParams[HAL_MTR2].motor_type == MOTOR_Type_Pm)
			{
			  ENC_setZeroOffset(encHandle[HAL_MTR2], (uint32_t)(HAL_getQepPosnMaximum(halHandleMtr[HAL_MTR2]) - HAL_getQepPosnCounts(halHandleMtr[HAL_MTR2])));
			}

			// if alignment counter exceeds threshold, exit alignment
			if(gAlignCount[HAL_MTR2]++ >= gUserParams[HAL_MTR2].ctrlWaitTime[CTRL_State_OffLine]) {
				gMotorVars[HAL_MTR2].Flag_enableAlignment = false;
				gAlignCount[HAL_MTR2] = 0;
				gIdq_ref_pu[HAL_MTR2].value[0] = _IQ(0.0);
			}
		}

		// Get the reference value from the trajectory module
		refValue = TRAJ_getIntValue(trajHandle_Id[HAL_MTR2]);

		// Get the actual value of Id
		fbackValue = gIdq_pu[HAL_MTR2].value[0];

		// The next instruction executes the PI current controller for the
		// d axis and places its output in Vdq_pu.value[0], which is the
		// control voltage along the d-axis (Vd)
		PID_run(pidHandle[HAL_MTR2][1], refValue, fbackValue,
				&(gVdq_out_pu[HAL_MTR2].value[0]));

		// get the Iq reference value
		refValue = gIdq_ref_pu[HAL_MTR2].value[1];

		// set Iq reference to zero when doing Rs recalculation
		if (gMotorVars[HAL_MTR2].Flag_enableRsRecalc)
			gIdq_ref_pu[HAL_MTR2].value[1] = _IQ(0.0);

		// get the actual value of Iq
		fbackValue = gIdq_pu[HAL_MTR2].value[1];

		// The voltage limits on the output of the q-axis current controller
		// are dynamic, and are dependent on the output voltage from the d-axis
		// current controller.  In other words, the d-axis current controller
		// gets first dibs on the available voltage, and the q-axis current
		// controller gets what's left over.  That is why the d-axis current
		// controller executes first. The next instruction calculates the
		// maximum limits for this voltage as:
		// Vq_min_max = +/- sqrt(Vbus^2 - Vd^2)
		_iq max_vs = _IQmpy(_IQ(USER_MAX_VS_MAG_PU_M2),
				EST_getDcBus_pu(estHandle[HAL_MTR2]));
		outMax_pu =
				_IQsqrt(
						_IQmpy(max_vs,max_vs) - _IQmpy(gVdq_out_pu[HAL_MTR2].value[0],gVdq_out_pu[HAL_MTR2].value[0]));

		// Set the limits to +/- outMax_pu
		PID_setMinMax(pidHandle[HAL_MTR2][2], -outMax_pu, outMax_pu);

		// The next instruction executes the PI current controller for the
		// q axis and places its output in Vdq_pu.value[1], which is the
		// control voltage vector along the q-axis (Vq)
		PID_run(pidHandle[HAL_MTR2][2], refValue, fbackValue,
				&(gVdq_out_pu[HAL_MTR2].value[1]));

		// The voltage vector is now calculated and ready to be applied to the
		// motor in the form of three PWM signals.  However, even though the
		// voltages may be supplied to the PWM module now, they won't be
		// applied to the motor until the next PWM cycle. By this point, the
		// motor will have moved away from the angle that the voltage vector
		// was calculated for, by an amount which is proportional to the
		// sampling frequency and the speed of the motor.  For steady-state
		// speeds, we can calculate this angle delay and compensate for it.
		ANGLE_COMP_run(angleCompHandle[HAL_MTR2], speed_pu, angle_pu);
		angle_pu = ANGLE_COMP_getAngleComp_pu(angleCompHandle[HAL_MTR2]);

		// compute the sine and cosine phasor values which are part of the inverse
		// Park transform calculations. Once these values are computed,
		// they are copied into the IPARK module, which then uses them to
		// transform the voltages from DQ to Alpha/Beta reference frames.
		phasor.value[0] = _IQcosPU(angle_pu);
		phasor.value[1] = _IQsinPU(angle_pu);

		// set the phasor in the inverse Park transform
		IPARK_setPhasor(iparkHandle[HAL_MTR2], &phasor);

		// Run the inverse Park module.  This converts the voltage vector from
		// synchronous frame values to stationary frame values.
		IPARK_run(iparkHandle[HAL_MTR2], &gVdq_out_pu[HAL_MTR2], &Vab_pu);

		// These 3 statements compensate for variations in the DC bus by adjusting the
		// PWM duty cycle. The goal is to achieve the same volt-second product
		// regardless of the DC bus value.  To do this, we must divide the desired voltage
		// values by the DC bus value.  Or...it is easier to multiply by 1/(DC bus value).
		oneOverDcBus = EST_getOneOverDcBus_pu(estHandle[HAL_MTR2]);
		Vab_pu.value[0] = _IQmpy(Vab_pu.value[0], oneOverDcBus);
		Vab_pu.value[1] = _IQmpy(Vab_pu.value[1], oneOverDcBus);

		// Now run the space vector generator (SVGEN) module.
		// There is no need to do an inverse CLARKE transform, as this is
		// handled in the SVGEN_run function.
		SVGEN_run(svgenHandle[HAL_MTR2], &Vab_pu, &(gPwmData[HAL_MTR2].Tabc));

		gTrjCnt[HAL_MTR2]++;
	} else if (gMotorVars[HAL_MTR2].Flag_enableOffsetcalc == true) {
		runOffsetsCalculation(HAL_MTR2);
	} else  // gMotorVars.Flag_Run_Identify = 0
	{
		// disable the PWM
		HAL_disablePwm(halHandleMtr[HAL_MTR2]);

		// Set the PWMs to 50% duty cycle
		gPwmData[HAL_MTR2].Tabc.value[0] = _IQ(0.0);
		gPwmData[HAL_MTR2].Tabc.value[1] = _IQ(0.0);
		gPwmData[HAL_MTR2].Tabc.value[2] = _IQ(0.0);
	}

	// write to the PWM compare registers, and then we are done!
	HAL_writePwmData(halHandleMtr[HAL_MTR2], &gPwmData[HAL_MTR2]);

	if (gTrjCnt[HAL_MTR2] >= gUserParams[HAL_MTR2].numCtrlTicksPerTrajTick) {
		// clear counter
		gTrjCnt[HAL_MTR2] = 0;

		// run a trajectory for speed reference, so the reference changes with a ramp instead of a step
		TRAJ_run(trajHandle_spd[HAL_MTR2]);
	}



	// read the timer 2 value and update the CPU usage module
	timer1Cnt = HAL_readTimerCnt(halHandle, 2);
	CPU_TIME_run(cpu_timeHandle[HAL_MTR2], timer1Cnt);

	HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
	return;
} // end of motor2_ISR() function

void pidSetup(HAL_MtrSelect_e mtrNum) {
	// This equation defines the relationship between per unit current and
	// real-world current. The resulting value in per units (pu) is then
	// used to configure the controllers
	_iq maxCurrent_pu = _IQ(
			gUserParams[mtrNum].maxCurrent
					/ gUserParams[mtrNum].iqFullScaleCurrent_A);

	// This equation uses the scaled maximum voltage vector, which is
	// already in per units, hence there is no need to include the #define
	// for USER_IQ_FULL_SCALE_VOLTAGE_V
	_iq maxVoltage_pu = _IQ(
			gUserParams[mtrNum].maxVsMag_pu * gUserParams[mtrNum].voltage_sf);

	float_t fullScaleCurrent = gUserParams[mtrNum].iqFullScaleCurrent_A;
	float_t fullScaleVoltage = gUserParams[mtrNum].iqFullScaleVoltage_V;
	float_t IsrPeriod_sec = 1.0e-6 * gUserParams[mtrNum].pwmPeriod_usec
			* gUserParams[mtrNum].numPwmTicksPerIsrTick;
	float_t Ls_d = gUserParams[mtrNum].motor_Ls_d;
	float_t Ls_q = gUserParams[mtrNum].motor_Ls_q;
	float_t Rs = gUserParams[mtrNum].motor_Rs;

	// This lab assumes that motor parameters are known, and it does not
	// perform motor ID, so the R/L parameters are known and defined in
	// user.h
	float_t RoverLs_d = Rs / Ls_d;
	float_t RoverLs_q = Rs / Ls_q;

	// For the current controller, Kp = Ls*bandwidth(rad/sec)  But in order
	// to be used, it must be converted to per unit values by multiplying
	// by fullScaleCurrent and then dividing by fullScaleVoltage.  From the
	// statement below, we see that the bandwidth in rad/sec is equal to
	// 0.25/IsrPeriod_sec, which is equal to USER_ISR_FREQ_HZ/4. This means
	// that by setting Kp as described below, the bandwidth in Hz is
	// USER_ISR_FREQ_HZ/(8*pi).
	_iq Kp_Id = _IQ(
			(0.25 * Ls_d * fullScaleCurrent)
					/ (IsrPeriod_sec * fullScaleVoltage));

	// In order to achieve pole/zero cancellation (which reduces the
	// closed-loop transfer function from a second-order system to a
	// first-order system), Ki must equal Rs/Ls.  Since the output of the
	// Ki gain stage is integrated by a DIGITAL integrator, the integrator
	// input must be scaled by 1/IsrPeriod_sec.  That's just the way
	// digital integrators work.  But, since IsrPeriod_sec is a constant,
	// we can save an additional multiplication operation by lumping this
	// term with the Ki value.
	_iq Ki_Id = _IQ(RoverLs_d * IsrPeriod_sec);

	// Now do the same thing for Kp for the q-axis current controller.
	// If the motor is not an IPM motor, Ld and Lq are the same, which
	// means that Kp_Iq = Kp_Id
	_iq Kp_Iq = _IQ(
			(0.25 * Ls_q * fullScaleCurrent)
					/ (IsrPeriod_sec * fullScaleVoltage));

	// Do the same thing for Ki for the q-axis current controller.  If the
	// motor is not an IPM motor, Ld and Lq are the same, which means that
	// Ki_Iq = Ki_Id.
	_iq Ki_Iq = _IQ(RoverLs_q * IsrPeriod_sec);

	// There are three PI controllers; one speed controller and two current
	// controllers.  Each PI controller has two coefficients; Kp and Ki.
	// So you have a total of six coefficients that must be defined.
	// This is for the speed controller
	pidHandle[mtrNum][0] = PID_init(&pid[mtrNum][0], sizeof(pid[mtrNum][0]));
	// This is for the Id current controller
	pidHandle[mtrNum][1] = PID_init(&pid[mtrNum][1], sizeof(pid[mtrNum][1]));
	// This is for the Iq current controller
	pidHandle[mtrNum][2] = PID_init(&pid[mtrNum][2], sizeof(pid[mtrNum][2]));

	// The following instructions load the parameters for the speed PI
	// controller.
	PID_setGains(pidHandle[mtrNum][0], _IQ(1.0), _IQ(0.01), _IQ(0.0));

	// The current limit is performed by the limits placed on the speed PI
	// controller output.  In the following statement, the speed
	// controller's largest negative current is set to -maxCurrent_pu, and
	// the largest positive current is set to maxCurrent_pu.
	PID_setMinMax(pidHandle[mtrNum][0], -maxCurrent_pu, maxCurrent_pu);
	PID_setUi(pidHandle[mtrNum][0], _IQ(0.0)); // Set the initial condition value
	// for the integrator output to 0

	pidCntSpeed[mtrNum] = 0;  // Set the counter for decimating the speed
							  // controller to 0

	// The following instructions load the parameters for the d-axis
	// current controller.
	// P term = Kp_Id, I term = Ki_Id, D term = 0
	PID_setGains(pidHandle[mtrNum][1], Kp_Id, Ki_Id, _IQ(0.0));

	// Largest negative voltage = -maxVoltage_pu, largest positive
	// voltage = maxVoltage_pu
	PID_setMinMax(pidHandle[mtrNum][1], -maxVoltage_pu, maxVoltage_pu);

	// Set the initial condition value for the integrator output to 0
	PID_setUi(pidHandle[mtrNum][1], _IQ(0.0));

	// The following instructions load the parameters for the q-axis
	// current controller.
	// P term = Kp_Iq, I term = Ki_Iq, D term = 0
	PID_setGains(pidHandle[mtrNum][2], Kp_Iq, Ki_Iq, _IQ(0.0));

	// The largest negative voltage = 0 and the largest positive
	// voltage = 0.  But these limits are updated every single ISR before
	// actually executing the Iq controller. The limits depend on how much
	// voltage is left over after the Id controller executes. So having an
	// initial value of 0 does not affect Iq current controller execution.
	PID_setMinMax(pidHandle[mtrNum][2], _IQ(0.0), _IQ(0.0));

	// Set the initial condition value for the integrator output to 0
	PID_setUi(pidHandle[mtrNum][2], _IQ(0.0));
}

void runOffsetsCalculation(HAL_MtrSelect_e mtrNum) {
	uint16_t cnt;

	// enable the PWM
	HAL_enablePwm(halHandleMtr[mtrNum]);

	for (cnt = 0; cnt < 3; cnt++) {
		// Set the PWMs to 50% duty cycle
		gPwmData[mtrNum].Tabc.value[cnt] = _IQ(0.0);

		// reset offsets used
		gOffsets_I_pu[mtrNum].value[cnt] = _IQ(0.0);
		gOffsets_V_pu[mtrNum].value[cnt] = _IQ(0.0);

		// run offset estimation
		FILTER_FO_run(filterHandle[mtrNum][cnt], gAdcData[mtrNum].I.value[cnt]);
		FILTER_FO_run(filterHandle[mtrNum][cnt + 3],
				gAdcData[mtrNum].V.value[cnt]);
	}

	if (gOffsetCalcCount[mtrNum]++
			>= gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
		gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
		gOffsetCalcCount[mtrNum] = 0;

		for (cnt = 0; cnt < 3; cnt++) {
			// get calculated offsets from filter
			gOffsets_I_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(
					filterHandle[mtrNum][cnt]);
			gOffsets_V_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(
					filterHandle[mtrNum][cnt + 3]);

			// clear filters
			FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt], _IQ(0.0),
					_IQ(0.0));
			FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt + 3],
					_IQ(0.0), _IQ(0.0));
		}
	}

	return;
} // end of runOffsetsCalculation() function

//! \brief  Call this function to fix 1p6. This is only used for F2806xF/M
//! \brief  implementation of InstaSPIN (version 1.6 of ROM) since the
//! \brief  inductance calculation is not done correctly in ROM, so this
//! \brief  function fixes that ROM bug.
void softwareUpdate1p6(EST_Handle handle, USER_Params *pUserParams) {
	float_t iqFullScaleVoltage_V = pUserParams->iqFullScaleVoltage_V;
	float_t iqFullScaleCurrent_A = pUserParams->iqFullScaleCurrent_A;
	float_t voltageFilterPole_rps = pUserParams->voltageFilterPole_rps;
	float_t motorLs_d = pUserParams->motor_Ls_d;
	float_t motorLs_q = pUserParams->motor_Ls_q;

	float_t fullScaleInductance = iqFullScaleVoltage_V
			/ (iqFullScaleCurrent_A * voltageFilterPole_rps);
	float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
	int_least8_t lShift = ceil(
			log(motorLs_d / (Ls_coarse_max * fullScaleInductance)) / log(2.0));
	uint_least8_t Ls_qFmt = 30 - lShift;
	float_t L_max = fullScaleInductance * pow(2.0, lShift);
	_iq Ls_d_pu = _IQ30(motorLs_d / L_max);
	_iq Ls_q_pu = _IQ30(motorLs_q / L_max);

	// store the results
	EST_setLs_d_pu(handle, Ls_d_pu);
	EST_setLs_q_pu(handle, Ls_q_pu);
	EST_setLs_qFmt(handle, Ls_qFmt);

	return;
} // end of softwareUpdate1p6() function

//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
void setupClarke_I(CLARKE_Handle handle, const uint_least8_t numCurrentSensors) {
	_iq alpha_sf, beta_sf;

	// initialize the Clarke transform module for current
	if (numCurrentSensors == 3) {
		alpha_sf = _IQ(MATH_ONE_OVER_THREE);
		beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
	} else if (numCurrentSensors == 2) {
		alpha_sf = _IQ(1.0);
		beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
	} else {
		alpha_sf = _IQ(0.0);
		beta_sf = _IQ(0.0);
	}

	// set the parameters
	CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
	CLARKE_setNumSensors(handle, numCurrentSensors);

	return;
} // end of setupClarke_I() function

//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
void setupClarke_V(CLARKE_Handle handle, const uint_least8_t numVoltageSensors) {
	_iq alpha_sf, beta_sf;

	// initialize the Clarke transform module for voltage
	if (numVoltageSensors == 3) {
		alpha_sf = _IQ(MATH_ONE_OVER_THREE);
		beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
	} else {
		alpha_sf = _IQ(0.0);
		beta_sf = _IQ(0.0);
	}

	// In other words, the only acceptable number of voltage sensors is three.
	// set the parameters
	CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
	CLARKE_setNumSensors(handle, numVoltageSensors);

	return;
} // end of setupClarke_V() function

//! \brief     Update the global variables (gMotorVars).
//! \param[in] handle  The estimator (EST) handle
void updateGlobalVariables(EST_Handle handle, const uint_least8_t mtrNum) {
	// get the speed estimate
	//gMotorVars[mtrNum].Speed_krpm = EST_getSpeed_krpm(handle);
	gMotorVars[mtrNum].Speed_krpm = ENC_getSpeedRPM(encHandle[HAL_MTR2]) / 1000.0;
	// gegMotorVars[mtrNum].Speed_krpm = EST_getSpeed_krpm(handle);t the torque estimate
	{
		_iq Flux_pu = EST_getFlux_pu(handle);
		_iq Id_pu = PID_getFbackValue(pidHandle[mtrNum][1]);
		_iq Iq_pu = PID_getFbackValue(pidHandle[mtrNum][2]);
		_iq Ld_minus_Lq_pu = _IQ30toIQ(
				EST_getLs_d_pu(handle) - EST_getLs_q_pu(handle));

		// Reactance Torque
		_iq Torque_Flux_Iq_Nm = _IQmpy(_IQmpy(Flux_pu,Iq_pu),
				gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum]);

		// Reluctance Torque
		_iq Torque_Ls_Id_Iq_Nm = _IQmpy(
				_IQmpy(_IQmpy(Ld_minus_Lq_pu,Id_pu), Iq_pu),
				gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum]);

		// Total torque is sum of reactance torque and reluctance torque
		_iq Torque_Nm = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

		gMotorVars[mtrNum].Torque_Nm = Torque_Nm;
	}

	// get the magnetizing current
	gMotorVars[mtrNum].MagnCurr_A = EST_getIdRated(handle);

	// get the rotor resistance
	gMotorVars[mtrNum].Rr_Ohm = EST_getRr_Ohm(handle);

	// get the stator resistance
	gMotorVars[mtrNum].Rs_Ohm = EST_getRs_Ohm(handle);

	// get the stator inductance in the direct coordinate direction
	gMotorVars[mtrNum].Lsd_H = EST_getLs_d_H(handle);

	// get the stator inductance in the quadrature coordinate direction
	gMotorVars[mtrNum].Lsq_H = EST_getLs_q_H(handle);

	// get the flux in V/Hz in floating point
	gMotorVars[mtrNum].Flux_VpHz = EST_getFlux_VpHz(handle);

	// get the flux in Wb in fixed point
	gMotorVars[mtrNum].Flux_Wb = _IQmpy(EST_getFlux_pu(handle),
			gFlux_pu_to_Wb_sf[mtrNum]);

	// get the estimator state
	gMotorVars[mtrNum].EstState = EST_getState(handle);

	// Get the DC buss voltage
	gMotorVars[mtrNum].VdcBus_kV = _IQmpy(gAdcData[mtrNum].dcBus,
			_IQ(gUserParams[mtrNum].iqFullScaleVoltage_V / 1000.0));

	// read Vd and Vq vectors per units
	gMotorVars[mtrNum].Vd = gVdq_out_pu[mtrNum].value[0];
	gMotorVars[mtrNum].Vq = gVdq_out_pu[mtrNum].value[1];

	// calculate vector Vs in per units: (Vs = sqrt(Vd^2 + Vq^2))
	gMotorVars[mtrNum].Vs =
			_IQsqrt(
					_IQmpy(gMotorVars[mtrNum].Vd,gMotorVars[mtrNum].Vd) + _IQmpy(gMotorVars[mtrNum].Vq,gMotorVars[mtrNum].Vq));

	// read Id and Iq vectors in amps
	gMotorVars[mtrNum].Id_A = _IQmpy(gIdq_pu[mtrNum].value[0],
			_IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
	gMotorVars[mtrNum].Iq_A = _IQmpy(gIdq_pu[mtrNum].value[1],
			_IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));

	// calculate vector Is in amps:  (Is_A = sqrt(Id_A^2 + Iq_A^2))
	gMotorVars[mtrNum].Is_A =
			_IQsqrt(
					_IQmpy(gMotorVars[mtrNum].Id_A,gMotorVars[mtrNum].Id_A) + _IQmpy(gMotorVars[mtrNum].Iq_A,gMotorVars[mtrNum].Iq_A));

	return;
} // end of updateGlobalVariables() function

//@} //defgroup
// end of file
//
///* --COPYRIGHT--,BSD
// * Copyright (c) 2012, Texas Instruments Incorporated
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * *  Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// *
// * *  Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in the
// *    documentation and/or other materials provided with the distribution.
// *
// * *  Neither the name of Texas Instruments Incorporated nor the names of
// *    its contributors may be used to endorse or promote products derived
// *    from this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// * --/COPYRIGHT--*/
////! \file   solutions/instaspin_foc/src/proj_lab10d.c
////! \brief Dual motor project
////!
////! (C) Copyright 2011, Texas Instruments, Inc.
//
////! \defgroup PROJ_LAB10d PROJ_LAB10d
////@{
//
////! \defgroup PROJ_LAB10d_OVERVIEW Project Overview
////!
////! Dual Motor Sensorless Velocity Control
////!
//
//// **************************************************************************
//// the includes
//
//// system includes
//#include <math.h>
//#include "main.h"
//
//#ifdef FLASH
//#pragma CODE_SECTION(motor1_ISR,"ramfuncs");
//#pragma CODE_SECTION(motor2_ISR,"ramfuncs");
//#endif
//
//// Include header files used in the main function
//
//
//// **************************************************************************
//// the defines
//
//#define LED_BLINK_FREQ_Hz   5
//
//#define EST_Number1 		0
//#define CTRL_Number1		0
//
//
//// **************************************************************************
//// the globals
//CTRL_Handle ctrlHandle[2];
//
//uint_least16_t gCounter_updateGlobals[2] = {0, 0};
//uint_least8_t motorNum = 0;
//uint_least8_t estNumber[2] = {0, 1};
//uint_least8_t ctrlNumber[2] = {0, 1};
//
//bool Flag_Latch_softwareUpdate[2] = {true, true};
//
//
//#ifdef CSM_ENABLE
//#pragma DATA_SECTION(halHandle,"rom_accessed_data");
//#endif
//
//HAL_Handle halHandle;
//
//#ifdef CSM_ENABLE
//#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
//#endif
//
//USER_Params gUserParams[2];
//
//// the pwm voltage values for the three phases.
//HAL_PwmData_t gPwmData[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};
//
//// the voltage and current adc values for the CTRL controller and the FAST estimator.
//HAL_AdcData_t gAdcData[2];
//
//// the PWMDAC variable
//HAL_DacData_t gDacData;
//
//_iq gMaxCurrentSlope[2] = {_IQ(0.0), _IQ(0.0)};
//
//#ifdef FAST_ROM_V1p6
//CTRL_Obj *controller_obj[2];
//#else
//#ifdef CSM_ENABLE
//#pragma DATA_SECTION(ctrl,"rom_accessed_data");
//#endif
//CTRL_Obj ctrl[2];				//v1p7 format
//#endif
//
//uint16_t gLEDcnt[2] = {0, 0};
//
//// the variables to turn on and adjust InstaSPIN
//volatile MOTOR_Vars_t gMotorVars[2] = {MOTOR_Vars_INIT_Mtr1,MOTOR_Vars_INIT_Mtr2};
//
////
//volatile SYSTEM_Vars_t gSystemVars = SYSTEM_Vars_INIT;
//
//#ifdef FLASH
//// Used for running BackGround in flash, and ISR in RAM
//extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
//
//#ifdef CSM_ENABLE
//extern uint16_t *econst_start, *econst_end, *econst_ram_load;
//extern uint16_t *switch_start, *switch_end, *switch_ram_load;
//#endif
//#endif
//
//SVGENCURRENT_Obj svgencurrent[2];
//SVGENCURRENT_Handle svgencurrentHandle[2];
//
//// set the offset, default value of 1 microsecond
//int16_t gCmpOffset[2] = {(int16_t)(1.0 * USER_SYSTEM_FREQ_MHz_M1), (int16_t)(1.0 * USER_SYSTEM_FREQ_MHz_M2)};
//
//MATH_vec3 gIavg[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};
//uint16_t gIavg_shift[2] = {1, 1};
//MATH_vec3 gPwmData_prev[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};
//
//#ifdef DRV8301_SPI
//// Watch window interface to the 8301 SPI
//DRV_SPI_8301_Vars_t gDrvSpi8301Vars[2];
//#endif
//
//#ifdef DRV8305_SPI
//// Watch window interface to the 8305 SPI
//DRV_SPI_8305_Vars_t gDrvSpi8305Vars[2];
//#endif
//
//_iq gFlux_pu_to_Wb_sf[2];
//
//_iq gFlux_pu_to_VpHz_sf[2];
//
//_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf[2];
//
//_iq gTorque_Flux_Iq_pu_to_Nm_sf[2];
//
//HAL_Handle      halHandle;       	//!< the handle for the hardware abstraction
//                                 	 //!< layer for common CPU setup
//HAL_Obj         hal;             	//!< the hardware abstraction layer object
//
//#ifdef F2802xF
//#pragma DATA_SECTION(halHandleMtr,"rom_accessed_data");
//#endif
//
//HAL_Handle_mtr  halHandleMtr[2]; 	//!< the handle for the hardware abstraction
//                                 	 //!< layer specific to the motor board.
//HAL_Obj_mtr     halMtr[2];       	//!< the hardware abstraction layer object
//                                 	 //!< specific to the motor board.
//
//// define Flying Start (FS) variables
//FS_Obj fs[2];
//FS_Handle fsHandle[2];
//
//// define cpu_time object and handle for CPU usage time calculation
//CPU_TIME_Handle  cpu_timeHandle[2];
//CPU_TIME_Obj     cpu_time[2];
//
//// **************************************************************************
//// the functions
//
//void main(void)
//{
//  // Only used if running from FLASH
//  // Note that the variable FLASH is defined by the project
//  #ifdef FLASH
//  // Copy time critical code and Flash setup code to RAM
//  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
//  // symbols are created by the linker. Refer to the linker files.
//  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
//
//  #ifdef CSM_ENABLE
//  //copy .econst to unsecure RAM
//  if(*econst_end - *econst_start)
//    {
//      memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
//    }
//
//  //copy .switch ot unsecure RAM
//  if(*switch_end - *switch_start)
//    {
//      memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
//    }
//  #endif
//  #endif
//
//  // initialize the hardware abstraction layer
//  halHandle = HAL_init(&hal,sizeof(hal));
//
//  // initialize the individual motor hal files
//  halHandleMtr[HAL_MTR1] = HAL_init_mtr(&halMtr[HAL_MTR1], sizeof(halMtr[HAL_MTR1]), (HAL_MtrSelect_e)HAL_MTR1);
//
//  // initialize the individual motor hal files
//  halHandleMtr[HAL_MTR2] = HAL_init_mtr(&halMtr[HAL_MTR2], sizeof(halMtr[HAL_MTR2]), (HAL_MtrSelect_e)HAL_MTR2);
//
//  // initialize the controller
//#ifdef FAST_ROM_V1p6
////  ctrlHandle[HAL_MTR1] = CTRL_initCtrl(CTRL_Number1, EST_Number1);  		//v1p6 format (06xF and 06xM devices)
//  ctrlHandle[HAL_MTR1] = CTRL_initCtrl(ctrlNumber[HAL_MTR1], estNumber[HAL_MTR1]);  					//v1p6 format (06xF and 06xM devices)
//  ctrlHandle[HAL_MTR2] = CTRL_initCtrl(ctrlNumber[HAL_MTR2], estNumber[HAL_MTR2]);  					//v1p6 format (06xF and 06xM devices)
//#else
//  ctrlHandle[HAL_MTR1] = CTRL_initCtrl(estNumber[HAL_MTR1], &ctrl[HAL_MTR1], sizeof(ctrl[HAL_MTR1]));	//v1p7 format default
//  ctrlHandle[HAL_MTR2] = CTRL_initCtrl(estNumber[HAL_MTR2], &ctrl[HAL_MTR2], sizeof(ctrl[HAL_MTR2]));	//v1p7 format default
//#endif
//
//  // Initialize and setup the 100% SVM generator
//  svgencurrentHandle[HAL_MTR1] = SVGENCURRENT_init(&svgencurrent[HAL_MTR1],sizeof(svgencurrent[HAL_MTR1]));
//
//  // Initialize and setup the 100% SVM generator
//  svgencurrentHandle[HAL_MTR2] = SVGENCURRENT_init(&svgencurrent[HAL_MTR2],sizeof(svgencurrent[HAL_MTR2]));
//
//  // initialize the user parameters
//  // This function initializes all values of structure gUserParams with
//  // values defined in user.h. The values in gUserParams will be then used by
//  // the hardware abstraction layer (HAL) to configure peripherals such as
//  // PWM, ADC, interrupts, etc.
//  USER_setParamsMtr1(&gUserParams[HAL_MTR1]);
//  USER_setParamsMtr2(&gUserParams[HAL_MTR2]);
//
//  //HAL_setParams
//  // set the common hardware abstraction layer parameters
//  HAL_setParams(halHandle,&gUserParams[HAL_MTR1]);
//
//  #ifdef _SINGLE_ISR_EN_
//  // Setup each motor board to its specific setting
//  HAL_setParamsDualMtr(halHandleMtr[HAL_MTR1], halHandleMtr[HAL_MTR2], halHandle, &gUserParams[HAL_MTR1], &gUserParams[HAL_MTR2]);
//  #else
//  // Setup each motor board to its specific setting
//  HAL_setParamsMtr(halHandleMtr[HAL_MTR1],halHandle, &gUserParams[HAL_MTR1]);
//
//  // Setup each motor board to its specific setting
//  HAL_setParamsMtr(halHandleMtr[HAL_MTR2],halHandle, &gUserParams[HAL_MTR2]);
//  #endif
//
//  gMotorVars[0].Flag_enableUserParams = true;
////  gMotorVars[0].Flag_enableUserParams = false;
//
//  gMotorVars[1].Flag_enableUserParams = true;
////  gMotorVars[1].Flag_enableUserParams = false;
//
//  gMotorVars[0].Flag_enableSpeedCtrl = true;
//  gMotorVars[1].Flag_enableSpeedCtrl = true;
//
//  for(motorNum=HAL_MTR1;motorNum<=HAL_MTR2;motorNum++)
//  {
//	  // set the default controller parameters
//	  CTRL_setParams(ctrlHandle[motorNum], &gUserParams[motorNum]);
//
//	  // set the default controller parameters (Reset the control to re-identify the motor)
//      CTRL_setParams(ctrlHandle[motorNum],&gUserParams[motorNum]);
//
//	  {
//		// initialize the CPU usage module
//		cpu_timeHandle[motorNum] = CPU_TIME_init(&cpu_time[motorNum],sizeof(cpu_time[motorNum]));
//		CPU_TIME_setParams(cpu_timeHandle[motorNum], PWM_getPeriod(halHandleMtr[motorNum]->pwmHandle[0]));
//	  }
//
//	  // set overmodulation to maximum value
//	  gMotorVars[motorNum].OverModulation = _IQ(MATH_TWO_OVER_THREE);
//
//	  // setup faults
//	  HAL_setupFaults(halHandleMtr[motorNum]);
//
//	  // enable DC bus compensation
//	  CTRL_setFlag_enableDcBusComp(ctrlHandle[motorNum], true);
//
//	  // compute scaling factors for flux and torque calculations
//	  gFlux_pu_to_Wb_sf[motorNum] = USER_computeFlux_pu_to_Wb_sf(&gUserParams[motorNum]);
//	  gFlux_pu_to_VpHz_sf[motorNum] = USER_computeFlux_pu_to_VpHz_sf(&gUserParams[motorNum]);
//	  gTorque_Ls_Id_Iq_pu_to_Nm_sf[motorNum] = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(&gUserParams[motorNum]);
//	  gTorque_Flux_Iq_pu_to_Nm_sf[motorNum] = USER_computeTorque_Flux_Iq_pu_to_Nm_sf(&gUserParams[motorNum]);
//
//	  gMotorVars[motorNum].current_pu_to_A_sf = _IQ(gUserParams[motorNum].iqFullScaleCurrent_A);
//	  gMotorVars[motorNum].voltage_pu_to_kv_sf = _IQ(gUserParams[motorNum].iqFullScaleVoltage_V/(float_t)1000.0);
//	  gMotorVars[motorNum].current_A_to_pu_sf = _IQdiv(_IQ(1.0), gMotorVars[motorNum].current_pu_to_A_sf);
//
//	#ifdef DRV8301_SPI
//	  // turn on the DRV8301 if present
//	  HAL_enableDrv(halHandleMtr[motorNum]);
//	  // initialize the DRV8301 interface
//	  HAL_setupDrvSpi(halHandleMtr[motorNum],&gDrvSpi8301Vars[motorNum]);
//	#endif
//
//	#ifdef DRV8305_SPI
//	  // turn on the DRV8305 if present
//	  HAL_enableDrv(halHandleMtr[motorNum]);
//	  // initialize the DRV8305 interface
//	  HAL_setupDrvSpi(halHandleMtr[motorNum],&gDrvSpi8305Vars[motorNum]);
//	#endif
//
//	  gCounter_updateGlobals[motorNum] = 0;
//  }
//
//  // setup svgen current for Motor #1
//  {
//    float_t minWidth_microseconds_M1 = 2.0;
//    uint16_t minWidth_counts = (uint16_t)(minWidth_microseconds_M1 * USER_SYSTEM_FREQ_MHz);
//    float_t fdutyLimit = 0.5-(2.0*minWidth_microseconds_M1*USER_PWM_FREQ_kHz_M1*0.001);
//    _iq dutyLimit = _IQ(fdutyLimit);
//
//    SVGENCURRENT_setMinWidth(svgencurrentHandle[HAL_MTR1], minWidth_counts);
//    SVGENCURRENT_setIgnoreShunt(svgencurrentHandle[HAL_MTR1], use_all);
//    SVGENCURRENT_setMode(svgencurrentHandle[HAL_MTR1],all_phase_measurable);
//    SVGENCURRENT_setVlimit(svgencurrentHandle[HAL_MTR1],dutyLimit);
//  }
//
//  // setup svgen current for Motor #2
//  {
//    float_t minWidth_microseconds_M2 = 2.0;
//    uint16_t minWidth_counts = (uint16_t)(minWidth_microseconds_M2 * USER_SYSTEM_FREQ_MHz);
//    float_t fdutyLimit = 0.5-(2.0*minWidth_microseconds_M2*USER_PWM_FREQ_kHz_M2*0.001);
//    _iq dutyLimit = _IQ(fdutyLimit);
//
//    SVGENCURRENT_setMinWidth(svgencurrentHandle[HAL_MTR2], minWidth_counts);
//    SVGENCURRENT_setIgnoreShunt(svgencurrentHandle[HAL_MTR2], use_all);
//    SVGENCURRENT_setMode(svgencurrentHandle[HAL_MTR2],all_phase_measurable);
//    SVGENCURRENT_setVlimit(svgencurrentHandle[HAL_MTR2],dutyLimit);
//  }
//
//  {
//    CTRL_Version version;
//
//    // get the version number
//    CTRL_getVersion(ctrlHandle[HAL_MTR1],&version);
//
//    gMotorVars[HAL_MTR1].CtrlVersion = version;
//  }
//
//  // set DAC parameters
//  HAL_setDacParameters(halHandle, &gDacData);
//
//  // initialize the interrupt vector table
//  HAL_initIntVectorTable(halHandle);
//
//
//  // enable the ADC interrupts
//  HAL_enableAdcInts(halHandle);
//
//
//  // enable global interrupts
//  HAL_enableGlobalInts(halHandle);
//
//
//  // enable debug interrupts
//  HAL_enableDebugInt(halHandle);
//
//
//  // disable the PWM
//  HAL_disablePwm(halHandleMtr[HAL_MTR1]);
//  HAL_disablePwm(halHandleMtr[HAL_MTR2]);
//
//  gSystemVars.Flag_enableSystem = true;
//
//  // Below two lines code for Flash Testing, need to be commented
////  gSystemVars.Flag_enableSynControl = true;
////  gSystemVars.Flag_enableRun = true;
//
//  for(;;)
//  {
//    // Waiting for enable system flag to be set
//    // Motor 1 Flag_enableSys is the master control.
////    while(!(gMotorVars[HAL_MTR1].Flag_enableSys));
//    while(!(gSystemVars.Flag_enableSystem));
//
//    // Enable the Library internal PI.  Iq is referenced by the speed PI now
////    CTRL_setFlag_enableSpeedCtrl(ctrlHandle[HAL_MTR1], true);
//
//    // Enable the Library internal PI.  Iq is referenced by the speed PI now
////    CTRL_setFlag_enableSpeedCtrl(ctrlHandle[HAL_MTR2], true);
//
//    // loop while the enable system flag is true
//    // Motor 1 Flag_enableSys is the master control.
////    while(gMotorVars[HAL_MTR1].Flag_enableSys)
//    while(gSystemVars.Flag_enableSystem)
//    {
//		// toggle status LED
//		if(gLEDcnt[HAL_MTR1]++ > (uint_least32_t)(USER_ISR_FREQ_Hz_M1 / LED_BLINK_FREQ_Hz))
//		{
//			HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
//			gLEDcnt[HAL_MTR1] = 0;
//		}
//
//		// toggle status LED
//		if(gLEDcnt[HAL_MTR2]++ > (uint_least32_t)(USER_ISR_FREQ_Hz_M2 / LED_BLINK_FREQ_Hz))
//		{
//			HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED3);
//			gLEDcnt[HAL_MTR2] = 0;
//		}
//
//    	if(gSystemVars.Flag_enableSynControl == true)
//    	{
//    		gMotorVars[HAL_MTR1].Flag_Run_Identify = gSystemVars.Flag_enableRun;
//    		gMotorVars[HAL_MTR1].SpeedRef_krpm = gSystemVars.SpeedSet_krpm;
//    		gMotorVars[HAL_MTR1].MaxAccel_krpmps = gSystemVars.MaxAccelSet_krpmps;
//
//    		gMotorVars[HAL_MTR2].Flag_Run_Identify = gSystemVars.Flag_enableRun;
//    		gMotorVars[HAL_MTR2].SpeedRef_krpm = gSystemVars.SpeedSet_krpm;
//    		gMotorVars[HAL_MTR2].MaxAccel_krpmps = gSystemVars.MaxAccelSet_krpmps;
//    	}
//
//        for(motorNum=HAL_MTR1;motorNum<=HAL_MTR2;motorNum++)
//        {
//        	CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle[motorNum];
//
//        	// increment counters
//        	gCounter_updateGlobals[motorNum]++;
//
//        	if(CTRL_isError(ctrlHandle[motorNum]))
//        	{
//        		// set the enable controller flag to false
//        		CTRL_setFlag_enableCtrl(ctrlHandle[motorNum],false);
//
//        		// set the enable system flag to false
//        		gMotorVars[motorNum].Flag_enableSys = false;
//
//        		// disable the PWM
//        		HAL_disablePwm(halHandleMtr[motorNum]);
//        	}
//        	else
//        	{
//        		// update the controller state
//        		bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle[motorNum]);
//
//        		// enable or disable the control
//        		CTRL_setFlag_enableCtrl(ctrlHandle[motorNum], gMotorVars[motorNum].Flag_Run_Identify);
//
//        		if(flag_ctrlStateChanged)
//        		{
//        			CTRL_State_e ctrlState = CTRL_getState(ctrlHandle[motorNum]);
//                    EST_State_e estState = EST_getState(obj->estHandle);
//
//        			if(ctrlState == CTRL_State_OffLine)
//        			{
//        				// enable the PWM
//        				HAL_enablePwm(halHandleMtr[motorNum]);
//        			}
//        			else if(ctrlState == CTRL_State_OnLine)
//        			{
////        				if(gMotorVars[motorNum].Flag_enableOffsetcalc == true)
////        				{
////        					// update the ADC bias values
////        					HAL_updateAdcBias(halHandleMtr[motorNum]);
////        				}
////        				else
////        				{
////        					// set the current bias
////        					HAL_setBias(halHandleMtr[motorNum],HAL_SensorType_Current,0,_IQ(I_A_offset));
////        					HAL_setBias(halHandleMtr[motorNum],HAL_SensorType_Current,1,_IQ(I_B_offset));
////        					HAL_setBias(halHandleMtr[motorNum],HAL_SensorType_Current,2,_IQ(I_C_offset));
////
////        					// set the voltage bias
////        					HAL_setBias(halHandleMtr[motorNum],HAL_SensorType_Voltage,0,_IQ(V_A_offset));
////        					HAL_setBias(halHandleMtr[motorNum],HAL_SensorType_Voltage,1,_IQ(V_B_offset));
////        					HAL_setBias(halHandleMtr[motorNum],HAL_SensorType_Voltage,2,_IQ(V_C_offset));
////        				}
//
//                        if((estState < EST_State_LockRotor) || (estState > EST_State_MotorIdentified))
//                        {
//                            // update the ADC bias values
//                            HAL_updateAdcBias(halHandleMtr[motorNum]);
//                        }
//
//						// Return the bias value for currents
//						gMotorVars[motorNum].I_bias.value[0] = HAL_getBias(halHandleMtr[motorNum],HAL_SensorType_Current,0);
//						gMotorVars[motorNum].I_bias.value[1] = HAL_getBias(halHandleMtr[motorNum],HAL_SensorType_Current,1);
//						gMotorVars[motorNum].I_bias.value[2] = HAL_getBias(halHandleMtr[motorNum],HAL_SensorType_Current,2);
//
//						// Return the bias value for voltages
//						gMotorVars[motorNum].V_bias.value[0] = HAL_getBias(halHandleMtr[motorNum],HAL_SensorType_Voltage,0);
//						gMotorVars[motorNum].V_bias.value[1] = HAL_getBias(halHandleMtr[motorNum],HAL_SensorType_Voltage,1);
//						gMotorVars[motorNum].V_bias.value[2] = HAL_getBias(halHandleMtr[motorNum],HAL_SensorType_Voltage,2);
//
//						// enable the PWM
//						HAL_enablePwm(halHandleMtr[motorNum]);
//        			}
//        			else if(ctrlState == CTRL_State_Idle)
//        			{
//        				// disable the PWM
//        				HAL_disablePwm(halHandleMtr[motorNum]);
//        				gMotorVars[motorNum].Flag_Run_Identify = false;
//        			}		// ctrlState=?
//
//        			if((CTRL_getFlag_enableUserMotorParams(ctrlHandle[motorNum]) == true) &&
//        												  (ctrlState > CTRL_State_Idle) &&
//												  (gMotorVars[motorNum].CtrlVersion.minor == 6))
//        			{
//        				// call this function to fix 1p6
//        				USER_softwareUpdate1p6(ctrlHandle[motorNum], &gUserParams[motorNum]);
//        			}
//
//        		}	// flag_ctrlStateChanged=?
//        	}	// CTRL_isError=?
//
//
//        	if(EST_isMotorIdentified(obj->estHandle))
//        	{
//        		_iq Id_squared_pu = _IQmpy(CTRL_getId_ref_pu(ctrlHandle[motorNum]),CTRL_getId_ref_pu(ctrlHandle[motorNum]));
//
//				//Set the maximum current controller output for the Iq and Id current controllers to enable
//				//over-modulation.
//				//An input into the SVM above 1/SQRT(3) = 0.5774 is in the over-modulation region.  An input of 0.5774 is where
//				//the crest of the sinewave touches the 100% duty cycle.  At an input of 2/3, the SVM generator
//				//produces a trapezoidal waveform touching every corner of the hexagon
//				CTRL_setMaxVsMag_pu(ctrlHandle[motorNum],gMotorVars[motorNum].OverModulation);
//
//				// set the current ramp
//				EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope[motorNum]);
//
//				gMotorVars[motorNum].Flag_MotorIdentified = true;
//
//				// set the speed reference
//				CTRL_setSpd_ref_krpm(ctrlHandle[motorNum],gMotorVars[motorNum].SpeedRef_krpm);
//
//				// set the speed acceleration
////				CTRL_setMaxAccel_pu(ctrlHandle[motorNum],_IQmpy(MAX_ACCEL_KRPMPS_SF_M1,gMotorVars[motorNum].MaxAccel_krpmps));
//				CTRL_setMaxAccel_pu(ctrlHandle[motorNum],_IQmpy(MAX_ACCEL_KRPMPS_SF_M1,gMotorVars[motorNum].MaxAccel_krpmps));
//
//				// set the Id reference
////				CTRL_setId_ref_pu(ctrlHandle[motorNum], _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));
//				CTRL_setId_ref_pu(ctrlHandle[motorNum], _IQmpy(gMotorVars[motorNum].IdRef_A, gMotorVars[motorNum].current_A_to_pu_sf));
//
//				if(Flag_Latch_softwareUpdate[motorNum])
//				{
//				  Flag_Latch_softwareUpdate[motorNum] = false;
//
//				  USER_calcPIgains(ctrlHandle[motorNum], &gUserParams[motorNum]);
//
//				  // initialize the watch window kp and ki current values with pre-calculated values
//				  gMotorVars[motorNum].Kp_Idq = CTRL_getKp(ctrlHandle[motorNum],CTRL_Type_PID_Id);
//				  gMotorVars[motorNum].Ki_Idq = CTRL_getKi(ctrlHandle[motorNum],CTRL_Type_PID_Id);
//
//				  // initialize the watch window kp and ki current values with pre-calculated values
//				  gMotorVars[motorNum].Kp_spd = CTRL_getKp(ctrlHandle[motorNum],CTRL_Type_PID_spd);
//				  gMotorVars[motorNum].Ki_spd = CTRL_getKi(ctrlHandle[motorNum],CTRL_Type_PID_spd);
//				}
//        	}
//        	else
//        	{
//        		Flag_Latch_softwareUpdate[motorNum] = true;
//
//        		// initialize the watch window kp and ki values with pre-calculated values
////        		gMotorVars[motorNum].Kp_spd = CTRL_getKp(ctrlHandle[motorNum],CTRL_Type_PID_spd);
////        		gMotorVars[motorNum].Ki_spd = CTRL_getKi(ctrlHandle[motorNum],CTRL_Type_PID_spd);
//
//        		// the estimator sets the maximum current slope during identification
//        		gMaxCurrentSlope[motorNum] = EST_getMaxCurrentSlope_pu(obj->estHandle);
//        	}
//
//
//        	// when appropriate, update the global variables
//        	if(gCounter_updateGlobals[motorNum] >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
//        	{
//        		// reset the counter
//        		gCounter_updateGlobals[motorNum] = 0;
//
//        		updateGlobalVariables_motor(ctrlHandle[motorNum], motorNum);
//        	}
//
//            // recalculate Kp and Ki gains to fix the R/L limitation of 2000.0, and Kp limit to 0.11
//            recalcKpKi(ctrlHandle[motorNum], motorNum);
//
//        	// update Kp and Ki gains
//        	updateKpKiGains(ctrlHandle[motorNum], motorNum);
//
//        	// enable/disable the forced angle
//        	EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars[motorNum].Flag_enableForceAngle);
//
//        	// enable or disable power warp
//        	CTRL_setFlag_enablePowerWarp(ctrlHandle[motorNum],gMotorVars[motorNum].Flag_enablePowerWarp);
//
//			#ifdef DRV8301_SPI
//			HAL_writeDrvData(halHandleMtr[motorNum],&gDrvSpi8301Vars[motorNum]);
//
//			HAL_readDrvData(halHandleMtr[motorNum],&gDrvSpi8301Vars[motorNum]);
//			#endif
//
//			#ifdef DRV8305_SPI
//			HAL_writeDrvData(halHandleMtr[motorNum],&gDrvSpi8305Vars[motorNum]);
//
//			HAL_readDrvData(halHandleMtr[motorNum],&gDrvSpi8305Vars[motorNum]);
//			#endif
//        }
//
//      } // end of while(gFlag_enableSys) loop
//
//      // disable the PWM
//      HAL_disablePwm(halHandleMtr[HAL_MTR1]);
//      HAL_disablePwm(halHandleMtr[HAL_MTR2]);
//
//      gMotorVars[HAL_MTR1].Flag_Run_Identify = false;
//      gMotorVars[HAL_MTR2].Flag_Run_Identify = false;
//
//      // set the default controller parameters (Reset the control to re-identify the motor)
//      CTRL_setParams(ctrlHandle[HAL_MTR1],&gUserParams[HAL_MTR1]);
//      CTRL_setParams(ctrlHandle[HAL_MTR2],&gUserParams[HAL_MTR2]);
//   } // end of for(;;) loop
//
//} // end of main() function
//
//#ifdef _SINGLE_ISR_EN_
//// motor_ISR
//interrupt void motor_ISR(void)
//{
//  // read the timer 1 value and update the CPU usage module
//  uint32_t timer1Cnt = HAL_readTimerCnt(halHandle,2);
//  CPU_TIME_updateCnts(cpu_timeHandle[HAL_MTR1],timer1Cnt);
//
//  #ifdef _ENABLE_OVM_
//  SVGENCURRENT_IgnoreShunt_e ignoreShuntThisCycle_M1 = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle[HAL_MTR1]);
//
//  // acknowledge the ADC interrupt
//  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);
//
//  // convert the ADC data
//  HAL_readAdcData(halHandle,halHandleMtr[HAL_MTR1],&gAdcData[HAL_MTR1]);
//
//  // run the current reconstruction algorithm
//  SVGENCURRENT_RunRegenCurrent(svgencurrentHandle[HAL_MTR1], (MATH_vec3 *)(gAdcData[HAL_MTR1].I.value));
//
//  gIavg[HAL_MTR1].value[0] += (gAdcData[HAL_MTR1].I.value[0] - gIavg[HAL_MTR1].value[0])>>gIavg_shift[HAL_MTR1];
//  gIavg[HAL_MTR1].value[1] += (gAdcData[HAL_MTR1].I.value[1] - gIavg[HAL_MTR1].value[1])>>gIavg_shift[HAL_MTR1];
//  gIavg[HAL_MTR1].value[2] += (gAdcData[HAL_MTR1].I.value[2] - gIavg[HAL_MTR1].value[2])>>gIavg_shift[HAL_MTR1];
//
//  if(ignoreShuntThisCycle_M1 == ignore_ab)
//  {
//    gAdcData[HAL_MTR1].I.value[0] = gIavg[HAL_MTR1].value[0];
//    gAdcData[HAL_MTR1].I.value[1] = gIavg[HAL_MTR1].value[1];
//  }
//  else if(ignoreShuntThisCycle_M1 == ignore_ac)
//  {
//    gAdcData[HAL_MTR1].I.value[0] = gIavg[HAL_MTR1].value[0];
//    gAdcData[HAL_MTR1].I.value[2] = gIavg[HAL_MTR1].value[2];
//  }
//  else if(ignoreShuntThisCycle_M1 == ignore_bc)
//  {
//    gAdcData[HAL_MTR1].I.value[1] = gIavg[HAL_MTR1].value[1];
//    gAdcData[HAL_MTR1].I.value[2] = gIavg[HAL_MTR1].value[2];
//  }
//  #else
//  // acknowledge the ADC interrupt
//  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);
//
//  // convert the ADC data
//  HAL_readDualAdcDataWithOffsets(halHandle,halHandleMtr[HAL_MTR1],halHandleMtr[HAL_MTR2],&gAdcData[HAL_MTR1],&gAdcData[HAL_MTR2]);
//  #endif
//
//  // run the flying start
//  FS_run(ctrlHandle[HAL_MTR1], fsHandle[HAL_MTR1]);
//
//  // run the controller
//  CTRL_run(ctrlHandle[HAL_MTR1],halHandleMtr[HAL_MTR1],&gAdcData[HAL_MTR1],&gPwmData[HAL_MTR1]);
//
//  // write the PWM compare values
//  HAL_writePwmData(halHandleMtr[HAL_MTR1],&gPwmData[HAL_MTR1]);
//
//  #ifdef _ENABLE_OVM_
//  // run the current ignore algorithm
//  {
//    uint16_t cmp1_M1 = HAL_readPwmCmpA(halHandleMtr[HAL_MTR1],PWM_Number_1);
//    uint16_t cmp2_M1 = HAL_readPwmCmpA(halHandleMtr[HAL_MTR1],PWM_Number_2);
//    uint16_t cmp3_M1 = HAL_readPwmCmpA(halHandleMtr[HAL_MTR1],PWM_Number_3);
//
//    uint16_t cmpM1_M1 = HAL_readPwmCmpAM(halHandleMtr[HAL_MTR1],PWM_Number_1);
//    uint16_t cmpM2_M1 = HAL_readPwmCmpAM(halHandleMtr[HAL_MTR1],PWM_Number_2);
//    uint16_t cmpM3_M1 = HAL_readPwmCmpAM(halHandleMtr[HAL_MTR1],PWM_Number_3);
//
//    // run the current ignore algorithm
//    SVGENCURRENT_RunIgnoreShunt(svgencurrentHandle[HAL_MTR1],cmp1_M1,cmp2_M1,cmp3_M1,cmpM1_M1,cmpM2_M1,cmpM3_M1);
//  }
//
//  {
//    int16_t minwidth_M1 = SVGENCURRENT_getMinWidth(svgencurrentHandle[HAL_MTR1]);
//    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle_M1 = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle[HAL_MTR1]);
//
//    // Set trigger point in the middle of the low side pulse
//    HAL_setTrigger(halHandleMtr[HAL_MTR1], ignoreShuntNextCycle_M1, minwidth_M1, gCmpOffset[HAL_MTR1]);
//  }
//  #endif
//
//  #ifdef _ENABLE_OVM_
//  SVGENCURRENT_IgnoreShunt_e ignoreShuntThisCycle_M2 = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle[HAL_MTR2]);
//
//  // convert the ADC data
//  HAL_readAdcData(halHandle,halHandleMtr[HAL_MTR2],&gAdcData[HAL_MTR2]);
//
//  // run the current reconstruction algorithm
//  SVGENCURRENT_RunRegenCurrent(svgencurrentHandle[HAL_MTR2], (MATH_vec3 *)(gAdcData[HAL_MTR2].I.value));
//
//  gIavg[HAL_MTR2].value[0] += (gAdcData[HAL_MTR2].I.value[0] - gIavg[HAL_MTR2].value[0])>>gIavg_shift[HAL_MTR2];
//  gIavg[HAL_MTR2].value[1] += (gAdcData[HAL_MTR2].I.value[1] - gIavg[HAL_MTR2].value[1])>>gIavg_shift[HAL_MTR2];
//  gIavg[HAL_MTR2].value[2] += (gAdcData[HAL_MTR2].I.value[2] - gIavg[HAL_MTR2].value[2])>>gIavg_shift[HAL_MTR2];
//
//  if(ignoreShuntThisCycle_M2 == ignore_ab)
//  {
//	gAdcData[HAL_MTR2].I.value[0] = gIavg[HAL_MTR2].value[0];
//	gAdcData[HAL_MTR2].I.value[1] = gIavg[HAL_MTR2].value[1];
//  }
//  else if(ignoreShuntThisCycle_M2 == ignore_ac)
//  {
//	gAdcData[HAL_MTR2].I.value[0] = gIavg[HAL_MTR2].value[0];
//	gAdcData[HAL_MTR2].I.value[2] = gIavg[HAL_MTR2].value[2];
//  }
//  else if(ignoreShuntThisCycle_M2 == ignore_bc)
//  {
//	gAdcData[HAL_MTR2].I.value[1] = gIavg[HAL_MTR2].value[1];
//	gAdcData[HAL_MTR2].I.value[2] = gIavg[HAL_MTR2].value[2];
//  }
//  #endif
//
//  // run the flying start
//  FS_run(ctrlHandle[HAL_MTR2], fsHandle[HAL_MTR2]);
//
//  // run the controller
//  CTRL_run(ctrlHandle[HAL_MTR2],halHandleMtr[HAL_MTR2],&gAdcData[HAL_MTR2],&gPwmData[HAL_MTR2]);
//
//  // write the PWM compare values
//  HAL_writePwmData(halHandleMtr[HAL_MTR2],&gPwmData[HAL_MTR2]);
//
//  #ifdef _ENABLE_OVM_
//  // run the current ignore algorithm
//  {
//	uint16_t cmp1_M2 = HAL_readPwmCmpA(halHandleMtr[HAL_MTR2],PWM_Number_1);
//	uint16_t cmp2_M2 = HAL_readPwmCmpA(halHandleMtr[HAL_MTR2],PWM_Number_2);
//	uint16_t cmp3_M2 = HAL_readPwmCmpA(halHandleMtr[HAL_MTR2],PWM_Number_3);
//
//	uint16_t cmpM1_M2 = HAL_readPwmCmpAM(halHandleMtr[HAL_MTR2],PWM_Number_1);
//	uint16_t cmpM2_M2 = HAL_readPwmCmpAM(halHandleMtr[HAL_MTR2],PWM_Number_2);
//	uint16_t cmpM3_M2 = HAL_readPwmCmpAM(halHandleMtr[HAL_MTR2],PWM_Number_3);
//
//	// run the current ignore algorithm
//	SVGENCURRENT_RunIgnoreShunt(svgencurrentHandle[HAL_MTR2],cmp1_M2,cmp2_M2,cmp3_M2,cmpM1_M2,cmpM2_M2,cmpM3_M2);
//  }
//
//  {
//	int16_t minwidth_M2 = SVGENCURRENT_getMinWidth(svgencurrentHandle[HAL_MTR2]);
//	SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle_M2 = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle[HAL_MTR2]);
//
//	// Set trigger point in the middle of the low side pulse
//	HAL_setTrigger(halHandleMtr[HAL_MTR2], ignoreShuntNextCycle_M2, minwidth_M2, gCmpOffset[HAL_MTR2]);
//  }
//  #endif
//
//  // setup the controller
//  CTRL_setup(ctrlHandle[HAL_MTR1]);
//  CTRL_setup(ctrlHandle[HAL_MTR2]);
//
//  // read the timer 1 value and update the CPU usage module
//  timer1Cnt = HAL_readTimerCnt(halHandle,2);
//  CPU_TIME_run(cpu_timeHandle[HAL_MTR1],timer1Cnt);
//
//  return;
//} // end of motor1_ISR() function
//#else
//// motor1_ISR
//interrupt void motor1_ISR(void)
//{
//  // read the timer 1 value and update the CPU usage module
//  uint32_t timer1Cnt = HAL_readTimerCnt(halHandle,2);
//  CPU_TIME_updateCnts(cpu_timeHandle[HAL_MTR1],timer1Cnt);
//
//  SVGENCURRENT_MeasureShunt_e measurableShuntThisCycle_M1 = SVGENCURRENT_getMode(svgencurrentHandle[HAL_MTR1]);
//
//  // acknowledge the ADC interrupt
//  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);
//
//
//  // convert the ADC data
//  HAL_readAdcData(halHandle,halHandleMtr[HAL_MTR1],&gAdcData[HAL_MTR1]);
//
//
//  // run the current reconstruction algorithm
//  SVGENCURRENT_RunRegenCurrent(svgencurrentHandle[HAL_MTR1], (MATH_vec3 *)(gAdcData[HAL_MTR1].I.value));
//
//
//  gIavg[HAL_MTR1].value[0] += (gAdcData[HAL_MTR1].I.value[0] - gIavg[HAL_MTR1].value[0])>>gIavg_shift[HAL_MTR1];
//  gIavg[HAL_MTR1].value[1] += (gAdcData[HAL_MTR1].I.value[1] - gIavg[HAL_MTR1].value[1])>>gIavg_shift[HAL_MTR1];
//  gIavg[HAL_MTR1].value[2] += (gAdcData[HAL_MTR1].I.value[2] - gIavg[HAL_MTR1].value[2])>>gIavg_shift[HAL_MTR1];
//
//  if(measurableShuntThisCycle_M1 > two_phase_measurable)
//  {
//	  gAdcData[HAL_MTR1].I.value[0] = gIavg[HAL_MTR1].value[0];
//	  gAdcData[HAL_MTR1].I.value[1] = gIavg[HAL_MTR1].value[1];
//	  gAdcData[HAL_MTR1].I.value[2] = gIavg[HAL_MTR1].value[2];
//  }
//
//  // run the controller
//  CTRL_run(ctrlHandle[HAL_MTR1],halHandleMtr[HAL_MTR1],&gAdcData[HAL_MTR1],&gPwmData[HAL_MTR1]);
//
//
//  // run the PWM compensation and current ignore algorithm
//  SVGENCURRENT_compPwmData(svgencurrentHandle[HAL_MTR1],&(gPwmData[HAL_MTR1].Tabc),&gPwmData_prev[HAL_MTR1]);
//
//
//  // write the PWM compare values
//  HAL_writePwmData(halHandleMtr[HAL_MTR1],&gPwmData[HAL_MTR1]);
//
//
//  {
//    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle_M1 = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle[HAL_MTR1]);
//    SVGENCURRENT_VmidShunt_e midVolShunt_M1 = SVGENCURRENT_getVmid(svgencurrentHandle[HAL_MTR1]);
//
//    // Set trigger point in the middle of the low side pulse
//    HAL_setTrigger(halHandleMtr[HAL_MTR1],ignoreShuntNextCycle_M1,midVolShunt_M1);
//  }
//
//
//  // setup the controller
//  CTRL_setup(ctrlHandle[HAL_MTR1]);
//
//
//  // get the estimator angle and frequency values
//  gMotorVars[HAL_MTR1].angle_est_pu = EST_getAngle_pu(ctrlHandle[HAL_MTR1]->estHandle);
//  gMotorVars[HAL_MTR1].speed_est_pu = EST_getFm_pu(ctrlHandle[HAL_MTR1]->estHandle);
//
//  gDacData.value[0] = gMotorVars[HAL_MTR1].angle_est_pu;
//  gDacData.value[1] = gMotorVars[HAL_MTR1].speed_est_pu;
//  gDacData.value[2] = gMotorVars[HAL_MTR2].angle_est_pu;
//  gDacData.value[3] = gMotorVars[HAL_MTR2].speed_est_pu;
//
//  HAL_writeDacData(halHandle,&gDacData);
//
//
//  // read the timer 1 value and update the CPU usage module
//  timer1Cnt = HAL_readTimerCnt(halHandle,2);
//  CPU_TIME_run(cpu_timeHandle[HAL_MTR1],timer1Cnt);
//
//  return;
//} // end of motor1_ISR() function
//
//
////motor2_ISR
//interrupt void motor2_ISR(void)
//{
//  // read the timer 1 value and update the CPU usage module
//  uint32_t timer1Cnt = HAL_readTimerCnt(halHandle,2);
//  CPU_TIME_updateCnts(cpu_timeHandle[HAL_MTR2],timer1Cnt);
//  SVGENCURRENT_MeasureShunt_e measurableShuntThisCycle_M2 = SVGENCURRENT_getMode(svgencurrentHandle[HAL_MTR2]);
//
//  // acknowledge the ADC interrupt
//  HAL_acqAdcInt(halHandle,ADC_IntNumber_2);
//
//  // convert the ADC data
//  HAL_readAdcData(halHandle,halHandleMtr[HAL_MTR2],&gAdcData[HAL_MTR2]);
//
//  // run the current reconstruction algorithm
//  SVGENCURRENT_RunRegenCurrent(svgencurrentHandle[HAL_MTR2], (MATH_vec3 *)(gAdcData[HAL_MTR2].I.value));
//
//
//  gIavg[HAL_MTR2].value[0] += (gAdcData[HAL_MTR2].I.value[0] - gIavg[HAL_MTR2].value[0])>>gIavg_shift[HAL_MTR2];
//  gIavg[HAL_MTR2].value[1] += (gAdcData[HAL_MTR2].I.value[1] - gIavg[HAL_MTR2].value[1])>>gIavg_shift[HAL_MTR2];
//  gIavg[HAL_MTR2].value[2] += (gAdcData[HAL_MTR2].I.value[2] - gIavg[HAL_MTR2].value[2])>>gIavg_shift[HAL_MTR2];
//
//  if(measurableShuntThisCycle_M2 > two_phase_measurable)
//  {
//	  gAdcData[HAL_MTR2].I.value[0] = gIavg[HAL_MTR2].value[0];
//	  gAdcData[HAL_MTR2].I.value[1] = gIavg[HAL_MTR2].value[1];
//	  gAdcData[HAL_MTR2].I.value[2] = gIavg[HAL_MTR2].value[2];
//  }
//
//
//  // run the controller
//  CTRL_run(ctrlHandle[HAL_MTR2],halHandleMtr[HAL_MTR2],&gAdcData[HAL_MTR2],&gPwmData[HAL_MTR2]);
//
//
//  // run the PWM compensation and current ignore algorithm
//  SVGENCURRENT_compPwmData(svgencurrentHandle[HAL_MTR2],&(gPwmData[HAL_MTR2].Tabc),&gPwmData_prev[HAL_MTR2]);
//
//
//  // write the PWM compare values
//  HAL_writePwmData(halHandleMtr[HAL_MTR2],&gPwmData[HAL_MTR2]);
//
//
//  {
//    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle_M2 = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle[HAL_MTR2]);
//    SVGENCURRENT_VmidShunt_e midVolShunt_M2 = SVGENCURRENT_getVmid(svgencurrentHandle[HAL_MTR2]);
//
//    // Set trigger point in the middle of the low side pulse
//    HAL_setTrigger(halHandleMtr[HAL_MTR2],ignoreShuntNextCycle_M2,midVolShunt_M2);
//  }
//
//
//  // setup the controller
//  CTRL_setup(ctrlHandle[HAL_MTR2]);
//
//
//  // get the estimator angle and frequency values
//  gMotorVars[HAL_MTR2].angle_est_pu = EST_getAngle_pu(ctrlHandle[HAL_MTR2]->estHandle);
//  gMotorVars[HAL_MTR2].speed_est_pu = EST_getFm_pu(ctrlHandle[HAL_MTR2]->estHandle);
//
//  // read the timer 1 value and update the CPU usage module
//  timer1Cnt = HAL_readTimerCnt(halHandle,2);
//  CPU_TIME_run(cpu_timeHandle[HAL_MTR2],timer1Cnt);
//
//  return;
//} // end of mainISR() function
//#endif
//
//void updateGlobalVariables_motor(CTRL_Handle handle, const uint_least8_t mtrNum)
//{
//  CTRL_Obj *obj = (CTRL_Obj *)handle;
//
//  // get the speed estimate
//  gMotorVars[mtrNum].Speed_krpm = EST_getSpeed_krpm(obj->estHandle);
//
//  // get the real time speed reference coming out of the speed trajectory generator
//  gMotorVars[mtrNum].SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));
//
//  // get the torque estimate
////  gMotorVars[mtrNum].Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum], gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum]);
//
//  // get the magnetizing current
//  gMotorVars[mtrNum].MagnCurr_A = EST_getIdRated(obj->estHandle);
//
//  // get the rotor resistance
//  gMotorVars[mtrNum].Rr_Ohm = EST_getRr_Ohm(obj->estHandle);
//
//  // get the stator resistance
//  gMotorVars[mtrNum].Rs_Ohm = EST_getRs_Ohm(obj->estHandle);
//
//  // get the stator inductance in the direct coordinate direction
//  gMotorVars[mtrNum].Lsd_H = EST_getLs_d_H(obj->estHandle);
//
//  // get the stator inductance in the quadrature coordinate direction
//  gMotorVars[mtrNum].Lsq_H = EST_getLs_q_H(obj->estHandle);
//
//  // get the flux in V/Hz in floating point
//  gMotorVars[mtrNum].Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);
//
//  // get the flux in Wb in fixed point
//  gMotorVars[mtrNum].Flux_Wb = _IQmpy(EST_getFlux_pu(obj->estHandle),gFlux_pu_to_Wb_sf[mtrNum]);
//
//  // get the controller state
//  gMotorVars[mtrNum].CtrlState = CTRL_getState(handle);
//
//  // get the estimator state
//  gMotorVars[mtrNum].EstState = EST_getState(obj->estHandle);
//
//  // read Vd and Vq vectors per units
//  gMotorVars[mtrNum].Vd = CTRL_getVd_out_pu(handle);
//  gMotorVars[mtrNum].Vq = CTRL_getVq_out_pu(handle);
//
//  // calculate vector Vs in per units
//  gMotorVars[mtrNum].Vs = _IQsqrt(_IQmpy(gMotorVars[mtrNum].Vd, gMotorVars[mtrNum].Vd) + _IQmpy(gMotorVars[mtrNum].Vq, gMotorVars[mtrNum].Vq));
//
//  // read Id and Iq vectors in amps
////  gMotorVars[mtrNum].Id_A = _IQmpy(CTRL_getId_in_pu(handle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
////  gMotorVars[mtrNum].Iq_A = _IQmpy(CTRL_getIq_in_pu(handle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
//  gMotorVars[mtrNum].Id_A = _IQmpy(CTRL_getId_in_pu(handle), gMotorVars[mtrNum].current_pu_to_A_sf);
//  gMotorVars[mtrNum].Iq_A = _IQmpy(CTRL_getIq_in_pu(handle), gMotorVars[mtrNum].current_pu_to_A_sf);
//
//  // calculate vector Is in amps
//  gMotorVars[mtrNum].Is_A = _IQsqrt(_IQmpy(gMotorVars[mtrNum].Id_A, gMotorVars[mtrNum].Id_A) + _IQmpy(gMotorVars[mtrNum].Iq_A, gMotorVars[mtrNum].Iq_A));
//
//  // Get the DC buss voltage
////  gMotorVars[mtrNum].VdcBus_kV = _IQmpy(gAdcData[mtrNum].dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));
//  gMotorVars[mtrNum].VdcBus_kV = _IQmpy(gAdcData[mtrNum].dcBus, gMotorVars[mtrNum].voltage_pu_to_kv_sf);
//
//  // enable/disable the use of motor parameters being loaded from user.h
//  CTRL_setFlag_enableUserMotorParams(ctrlHandle[mtrNum],gMotorVars[mtrNum].Flag_enableUserParams);
//
//  // Enable the Library internal PI.  Iq is referenced by the speed PI now
//  CTRL_setFlag_enableSpeedCtrl(ctrlHandle[mtrNum], gMotorVars[mtrNum].Flag_enableSpeedCtrl);
//
//  // enable/disable automatic calculation of bias values
//  CTRL_setFlag_enableOffset(ctrlHandle[mtrNum],gMotorVars[mtrNum].Flag_enableOffsetcalc);
//
//  // enable/disable dc bus voltage compensation
//  CTRL_setFlag_enableDcBusComp(ctrlHandle[mtrNum], gMotorVars[mtrNum].Flag_enableDcBusComp);
//
//  // enable/disable Rs recalibration during motor startup
//  EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars[mtrNum].Flag_enableRsRecalc);
//
//  return;
//} // end of updateGlobalVariables_motor() function
//
//
//void updateKpKiGains(CTRL_Handle handle, const uint_least8_t mtrNum)
//{
//  if((gMotorVars[mtrNum].CtrlState == CTRL_State_OnLine) && (gMotorVars[mtrNum].Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate[mtrNum] == false))
//    {
//      // set the kp and ki speed values from the watch window
//      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars[mtrNum].Kp_spd);
//      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars[mtrNum].Ki_spd);
//
//      // set the kp and ki current values for Id and Iq from the watch window
//      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars[mtrNum].Kp_Idq);
//      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars[mtrNum].Ki_Idq);
//      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars[mtrNum].Kp_Idq);
//      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars[mtrNum].Ki_Idq);
//	}
//
//  return;
//} // end of updateKpKiGains() function
//
//void recalcKpKi(CTRL_Handle handle, const uint_least8_t mtrNum)
//{
//  CTRL_Obj *obj = (CTRL_Obj *)handle;
//  EST_State_e EstState = EST_getState(obj->estHandle);
//
//  if((EST_isMotorIdentified(obj->estHandle) == false) && (EstState == EST_State_Rs))
//    {
//      float_t Lhf = CTRL_getLhf(handle);
//      float_t Rhf = CTRL_getRhf(handle);
//      float_t RhfoverLhf = Rhf/Lhf;
////      _iq Kp = _IQ(0.25*Lhf*USER_IQ_FULL_SCALE_CURRENT_A/(USER_CTRL_PERIOD_sec*USER_IQ_FULL_SCALE_VOLTAGE_V));
//      _iq Kp = _IQ(0.25*Lhf*gUserParams[mtrNum].iqFullScaleCurrent_A/(gUserParams[mtrNum].ctrlPeriod_sec*gUserParams[mtrNum].iqFullScaleVoltage_V));
//
////      _iq Ki = _IQ(RhfoverLhf*USER_CTRL_PERIOD_sec);
//      _iq Ki = _IQ(RhfoverLhf*gUserParams[mtrNum].ctrlPeriod_sec);
//
//      // set Rhf/Lhf
//      CTRL_setRoverL(handle,RhfoverLhf);
//
//      // set the controller proportional gains
//      CTRL_setKp(handle,CTRL_Type_PID_Id,Kp);
//      CTRL_setKp(handle,CTRL_Type_PID_Iq,Kp);
//
//      // set the Id controller gains
//      CTRL_setKi(handle,CTRL_Type_PID_Id,Ki);
//      PID_setKi(obj->pidHandle_Id,Ki);
//
//      // set the Iq controller gains
//      CTRL_setKi(handle,CTRL_Type_PID_Iq,Ki);
//      PID_setKi(obj->pidHandle_Iq,Ki);
//    }
//
//  return;
//} // end of recalcKpKi() function
//
////@} //defgroup
//// end of file
//
//
//

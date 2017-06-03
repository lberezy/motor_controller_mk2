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
//! \brief Dual motor implementation of InstaSPIN-FOC
//!
//! (C) Copyright 2015, Texas Instruments, Inc.
//!
//! Dual Motor Implementation of InstaSPIN-FOC produced by IS2 Capstone Group
//!
//!
// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"
#ifdef FLASH
// when running from FLASH, load the ISRs to fast executing RAM
#pragma CODE_SECTION(motor1_ISR,"ramfuncs");
#pragma CODE_SECTION(motor2_ISR,"ramfuncs");
#endif
// for encoder attached to motor channel 2
#define USER_ENC_SAMPLE_PERIOD	(USER_ISR_FREQ_Hz_M2/USER_NUM_PWM_TICKS_PER_ISR_TICK_M2)/(USER_MOTOR_MAX_SPEED_KRPM_M2*1000/60 *2)

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

//!< PID controllers for platform stabilisation
//!< 0 - Platform Velocity, 1 - Platform Position
PID_Obj pidPlat[2];
PID_Handle pidPlatHandle[2];
uint16_t pidPlatDecimate[2];     //!< count variable to decimate the execution of platform controllers

// variables to connect controller paths
volatile _iq gPlatPosRef = _IQ(0.0);
volatile _iq gPlatPosFB = _IQ(0.0);
volatile _iq gPlatPosCtrlOut = _IQ(0.0);

volatile _iq gPlatVelFB = _IQ(0.0);
volatile _iq gPlatVelCtrlOut = _IQ(0.0);

PARK_Handle     parkHandle[2];      //!< the handle for the current Park transform
PARK_Obj        park[2];            //!< the current Park transform object
IPARK_Handle iparkHandle[2];     //!< the handle for the inverse Park transform
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
// va, vb, vc, ia, ib, ic (ordered)
FILTER_FO_Obj filter[2][6]; //!< the 3-current and 3-voltage filters for offset calculation
uint32_t gOffsetCalcCount[2] = { 0, 0 };
uint32_t gAlignCount[2] = {0,0};

// define CPU timecounting object
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

volatile SYSTEM_Vars_t gSystemVars = SYSTEM_Vars_INIT; // initial system control state

#ifdef FLASH
// Used for running ISR in RAM and main() from flash
		extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#ifdef CSM_ENABLE
		extern uint16_t *econst_start, *econst_end, *econst_ram_load;
		extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif


#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI, debugging use
DRV_SPI_8305_Vars_t gDrvSpi8305Vars[2];
#endif


uint16_t gCounter_updateGlobals[2] = { 0, 0 };
uint16_t gTrjCnt[2] = { 0, 0 };

_iq gSpeed_krpm_to_pu_sf[2];
_iq gSpeed_hz_to_krpm_sf[2];
_iq gId_MinValue[2];
_iq gId_MaxDelta[2];
_iq gId_RsEst[2];

uint16_t gCounter_getGyro = 0;
bool gGetGyroFlag = 0;

//_iq gGyro_angle = _IQ(0.0);

uint16_t gData = 0;

float gGyro_rate = 0.0;
float gGyro_angle = 0.0;
int16_t gGyro_bias = -60;

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
	if(*econst_end - *econst_start) {
		memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
	}
	//copy .switch to unsecure RAM
	if(*switch_end - *switch_start) {
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
	//encHandle[HAL_MTR1] = ENC_init(&enc[HAL_MTR1], sizeof(enc[HAL_MTR1])); // not using first encoder channel
	encHandle[HAL_MTR2] = ENC_init(&enc[HAL_MTR2], sizeof(enc[HAL_MTR2]));
    //ENC_setup(encHandle[HAL_MTR1], 1, USER_MOTOR_NUM_POLE_PAIRS_M1, USER_MOTOR_ENCODER_LINES_M1, 0, USER_IQ_FULL_SCALE_FREQ_Hz_M1, USER_ISR_FREQ_Hz_M1, 8000.0);
    ENC_setup(encHandle[HAL_MTR2], USER_ENC_SAMPLE_PERIOD, USER_MOTOR_NUM_POLE_PAIRS_M2, USER_MOTOR_ENCODER_LINES_M2, 0, USER_IQ_FULL_SCALE_FREQ_Hz_M2, USER_ISR_FREQ_Hz_M2, 8000.0);
#endif
    // set up for each motor channel
	{
		uint_least8_t mtrNum;
		for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {
			// initialize the individual motor hal files
			halHandleMtr[mtrNum] = HAL_init_mtr(&halMtr[mtrNum],
					sizeof(halMtr[mtrNum]), (HAL_MtrSelect_e) mtrNum);
			// Setup each motor channel to its specific setting
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
			// compute speed scale factor constants
			gSpeed_krpm_to_pu_sf[mtrNum] = _IQ(
					(float_t )gUserParams[mtrNum].motor_numPolePairs * 1000.0
							/ (gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0));
			gSpeed_hz_to_krpm_sf[mtrNum] = _IQ(
					60.0 / (float_t )gUserParams[mtrNum].motor_numPolePairs / 1000.0);
			// set the number of current sensors
			setupClarke_I(clarkeHandle_I[mtrNum], gUserParams[mtrNum].numCurrentSensors);
			// set the number of voltage sensors
			setupClarke_V(clarkeHandle_V[mtrNum], gUserParams[mtrNum].numVoltageSensors);
			// initialize the PID controllers
			pidSetup((HAL_MtrSelect_e) mtrNum);
	        // Park handle initialization for current signals
	        parkHandle[mtrNum] = PARK_init(&park[mtrNum], sizeof park[mtrNum]);
			// initialize the inverse Park module
			iparkHandle[mtrNum] = IPARK_init(&ipark[mtrNum], sizeof(ipark[mtrNum]));
			// initialize the space vector generator module
			svgenHandle[mtrNum] = SVGEN_init(&svgen[mtrNum], sizeof(svgen[mtrNum]));
			// initialize the Id reference trajectory
			trajHandle_Id[mtrNum] = TRAJ_init(&traj_Id[mtrNum], sizeof(traj_Id[mtrNum]));
			// initialize and configure offsets using filters
			{
				uint16_t cnt = 0;
				// set up first order lowpass filter, pole at 20 rad/s (default)
				_iq b0 = _IQ(gUserParams[mtrNum].offsetPole_rps / (float_t )gUserParams[mtrNum].ctrlFreq_Hz);
				_iq a1 = (b0 - _IQ(1.0));
				_iq b1 = _IQ(0.0);

				for (cnt = 0; cnt < 6; cnt++) {
					filterHandle[mtrNum][cnt] = FILTER_FO_init( &filter[mtrNum][cnt], sizeof(filter[mtrNum][0]));
					FILTER_FO_setDenCoeffs(filterHandle[mtrNum][cnt], a1);
					FILTER_FO_setNumCoeffs(filterHandle[mtrNum][cnt], b0, b1);
					FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt], _IQ(0.0), _IQ(0.0));
				}
				gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
			}
			// setup faults
			HAL_setupFaults(halHandleMtr[mtrNum]);
#ifdef DRV8305_SPI
			// turn on the DRV8305 if present
			HAL_enableDrv(halHandleMtr[mtrNum]);
			// initialize the DRV8305 interface
			HAL_setupDrvSpi(halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
#endif
			gCounter_updateGlobals[mtrNum] = 0;
		}
	}

	// configure the Id reference trajectory (for each motor channel)
	TRAJ_setTargetValue(trajHandle_Id[HAL_MTR1], _IQ(0.0));
	TRAJ_setIntValue(trajHandle_Id[HAL_MTR1], _IQ(0.0));
	TRAJ_setMinValue(trajHandle_Id[HAL_MTR1], _IQ(-USER_MOTOR_MAX_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1));
	TRAJ_setMaxValue(trajHandle_Id[HAL_MTR1], _IQ(USER_MOTOR_MAX_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1));
	TRAJ_setMaxDelta(trajHandle_Id[HAL_MTR1], _IQ( USER_MOTOR_RES_EST_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1 / USER_ISR_FREQ_Hz_M1));

	TRAJ_setTargetValue(trajHandle_Id[HAL_MTR2], _IQ(0.0));
	TRAJ_setIntValue(trajHandle_Id[HAL_MTR2], _IQ(0.0));
	TRAJ_setMinValue(trajHandle_Id[HAL_MTR2], _IQ(-USER_MOTOR_MAX_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2));
	TRAJ_setMaxValue(trajHandle_Id[HAL_MTR2], _IQ(USER_MOTOR_MAX_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2));
	TRAJ_setMaxDelta(trajHandle_Id[HAL_MTR2], _IQ( USER_MOTOR_RES_EST_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2 / USER_ISR_FREQ_Hz_M2));

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

	// initialise safe current value to run through motor for rotor alignment (or phase resistance estimation)
	gId_RsEst[HAL_MTR1] = _IQ(USER_MOTOR_RES_EST_CURRENT_M1 / USER_IQ_FULL_SCALE_CURRENT_A_M1);
	gId_RsEst[HAL_MTR2] = _IQ(USER_MOTOR_RES_EST_CURRENT_M2 / USER_IQ_FULL_SCALE_CURRENT_A_M2);

	HAL_initIntVectorTable(halHandle); 	// initialize the interrupt vector table
	HAL_enableAdcInts(halHandle); 	// enable the ADC interrupts
	//HAL_enableI2cInt(halHandle); 	// enable the I2C interrupt
	HAL_enableGlobalInts(halHandle); 	// enable global interrupts
	HAL_enableDebugInt(halHandle); 	// enable debug interrupts

	// disable the PWM
	HAL_disablePwm(halHandleMtr[HAL_MTR1]);
	HAL_disablePwm(halHandleMtr[HAL_MTR2]);

	// enable the system by default
	gSystemVars.Flag_enableSystem = true;

	printf("System initialised\r\n");
	// Begin the background loop
	while(1) {
		// Waiting for enable system flag to be set
		while (!(gSystemVars.Flag_enableSystem)) {
			HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED1);
			float avg = 0.0;
			const int count = 100;
			int i = 0;
			for(i = 0; i < count; i++) {
				int16_t raw = (-1) * MPU6050_getGyro_Z(halHandle->mpu6050Handle);
				avg += raw;
			}
			avg /= (1.0 * count);
			// truncate bias average to integer and store
			gGyro_bias = (int16_t)avg;
			//printf("%d\r\n", data_x);
			//MPU6050_getGyro_All(halHandle->mpu6050Handle, data);
			//printf("X: %6d\tY: %6d\tZ: %6d\t\r\n", data[0], data[1], data[2]);
			//printf(PRINTF_BINARY_PATTERN_INT16 "\r\n", PRINTF_BYTE_TO_BINARY_INT16(data));
			HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED1);
		}
		// loop while the enable system flag is true
		while (gSystemVars.Flag_enableSystem) {
			uint_least8_t mtrNum = HAL_MTR1;
			// run state machine for each motor channel
			for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {
				// increment counters
				//gCounter_updateGlobals[mtrNum]++;

				// If Flag_enableSys is set AND Flag_Run_Identify is set THEN
				// enable PWMs and set the speed reference
				if (gMotorVars[mtrNum].Flag_Run_Identify) {
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
					// set trajectory target for Id reference
					TRAJ_setTargetValue(trajHandle_Id[mtrNum],
							gIdq_ref_pu[mtrNum].value[0]);
				} else { // Flag_enableSys is set AND Flag_Run_Identify is not set
					// set estimator to Idle
					EST_setIdle(estHandle[mtrNum]);
					// disable the PWM
					if (!gMotorVars[mtrNum].Flag_enableOffsetcalc)
						HAL_disablePwm(halHandleMtr[mtrNum]);
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
				// when appropriate, update the global variables

				if(pidPlatDecimate[mtrNum]) {
					//
				}

				if (gCounter_updateGlobals[mtrNum]++ >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE) {
					// reset the counter
					gCounter_updateGlobals[mtrNum] = 0;
					updateGlobalVariables(mtrNum);
				}
#ifdef DRV8305_SPI
				HAL_writeDrvData(halHandleMtr[mtrNum],
						&gDrvSpi8305Vars[mtrNum]);
				HAL_readDrvData(halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
#endif
			} // end of per motor loop

			if (gGetGyroFlag) { // produce a sample from the gyro when indicated to by ISR (constant rate)
				gGetGyroFlag = false; //

				HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
				const int16_t bias = -60; //!!! FIX THIS
#define PI 3.141592
#define sample_time (USER_NUM_CTRL_TICKS_PER_GYRO_UPDATE_TICK / (USER_PWM_FREQ_kHz_M2 * 1E3)) // 0.005 seconds
#define scale_factor (131.072 * (180.0/PI))// LSB/rad/s scale factor (2^16/(+- 250 dps = 500) = 131.072)
				int16_t raw = 0;
				// I2C read of latest gyro data
				raw = (-1) * MPU6050_getGyro_Z(halHandle->mpu6050Handle);
				printf("RAW X: %d\r\n", raw); // for bias estimation

				// scale gyro data
				raw -= gGyro_bias;
				float gyro_rate = raw / scale_factor; // rad / s
				gGyro_rate = gyro_rate;
				gGyro_angle += (gyro_rate * sample_time);
				// integrate the thing
				//gGyro_angle += _IQmpy(gyro_rate, sample_time); // degrees
				HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
			}
		} // end of while(gFlag_enableSys) loop
		// disable the PWM
		HAL_disablePwm(halHandleMtr[HAL_MTR1]);
		HAL_disablePwm(halHandleMtr[HAL_MTR2]);
		gMotorVars[HAL_MTR1].Flag_Run_Identify = false;
		gMotorVars[HAL_MTR2].Flag_Run_Identify = false;
	} // end of main loop
} // end of main() function


//! \brief     The main ISR that implements the motor control.
interrupt void motor1_ISR(void) {
 // motor1_ISR is a duplicate of motor2_ISR, removed to save space
	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_1);
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
	//HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED2); // flash board LED
	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_2);
	// convert the ADC data
	HAL_readAdcDataWithOffsets(halHandle,halHandleMtr[HAL_MTR2],&gAdcData[HAL_MTR2]);
	// remove offsets, measured data is now in per unit quantities
	gAdcData[HAL_MTR2].I.value[0] = gAdcData[HAL_MTR2].I.value[0] - gOffsets_I_pu[HAL_MTR2].value[0];
	gAdcData[HAL_MTR2].I.value[1] = gAdcData[HAL_MTR2].I.value[1] - gOffsets_I_pu[HAL_MTR2].value[1];
	gAdcData[HAL_MTR2].I.value[2] = gAdcData[HAL_MTR2].I.value[2] - gOffsets_I_pu[HAL_MTR2].value[2];
	gAdcData[HAL_MTR2].V.value[0] = gAdcData[HAL_MTR2].V.value[0] - gOffsets_V_pu[HAL_MTR2].value[0];
	gAdcData[HAL_MTR2].V.value[1] = gAdcData[HAL_MTR2].V.value[1] - gOffsets_V_pu[HAL_MTR2].value[1];
	gAdcData[HAL_MTR2].V.value[2] = gAdcData[HAL_MTR2].V.value[2] - gOffsets_V_pu[HAL_MTR2].value[2];

	// Clark transform phase current/voltages to alpha/beta stationary reference frame
	// run Clarke transform on current.
	CLARKE_run(clarkeHandle_I[HAL_MTR2], &gAdcData[HAL_MTR2].I, &Iab_pu);
	// run Clarke transform on voltage.
	CLARKE_run(clarkeHandle_V[HAL_MTR2], &gAdcData[HAL_MTR2].V, &Vab_pu);

	EST_run(estHandle[HAL_MTR2], &Iab_pu, &Vab_pu, gAdcData[HAL_MTR2].dcBus,
				TRAJ_getIntValue(trajHandle_spd[HAL_MTR2]));
	// update the value from the quadrature encoder (rotor mechanical angle)
	ENC_run(encHandle[HAL_MTR2], HAL_getQepPosnCounts(halHandleMtr[HAL_MTR2]), HAL_getQepIndex(halHandleMtr[HAL_MTR2]), HAL_getQepDirection(halHandleMtr[HAL_MTR2]), true);
	// generate the motor electrical angle
#ifdef QEP
    // compute the electrical angle from measured mechanical angle
    ENC_calcElecAngle(encHandle[HAL_MTR2], HAL_getQepPosnCounts(halHandleMtr[HAL_MTR2]));
	angle_pu = ENC_getElecAngle(encHandle[HAL_MTR2]);
	gAngle_pu = angle_pu;
#endif

	// compute the sine and cosine phasor values which are part of the
	// Park transform calculations. Once these values are computed,
	// they are copied into the PARK module, which then uses them to
	// transform the voltages from Alpha/Beta to DQ reference frames.
	phasor.value[0] = _IQcosPU(angle_pu);
	phasor.value[1] = _IQsinPU(angle_pu);
	PARK_setPhasor(parkHandle[HAL_MTR2],&phasor);
	// Run forward Park transform on currents. Converts alpha/beta stationary reference frame
	// quantities to direct-quadrature rotating reference frame.
	PARK_run(parkHandle[HAL_MTR2], &Iab_pu, &gIdq_pu[HAL_MTR2]);

	// run a trajectory for Id reference, so the reference changes with a ramp instead of a step
	// needed for practical implementation
	TRAJ_run(trajHandle_Id[HAL_MTR2]);

	speed_pu = ENC_getFilteredSpeed(encHandle[HAL_MTR2]);


	// run the appropriate controller
	if (gMotorVars[HAL_MTR2].Flag_Run_Identify) {
		// Declaration of local variables.
		_iq refValue;
		_iq fbackValue;
		_iq outMax_pu;

		if(gMotorVars[HAL_MTR2].Flag_enableAlignment == true) {
			// the alignment procedure is in effect
			// This procedure injects current into the direct axis of the motor such that the
			// rotor aligns with the field. Once the rotor is aligned (there should be minimal load on the rotor),
			// the current mechanical angle of the rotor is taken as the zero point for the encoder. Thus an angle of
			// 0 PU is aligned with the direct axis and thus provides a reference for the forward Park transform
			// the quadrature axis is now physically aligned with the portion of the rotor field that produces maximum
			// torque.
			// force motor angle to 0 (if coming from other control state)
			angle_pu = _IQ(0.0);
			// set D-axis current to alignment current (PU)
			gIdq_ref_pu[HAL_MTR2].value[0] = _IQ(USER_MOTOR_RES_EST_CURRENT_M2/USER_IQ_FULL_SCALE_CURRENT_A_M2);
			// set Q-axis current to 0 (we want no continuous motor torque)
			gIdq_ref_pu[HAL_MTR2].value[1] = _IQ(0.0);
			// save encoder reading when forcing motor into alignment
			if(gUserParams[HAL_MTR2].motor_type == MOTOR_Type_Pm) {
				ENC_setZeroOffset(encHandle[HAL_MTR2], (uint32_t)(HAL_getQepPosnMaximum(halHandleMtr[HAL_MTR2]) - HAL_getQepPosnCounts(halHandleMtr[HAL_MTR2])));
			}
			// after elapsed time (5 seconds), exit the alignment procedure
			if(gAlignCount[HAL_MTR2]++ >= gUserParams[HAL_MTR2].ctrlWaitTime[CTRL_State_OffLine]) {
				gMotorVars[HAL_MTR2].Flag_enableAlignment = false;
				gAlignCount[HAL_MTR2] = 0;
				gIdq_ref_pu[HAL_MTR2].value[0] = _IQ(0.0);
			}
		} else {
			// when appropriate, run the platform speed and position controllers
			if (pidPlatDecimate[CTRL_PLAT_POS]++ >= USER_NUM_CTRL_TICKS_PER_PLAT_CTRL_TICK) {
				pidPlatDecimate[CTRL_PLAT_POS] = 0;
				// todo: scale constants by update rate of controller
				gPlatPosFB =  _IQ(gGyro_angle); // radians
				PID_run(pidPlatHandle[CTRL_PLAT_POS], gPlatPosRef, gPlatPosFB, &gPlatPosCtrlOut);
				//PID_run(PID_Handle handle,const _iq refValue,const _iq fbackValue,_iq *pOutValue)
			}
			if (pidPlatDecimate[CTRL_PLAT_VEL]++ >= USER_NUM_CTRL_TICKS_PER_PLAT_CTRL_TICK) {
				pidPlatDecimate[CTRL_PLAT_VEL] = 0;
				// the output of the position controller feeds
				gPlatVelFB = _IQ(gGyro_rate); // rad / s
				PID_run(pidPlatHandle[CTRL_PLAT_VEL], gPlatPosCtrlOut, gPlatVelFB, &gPlatVelCtrlOut);
			}
			if(gCounter_getGyro++ >= USER_NUM_CTRL_TICKS_PER_GYRO_UPDATE_TICK) {
				gCounter_getGyro = 0; // reset counter
				gGetGyroFlag = true; // set flag that main loop polls
			}

		}
		// Run the direct-axis current PI controller
		refValue = TRAJ_getIntValue(trajHandle_Id[HAL_MTR2]); // typically 0
		fbackValue = gIdq_pu[HAL_MTR2].value[0]; // feedback loop is measured direct-axis current
		// execute PI controller
		PID_run(pidHandle[HAL_MTR2][1], refValue, fbackValue, &(gVdq_out_pu[HAL_MTR2].value[0]));

		// Run the quadrature-axis current PI controller
		refValue = gIdq_ref_pu[HAL_MTR2].value[1]; // reference value is output of platform velocity controller // todo: change this
		refValue += gPlatVelCtrlOut;
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

		// Get the DC bus voltage
		//gMotorVars[mtrNum].VdcBus_kV = _IQmpy(gAdcData[mtrNum].dcBus,
		//		_IQ(gUserParams[mtrNum].iqFullScaleVoltage_V / 1000.0));

		// measured dc_bus voltage, PU
		//_iq max_vs = _IQmpy(_IQ(USER_MAX_VS_MAG_PU_M2), gAdcData[HAL_MTR2].dcBus); // live value?
		_iq max_vs = _IQmpy(_IQ(USER_MAX_VS_MAG_PU_M2), _IQ(1.0));
		//_iq max_vs = _IQmpy(_IQ(USER_MAX_VS_MAG_PU_M2), EST_getDcBus_pu(estHandle[HAL_MTR2]));
		outMax_pu =_IQsqrt(_IQmpy(max_vs,max_vs) - _IQmpy(gVdq_out_pu[HAL_MTR2].value[0],gVdq_out_pu[HAL_MTR2].value[0]));
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
		//oneOverDcBus = EST_getOneOverDcBus_pu(estHandle[HAL_MTR2]);
		//oneOverDcBus = _IQdiv(1, gAdcData[HAL_MTR2].dcBus); // PU
		oneOverDcBus = _IQ(1.0);
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
		//TRAJ_run(trajHandle_spd[HAL_MTR2]);
	}
	// read the timer 2 value and update the CPU usage module
	timer1Cnt = HAL_readTimerCnt(halHandle, 2);
	CPU_TIME_run(cpu_timeHandle[HAL_MTR2], timer1Cnt);
	//HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
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
	// 1E-6 * (1000/20) * (2) = 1/10000 Hz = 0.0001 s;
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
	// 0.25 is a rule of thumb
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

	// Initialise the platform PID controllers
	// initialise handles to platform controllers
	pidPlatHandle[CTRL_PLAT_VEL] = PID_init(&pidPlat[CTRL_PLAT_VEL], sizeof pidPlat[CTRL_PLAT_VEL]);
	pidPlatHandle[CTRL_PLAT_POS] = PID_init(&pidPlat[CTRL_PLAT_POS], sizeof pidPlat[CTRL_PLAT_POS]);
	// reset decimation counters
	pidPlatDecimate[CTRL_PLAT_VEL] = 0;
	pidPlatDecimate[CTRL_PLAT_POS] = 0;
	// setup platform velocity controller
	PID_setGains(pidPlatHandle[CTRL_PLAT_VEL], _IQ(-0.124), _IQ(0.0), _IQ(0.0)); // !!
	PID_setMinMax(pidPlatHandle[CTRL_PLAT_VEL], _IQ(-0.1), _IQ(+0.1));
	PID_setUi(pidPlatHandle[CTRL_PLAT_VEL], _IQ(0.0));
	// setup position controller
	PID_setGains(pidPlatHandle[CTRL_PLAT_POS], _IQ(10), _IQ(0.0), _IQ(0.0)); // !!
	PID_setMinMax(pidPlatHandle[CTRL_PLAT_POS], _IQ(-100.0), _IQ(100.0));
	PID_setUi(pidPlatHandle[CTRL_PLAT_POS], _IQ(0.0));
}


interrupt void i2c_ISR(void) {
//	//Read the interrupt source
//	//acknowledge the I2C interrupt
	HAL_acqI2cInt(halHandle);
//	// I2C_IntSource_e source;
//
	//HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED1);
//
//	I2cSource = HAL_getIntSourceI2c(halHandle);
//	I2cStatus = HAL_getStatusI2c(halHandle);
//	switch(I2cSource) {
//		case I2C_IntSrc_None: //!< No Interrupt
//		case I2C_IntSrc_Arb_Lost : //!< Arbitration Lost
//		case I2C_IntSrc_NACK: //!< No-Acknowledge Detected
//		case I2C_IntSrc_Stop: //!< Stop Condition Detected
//		case I2C_IntSrc_Slave_Addr: //!< Addressed as Slave
//		case I2C_IntSrc_Reg_Rdy: //!< Register Ready for Access
//			break;
//		case I2C_IntSrc_Rx_Rdy: //!< Receive Data Ready
//			break;
//		case I2C_IntSrc_Tx_Rdy: //!< Transmit Data Ready
//			break;
//	}
}

// Measures the steady state ADC offsets in the current and voltage measurement channels
// by running a known current through them (50% duty cycle, synchronous across all output phases)
// measured values are averaged with a LPF and then stored. Offsets are then later subtracted from ADC readings in ISR
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
		FILTER_FO_run(filterHandle[mtrNum][cnt + 3], gAdcData[mtrNum].V.value[cnt]);
	}
	// when offset calc has run for enough time
	if (gOffsetCalcCount[mtrNum]++ >= gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
		gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
		gOffsetCalcCount[mtrNum] = 0;
		// get the low-pass output values for the current and voltage measurement offsets
		for (cnt = 0; cnt < 3; cnt++) {
			// get calculated offsets from filter
			gOffsets_I_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(filterHandle[mtrNum][cnt]);
			gOffsets_V_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(filterHandle[mtrNum][cnt + 3]);
			// clear filters
			FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt], _IQ(0.0), _IQ(0.0));
			FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt + 3], _IQ(0.0), _IQ(0.0));
		}
	}
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

	float_t fullScaleInductance = iqFullScaleVoltage_V / (iqFullScaleCurrent_A * voltageFilterPole_rps);
	float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
	int_least8_t lShift = ceil(log(motorLs_d / (Ls_coarse_max * fullScaleInductance)) / log(2.0));
	uint_least8_t Ls_qFmt = 30 - lShift;
	float_t L_max = fullScaleInductance * pow(2.0, lShift);
	_iq Ls_d_pu = _IQ30(motorLs_d / L_max);
	_iq Ls_q_pu = _IQ30(motorLs_q / L_max);
	// store the results
	EST_setLs_d_pu(handle, Ls_d_pu);
	EST_setLs_q_pu(handle, Ls_q_pu);
	EST_setLs_qFmt(handle, Ls_qFmt);
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
void updateGlobalVariables(const uint_least8_t mtrNum) {

	// get filtered speed in kRPM from encoder module
	gMotorVars[mtrNum].Speed_krpm = _IQmpy((uint32_t)ENC_getSpeedRPM(encHandle[HAL_MTR2]), 1/1000.0);
	// Get the DC bus voltage
	gMotorVars[mtrNum].VdcBus_kV = _IQmpy(gAdcData[mtrNum].dcBus, _IQ(gUserParams[mtrNum].iqFullScaleVoltage_V / 1000.0));
	// read Vd and Vq vectors per units
	gMotorVars[mtrNum].Vd = gVdq_out_pu[mtrNum].value[0];
	gMotorVars[mtrNum].Vq = gVdq_out_pu[mtrNum].value[1];
	// calculate vector Vs in per units: (Vs = sqrt(Vd^2 + Vq^2))
	gMotorVars[mtrNum].Vs = _IQsqrt(
					_IQmpy(gMotorVars[mtrNum].Vd,gMotorVars[mtrNum].Vd) + _IQmpy(gMotorVars[mtrNum].Vq,gMotorVars[mtrNum].Vq));
	// read Id and Iq vectors in amps
	gMotorVars[mtrNum].Id_A = _IQmpy(gIdq_pu[mtrNum].value[0], _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
	gMotorVars[mtrNum].Iq_A = _IQmpy(gIdq_pu[mtrNum].value[1], _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
	// calculate vector Is in amps:  (Is_A = sqrt(Id_A^2 + Iq_A^2))
	gMotorVars[mtrNum].Is_A =_IQsqrt(
					_IQmpy(gMotorVars[mtrNum].Id_A,gMotorVars[mtrNum].Id_A) + _IQmpy(gMotorVars[mtrNum].Iq_A,gMotorVars[mtrNum].Iq_A));
} // end of updateGlobalVariables() function

//@} //defgroup
// end of file

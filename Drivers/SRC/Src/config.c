/*
 * config.c
 *
 *  Created on: Mar 30, 2026
 *      Author: lenovo
 */
#include "config.h"

void init_eepromConfig(bool eepromReset)
{
	if (eepromReset)
	    {
	        // Default settings
	        // eepromConfig.version = checkNewEEPROMConf;

	        ///////////////////////////////

	        eepromConfig.accelTCBiasSlope[XAXIS] = 0.0f;
	        eepromConfig.accelTCBiasSlope[YAXIS] = 0.0f;
	        eepromConfig.accelTCBiasSlope[ZAXIS] = 0.0f;

	        ///////////////////////////////

	        eepromConfig.accelTCBiasIntercept[XAXIS] = 0.0f;
	        eepromConfig.accelTCBiasIntercept[YAXIS] = 0.0f;
	        eepromConfig.accelTCBiasIntercept[ZAXIS] = 0.0f;

	        ///////////////////////////////

	        eepromConfig.gyroTCBiasSlope[ROLL ] = 0.0f;
	        eepromConfig.gyroTCBiasSlope[PITCH] = 0.0f;
	        eepromConfig.gyroTCBiasSlope[YAW  ] = 0.0f;

	        ///////////////////////////////

	        eepromConfig.gyroTCBiasIntercept[ROLL ] = 0.0f;
	        eepromConfig.gyroTCBiasIntercept[PITCH] = 0.0f;
	        eepromConfig.gyroTCBiasIntercept[YAW  ] = 0.0f;

	        ///////////////////////////////

	        eepromConfig.magBias[XAXIS] = 0.0f;
	        eepromConfig.magBias[YAXIS] = 0.0f;
	        eepromConfig.magBias[ZAXIS] = 0.0f;

	        ///////////////////////////////

	        eepromConfig.accelCutoff = 1.0f;

	        ///////////////////////////////

	        eepromConfig.KpAcc = 0.98f;    // proportional gain governs rate of convergence to accelerometer
	        eepromConfig.KiAcc = 0.02f;    // integral gain governs rate of convergence of gyroscope biases
	        eepromConfig.KpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer
	        eepromConfig.KiMag = 0.0f;    // integral gain governs rate of convergence of gyroscope biases

	        ///////////////////////////////

	        eepromConfig.dlpfSetting = DLPF_CFG_184HZ;

	        ///////////////////////////////

	        eepromConfig.midCommand = 3000.0f;

	        ///////////////////////////////

	        eepromConfig.PID[ROLL_PID].B               =    1.0f;
	        eepromConfig.PID[ROLL_PID].P               =   10.0f;
	        eepromConfig.PID[ROLL_PID].I               =    5.0f;
	        eepromConfig.PID[ROLL_PID].D               =    0.1f;
	        eepromConfig.PID[ROLL_PID].iTerm           =    0.0f;
	        eepromConfig.PID[ROLL_PID].windupGuard     = 1000.0f;  // PWMs
	        eepromConfig.PID[ROLL_PID].lastDcalcValue  =    0.0f;
	        eepromConfig.PID[ROLL_PID].lastDterm       =    0.0f;
	        eepromConfig.PID[ROLL_PID].lastLastDterm   =    0.0f;
	        eepromConfig.PID[ROLL_PID].dErrorCalc      =    D_ERROR;
	        eepromConfig.PID[ROLL_PID].type            =    ANGULAR;

	        eepromConfig.PID[PITCH_PID].B              =    1.0f;
	        eepromConfig.PID[PITCH_PID].P              =    2.5f;
	        eepromConfig.PID[PITCH_PID].I              =    5.0f;
	        eepromConfig.PID[PITCH_PID].D              =    0.05f;
	        eepromConfig.PID[PITCH_PID].iTerm          =    0.0f;
	        eepromConfig.PID[PITCH_PID].windupGuard    = 	100.0f;  // PWMs
	        eepromConfig.PID[PITCH_PID].lastDcalcValue =    0.0f;
	        eepromConfig.PID[PITCH_PID].lastDterm      =    0.0f;
	        eepromConfig.PID[PITCH_PID].lastLastDterm  =    0.0f;
	        eepromConfig.PID[PITCH_PID].dErrorCalc     =    D_ERROR;
	        eepromConfig.PID[PITCH_PID].type           =    ANGULAR;

	        eepromConfig.PID[YAW_PID].B                =    1.0f;
	        eepromConfig.PID[YAW_PID].P                =   10.0f;
	        eepromConfig.PID[YAW_PID].I                =    5.0f;
	        eepromConfig.PID[YAW_PID].D                =    0.1f;
	        eepromConfig.PID[YAW_PID].iTerm            =    0.0f;
	        eepromConfig.PID[YAW_PID].windupGuard      = 1000.0f;  // PWMs
	        eepromConfig.PID[YAW_PID].lastDcalcValue   =    0.0f;
	        eepromConfig.PID[YAW_PID].lastDterm        =    0.0f;
	        eepromConfig.PID[YAW_PID].lastLastDterm    =    0.0f;
	        eepromConfig.PID[YAW_PID].dErrorCalc       =    D_ERROR;
	        eepromConfig.PID[YAW_PID].type             =    ANGULAR;

	        eepromConfig.rollPower    = 55.0f;
	        eepromConfig.pitchPower   = 55.0f;
	        eepromConfig.yawPower     = 55.0f;

	        eepromConfig.rollEnabled  = false;
	        eepromConfig.pitchEnabled = false;
	        eepromConfig.yawEnabled   = false;

	        eepromConfig.rollAutoPanEnabled  = false;
	        eepromConfig.pitchAutoPanEnabled = false;
	        eepromConfig.yawAutoPanEnabled   = false;

	        eepromConfig.imuOrientation = 4;

	        eepromConfig.rollMotorPoles  = 14.0f;
	        eepromConfig.pitchMotorPoles = 14.0f;
	        eepromConfig.yawMotorPoles   = 14.0f;

	        eepromConfig.rateLimit = 45.0f * D2R;  // Note this is rate limiting electrical degrees of rotation, not mechanical

	        eepromConfig.rollRateCmdInput  = true;
	        eepromConfig.pitchRateCmdInput = true;
	        eepromConfig.yawRateCmdInput   = true;

	        eepromConfig.gimbalRollRate  = 40.0f * D2R;
	        eepromConfig.gimbalPitchRate = 40.0f * D2R;
	        eepromConfig.gimbalYawRate   = 40.0f * D2R;

	        eepromConfig.gimbalRollLeftLimit  = 75.0f * D2R;
	        eepromConfig.gimbalRollRightLimit = 75.0f * D2R;
	        eepromConfig.gimbalPitchDownLimit = 75.0f * D2R;
	        eepromConfig.gimbalPitchUpLimit   = 30.0f * D2R;
	        eepromConfig.gimbalYawLeftLimit   = 75.0f * D2R;
	        eepromConfig.gimbalYawRightLimit  = 75.0f * D2R;

	        eepromConfig.accelX500HzLowPassTau = 0.1f;
	        eepromConfig.accelY500HzLowPassTau = 0.1f;
	        eepromConfig.accelZ500HzLowPassTau = 0.1f;

	        eepromConfig.rollRatePointingCmd50HzLowPassTau  = 0.0f;
	        eepromConfig.pitchRatePointingCmd50HzLowPassTau = 0.0f;
	        eepromConfig.yawRatePointingCmd50HzLowPassTau   = 0.0f;

	        eepromConfig.rollAttPointingCmd50HzLowPassTau  = 0.25f;
	        eepromConfig.pitchAttPointingCmd50HzLowPassTau = 0.25f;
	        eepromConfig.yawAttPointingCmd50HzLowPassTau   = 0.25f;

	       // writeEEPROM();
	    }
}

/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 *
 * --------------------------------------------------------------------
 * --------------------------------------------------------------------
 * Changes made for ModQuad are denoted with ***...*** comment blocks
 *
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"

/* stock */
static bool motorSetEnable = false;

/* modquad var */
static uint16_t motor_set_timer = 0;

/* more modquad vars */
static float r1 = -1.0;
static float r2 = -1.0;
static float r3 = 1.0;
static float r4 = 1.0;
static float p1 = 1.0;
static float p2 = -1.0;
static float p3 = -1.0;
static float p4 = 1.0;
static float cz = 1.0;

// F-plane parameters
static float rp11 = 1.0;
static float rp12 = 0.0;
static float rp13 = 0.0;
static float rp21 = 0.0;
static float rp22 = 1.0;
static float rp23 = 0.0;
static float rp31 = 0.0;
static float rp32 = 0.0;
static float rp33 = 1.0;

/* stock */
static struct {
	uint32_t m1;
	uint32_t m2;
	uint32_t m3;
	uint32_t m4;
} motorPower;

static struct {
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
	motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
#ifdef QUAD_FORMATION_X
	int16_t r = control->roll / 2.0f;
	int16_t p = control->pitch / 2.0f;

	/*** Modified for H-ModQuad ***/
  // TODO: use the following script to derive the PD:
  // inverse[{{1, 1, 1, 1},{p_1*r_33-c*r_13, p_2*r_33+c*r_13, p_3*r_33-c*r_13, p_4*r_33+c*r_13},{-p_1*r_33-c*r_23, -p_2*r_33+c*r_23, -p_3*r_33-c*r_23, -p_4*r_33+c*r_23},{-p_1*r_13+r_1*r_23 - c*r_33, -p_2*r_13+r_2*r_23 + c*r_33, -p_3*r_13+r_3*r_23 - c*r_33, -p_4*r_13+r_4*r_23 - c*r_33}}]
	motorPower.m1 = limitThrust(control->thrust  + r1*r + p1*p + cz*control->yaw);
	motorPower.m2 = limitThrust(control->thrust  + r2*r + p2*p - cz*control->yaw);
	motorPower.m3 = limitThrust(control->thrust + r3*r + p3*p + cz*control->yaw);
	motorPower.m4 = limitThrust(control->thrust + r4*r + p4*p - cz*control->yaw);
	/*** End Modified for ModQuad ***/

#else /* QUAD_FORMATION_NORMAL */
	motorPower.m1 = limitThrust(control->thrust + control->pitch +
			control->yaw);
	motorPower.m2 = limitThrust(control->thrust - control->roll -
			control->yaw);
	motorPower.m3 =  limitThrust(control->thrust - control->pitch +
			control->yaw);
	motorPower.m4 =  limitThrust(control->thrust + control->roll -
			control->yaw);
#endif


	/*** Added for ModQuad ***/
	/* The timer is reduced on every tick */
	if (motor_set_timer){
		motor_set_timer--;
	}
	/*** End Added for ModQuad ***/

	if (motorSetEnable)
	{
		motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
		motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
		motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
		motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
	}
	else
	{
		motorsSetRatio(MOTOR_M1, motorPower.m1);
		motorsSetRatio(MOTOR_M2, motorPower.m2);
		motorsSetRatio(MOTOR_M3, motorPower.m3);
		motorsSetRatio(MOTOR_M4, motorPower.m4);
	}
}

/*** Add for ModQuad ***/
PARAM_GROUP_START(var)
PARAM_ADD(PARAM_FLOAT, roll1, &r1)
PARAM_ADD(PARAM_FLOAT, roll2, &r2)
PARAM_ADD(PARAM_FLOAT, roll3, &r3)
PARAM_ADD(PARAM_FLOAT, roll4, &r4)
PARAM_ADD(PARAM_FLOAT, pitch1, &p1)
PARAM_ADD(PARAM_FLOAT, pitch2, &p2)
PARAM_ADD(PARAM_FLOAT, pitch3, &p3)
PARAM_ADD(PARAM_FLOAT, pitch4, &p4)
PARAM_ADD(PARAM_FLOAT, czz, &cz)
PARAM_GROUP_STOP(var)
/*** End Add for ModQuad ***/

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)

/*** Add for ModQuad ***/
/* Enable based on timer */
PARAM_ADD(PARAM_UINT16, motor_timer, &motor_set_timer)
/*** End Add for ModQuad ***/

PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)

/*** Add for H-ModQuad ***/
PARAM_GROUP_START(FplaneParams)
PARAM_ADD(PARAM_FLOAT, rp11, &rp11)
PARAM_ADD(PARAM_FLOAT, rp12, &rp12)
PARAM_ADD(PARAM_FLOAT, rp13, &rp13)
PARAM_ADD(PARAM_FLOAT, rp21, &rp21)
PARAM_ADD(PARAM_FLOAT, rp22, &rp22)
PARAM_ADD(PARAM_FLOAT, rp23, &rp23)
PARAM_ADD(PARAM_FLOAT, rp31, &rp31)
PARAM_ADD(PARAM_FLOAT, rp32, &rp32)
PARAM_ADD(PARAM_FLOAT, rp33, &rp33)
PARAM_GROUP_STOP(FplaneParams)
/*** End add for H-ModQuad ***/

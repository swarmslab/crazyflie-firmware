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

static bool motorSetEnable = false;

// F-frame parameters
static float Ai[8][6] = {{4.95065, -0.950647, 1.41421, -41.7284, -41.7284, 20.},
                        {-4.95065, -0.950647, 1.41421, -41.7284, 41.7284, -20.},
                        {-4.95065, 0.950647, 1.41421, 41.7284, 41.7284, 20.},
                        {4.95065, 0.950647, 1.41421, 41.7284, -41.7284, -20.},
                        {0.950647, -4.95065, -1.41421, -41.7284, -41.7284, -20.},
                        {-0.950647, -4.95065, -1.41421, -41.7284, 41.7284, 20.},
                        {-0.950647, 4.95065, -1.41421, 41.7284, 41.7284, -20.},
                        {0.950647, 4.95065, -1.41421, 41.7284, -41.7284, 20.}};
static float null_base[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

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

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

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

#define limitThrust(VAL) ((VAL>8000)?limitUint16(VAL):8000)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  float motorPowerTemp[8];
  #ifdef QUAD_FORMATION_X
    float r = control->roll;
    float p = control->pitch;
    float y = control->yaw;
    //
    /*** Modified for H-ModQuad ***/
    int module_index = 0;
    float min_force_div = 0.0;
    for (int i = 0; i < 8; i++) {
      motorPowerTemp[i] = Ai[i][0] * control->thrustx +
                          Ai[i][1] * control->thrusty +
                          Ai[i][2] * control->thrust +
                          Ai[i][3] * r +
                          Ai[i][4] * p +
                          Ai[i][5] * y;
      if(motorPowerTemp[i] / null_base[i] < min_force_div) {
        min_force_div = motorPowerTemp[i] / null_base[i];
      }
    }

    if(control->thrust>0 || control->thrustx>0 || control->thrusty>0) {
      motorPower.m1 = limitThrust(motorPowerTemp[0 + 4*module_index] - null_base[0 + 4*module_index]*min_force_div);
      motorPower.m2 = limitThrust(motorPowerTemp[1 + 4*module_index] - null_base[1 + 4*module_index]*min_force_div);
      motorPower.m3 = limitThrust(motorPowerTemp[2 + 4*module_index] - null_base[2 + 4*module_index]*min_force_div);
      motorPower.m4 = limitThrust(motorPowerTemp[3 + 4*module_index] - null_base[3 + 4*module_index]*min_force_div);
    }
    /*** End Modified for ModQuad ***/
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    if (motorPower.m1 < idleThrust) {
      motorPower.m1 = idleThrust;
    }
    if (motorPower.m2 < idleThrust) {
      motorPower.m2 = idleThrust;
    }
    if (motorPower.m3 < idleThrust) {
      motorPower.m3 = idleThrust;
    }
    if (motorPower.m4 < idleThrust) {
      motorPower.m4 = idleThrust;
    }

    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)

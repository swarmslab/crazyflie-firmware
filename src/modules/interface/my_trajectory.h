/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Jiawei Xu
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
 * myTrjectory.c
 */

#ifndef __MY_TRAJECTORY_H__
#define __MY_TRAJECTORY_H__
#include "stabilizer_types.h"
void trajectory_square(float t, setpoint_t *setpoint);
void trajectory_circle(float t, setpoint_t *setpoint);
void trajectory_pitchosci(float t, setpoint_t *setpoint);
void trajectory_takeoff(float t, setpoint_t *setpoint);
void trajectory_landing(float t, setpoint_t *setpoint);

#endif //__MY_TRAJECTORY_H__

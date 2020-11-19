/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * my_trajectory.c trajectories as setpoint functions of time
 */

#include "my_trajectory.h"
#include "stabilizer_types.h"
#include "debug.h"
#include <math.h>
#include "math3d.h"

static float xy_mag = 0.6; // m
static float interval = 5.0; // time interval for several trajectories (s)
static float x_scale = 1.5; // scale motions on x a little bit
static float oscillation_mag = 5.0; // degrees
static float pitch_trim = -5.0;

void trajectory_square(float time_instance, setpoint_t *setpoint){
  // A square of side length xy_mag (m)
  float x, y, z;
  float x_vel = 0;
  float y_vel = 0;

  if (time_instance < interval) {
    y = -xy_mag/2;
    x = x_scale*(-xy_mag/2+time_instance*xy_mag/interval);
    // x_vel = ((float)-fabs(2*xy_mag/(interval*interval)*(time_instance-(float)0.5*interval))+xy_mag/interval)*x_scale;
    x_vel = 0;
    y_vel = 0;
  } else if (time_instance < 2*interval) {
    y = -xy_mag/2+(time_instance-interval)*xy_mag/interval;
    x = x_scale*(xy_mag/2);
    x_vel = 0;
    y_vel = 0;
    // y_vel = (float)-fabs(2*xy_mag/(interval*interval)*(time_instance-(float)1.5*interval))+xy_mag/interval;
  } else if (time_instance < 3*interval) {
    y = xy_mag/2;
    x = x_scale*(xy_mag/2-(time_instance-2*interval)*xy_mag/interval);
    // x_vel = ((float)-fabs(2*xy_mag/(interval*interval)*(time_instance-(float)2.5*interval))+xy_mag/interval)*x_scale;
    x_vel = 0;
    y_vel = 0;
  } else {
    y = xy_mag/2-(time_instance-3*interval)*xy_mag/interval;
    x = x_scale*(-xy_mag/2);
    x_vel = 0;
    y_vel = 0;
    // y_vel = (float)-fabs(2*xy_mag/(interval*interval)*(time_instance-(float)3.5*interval))+xy_mag/interval;
  }
  z = 0.45;
  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->position.z = z;
  setpoint->attitude.roll = 0;
  setpoint->attitude.pitch = 10;
  setpoint->attitude.yaw = 0;
  setpoint->velocity.x = x_vel;
  setpoint->velocity.y = y_vel;
  setpoint->velocity.z = 0;
  setpoint->attitude.roll = 0;
  setpoint->attitude.pitch = 0;
  setpoint->attitude.yaw = 0;
  setpoint->attitudeRate.roll = 0;
  setpoint->attitudeRate.pitch = 0;
  setpoint->attitudeRate.yaw = 0;
  setpoint->acceleration.x = 0;
  setpoint->acceleration.y = 0;
  setpoint->acceleration.z = 0;
}

void trajectory_circle(float t, setpoint_t *setpoint) {
  setpoint->position.x = -xy_mag/2*cosf(radians(18*t))+xy_mag;
  setpoint->position.y = xy_mag/2*sinf(radians(18*t));
  setpoint->position.z = 0.3;
  setpoint->attitude.roll = 0;
  setpoint->attitude.yaw = 0;
  setpoint->velocity.x = 0;
  setpoint->velocity.y = 0;
  setpoint->velocity.z = 0;
  setpoint->attitude.roll = 0;
  setpoint->attitude.pitch = 0;
  setpoint->attitude.yaw = 0;
  setpoint->attitudeRate.roll = 0;
  setpoint->attitudeRate.pitch = 0;
  setpoint->attitudeRate.yaw = 0;
  setpoint->acceleration.x = 0;
  setpoint->acceleration.y = 0;
  setpoint->acceleration.z = 0;
}

void trajectory_pitchosci(float t, setpoint_t *setpoint) {
  setpoint->position.x = 0;
  setpoint->position.y = 0;
  setpoint->position.z = 0.45;
  setpoint->attitude.roll = 0;
  setpoint->attitude.yaw = 0;
  setpoint->velocity.x = 0;
  setpoint->velocity.y = 0;
  setpoint->velocity.z = 0;
  setpoint->attitude.roll = 0;
  setpoint->attitude.pitch = 0;
  setpoint->attitude.yaw = 0;
  setpoint->attitudeRate.roll = 0;
  setpoint->attitudeRate.pitch = 0;
  setpoint->attitudeRate.yaw = 0;
  setpoint->acceleration.x = 0;
  setpoint->acceleration.y = 0;
  setpoint->acceleration.z = 0;
  if (t<=0){
    setpoint->attitude.pitch = 10;
  } else {
    setpoint->attitude.pitch = oscillation_mag*cosf(radians(18*t))-pitch_trim;
  }
}

void trajectory_takeoff(float t, setpoint_t *setpoint) {
  setpoint->position.x = 0;
  setpoint->position.y = 0;
  setpoint->position.z = 0.45;
  setpoint->attitude.roll = 0;
  setpoint->attitude.yaw = 0;
  setpoint->velocity.x = 0;
  setpoint->velocity.y = 0;
  setpoint->velocity.z = 0;
  setpoint->attitude.roll = 0;
  setpoint->attitude.pitch = 0;
  setpoint->attitude.yaw = 0;
  setpoint->attitudeRate.roll = 0;
  setpoint->attitudeRate.pitch = 0;
  setpoint->attitudeRate.yaw = 0;
  setpoint->acceleration.x = 0;
  setpoint->acceleration.y = 0;
  setpoint->acceleration.z = 0;
}

void trajectory_landing(float t, setpoint_t *setpoint) {}

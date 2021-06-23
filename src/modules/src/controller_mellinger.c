/*
The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This controller is based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.

We added the following:
 * Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmmetries, and uneven wear on propellers and motors)
 * D-term for angular velocity
 * Support to use this controller as an attitude-only controller for manual flight
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "controller_mellinger.h"
#include "physicalConstants.h"
#include "my_trajectory.h"
#include "debug.h"

static float g_vehicleMass = CF_MASS;
static float massThrust = 132000;

// XY Position PID
static float kp_x = 0.4;       // P
static float kd_x = 0.2;       // D
static float ki_x = 0.05;      // I
static float i_range_x = 2.0;

static float kp_y = 0.4;       // P
static float kd_y = 0.2;       // D
static float ki_y = 0.05;      // I
static float i_range_y = 2.0;

// Z Position
static float kp_z = 1.25;       // P
static float kd_z = 0.4;        // D
static float ki_z = 0.05;       // I
static float i_range_z  = 0.4;

// Attitude
static float kR_x = 56000; // P
static float kw_x = 20000; // D
static float ki_m_x = 0.0; // I
static float i_range_m_x = 1.0;

static float kR_y = 65000; // P
static float kw_y = 14000; // D
static float ki_m_y = 0.0; // I
static float i_range_m_y = 3.0;

// Yaw
static float kR_z = 60000; // P
static float kw_z = 12000; // D
static float ki_m_z = 500; // I
static float i_range_m_z  = 1500;

// roll and pitch angular velocity
static float kd_omega_rp = 200; // D


// Helper variables
static float i_error_x = 0;
static float i_error_y = 0;
static float i_error_z = 0;

static float prev_omega_roll;
static float prev_omega_pitch;
static float prev_setpoint_omega_roll;
static float prev_setpoint_omega_pitch;

static float i_error_m_x = 0;
static float i_error_m_y = 0;
static float i_error_m_z = 0;

// Logging variables
static struct vec z_axis_desired;

static float cmd_thrust;
static float cmd_thrust_x;
static float cmd_thrust_y;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

struct vec eR;

// counter:
// static uint16_t counter = 0;

// trajectory parameters
static float time_instance = 0.0;
static uint8_t trajectory_type = 4;

void controllerMellingerReset(void)
{
  i_error_x = 0;
  i_error_y = 0;
  i_error_z = 0;
  i_error_m_x = 0;
  i_error_m_y = 0;
  i_error_m_z = 0;
}

void controllerMellingerInit(void)
{
  controllerMellingerReset();
}

bool controllerMellingerTest(void)
{
  return true;
}

void controllerMellinger(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  struct vec r_error;
  struct vec v_error;
  struct vec target_thrust;
  struct vec z_axis;
  struct vec x_axis;
  struct vec y_axis;
  float current_thrust;
  float current_thrust_x;
  float current_thrust_y;
  struct vec x_axis_desired;
  struct vec y_axis_desired;
  struct vec z_axis_desired;

  struct vec ew, M;
  float dt;
  float desiredRoll = 0.0; // degrees
  float desiredPitch = 0.0; // degrees
  float desiredYaw = 0.0; // degrees

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  dt = (float)(1.0f/ATTITUDE_RATE);
  /* modified for H-ModQuad Demo */
  switch (trajectory_type) {
    case 0: trajectory_square(time_instance, setpoint); break;
    case 1: trajectory_circle(time_instance, setpoint); break;
    case 2: trajectory_pitchosci(time_instance, setpoint); break;
    case 3: trajectory_rollosci(time_instance, setpoint); break;
    case 4: trajectory_takeoff(time_instance, setpoint); break;
    case 5: trajectory_landing(time_instance, setpoint); break;
    case 6: trajectory_fliphover(time_instance, setpoint); break;
  }

  /* *****************************/
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

  // Position Error (ep)
  r_error = vsub(setpointPos, statePos);

  // Velocity Error (ev)
  v_error = vsub(setpointVel, stateVel);

  // Integral Error
  i_error_z += r_error.z * dt;
  i_error_z = clamp(i_error_z, -i_range_z, i_range_z);

  i_error_x += r_error.x * dt;
  i_error_x = clamp(i_error_x, -i_range_x, i_range_x);

  i_error_y += r_error.y * dt;
  i_error_y = clamp(i_error_y, -i_range_y, i_range_y);

  // Desired thrust [F_des]
  target_thrust.x = g_vehicleMass * setpoint->acceleration.x                       + kp_x * r_error.x + kd_x * v_error.x + ki_x * i_error_x;
  target_thrust.y = g_vehicleMass * setpoint->acceleration.y                       + kp_y * r_error.y + kd_y * v_error.y + ki_y * i_error_y;
  target_thrust.z = g_vehicleMass * (setpoint->acceleration.z + GRAVITY_MAGNITUDE) + kp_z  * r_error.z + kd_z  * v_error.z + ki_z  * i_error_z;

  desiredYaw = setpoint->attitude.yaw;
  desiredPitch = setpoint->attitude.pitch;
  desiredRoll = setpoint->attitude.roll;

  // Z-Axis [zB]
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);
  z_axis = mcolumn(R, 2);
  y_axis = mcolumn(R, 1);
  x_axis = mcolumn(R, 0);

  // thrust in F-plane
  // fz
  current_thrust = vdot(target_thrust, z_axis);
  // fx
  current_thrust_x = vdot(target_thrust, x_axis);
  // fy
  current_thrust_y = vdot(target_thrust, y_axis);

  // Rd
  struct mat33 Rd = quat2rotmat(rpy2quat(mkvec(desiredRoll, desiredPitch, desiredYaw)));
  z_axis_desired = mcolumn(Rd, 2);
  y_axis_desired = mcolumn(Rd, 1);
  x_axis_desired = mcolumn(Rd, 0);

  // [eR]
  // Slow version
  // struct mat33 Rdes = mcolumns(
  //   mkvec(x_axis_desired.x, x_axis_desired.y, x_axis_desired.z),
  //   mkvec(y_axis_desired.x, y_axis_desired.y, y_axis_desired.z),
  //   mkvec(z_axis_desired.x, z_axis_desired.y, z_axis_desired.z));

  // struct mat33 R_transpose = mtranspose(R);
  // struct mat33 Rdes_transpose = mtranspose(Rdes);

  // struct mat33 eRM = msub(mmult(Rdes_transpose, R), mmult(R_transpose, Rdes));

  // eR.x = eRM.m[2][1];
  // eR.y = -eRM.m[0][2];
  // eR.z = eRM.m[1][0];

  // Fast version (generated using Mathematica)
  float x = q.x;
  float y = q.y;
  float z = q.z;
  float w = q.w;
  eR.x = (-1 + 2*fsqr(x) + 2*fsqr(y))*y_axis_desired.z + z_axis_desired.y - 2*(x*y_axis_desired.x*z + y*y_axis_desired.y*z - x*y*z_axis_desired.x + fsqr(x)*z_axis_desired.y + fsqr(z)*z_axis_desired.y - y*z*z_axis_desired.z) +    2*w*(-(y*y_axis_desired.x) - z*z_axis_desired.x + x*(y_axis_desired.y + z_axis_desired.z));
  eR.y = x_axis_desired.z - z_axis_desired.x - 2*(fsqr(x)*x_axis_desired.z + y*(x_axis_desired.z*y - x_axis_desired.y*z) - (fsqr(y) + fsqr(z))*z_axis_desired.x + x*(-(x_axis_desired.x*z) + y*z_axis_desired.y + z*z_axis_desired.z) + w*(x*x_axis_desired.y + z*z_axis_desired.y - y*(x_axis_desired.x + z_axis_desired.z)));
  eR.z = y_axis_desired.x - 2*(y*(x*x_axis_desired.x + y*y_axis_desired.x - x*y_axis_desired.y) + w*(x*x_axis_desired.z + y*y_axis_desired.z)) + 2*(-(x_axis_desired.z*y) + w*(x_axis_desired.x + y_axis_desired.y) + x*y_axis_desired.z)*z - 2*y_axis_desired.x*fsqr(z) + x_axis_desired.y*(-1 + 2*fsqr(x) + 2*fsqr(z));

  // Account for Crazyflie coordinate system
  eR.y = -eR.y;

  // [ew]
  float err_d_roll = 0;
  float err_d_pitch = 0;

  float stateAttitudeRateRoll = radians(sensors->gyro.x);
  float stateAttitudeRatePitch = -radians(sensors->gyro.y);
  float stateAttitudeRateYaw = radians(sensors->gyro.z);

  ew.x = radians(setpoint->attitudeRate.roll) - stateAttitudeRateRoll;
  ew.y = -radians(setpoint->attitudeRate.pitch) - stateAttitudeRatePitch;
  ew.z = radians(setpoint->attitudeRate.yaw) - stateAttitudeRateYaw;
  if (prev_omega_roll == prev_omega_roll) { /*d part initialized*/
    err_d_roll = ((radians(setpoint->attitudeRate.roll) - prev_setpoint_omega_roll) - (stateAttitudeRateRoll - prev_omega_roll)) / dt;
    err_d_pitch = (-(radians(setpoint->attitudeRate.pitch) - prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - prev_omega_pitch)) / dt;
  }
  prev_omega_roll = stateAttitudeRateRoll;
  prev_omega_pitch = stateAttitudeRatePitch;
  prev_setpoint_omega_roll = radians(setpoint->attitudeRate.roll);
  prev_setpoint_omega_pitch = radians(setpoint->attitudeRate.pitch);

  // Integral Error
  i_error_m_x += (-eR.x) * dt;
  i_error_m_x = clamp(i_error_m_x, -i_range_m_x, i_range_m_x);

  i_error_m_y += (-eR.y) * dt;
  i_error_m_y = clamp(i_error_m_y, -i_range_m_y, i_range_m_y);

  i_error_m_z += (-eR.z) * dt;
  i_error_m_z = clamp(i_error_m_z, -i_range_m_z, i_range_m_z);

  // Moment:
  M.x = -kR_x * eR.x + kw_x * ew.x + ki_m_x * i_error_m_x + kd_omega_rp * err_d_roll;
  M.y = -kR_y * eR.y + kw_y * ew.y + ki_m_y * i_error_m_y + kd_omega_rp * err_d_pitch;
  M.z = -kR_z  * eR.z + kw_z  * ew.z + ki_m_z  * i_error_m_z;

  // Output
  if (setpoint->mode.z == modeDisable) {
    // This is the really executed command
    // if (setpoint->thrust > 0){
    if (time_instance > 0){
      control->thrust = massThrust * current_thrust;
      // control->thrustx = clamp(current_thrust_x*massThrust, -2000, 2000);
      // control->thrusty = clamp(current_thrust_y*massThrust, -2000, 2000);
      control->thrustx = current_thrust_x*massThrust;
      control->thrusty = current_thrust_y*massThrust;
    } else {
      control->thrust = 0;
      control->thrustx = 0;
      control->thrusty = 0;
    }
    // control->thrust = setpoint->thrust;
    // control->thrustx = clamp(current_thrust_x*setpoint->thrust/current_thrust, -2000, 2000);
    // control->thrusty = clamp(current_thrust_y*setpoint->thrust/current_thrust, -2000, 2000);
    // control->thrustx = current_thrust_x*setpoint->thrust/current_thrust;
    // control->thrusty = current_thrust_y*setpoint->thrust/current_thrust;
  } else {
    control->thrust = massThrust * current_thrust;
    // control->thrustx = clamp(current_thrust_x*massThrust, -2000, 2000);
    // control->thrusty = clamp(current_thrust_y*massThrust, -2000, 2000);
    control->thrustx = current_thrust_x*massThrust;
    control->thrusty = current_thrust_y*massThrust;
  }

  cmd_thrust = control->thrust;
  cmd_thrust_x = control->thrustx;
  cmd_thrust_y = control->thrusty;
  r_roll = radians(sensors->gyro.x);
  r_pitch = -radians(sensors->gyro.y);
  r_yaw = radians(sensors->gyro.z);
  accelz = sensors->acc.z;

  if (control->thrust > 0) {
    // control->thrust = massThrust*current_thrust;
    // control->roll = clamp(M.x, -32000, 32000);
    // control->pitch = clamp(M.y, -32000, 32000);
    // control->yaw = clamp(-M.z, -32000, 32000);
    control->roll = M.x;
    control->pitch = M.y;
    control->yaw = -M.z;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

  } else {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    control->thrustx = 0;
    control->thrusty = 0;

    cmd_thrust_x = control->thrustx;
    cmd_thrust_y = control->thrusty;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    controllerMellingerReset();
  }
}

PARAM_GROUP_START(ctrlMel)
PARAM_ADD(PARAM_FLOAT, kp_x, &kp_x)
PARAM_ADD(PARAM_FLOAT, kd_x, &kd_x)
PARAM_ADD(PARAM_FLOAT, ki_x, &ki_x)
PARAM_ADD(PARAM_FLOAT, kp_y, &kp_y)
PARAM_ADD(PARAM_FLOAT, kd_y, &kd_y)
PARAM_ADD(PARAM_FLOAT, ki_y, &ki_y)
PARAM_ADD(PARAM_FLOAT, i_range_x, &i_range_x)
PARAM_ADD(PARAM_FLOAT, i_range_y, &i_range_y)
PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)
PARAM_ADD(PARAM_FLOAT, kd_z, &kd_z)
PARAM_ADD(PARAM_FLOAT, ki_z, &ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)
PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
// PARAM_ADD(PARAM_FLOAT, desiredPitch, &desiredPitch)
PARAM_ADD(PARAM_FLOAT, kR_x, &kR_x)
PARAM_ADD(PARAM_FLOAT, kR_y, &kR_y)
PARAM_ADD(PARAM_FLOAT, kR_z, &kR_z)
PARAM_ADD(PARAM_FLOAT, kw_x, &kw_x)
PARAM_ADD(PARAM_FLOAT, kw_y, &kw_y)
PARAM_ADD(PARAM_FLOAT, kw_z, &kw_z)
PARAM_ADD(PARAM_FLOAT, ki_m_x, &ki_m_x)
PARAM_ADD(PARAM_FLOAT, ki_m_y, &ki_m_y)
PARAM_ADD(PARAM_FLOAT, ki_m_z, &ki_m_z)
PARAM_ADD(PARAM_FLOAT, kd_omega_rp, &kd_omega_rp)
PARAM_ADD(PARAM_FLOAT, i_range_m_x, &i_range_m_x)
PARAM_ADD(PARAM_FLOAT, i_range_m_y, &i_range_m_y)
PARAM_ADD(PARAM_FLOAT, i_range_m_z, &i_range_m_z)
PARAM_ADD(PARAM_FLOAT, time_instance, &time_instance)
PARAM_ADD(PARAM_UINT8, trajectory_type, &trajectory_type)
PARAM_GROUP_STOP(ctrlMel)

LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_thrust_x, &cmd_thrust_x)
LOG_ADD(LOG_FLOAT, cmd_thrust_y, &cmd_thrust_y)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, zdx, &z_axis_desired.x)
LOG_ADD(LOG_FLOAT, zdy, &z_axis_desired.y)
LOG_ADD(LOG_FLOAT, zdz, &z_axis_desired.z)
LOG_ADD(LOG_FLOAT, i_err_x, &i_error_x)
LOG_ADD(LOG_FLOAT, i_err_y, &i_error_y)
LOG_ADD(LOG_FLOAT, i_err_z, &i_error_z)
LOG_ADD(LOG_FLOAT, eRx, &eR.x)
LOG_ADD(LOG_FLOAT, eRy, &eR.y)
LOG_ADD(LOG_FLOAT, eRz, &eR.z)
LOG_GROUP_STOP(ctrlMel)

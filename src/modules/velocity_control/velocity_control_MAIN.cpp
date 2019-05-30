/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

// PX4 INCLUDES
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/velocity_filter.h>
#include <uORB/topics/external_cmd.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/velocity_control.h>
#include <matrix/matrix/math.hpp>

// Namespaces
using namespace std;
using namespace matrix;

// MAIN for velocity_control - must be included to choose 'main' function
extern "C" __EXPORT int velocity_control_main(int argc, char *argv[]);



// Velocity_Control_Class Definition
class Velocity_Control_Class
{
public:
  // Cosntructor and Deconstructor
  Velocity_Control_Class();
  ~Velocity_Control_Class();   // Also kills task
  // Start
  int start();

private:
  // Handles
  bool	_task_should_exit;	  /**< if true, task_main() should exit */
	int		_control_task;			  /**< task handle */

  // UORB subcriptions
  int   _params_sub;          /**< parameter updates subscription */
  int   _attitude_sub;        /**< vehicle attitude subcription */
  int   _v_filt_sub;          /**< velocity filter subcription */
  int   _ext_cmd_sub;         /**< external command subcription */
  int   _v_rc_channels_sub;   /**< rc channels subscription */
  int   _v_control_mode_sub;  /**< vehicle control mode subscription */
  // UORB publishers
  orb_advert_t    _v_ctrl_pub;
  // UORB structures
  struct  vehicle_attitude_s      _attitude;        /**< vehicle attitude */
  struct  velocity_filter_s       _v_filt;          /**< velocity filter */
  struct  external_cmd_s          _ext_cmd;         /**< external command */
  struct  rc_channels_s           _v_rc_channels;   /**< rc channels */
  struct  vehicle_control_mode_s  _v_control_mode;  /**< vehicle control mode */
  struct  velocity_control_s      _v_ctrl;          /**< velocity control */
 

  // Parameter Handling
  struct {
    param_t vel_rc_channel;
    param_t vel_rc_threshold;
    param_t vel_kpx;
    param_t vel_kpy;
    param_t vel_kpz;
    param_t vel_kix;
    param_t vel_kiy;  
    param_t vel_kiz;
    param_t vel_mg; 
    param_t tilt_max;
    param_t thrust_max;
    param_t thrust_min;
    param_t hovermode;
    param_t ctrlmode;
    param_t holdmodeu;
    param_t holdmoded;
    param_t haltmode;
  }   _params_handles;    /**< handles for interesting parameters */
  // Parameter Values
  struct {

    int vel_rc_channel;
    float vel_rc_threshold;
    float vel_kpx;
    float vel_kpy;
    float vel_kpz;
    float vel_kix;
    float vel_kiy;
    float vel_kiz;
    float vel_mg;
    float tilt_max;
    float thrust_max;
    float thrust_min;
    int hovermode;
    int ctrlmode;
    int holdmodeu;
    int holdmoded;
    int haltmode;
  }   _params;


  float vel_intx, vel_inty, vel_intz;



  // Parameter and UORB polling
  int     parameters_update();
  void    parameter_update_poll();  
  void    vehicle_attitude_poll();
  void    velocity_filter_poll();
  void    external_command_poll();
  void    vehicle_rc_poll();
  void    vehicle_control_mode_poll();
  


  // Task Handle
  static void	task_main_trampoline(int argc, char *argv[]);
  void task_main();
};
// Namespace used for memory purposes on class assignment
namespace vel_ctrl
{
  // Make class pointer
	Velocity_Control_Class	*g_vel_ctrl;
}


// Velocity_Control_Class
// Velocity_Control_Class: Constructor and Desconstructor
Velocity_Control_Class::Velocity_Control_Class() :
  _task_should_exit(false),
  _control_task(-1),

  /* subscriptions */
  _params_sub(-1),    
  _attitude_sub(-1),    
  _v_filt_sub(-1),
  _ext_cmd_sub(-1),
  _v_rc_channels_sub(-1),
  _v_control_mode_sub(-1),
  /* publications */
  _v_ctrl_pub(nullptr),
  /* structures */
  _attitude{},  
  _v_filt{},
  _ext_cmd{},
  _v_rc_channels{},
  _v_control_mode{},
  _v_ctrl{}


{

  _params.vel_rc_channel = 0;
  _params.vel_rc_threshold = 0;
  _params.vel_kpx = 0;
  _params.vel_kpy = 0;
  _params.vel_kpz = 0;
  _params.vel_kix = 0;
  _params.vel_kiy = 0;
  _params.vel_kiz = 0;
  _params.vel_mg = 0;
  _params.tilt_max = 0;
  _params.thrust_max = 0;
  _params.thrust_min = 0;
  _params.hovermode = 0;
  _params.ctrlmode = 0;
  _params.holdmodeu = 0;
  _params.holdmoded = 0;
  _params.haltmode = 0;

  _params_handles.vel_rc_channel =    param_find("VEL_EXT_CHN");
  _params_handles.vel_rc_threshold =  param_find("VEL_EXT_THR");
  _params_handles.vel_kpx =           param_find("MC_VEL_XP");
  _params_handles.vel_kpy =           param_find("MC_VEL_YP");
  _params_handles.vel_kpz =           param_find("MC_VEL_ZP");
  _params_handles.vel_kix =           param_find("MC_VEL_XI");
  _params_handles.vel_kiy =           param_find("MC_VEL_YI");
  _params_handles.vel_kiz =           param_find("MC_VEL_ZI");
  _params_handles.vel_mg =            param_find("MC_VEL_MG");
  _params_handles.tilt_max =          param_find("MPC_TILTMAX_AIR");
  _params_handles.thrust_max =        param_find("MPC_THR_MAX");
  _params_handles.thrust_min =        param_find("MPC_THR_MIN");
  _params_handles.hovermode =         param_find("TXC_HOVER");
  _params_handles.ctrlmode =          param_find("TXC_CTRL");
  _params_handles.holdmodeu =         param_find("TXC_HOLD_U");
  _params_handles.holdmoded =         param_find("TXC_HOLD_D");
  _params_handles.haltmode =          param_find("TXC_HALT");

  vel_intx = 0;
  vel_inty = 0;
  vel_intz = 0;

  parameters_update();
}
Velocity_Control_Class::~Velocity_Control_Class()
{
  // If task thread is active, delete it
  if (_control_task != -1)
  {
    // End task
    _task_should_exit = true;
    // Try to end it
    unsigned i = 0;
    do
    {
      // Wait 20ms
      usleep(20000);
      // If it is not responding to quit command, just do it
      if (++i < 50)
      {
        px4_task_delete(_control_task);
        break;
      }
    }
    while (_control_task != -1);
  }
}

int
Velocity_Control_Class::parameters_update()
{
  param_get(_params_handles.vel_kpx, &(_params.vel_kpx));
  param_get(_params_handles.vel_kpy, &(_params.vel_kpy));
  param_get(_params_handles.vel_kpz, &(_params.vel_kpz));
  param_get(_params_handles.vel_kix, &(_params.vel_kix));
  param_get(_params_handles.vel_kiy, &(_params.vel_kiy));
  param_get(_params_handles.vel_kiz, &(_params.vel_kiz));
  param_get(_params_handles.vel_mg, &(_params.vel_mg));

  param_get(_params_handles.vel_rc_channel, &(_params.vel_rc_channel));
  param_get(_params_handles.vel_rc_threshold, &(_params.vel_rc_threshold));
  param_get(_params_handles.tilt_max, &(_params.tilt_max));
  param_get(_params_handles.thrust_max, &(_params.thrust_max));
  param_get(_params_handles.thrust_min, &(_params.thrust_min));
  _params.thrust_max = _params.vel_mg + 0.4f;
  _params.thrust_min = _params.vel_mg - 0.3f;


  param_get(_params_handles.hovermode, &(_params.hovermode));
  param_get(_params_handles.ctrlmode, &(_params.ctrlmode));
  param_get(_params_handles.holdmodeu, &(_params.holdmodeu));
  param_get(_params_handles.holdmoded, &(_params.holdmoded));
  param_get(_params_handles.haltmode, &(_params.haltmode));

  return OK;
}

void
Velocity_Control_Class::parameter_update_poll()
{
  bool updated;

  /* Check if parameters have changed */
  orb_check(_params_sub, &updated);

  if (updated) {
    struct parameter_update_s param_update;
    orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
    parameters_update();
  }

}

void    
Velocity_Control_Class::vehicle_attitude_poll()
{
  bool updated;

  /* check if there is a new message */
  orb_check(_attitude_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(vehicle_attitude), _attitude_sub, &_attitude);
  }
}

void    
Velocity_Control_Class::velocity_filter_poll()
{
  bool updated;

  /* check if there is a new message */
  orb_check(_v_filt_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(velocity_filter), _v_filt_sub, &_v_filt);
  }
}

void    
Velocity_Control_Class::external_command_poll()
{
  bool updated;

  /* check if there is a new message */
  orb_check(_ext_cmd_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(external_cmd), _ext_cmd_sub, &_ext_cmd);
  }
}

void
Velocity_Control_Class::vehicle_rc_poll()
{
  bool updated;

  /* Check RC state if vehicle status has changed */
  orb_check(_v_rc_channels_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(rc_channels), _v_rc_channels_sub, &_v_rc_channels);
  }
}

void
Velocity_Control_Class::vehicle_control_mode_poll()
{
  bool updated;

  /* Check if vehicle control mode has changed */
  orb_check(_v_control_mode_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
  }
}



// Velocity_Control_Class: Starter
int
Velocity_Control_Class::start()
{
	ASSERT(_control_task == -1);

	// Start the task and create the thread
	_control_task = px4_task_spawn_cmd("velocity_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&Velocity_Control_Class::task_main_trampoline,
					   nullptr);

  // Check to make sure the task thread was created successfully
	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}
// Velocity_Control_Class: main task and trampoline
void
Velocity_Control_Class::task_main_trampoline(int argc, char *argv[])
{
  vel_ctrl::g_vel_ctrl->task_main();
}
void
Velocity_Control_Class::task_main()
{
  /*
   * do subscriptions
   */
  _params_sub =         orb_subscribe(ORB_ID(parameter_update));
  _attitude_sub =       orb_subscribe(ORB_ID(vehicle_attitude));
  _v_filt_sub =         orb_subscribe(ORB_ID(velocity_filter));
  _ext_cmd_sub =        orb_subscribe(ORB_ID(external_cmd));
  _v_rc_channels_sub =  orb_subscribe(ORB_ID(rc_channels));
  _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

  /* initialize parameters cache */
  parameters_update();

  uint64_t last_run = hrt_absolute_time();
  float dt;

  while(!_task_should_exit)
  {
    // time derivative (and break if too short)
    dt = (hrt_absolute_time() - last_run) / 1000000.0f;
    if (dt < 0.002f)
    { 
      usleep(2000);
      continue; 
    } //  break if less than 2ms
    last_run = hrt_absolute_time();

    // Update polls for data
    parameter_update_poll();
    vehicle_attitude_poll();
    velocity_filter_poll();
    external_command_poll();
    vehicle_rc_poll();
    vehicle_control_mode_poll();


    Vector3f eul_sp; // = R.to_euler();

    /* If External Velocity mode selected */
    // PX4_WARN("rc, armed, ext:    %d, %d, %d",_v_rc_channels.channels[_params.vel_rc_channel-1]>_params.vel_rc_threshold, _v_control_mode.flag_armed, _ext_cmd.cmd_valid);
    if (_v_rc_channels.channels[_params.vel_rc_channel-1]>_params.vel_rc_threshold  &&  _v_control_mode.flag_armed && _ext_cmd.cmd_valid)
    {
      // Velocity Errors
      float vbx_m, vby_m, viz_m;
      if (_ext_cmd.cmd == _params.hovermode)
      {
        vbx_m =   _ext_cmd.vbx_tx2;
        vby_m =   _ext_cmd.vby_tx2;
        viz_m =   _v_filt.viz_est;  // use baro
      }
      else
      {
        vbx_m =   _v_filt.vbx_est;
        vby_m =   _v_filt.vby_est;
        viz_m =   _v_filt.viz_est;
      }
 

      float vx_error =  _ext_cmd.vx - vbx_m;
      float vy_error =  _ext_cmd.vy - vby_m;
      float vz_error =  _ext_cmd.vz - viz_m; // Use inertial velocity

      // Increase Integrals
      vel_intx += dt*vx_error;
      vel_inty += dt*vy_error;
      vel_intz += dt*vz_error;

      // Calculate Attitude Setpoints
      float tilt_rad_max = _params.tilt_max*3.1415f/180.0f;
      eul_sp(1) =   -_params.vel_kpx*vx_error - _params.vel_kix*vel_intx;
      if (eul_sp(1)>tilt_rad_max){ eul_sp(1) = tilt_rad_max; }
      if (eul_sp(1)<-tilt_rad_max){ eul_sp(1) = -tilt_rad_max; }

      eul_sp(0) =   _params.vel_kpy*vy_error + _params.vel_kiy*vel_inty;
      if (eul_sp(0)>tilt_rad_max){ eul_sp(0) = tilt_rad_max; }
      if (eul_sp(0)<-tilt_rad_max){ eul_sp(0) = -tilt_rad_max; }

      eul_sp(2) =    _ext_cmd.yaw_sp;
      // eul_sp(2) = eul_sp(2);

      // Calculate Thrust Setpoint
      float _thrust_sp =  _params.vel_mg - _params.vel_kpz*vz_error - _params.vel_kiz*vel_intz; //
      if (_thrust_sp>_params.thrust_max){ _thrust_sp = _params.thrust_max; }
      if (_thrust_sp<_params.thrust_min){ _thrust_sp = _params.thrust_min; }


      // TEST
      // PX4_WARN("is at vel ctrl");
      // eul_sp(0) = 0.573f;

      // uORB setup for publish
      _v_ctrl.roll_d =    eul_sp(0);
      _v_ctrl.pitch_d =   eul_sp(1);
      _v_ctrl.yaw_d =     eul_sp(2);
      _v_ctrl.thrust_d =  _thrust_sp;

      // If halt mode active
      if(_ext_cmd.cmd == _params.haltmode)
      {
      	_v_ctrl.roll_d =    0;
      	_v_ctrl.pitch_d =   0;
     	_v_ctrl.yaw_d =     eul_sp(2);
      	vel_intx = 0;
      	vel_inty = 0;
      	vel_intz = 0;
      	_v_ctrl.thrust_d = _params.thrust_min;
      }


    }
    else  // Clear integrals if External Velocity Mode not set
    {
      vel_intx = 0;
      vel_inty = 0;
      vel_intz = 0;

      memset(&_v_ctrl, 0, sizeof(_v_ctrl));
      _v_ctrl.thrust_d = _params.thrust_min;

    }

    // PX4_WARN("e0 %0.3f", (double)_v_ctrl.roll_d);

    // Publish
    if (_v_ctrl_pub != nullptr) {
      orb_publish(ORB_ID(velocity_control), _v_ctrl_pub, &_v_ctrl);
    } else {
      _v_ctrl_pub = orb_advertise(ORB_ID(velocity_control), &_v_ctrl);
    }

    usleep(5000);

  }

  return;
}






// MAIN
int velocity_control_main(int argc, char *argv[])
{
  // Check for command
  if (argc < 2)
  {
    warnx("usage: velocity_control {start|stop|status}");
    return 1;
  }

  // Start?
  if (!strcmp(argv[1], "start"))
  {
    // Check if it is already running
    if (vel_ctrl::g_vel_ctrl != nullptr)
    {
      warnx("already running");
      return 1;
    }
    // Init Object (via namespace)
    vel_ctrl::g_vel_ctrl = new Velocity_Control_Class;
    // Check if initialised with memory allocated
    if (vel_ctrl::g_vel_ctrl == nullptr)
    {
			warnx("alloc failed");
			return 1;
		}
    // Run it, and check to make sure it is OK
    if (OK != vel_ctrl::g_vel_ctrl->start())
    {
			delete vel_ctrl::g_vel_ctrl;
			vel_ctrl::g_vel_ctrl = nullptr;
			warnx("start failed");
			return 1;
		}
    return 0;
  }

  // Status?
  if (!strcmp(argv[1], "status"))
  {
    if (vel_ctrl::g_vel_ctrl)
    { warnx("running"); }
    else
    { warnx("stopped"); }
    return 0;
  }

  // Stop?
  if (!strcmp(argv[1], "stop"))
  {
    // Check if it is running
		if (vel_ctrl::g_vel_ctrl == nullptr)
    {
			warnx("not running");
			return 1;
		}
    // Delete if Stop command used
		delete vel_ctrl::g_vel_ctrl;
		vel_ctrl::g_vel_ctrl = nullptr;
		return 0;
	}

  // If command not recognised, state it
  warnx("unrecognized command");
	return 1;
}

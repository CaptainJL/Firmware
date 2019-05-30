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
#include <matrix/matrix/math.hpp>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <platforms/px4_defines.h>

#include "systemlib/err.h"

// UORB
#include <uORB/uORB.h>
#include <uORB/topics/external_cmd.h>
#include <uORB/topics/actuator_controls.h>

// Namespaces
using namespace std;

// MAIN for gymbal_ctrl - must be included to choose 'main' function
extern "C" __EXPORT int gymbal_ctrl_main(int argc, char *argv[]);


// Gymbal_CTRL_Class Definition
class Gymbal_CTRL_Class
{
public:
  // Cosntructor and Deconstructor
  Gymbal_CTRL_Class();
  ~Gymbal_CTRL_Class();   // Also kills task
  // Start
  int start();

private:
  // Handles
  bool	_task_should_exit;		/**< if true, task_main() should exit */
  int	_control_task;			/**< task handle */

  // external command reference
  
  int _ext_cmd_sub;
  orb_advert_t _alt_acts_pub;
  orb_advert_t _actuators_pub;

  struct external_cmd_s         _ext_cmd;
  struct actuator_controls_s    _actuators;


  // Params
  param_t modeslotparam;
  int modeslot;
  param_t pitchslotparam;
  int pitchslot;
  param_t gymballockparam;
  float gymballock;
  param_t gymbalfollowparam;
  float gymbalfollow;
  param_t gymbalpitchparam;
  float gymbalpitch;

  param_t hovermodeparam;
  int hovermode;
  param_t holdmodeuparam;
  int holdmodeu;
  param_t holdmodedparam;
  int holdmoded;
  param_t ctrlmodeparam;
  int ctrlmode;
  param_t haltmodeparam;
  int haltmode;

  // Task Handle
  static void	task_main_trampoline(int argc, char *argv[]);
  void task_main();
};
// Namespace used for memory purposes on class assignment
namespace gymbal_ns
{
  // Make class pointer
	Gymbal_CTRL_Class	*g_gymbal_ns;
}





// Gymbal_CTRL_Class
// Gymbal_CTRL_Class: Constructor and Desconstructor
Gymbal_CTRL_Class::
Gymbal_CTRL_Class():

  _task_should_exit(false),
  _control_task(-1),

  _ext_cmd_sub(-1),
  _actuators_pub(nullptr),

  _ext_cmd{},
  _actuators{}
{

	  modeslotparam = 	param_find("GB_SLOT_MODE");
  	param_get(modeslotparam, &modeslot);
  	pitchslotparam = 	param_find("GB_SLOT_PITCH");
  	param_get(pitchslotparam, &pitchslot);
    gymballockparam = 	param_find("GB_MODE_LOCK");
  	param_get(gymballockparam, &gymballock);
    gymbalfollowparam = 	param_find("GB_MODE_FOLLOW");
  	param_get(gymbalfollowparam, &gymbalfollow);
    gymbalpitchparam = 	param_find("GB_MODE_PITCH");
  	param_get(gymbalpitchparam, &gymbalpitch);

    hovermodeparam =  param_find("TXC_HOVER");
    param_get(hovermodeparam, &hovermode);
    ctrlmodeparam =  param_find("TXC_CTRL");
    param_get(ctrlmodeparam, &ctrlmode);
    holdmodeuparam =  param_find("TXC_HOLD_U");
    param_get(holdmodeuparam, &holdmodeu);
    holdmodedparam =  param_find("TXC_HOLD_D");
    param_get(holdmodedparam, &holdmoded);    
    haltmodeparam =  param_find("TXC_HALT");
    param_get(haltmodeparam, &haltmode);   
}
Gymbal_CTRL_Class::
~Gymbal_CTRL_Class()
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

// Gymbal_CTRL_Class: Starter
int
Gymbal_CTRL_Class::
start()
{
	ASSERT(_control_task == -1);

	// Start the task and create the thread
	_control_task = px4_task_spawn_cmd("gymbal_ctrl",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&Gymbal_CTRL_Class::task_main_trampoline,
					   nullptr);

  // Check to make sure the task thread was created successfully
	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}
// Gymbal_CTRL_Class: main task and trampoline
void
Gymbal_CTRL_Class::
task_main_trampoline(int argc, char *argv[])
{
  gymbal_ns::g_gymbal_ns->task_main();
}
void
Gymbal_CTRL_Class::
task_main()
{
  	// subscribe to external commander topic
  	_ext_cmd_sub = orb_subscribe(ORB_ID(external_cmd));

  	PX4_WARN("Gymbal Control Started!");

    int cmd_old = -1;


  	while(!_task_should_exit)
  	{
        // check for cmd update
      bool updated;
      orb_check(_ext_cmd_sub, &updated);
      if (updated) 
      {
        orb_copy(ORB_ID(external_cmd), _ext_cmd_sub, &_ext_cmd);
      }

      if (cmd_old != _ext_cmd.cmd)
      {
        PX4_WARN("New gymbal mode: %d", _ext_cmd.cmd);
        cmd_old = _ext_cmd.cmd;
      }


      if (_ext_cmd.cmd_valid && _ext_cmd.cmd==ctrlmode)
      { 
        _actuators.control[0] = 0;
        _actuators.control[1] = 0;
      }
      else if (_ext_cmd.cmd_valid && _ext_cmd.cmd==hovermode)
      {
        _actuators.control[0] = -1.0f;
        _actuators.control[1] = 1.0f;
      }
      else if (_ext_cmd.cmd_valid && _ext_cmd.cmd==holdmodeu)
      {
        _actuators.control[0] = 0;
        _actuators.control[1] = 0;
      }
      else if (_ext_cmd.cmd_valid && _ext_cmd.cmd==holdmoded)
      {
        _actuators.control[0] = -1.0f;
        _actuators.control[1] = 1.0f;
      }
      else if (_ext_cmd.cmd_valid && _ext_cmd.cmd==haltmode)
      {
        _actuators.control[0] = -1.0f;
        _actuators.control[1] = 1.0f;
      }
      else
      {
        _actuators.control[0] = 0;
        _actuators.control[1] = 0;
      }



      if (_actuators_pub != nullptr) {
        orb_publish(ORB_ID(actuator_controls_2), _actuators_pub, &_actuators);
      } else {
        _actuators_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);
      }

      usleep(10000);
	  }
}





// MAIN
int gymbal_ctrl_main(int argc, char *argv[])
{
  // Check for command
  if (argc < 2)
  {
    warnx("usage: gymbal_ctrl {start|stop|status}");
    return 1;
  }

  // Start?
  if (!strcmp(argv[1], "start"))
  {
    // Check if it is already running
    if (gymbal_ns::g_gymbal_ns != nullptr)
    {
      warnx("already running");
      return 1;
    }
    // Init Object (via namespace)
    gymbal_ns::g_gymbal_ns = new Gymbal_CTRL_Class;
    // Check if initialised with memory allocated
    if (gymbal_ns::g_gymbal_ns == nullptr)
    {
			warnx("alloc failed");
			return 1;
		}
    // Run it, and check to make sure it is OK
    if (OK != gymbal_ns::g_gymbal_ns->start())
    {
			delete gymbal_ns::g_gymbal_ns;
			gymbal_ns::g_gymbal_ns = nullptr;
			warnx("start failed");
			return 1;
		}
    return 0;
  }

  // Status?
  if (!strcmp(argv[1], "status"))
  {
    if (gymbal_ns::g_gymbal_ns)
    { warnx("running"); }
    else
    { warnx("stopped"); }
    return 0;
  }

  // Stop?
  if (!strcmp(argv[1], "stop"))
  {
    // Check if it is running
		if (gymbal_ns::g_gymbal_ns == nullptr)
    {
			warnx("not running");
			return 1;
		}
    // Delete if Stop command used
		delete gymbal_ns::g_gymbal_ns;
		gymbal_ns::g_gymbal_ns = nullptr;
		return 0;
	}

  // If command not recognised, state it
  warnx("unrecognized command");
	return 1;
}

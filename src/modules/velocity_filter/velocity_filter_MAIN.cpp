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

// UORB INCLUDES
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/velocity_filter.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/parameter_update.h>







// Namespaces
using namespace std;
using namespace matrix;

// MAIN for velocity_filter - must be included to choose 'main' function
extern "C" __EXPORT int velocity_filter_main(int argc, char *argv[]);

// Defines
#define G 9.81f



// Vel_Filter_Class Definition
class Vel_Filter_Class
{
public:
	// Constructor and Deconstructor
	Vel_Filter_Class();
	~Vel_Filter_Class();   // Also kills task
	// Start - returns OK if succesful
	int start();


	private:
	// Task handles
	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	// UORB subcriptions
	int		_params_sub{-1};			/**< parameter updates subscription */
	int 	_actuators_state_sub{-1};	/**< actuator state subcription */
	int 	_sensors_sub{-1};			/**< sensor output subcription */
	int 	_attitude_sub{-1};			/**< vehicle attitude subcription */
	int 	_ctrl_mode_sub{-1};			/**< control mode subcription */
	int 	_global_pos_sub{-1};		/**< global position subcription */
	int 	_local_pos_sub{-1};			/**< local position subcription */
	int 	_airdata_sub{-1};
	// UORB publishers
	orb_advert_t	_vel_filter_pub;
	// UORB structures
	struct 	actuator_armed_s			_actuators_state;	/**< actuator state */
	struct 	sensor_combined_s			_sensors;			/**< sensor output */
	struct 	vehicle_attitude_s			_attitude;			/**< vehicle attitude */
	struct 	vehicle_control_mode_s		_ctrl_mode;			/**< control mode */
	struct 	vehicle_global_position_s	_global_pos;		/**< global position */
	struct 	vehicle_local_position_s	_local_pos;			/**< local position */
	struct 	velocity_filter_s			_vel_filter;		/**< velocity filter */
	struct 	vehicle_air_data_s 			_airdata;

	// Parameter Handling
	struct {
		param_t	kv1;
		param_t kv2;
		param_t	kz1;
		param_t	kz2;
		param_t cbar[2];
		param_t board_rotation;
		param_t board_offset[3];
	}		_params_handles;		/**< handles for interesting parameters */

	// Parameter Values
	struct {
		float kv1;
		float kv2;
		float kz1;
		float kz2;
		float cbar[2];
		int board_rotation;
		float board_offset[3];
	}		_params;


	// Values
	Dcmf  _I;			// Identity matrix 
	Dcmf  _R;			// Rotation matrix
	Dcmf  _R_off;		// Rotation matrix offset
	Dcmf  _P;			// Projection matrix
	Dcmf  _RP;		// Rotated projection matri
	Vector3f		_accel;		// accelerometer
	Matrix<float,3,1>	_e3;	// e_3
	Vector3f		_vi_est_prev;		
	float 				_z_baro_prev;

	// Offsets
	Vector3f		_accel_offsets;
	float 				_baro_offset;



	float _dt;
	uint64_t _time_p;
	bool _armed_last;


	// Parameter and UORB polling
	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();		

	/**
	 * Check state arming state of actuators
	 */
	void		actuators_state_poll();

	/**
	 * Update sensor readings
	 */
	void 		sensors_poll();

	/**
	 * Update vehicle attitude
	 */
	void		vehicle_attitude_poll();

	/**
	 * Check control mode selected
	 */
	void		control_mode_poll();

	/**
	 * Check position in global environment
	 */
	void		global_position_poll();

	/**
	 * Check position in local environment
	 */
	void		local_position_poll();

	/**
	 * Check barometer
	 */
	void		airdata_poll();

	// Task Handle
	static void	task_main_trampoline(int argc, char *argv[]);
	void task_main();
};
	// Namespace used for memory purposes on class assignment
namespace velfilter
{
	// Make class pointer
	Vel_Filter_Class	*g_velfilter;
}





// Vel_Filter_Class
// Vel_Filter_Class: Constructor and Desconstructor
Vel_Filter_Class::Vel_Filter_Class() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */

	/* publications */
	_vel_filter_pub(nullptr),
	/* structures */
	_actuators_state{},	
	_sensors{},			
	_attitude{},	
	_ctrl_mode{},	
	_global_pos{},	
	_local_pos{},	
	_vel_filter{}


{

	// Parameter Handling
	_params_handles.kv1				= param_find("VEL_FILT_KV1");
	_params_handles.kv2				= param_find("VEL_FILT_KV2");
	_params_handles.kz1				= param_find("VEL_FILT_KZ1");
	_params_handles.kz2				= param_find("VEL_FILT_KZ2");
	_params_handles.cbar[0]			= param_find("VEL_FILT_CBARX");
	_params_handles.cbar[1]			= param_find("VEL_FILT_CBARY");
	_params_handles.board_rotation 	= param_find("SENS_BOARD_ROT");		// Used if flight controller is not rotated as quadrotor
	_params_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");

	// Variable Init
	_I.identity();
	_R.identity();
	_P.identity();
	_RP.identity();
	_accel.zero();		
	_e3.zero();			_e3(2,0) = 1;
	_vi_est_prev.zero();	
	_z_baro_prev = 0;

	_R_off.identity();
	_accel_offsets.zero();
	_baro_offset = 0;


	_dt = 0;
	_time_p = 0;
	_armed_last = false;

	_P = _I - _e3*_e3.transpose();





}
Vel_Filter_Class::~Vel_Filter_Class()
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
Vel_Filter_Class::parameters_update()
{
	// Filter Gains
	param_get(_params_handles.kv1, &(_params.kv1));
	param_get(_params_handles.kv2, &(_params.kv2));
	param_get(_params_handles.kz1, &(_params.kz1));
	param_get(_params_handles.kz2, &(_params.kz2));

	// Drag Coefficients
	param_get(_params_handles.cbar[0], &(_params.cbar[0]));
	param_get(_params_handles.cbar[1], &(_params.cbar[1]));

	// Board Rotation
	param_get(_params_handles.board_rotation, &(_params.board_rotation));
	param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
	param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
	param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

	return OK;
}

void
Vel_Filter_Class::parameter_update_poll()
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
Vel_Filter_Class::actuators_state_poll()
{
	bool updated;

	orb_check(_actuators_state_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _actuators_state_sub, &_actuators_state);
	}
}
	
void	
Vel_Filter_Class::sensors_poll()
{
	bool updated;

	orb_check(_sensors_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_combined), _sensors_sub, &_sensors);
	}
}

void	
Vel_Filter_Class::vehicle_attitude_poll()
{
	bool updated;

	orb_check(_attitude_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _attitude_sub, &_attitude);
	}
}

void	
Vel_Filter_Class::control_mode_poll()
{
	bool updated;

	orb_check(_ctrl_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _ctrl_mode_sub, &_ctrl_mode);
	}
} 

void	
Vel_Filter_Class::global_position_poll()
{
	bool updated;

	orb_check(_global_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
} 

void	
Vel_Filter_Class::local_position_poll()
{
	bool updated;

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
} 

void	
Vel_Filter_Class::airdata_poll()
{
	bool updated;

	orb_check(_airdata_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_air_data), _airdata_sub, &_airdata);
	}
} 




// Vel_Filter_Class: Starter
int
Vel_Filter_Class::start()
{
	ASSERT(_control_task == -1);

	// Start the task and create the thread
	_control_task = px4_task_spawn_cmd("velocity_filter",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&Vel_Filter_Class::task_main_trampoline,
					   nullptr);

  // Check to make sure the task thread was created successfully
	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}
// Vel_Filter_Class: main task and trampoline
void
Vel_Filter_Class::task_main_trampoline(int argc, char *argv[])
{
	velfilter::g_velfilter->task_main();
}
void
Vel_Filter_Class::task_main()
{
	/*
	 * do subscriptions
	 */
	_params_sub = 			orb_subscribe(ORB_ID(parameter_update));
	_actuators_state_sub = 	orb_subscribe(ORB_ID(actuator_armed));	
	_sensors_sub = 			orb_subscribe(ORB_ID(sensor_combined));
	_attitude_sub = 		orb_subscribe(ORB_ID(vehicle_attitude));
	_ctrl_mode_sub = 		orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = 		orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = 		orb_subscribe(ORB_ID(vehicle_local_position));
	_airdata_sub = 			orb_subscribe(ORB_ID(vehicle_air_data));

	/* initialize parameters cache */
	parameters_update();


	/* fine tune the rotation */
	// Vector3f _R_off_v3(M_DEG_TO_RAD_F * _params.board_offset[0],
	// 				 M_DEG_TO_RAD_F * _params.board_offset[1],
	// 				 M_DEG_TO_RAD_F * _params.board_offset[2]);
	Eulerf _R_off_eul(M_DEG_TO_RAD_F * _params.board_offset[0],
					 M_DEG_TO_RAD_F * _params.board_offset[1],
					 M_DEG_TO_RAD_F * _params.board_offset[2]);

	_R_off = _R_off_eul;

	// Baro init
	float _baro_init = 0;
	for(int i = 0; i < 100; i++)	// 1s baro average
	{
		sensors_poll();
		_baro_init += _airdata.baro_alt_meter;
		usleep(10000);
	}
	_baro_offset = _baro_init/100.0f;
	warnx("baro offset: %0.6f", (double)_baro_offset);





	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	_time_p = hrt_absolute_time();
	// Run looping program parts in 'while' loop
	while(!_task_should_exit)
	{

		poll_fds.fd = _attitude_sub;

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for _task_should_exit */
		/*if (pret == 0) {
			continue;
		}*/


		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("velocity_filter: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		/* poll subscription updates */
		parameter_update_poll();		
		actuators_state_poll();
		sensors_poll();
		vehicle_attitude_poll();
		control_mode_poll();
		global_position_poll();
		local_position_poll();
		airdata_poll();

		// Time references
		_dt = (hrt_absolute_time() - _time_p) / 1000000.0f;
		Vector3f _dt_vec(_dt,_dt,_dt);
		_time_p = hrt_absolute_time();
		// In case dt = 0
		if (_dt <= 0)
		{
			usleep(1000);
			continue;
		}


		// If recently armed, calibrate baro offsets
		// if (!_armed_last && _actuators_state.armed)
		// {
		// 	// Baro init
		// 	_baro_init = 0;
		// 	for(int i = 0; i < 50; i++)	// 1s baro average
		// 	{
		// 		sensors_poll();
		// 		_baro_init += _sensors.baro_alt_meter;
		// 		usleep(10000);
		// 	}
		// 	_baro_offset = _baro_init/100.0f;
		// 	warnx("baro offset: %0.6f", (double)_baro_offset);
		// }


		// Matrix manipulations to make it work
		Vector3f _ge3(0,0,G);
		Vector3f _k1v_vec(_params.kv1,_params.kv1,0);
		Vector3f _k2v_vec(_params.kv2,_params.kv2,0);
		Vector3f _k2z_vec(0,0,_params.kz2);

		// Get current rotation matrix
		Quatf _q_att(_attitude.q[0],_attitude.q[1],_attitude.q[2],_attitude.q[3]);
		_R = _q_att.to_dcm();

		// Rotated Projection matrix
		matrix::Matrix<float, 3, 1> _Re3 = _R*_e3;
		_RP = _I - _Re3*_Re3.transpose();

		// Accelerometer measurements + board rotation compensation
		Vector3f _accel_adj;
		_accel_adj(0) = _sensors.accelerometer_m_s2[0];
		_accel_adj(1) = _sensors.accelerometer_m_s2[1];
		_accel_adj(2) = _sensors.accelerometer_m_s2[2];
		_accel = _R_off*_accel_adj;

		// Raw Aerodynamic Velocity Measurement
		Vector3f Aw(_params.cbar[0]*_accel(0)/_accel(2), _params.cbar[1]*_accel(1)/_accel(2), 0);

		// GPS/VICON measurement
		Vector3f Au(_local_pos.vx, _local_pos.vy, 0);


		// warnx("acc_z: %0.6f", (double)_accel(2));

		// Innovations (Delta)
		Vector3f _DeltaV1 = 		_RP*_vi_est_prev - _R*_P*Aw;
		Vector3f _DeltaV2 = 		_P*_vi_est_prev - Au;
		float DeltaZ = 					_z_baro_prev - (-_airdata.baro_alt_meter+_baro_offset);
		//warnx("baro %f ", (double)(-_sensors.baro_alt_meter+_baro_offset));
		Vector3f _DeltaZ_e3(0,0,DeltaZ);


		// Derivative Products of Inertial Filter
		Vector3f _dv_est = 	_ge3 + _R*_accel - _k1v_vec.emult(_DeltaV1) - _k2v_vec.emult(_DeltaV2) - _k2z_vec.emult(_DeltaZ_e3);
		float _dz_baro =			_vi_est_prev(2) - _params.kz1*DeltaZ;
		// _R**_accel


		// Update filter estimates
		Vector3f _vi_est = 		_dt_vec.emult(_dv_est) + _vi_est_prev;
		float _z_baro =					_dt*_dz_baro + _z_baro_prev;


		/* temp for yani */
		/*_vi_est(0) = _local_pos.vx;
		_vi_est(1) = _local_pos.vy;
		_vi_est(2) = _local_pos.vz;*/

		// BOdy-fixed velocity
		Vector3f _vb_est = 		_R.transpose()*_vi_est; //_R.transposed()*


		// Handle if not armed
		if (!_actuators_state.armed)
		{
			
			_vel_filter = {0};
			_vi_est.zero();
			_vb_est.zero();
			_z_baro = 0;
			_dz_baro = 0;
			_baro_offset = _airdata.baro_alt_meter;
		}
		// Update UORB
		_vel_filter.vix_est =	_vi_est(0);
		_vel_filter.viy_est =	_vi_est(1);
		_vel_filter.viz_est =	_vi_est(2);
		_vel_filter.vbx_est =	_vb_est(0);
		_vel_filter.vby_est =	_vb_est(1);
		_vel_filter.vbz_est =	_vb_est(2);
		_vel_filter.vbx_aero =  Aw(0);
		_vel_filter.vby_aero =  Aw(1);
		_vel_filter.z_baro =	_z_baro;
		_vel_filter.vz_baro = 	_dz_baro;





		// UORB publish
		if (_vel_filter_pub != nullptr) {
			orb_publish(ORB_ID(velocity_filter), _vel_filter_pub, &_vel_filter);
		} else {
			_vel_filter_pub = orb_advertise(ORB_ID(velocity_filter), &_vel_filter);
		}

		// Update values
		_vi_est_prev = 	_vi_est;
		_z_baro_prev =	_z_baro;

		//warnx("fuck it should be working");
		_armed_last = _actuators_state.armed;
		usleep(1000);

	}

	return;
}















// MAIN
int velocity_filter_main(int argc, char *argv[])
{
  // Check for command
  if (argc < 2)
  {
    warnx("usage: velocity_filter {start|stop|status}");
    return 1;
  }

  // Start?
  if (!strcmp(argv[1], "start"))
  {
    // Check if it is already running
    if (velfilter::g_velfilter != nullptr)
    {
      warnx("already running");
      return 1;
    }
    // Init Object (via namespace)
    velfilter::g_velfilter = new Vel_Filter_Class;
    // Check if initialised with memory allocated
    if (velfilter::g_velfilter == nullptr)
    {
			warnx("alloc failed");
			return 1;
		}
    // Run it, and check to make sure it is OK
    if (OK != velfilter::g_velfilter->start())
    {
			delete velfilter::g_velfilter;
			velfilter::g_velfilter = nullptr;
			warnx("start failed");
			return 1;
		}
    return 0;
  }

  // Status?
  if (!strcmp(argv[1], "status"))
  {
    if (velfilter::g_velfilter)
    { warnx("running"); }
    else
    { warnx("stopped"); }
    return 0;
  }

  // Stop?
  if (!strcmp(argv[1], "stop"))
  {
    // Check if it is running
		if (velfilter::g_velfilter == nullptr)
    {
			warnx("not running");
			return 1;
		}
    // Delete if Stop command used
		delete velfilter::g_velfilter;
		velfilter::g_velfilter = nullptr;
		return 0;
	}

  // If command not recognised, state it
  warnx("unrecognized command");
	return 1;
}

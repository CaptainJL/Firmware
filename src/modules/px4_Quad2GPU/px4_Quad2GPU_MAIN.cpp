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

/**
 * @file px4_Quad2GPU.cpp
 * UART/Serial Commuications channel to NVIDIA Tegra Mobile Development GPU
 *
 * @author Jean-Luc Stevens <Jean-Luc.Stevens@anu.edu.au>
**/

// INCLUDES
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <stdio.h> // standard input / output functions
#include <stdlib.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <sys/stat.h>
#include <sys/types.h>
#include <drivers/drv_hrt.h>
//#include <sys/fcntl.h>
#include <sys/time.h>
#include <pthread.h>
#include <cstdlib>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/velocity_filter.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/external_cmd.h>
#include <uORB/topics/vehicle_control_mode.h>


// Definitions
#define TELEM_FREQ 1	// TELEM_FREQ/2 reads and writes per second
#define BAUD_RATE B921600 //B115200
#define PORT_NAME "/dev/ttyS2"
#define SERIAL_HEADER

// Namespaces
using namespace std;

// MAIN for PX4
extern "C" __EXPORT int px4_Quad2GPU_main(int argc, char *argv[]);


// Serial_Port Definition
class Serial_Port
{
public:
  // Cosntructor and Deconstructor
  Serial_Port();
  ~Serial_Port();   // Also kills task
  // Start
  int start();
  int mode;

private:
  // Handles
  bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

  // Ports
  void _open_port();
  void _close_port();
  void _configure_port();
  int _port_id;
  bool _port_status;
  uint8_t *wBuffer;
  uint8_t *rBuffer;
  uint8_t *rBuffTemp;

  // Port IO
  uint32_t _serial_header;
  void _write_port();
  float _interpret_data(uint8_t);
  void _read_port();
  void _buffer_print(uint32_t, uint8_t); //uint8_t *wBuffer[],
  void _timeout(float);

  // pthread control
  pthread_mutex_t _pthread_lock;

  // UORB
  int	_v_att_sub;
  int _v_filt_sub;
  orb_advert_t	_lpos_sp_pub;
  orb_advert_t  _ext_cmd_pub;
  orb_advert_t  _v_mode_pub;

  struct vehicle_attitude_s                 _v_att;
  struct vehicle_local_position_setpoint_s  _lpos_sp;
  struct external_cmd_s                     _ext_cmd;
  struct velocity_filter_s                  _v_filt;
  struct vehicle_control_mode_s             _v_mode;

  int _ext_cmd_prev;

  // Task Handle
  static void	task_main_trampoline(int argc, char *argv[]);
  void task_main();

  // Union for read
  union i8_f
	{
		float f;
		uint32_t i;
	} v;
  // Performance Counter
  uint32_t _read_correct;
  uint32_t _read_total;
  double _read_ratio;

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

  hrt_abstime t1;
  hrt_abstime t2;

  bool first_recieved;


};
namespace serial_port
{
  // Make class pointer
	Serial_Port	*g_serial_port;
}


// Serial_Port Class
// Serial_Port: Constructor and Desconstructor
Serial_Port::
Serial_Port()
{
  _task_should_exit = false;
  _control_task = -1;
  _serial_header = 0x48234567;
  wBuffer = (uint8_t *) malloc(20*sizeof(uint32_t));
  rBuffer = (uint8_t *) malloc(10*sizeof(uint32_t));
  rBuffTemp = (uint8_t *) malloc(10*sizeof(uint32_t));
  mode = 1;
  first_recieved = false;

  pthread_mutex_init(&_pthread_lock, NULL);

  memset(&_lpos_sp, 0, sizeof(_lpos_sp));
  _lpos_sp_pub = nullptr;
  _ext_cmd_pub = nullptr;
  _v_mode_pub = nullptr;
  _v_att_sub = -1;
  _v_filt_sub = -1;

  hovermodeparam = 	param_find("TXC_HOVER");
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
Serial_Port::
~Serial_Port()
{
  if (_control_task != -1)
  {
    // End task
    _task_should_exit = true;
    // Close Port
    _close_port();
    free(wBuffer);
    free(rBuffer);
    free(rBuffTemp);
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
    pthread_mutex_destroy(&_pthread_lock);
  }
}


// Serial_Port: Open, Configure and Close Port
void
Serial_Port::
_open_port()
{
	_port_id = px4_open(PORT_NAME, O_RDWR);
	if(_port_id < 0) // if open is unsucessful
	{
		PX4_ERR("Unable to open port");
		PX4_ERR("Killing Program");
		_port_status = false;
    while(1)
      { usleep(100); }
	}
	else
	{
		fcntl(_port_id, F_SETFL, 0);
		PX4_INFO("Port is open.");
		_port_status = true;
		_configure_port();
	}
}
void
Serial_Port::
_close_port()
{
 	px4_close(_port_id);
  	PX4_WARN("Port closed");
}
// Serial_Port: Configure the port
void
Serial_Port::
_configure_port()
{
	PX4_INFO("Port is configuring...");
	struct termios config;      // structure to store the port settings in
	memset(&config, 0, sizeof(termios));
	// Set BAUD Rate
	if (cfsetispeed(&config, BAUD_RATE) < 0 || cfsetospeed(&config, BAUD_RATE) < 0)
	{
		PX4_ERR("Could not set desired baud rate");
	}
	// 8 Bits per packet
	// config.c_cflag |= CS8;
  	config.c_cflag     &=  ~PARENB;        // Make 8n
  	config.c_cflag     &=  ~CSTOPB;
  	config.c_cflag     &=  ~CSIZE;
  	config.c_cflag     |=  CS8;
  	config.c_cflag     &=  ~CRTSCTS;       // no flow control
  	config.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
  	config.c_oflag     =   0;                  // no remapping, no delays
  	config.c_cc[VMIN]      =   0;                  // read doesn't block
  	config.c_cc[VTIME]     =   5;  

    /*config.c_cflag |= CREAD | CLOCAL;
    config.c_iflag &= ~(IXON | IXOFF | IXANY);
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    config.c_oflag &= ~OPOST;*/
	// Set Attributes of the port
	if ( tcsetattr(_port_id, TCSANOW, &config) >= 0 )
	{
    // apply the settings to the port
	  PX4_INFO("Port configured and ready");
	  _port_status = true;
	}
	else
	{
	  PX4_ERR("Failed to configure");
		_port_status = false;
	}
}







// Serial_Port: Write to UART
void
Serial_Port::
_write_port()
{

  // hrt_abstime starttime = hrt_absolute_time();
  uint32_t len;

  if (mode == 1)
  {

    orb_copy(ORB_ID(velocity_filter), _v_filt_sub, &_v_filt);
    float xv =    _v_filt.vix_est;
    float yv =    _v_filt.viy_est;
    float zv =    _v_filt.viz_est;
    float baro =  _v_filt.z_baro;

    orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
    float q0 =    _v_att.q[0];
    float q1 =    _v_att.q[1];
    float q2 =    _v_att.q[2];
    float q3 =    _v_att.q[3];
    float roll =  atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    float pitch = asinf(2*(q0*q2 - q3*q1));
    float yaw =   atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

    float avg = (xv+yv+zv + roll+pitch+yaw + baro)/7.0f;
    usleep(100);
    len = 9; // # of int in buffer

    // Add to buffer
    _buffer_print(_serial_header, 0);
    _buffer_print(*(uint32_t *)&roll, 1);
    _buffer_print(*(uint32_t *)&pitch, 2);
    _buffer_print(*(uint32_t *)&yaw, 3);
    _buffer_print(*(uint32_t *)&xv, 4);
    _buffer_print(*(uint32_t *)&yv, 5);
    _buffer_print(*(uint32_t *)&zv, 6);
    _buffer_print(*(uint32_t *)&baro, 7);
    _buffer_print(*(uint32_t *)&avg, 8);

    // Write over UART
    // for(uint8_t ii = 0; ii < 44; ii++)
    // {
    //   printf("%02x", wBuffer[ii]);
    // }
    // printf("\n");

    pthread_mutex_lock(&_pthread_lock);
    px4_write(_port_id, wBuffer, len*sizeof(int));
    pthread_mutex_unlock(&_pthread_lock);

  }
  // PX4_INFO("Write_ loop time: %d", (uint32_t)hrt_elapsed_time(&starttime));

}
void
Serial_Port::
_buffer_print(uint32_t val, uint8_t cur_val)	// Print data to buffer function
{
	for (uint8_t i = 0; i < 4; i++)
	{
    wBuffer[4*cur_val+(3-i)] = (val >> (i * 8));
	}
}

void
Serial_Port::
_read_port()
{
  int byte_count = 10*sizeof(uint32_t);

  memset(rBuffTemp, 0, byte_count);
  memset(rBuffer, 0, byte_count);

  hrt_abstime time_readStart;
  time_readStart =   hrt_absolute_time();
  // PX4_INFO("time: %d",time_readStart);

  unsigned long rBufC;

  if (mode == 1)
  {
      uint8_t vh = 0;
      while(1)
      {
        // usleep(10);
        // PX4_INFO("before");
        px4_ioctl(_port_id, FIONREAD, (unsigned long)&rBufC);
        // px4_read(_port_id, &vh, 1);
        // printf("ioctl, read: %d, %d\n", rBufC, vh);
        if (rBufC>0)
        {
        	// PX4_INFO("read byte");
        	px4_read(_port_id, &vh, 1);
        }     
        rBufC=0;   
        // PX4_INFO("after");
        if( (vh == 0x48) )
        {
          // while(1)
          // {
            usleep(100);
            // px4_ioctl(_port_id, FIONREAD, rBufC);
            // if (rBufC >= 19)
            // {

              pthread_mutex_lock(&_pthread_lock);
              px4_read(_port_id, rBuffTemp, byte_count-1);
              pthread_mutex_unlock(&_pthread_lock);
              rBuffer[0] = vh;
              vh = 0;
              for (uint8_t rc = 1; rc < byte_count; rc++)
              {
                rBuffer[rc] = rBuffTemp[rc-1];
              }
              // PX4_INFO("read bytes");
              
              break;
            // }
          // }
        }

        // PX4_INFO("t= %d", (uint64_t)hrt_elapsed_time(&time_readStart));
        if (hrt_elapsed_time(&time_readStart) > 100000)
        //if ((time_readStart+100000)<hrt_absolute_time())
        {
          if (first_recieved)
          { 
            _timeout(0); 
            break;
          }
          else
          { 
          	// PX4_WARN("No serial data received!");
          	break; 
          }
        }
        // break;

        usleep(10);

      }
    }
    // PX4_INFO("interpreting bytes");
    // for (int gg = 0; gg < 28; gg++)
    // {
    //   printf("%02x", rBuffer[gg]);
    // }
    // printf("\n");
    // Analyse the data
    float sh =    	   _interpret_data(0);
    float xvi =   	   _interpret_data(1);
    float yvi =   	   _interpret_data(2);
    float zvi =   	   _interpret_data(3);
    float vbx =        _interpret_data(4);
    float vby =        _interpret_data(5);
    float viz =        _interpret_data(6); 
    float modedes =    _interpret_data(7);
    float yawdes =   	 _interpret_data(8);
    float avg =   	   _interpret_data(9);
    // Check data for errors
    float avg_check = (xvi+yvi+zvi + vbx+vby+viz + modedes+yawdes)/8.0f;
    _ext_cmd.timestamp = hrt_absolute_time();
    if ((*(uint32_t *)&sh == _serial_header) && ((avg_check+0.01f)>avg) && ((avg_check-0.01f)<avg))
    {
      // PX4_INFO("Correct Data!");
      first_recieved =      true;
      _ext_cmd.cmd_valid =  true;
      _ext_cmd.cmd =        ctrlmode; // 1 = hover
      _ext_cmd.vx =         xvi;
      _ext_cmd.vy =         yvi;
      _ext_cmd.vz =         zvi;
      _ext_cmd.vbx_tx2 =    vbx;
      _ext_cmd.vby_tx2 =    vby;
      _ext_cmd.viz_tx2 =    viz;
      // _lpos_sp.yaw =        yawdes;
      _ext_cmd.yaw_sp =     yawdes;

      if (modedes>=0 && modedes<5)
      {
        _ext_cmd.cmd = holdmodeu; 
      }
      if (modedes>=5 && modedes<10)
      {
        _ext_cmd.cmd = holdmoded; 
      }
      if (modedes<0 && modedes>-10)
      {
        _ext_cmd.cmd = hovermode; 
      }
      if (modedes<=-90 && modedes>-110)
      {
        _ext_cmd.cmd = haltmode; 
      }

      if (_ext_cmd_pub != nullptr) {
      	orb_publish(ORB_ID(external_cmd), _ext_cmd_pub, &_ext_cmd);
      } else {
        _ext_cmd_pub = orb_advertise(ORB_ID(external_cmd), &_ext_cmd);
      }
      // if (_lpos_sp_pub != nullptr) {
      // 	orb_publish(ORB_ID(vehicle_local_position_setpoint), _lpos_sp_pub, &_lpos_sp);
      // } else {
      //   _lpos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_lpos_sp);
      // }

    }
    else
    {
      // PX4_WARN("Serial data unconfirmed!");
    }
}
float
Serial_Port::
_interpret_data(uint8_t i)	// Covert Data from uint8_t to float
{
	v.i = 		0;
	v.i =  		rBuffer[0+4*i];
	v.i <<= 	8;
	v.i |= 		rBuffer[1+4*i];
	v.i <<= 	8;
	v.i |= 		rBuffer[2+4*i];
	v.i <<= 	8;
	v.i |= 		rBuffer[3+4*i];

	return v.f;
}
void
Serial_Port::
_timeout(float yawdes)
{
  bool ts_start = false;
  // while(1)
  // {
  	if (!ts_start)
  	{
  		PX4_ERR("Serial Port Timed-Out, velocity slowing to 0,0,0");
  		ts_start = true;
  	}


    _ext_cmd.vx = 0.9f*_ext_cmd.vx;
    _ext_cmd.vy = 0.9f*_ext_cmd.vy;
    _ext_cmd.vz = 0.9f*_ext_cmd.vz;
    //_lpos_sp.yaw = yawdes;
    _ext_cmd.yaw_sp = yawdes;

    if (abs(_ext_cmd.vx)< 0.2f) { _ext_cmd.vx = 0.0f; }
    if (abs(_ext_cmd.vy)< 0.2f) { _ext_cmd.vy = 0.0f; }
    if (abs(_ext_cmd.vz)< 0.2f) { _ext_cmd.vz = 0.2f; } // Land on spot

      // PX4_INFO("Correct Data!");
    _ext_cmd.cmd_valid = true;
    _ext_cmd.cmd = hovermode; // 1 = hover

    if (_ext_cmd_pub != nullptr) {
      orb_publish(ORB_ID(external_cmd), _ext_cmd_pub, &_ext_cmd);
    } else {
      _ext_cmd_pub = orb_advertise(ORB_ID(external_cmd), &_ext_cmd);
    }
    // if (_lpos_sp_pub != nullptr) {
    //   orb_publish(ORB_ID(vehicle_local_position_setpoint), _lpos_sp_pub, &_lpos_sp);
    // } else {
    //   _lpos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_lpos_sp);
    // }
    usleep(15000);
  // }
}











// Serial_Port: Starter
int
Serial_Port::
start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("px4_Quad2GPU",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&Serial_Port::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_ERR("task start failed");
		return -errno;
	}

	return OK;
}
// Serial_Port: main task and trampoline
void
Serial_Port::
task_main_trampoline(int argc, char *argv[])
{
  serial_port::g_serial_port->task_main();
}
void
Serial_Port::
task_main()
{
  // Open Port
  _open_port();
  // UORB Subscribe
  _v_att_sub =      orb_subscribe(ORB_ID(vehicle_attitude));
  _v_filt_sub=      orb_subscribe(ORB_ID(velocity_filter));
  // hrt_abstime freq_ctr;


  while(!_task_should_exit)
  {
    // freq_ctr = hrt_absolute_time();

    //t1 = hrt_absolute_time();
    _write_port();
    //PX4_INFO("Write in %dus", hrt_elapsed_time(&t1));
    //t2 = hrt_absolute_time();
    _read_port();
    //usleep(1000000);
    //PX4_INFO("Read in %dus", hrt_elapsed_time(&t2));
    // usleep(10000);
    // PX4_INFO("Loop-de-loop!");
    // PX4_INFO("Freq: %0.6f", 1000000.0/hrt_elapsed_time(&freq_ctr));
  }

  // Close Port
  _close_port();

  return;
}






// MAIN
int px4_Quad2GPU_main(int argc, char *argv[])
{
  // Check for command
  if (argc < 2)
  {
    PX4_WARN("usage: px4_Quad2GPU {start|stop|status}");
    return 1;
  }

  // Start?
  if (!strcmp(argv[1], "start"))
  {
    // Running?
    if (serial_port::g_serial_port != nullptr)
    {
      PX4_WARN("already running");
      return 1;
    }
    // Init Object
    serial_port::g_serial_port = new Serial_Port;
    // Check if initialised
    if (serial_port::g_serial_port == nullptr)
    {
			PX4_WARN("alloc failed");
			return 1;
		}
    // Run it
    if (OK != serial_port::g_serial_port->start())
    {
			delete serial_port::g_serial_port;
			serial_port::g_serial_port = nullptr;
			PX4_WARN("start failed");
			return 1;
		}
    return 0;
  }

  // Status?
  if (!strcmp(argv[1], "status"))
  {
    if (serial_port::g_serial_port)
    { PX4_WARN("running"); }
    else
    { PX4_WARN("stopped"); }
    return 0;
  }

  // Stop?
  if (!strcmp(argv[1], "stop"))
  {
    // Check if it is running
		if (serial_port::g_serial_port == nullptr)
    {
			PX4_WARN("not running");
			return 1;
		}
    // Delete if Stop command used
		delete serial_port::g_serial_port;
		serial_port::g_serial_port = nullptr;
		return 0;
	}

  // If command not recognised, state it
  PX4_WARN("unrecognized command");
	return 1;
}

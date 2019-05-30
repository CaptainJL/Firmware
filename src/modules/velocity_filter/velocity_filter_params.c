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
 * @file velocity_filter_params.c
 * Parameters for Velocity Filter
 *
 * @author Jean-Luc Stevens <Jean-Luc.Stevens@anu.edu.au>
 * @author Robert Mahony <Robert.Mahony@anu.edu.au>
 */

#include <systemlib/param/param.h>

/**
 * Velocity Filter Kv1 constant
 *
 * Adjust for Aerodynamic velocity effectiveness
 *
 * @increment 0.001
 * @group Velocity Filter
 */
PARAM_DEFINE_FLOAT(VEL_FILT_KV1, 0.5f);

/**
 * Velocity Filter Kv2 constant
 *
 * Adjust for adjusting GPS/VICON velocity effectiveness
 *
 * @increment 0.001
 * @group Velocity Filter
 */
PARAM_DEFINE_FLOAT(VEL_FILT_KV2, 0.5f);


/**
 * Velocity Filter Kz1 constant
 *
 * Adjust for z_hat component
 *
 * @increment 0.001
 * @group Velocity Filter
 */
PARAM_DEFINE_FLOAT(VEL_FILT_KZ1, 1.3f);

/**
 * Velocity Filter Kz2 constant
 *
 * Adjust for v_hat component
 *
 * @increment 0.001
 * @group Velocity Filter
 */
PARAM_DEFINE_FLOAT(VEL_FILT_KZ2, 1.0f);

/**
 * Velocity Filter Drag Coefficient X
 *
 * @increment 0.001
 * @group Velocity Filter
 */
PARAM_DEFINE_FLOAT(VEL_FILT_CBARX, 0.3f);

/**
 * Velocity Filter Drag Coefficient Y
 *
 * @increment 0.001
 * @group Velocity Filter
 */
PARAM_DEFINE_FLOAT(VEL_FILT_CBARY, 0.3f);
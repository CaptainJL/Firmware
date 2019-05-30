/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file gymbal_ctrl_params.c
 * thingo for thingy.
 *
 * @author Jean-Luc Stevens <Jean-Luc.Stevens@anu.edu.au>
 *
 */

#include <systemlib/param/param.h>

/**
 * Gymbal Mode Output
 *
 * @min 0
 * @max 7
 * @group Gymbal_CTRL
 */
PARAM_DEFINE_INT32(GB_SLOT_MODE, 5);

/**
 * Gymbal Yaw Output
 *
 * @min 0
 * @max 7
 * @group Gymbal_CTRL
 */
PARAM_DEFINE_INT32(GB_SLOT_PITCH, 6);

/**
 * Gymbal Mode Lock
 *
 * @min -1.0
 * @max 1.0
 * @group Gymbal_CTRL
 */
PARAM_DEFINE_FLOAT(GB_MODE_LOCK, -1.0f);

/**
 * Gymbal Mode Follow
 *
 * @min -1.0
 * @max 1.0
 * @group Gymbal_CTRL
 */
PARAM_DEFINE_FLOAT(GB_MODE_FOLLOW, 0.0f);

/**
 * Gymbal Mode Pitch
 *
 * @min -1.0
 * @max 1.0
 * @group Gymbal_CTRL
 */
PARAM_DEFINE_FLOAT(GB_MODE_PITCH, 1.0f);

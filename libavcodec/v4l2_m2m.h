/*
 * V4L2 mem2mem helper functions
 *
 * Copyright (C) 2017 Alexis Ballier <aballier@gentoo.org>
 * Copyright (C) 2017 Jorge Ramirez <jorge.ramirez-ortiz@linaro.org>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVCODEC_V4L2_M2M_H
#define AVCODEC_V4L2_M2M_H

#include "v4l2-buffers.h"
#include "v4l2-common.h"

#define V4L_M2M_DEFAULT_OPTS \
    { "device",\
        "Path to the device to use",\
        OFFSET(devname),\
        AV_OPT_TYPE_STRING,\
        {.str = NULL }, 0, 0, FLAGS },\
    { "input_memory",\
        "Input memory model: See V4L2_MEMORY_* in videodev2.h. This depends on the HW but default should work with most but would imply useless memcpy()'s if used improperly.",\
        OFFSET(output_pool.memory),\
        AV_OPT_TYPE_INT,\
        {.i64 = V4L2_MEMORY_MMAP},\
        0, INT_MAX, FLAGS },\
    { "output_memory",\
        "Output memory model: See V4L2_MEMORY_* in videodev2.h. This depends on the HW but default should work with most.",\
        OFFSET(capture_pool.memory),\
        AV_OPT_TYPE_INT,\
        {.i64 = V4L2_MEMORY_MMAP},\
        0, INT_MAX, FLAGS },\
    { "num_output_pool_buffers",\
        "Number of buffers in the output pool",\
        OFFSET(output_pool.num_buffers),\
        AV_OPT_TYPE_INT,\
        { .i64 = 16 },\
        4, INT_MAX, FLAGS }

typedef struct V4Lm2mContext
{
    AVClass *class;
    int fd;
    char *devname;
    struct v4l2_capability cap;
    V4LBufferPool output_pool;
    V4LBufferPool capture_pool;
} V4Lm2mContext;

int avpriv_v4lm2m_init(V4Lm2mContext* s, void* log_ctx);
int avpriv_set_pool_format(V4LBufferPool* bp, int set);
int avpriv_v4lm2m_end(V4Lm2mContext* ctx);

#endif /* AVCODEC_V4L2_M2M_H */

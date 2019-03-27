/*
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

#include "hwaccel.h"
#include "mpegvideo.h"
#include "v4l2_normal.h"

static int v4l2_normal_mpeg2_start_frame(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const MpegEncContext *s = avctx->priv_data;

    ff_v4l2_normal_reset_frame(avctx, s->current_picture_ptr->f);

    return ff_v4l2_normal_append_output_buffer(avctx, s->current_picture_ptr->f, buffer, size);
}

static int v4l2_normal_mpeg2_decode_slice(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    return 0;
}

static int v4l2_normal_mpeg2_end_frame(AVCodecContext *avctx)
{
    const MpegEncContext *s = avctx->priv_data;

    return ff_v4l2_normal_decode_frame(avctx, s->current_picture_ptr->f);
}

static int v4l2_normal_mpeg2_init(AVCodecContext *avctx)
{
    return ff_v4l2_normal_init(avctx, V4L2_PIX_FMT_MPEG2);
}

const AVHWAccel ff_mpeg2_v4l2normal_hwaccel = {
    .name           = "mpeg2_v4l2",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_MPEG2VIDEO,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_normal_mpeg2_start_frame,
    .decode_slice   = v4l2_normal_mpeg2_decode_slice,
    .end_frame      = v4l2_normal_mpeg2_end_frame,
    .init           = v4l2_normal_mpeg2_init,
    .uninit         = ff_v4l2_normal_uninit,
    .priv_data_size = sizeof(V4L2NormalContext),
    .frame_params   = ff_v4l2_normal_frame_params,
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};

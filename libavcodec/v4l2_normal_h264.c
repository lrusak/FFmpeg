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

#include "h264dec.h"
#include "hwaccel.h"
#include "v4l2_normal.h"

static int v4l2_normal_h264_start_frame(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const H264Context *h = avctx->priv_data;

    ff_v4l2_normal_reset_frame(avctx, h->cur_pic_ptr->f);

    if (h->is_avc == 1)
        return ff_v4l2_normal_append_output_buffer(avctx, h->cur_pic_ptr->f, buffer, size);

    return 0;
}

static int v4l2_normal_h264_end_frame(AVCodecContext *avctx)
{
    const H264Context *h = avctx->priv_data;

    return ff_v4l2_normal_decode_frame(avctx, h->cur_pic_ptr->f);
}

static int v4l2_normal_h264_decode_slice(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const H264Context *h = avctx->priv_data;

    if (h->is_avc == 1)
        return 0;

    return ff_v4l2_normal_append_output_buffer(avctx, h->cur_pic_ptr->f, buffer, size);
}

static int v4l2_normal_h264_init(AVCodecContext *avctx)
{
    return ff_v4l2_normal_init(avctx, V4L2_PIX_FMT_H264);
}

const AVHWAccel ff_h264_v4l2normal_hwaccel = {
    .name           = "h264_v4l2",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H264,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_normal_h264_start_frame,
    .decode_params  = v4l2_normal_h264_decode_slice,
    .decode_slice   = v4l2_normal_h264_decode_slice,
    .end_frame      = v4l2_normal_h264_end_frame,
    .init           = v4l2_normal_h264_init,
    .uninit         = ff_v4l2_normal_uninit,
    .priv_data_size = sizeof(V4L2NormalContext),
    .frame_params   = ff_v4l2_normal_frame_params,
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};

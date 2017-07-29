/*
 * V4L2 mem2mem encoders
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

#include <sys/ioctl.h>

#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "v4l2_m2m_avcodec.h"
#include "v4l2-buffers.h"
#include "v4l2-common.h"
#include "v4l2_m2m.h"
#include "avcodec.h"

#define SET_V4L_EXT_CTRL(TYPE, ID, VALUE, CLASS, NAME) \
{ \
    struct v4l2_ext_control ctrl = { 0 };\
    struct v4l2_ext_controls ctrls = { 0 };\
    ctrl.id = ID ;\
    ctrl.TYPE = VALUE ;\
    ctrls.ctrl_class = CLASS ;\
    ctrls.count = 1;\
    ctrls.controls = &ctrl;\
\
    if ((ret = ioctl(s->fd, VIDIOC_S_EXT_CTRLS, &ctrls)) < 0)\
        av_log(avctx, AV_LOG_WARNING, "Failed to set " NAME "\n");\
}

static inline int v4l_h264_profile_from_ff(int p)
{
    switch(p) {
        case FF_PROFILE_H264_BASELINE:
            return V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
        case FF_PROFILE_H264_CONSTRAINED_BASELINE:
            return V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE;
        case FF_PROFILE_H264_MAIN:
            return V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
        case FF_PROFILE_H264_EXTENDED:
            return V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED;
        case FF_PROFILE_H264_HIGH:
            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
        case FF_PROFILE_H264_HIGH_10:
            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10;
        case FF_PROFILE_H264_HIGH_10_INTRA:
            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10_INTRA;
        case FF_PROFILE_H264_HIGH_422:
            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422;
        case FF_PROFILE_H264_HIGH_422_INTRA:
            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA;
        case FF_PROFILE_H264_HIGH_444_PREDICTIVE:
            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE;
        case FF_PROFILE_H264_HIGH_444_INTRA:
            return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_INTRA;
    }

    return -1;
}

static inline int v4l_mpeg4_profile_from_ff(int p)
{
    switch(p) {
        case FF_PROFILE_MPEG4_SIMPLE:
            return V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE;
        case FF_PROFILE_MPEG4_ADVANCED_SIMPLE:
            return V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE;
        case FF_PROFILE_MPEG4_CORE:
            return V4L2_MPEG_VIDEO_MPEG4_PROFILE_CORE;
        case FF_PROFILE_MPEG4_SIMPLE_SCALABLE:
            return V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE_SCALABLE;
        case FF_PROFILE_MPEG4_ADVANCED_CODING:
            return V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY;
    }

    return -1;
}

static av_cold int v4lm2m_encode_init(AVCodecContext *avctx)
{
    V4Lm2mContext *s = avctx->priv_data;
    int ret, val;

    s->output_pool.cfg.format = avpriv_set_pool_format;
    s->output_pool.cfg.init = avpriv_init_v4lbufpool;
    s->output_pool.av_pix_fmt = avctx->pix_fmt;
    s->output_pool.width = avctx->width;
    s->output_pool.height = avctx->height;
    s->output_pool.av_codec_id = AV_CODEC_ID_RAWVIDEO;
    s->output_pool.default_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

    s->capture_pool.cfg.format = avpriv_set_pool_format;
    s->capture_pool.cfg.init = avpriv_init_v4lbufpool;
    s->capture_pool.min_queued_buffers = 1;
    s->capture_pool.av_pix_fmt  = AV_PIX_FMT_NONE;
    s->capture_pool.av_codec_id = avctx->codec_id;
    s->capture_pool.default_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

    if (ret = ff_v4lm2m_codec_init(avctx))
        return ret;

    SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_GOP_SIZE, avctx->gop_size, V4L2_CTRL_CLASS_MPEG, "gop size");
    SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_BITRATE , avctx->bit_rate, V4L2_CTRL_CLASS_MPEG, "bit rate");
    SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_HEADER_MODE, V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME, V4L2_CTRL_CLASS_MPEG, "header mode");
    SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_B_FRAMES, avctx->max_b_frames, V4L2_CTRL_CLASS_MPEG, "number of B-frames");

    switch(avctx->codec_id) {
        case AV_CODEC_ID_H264:
            val = v4l_h264_profile_from_ff(avctx->profile);
            if (val >= 0) {
                 SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_H264_PROFILE, val, V4L2_CTRL_CLASS_MPEG, "h264 profile");
            }
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_H264_MIN_QP, avctx->qmin, V4L2_CTRL_CLASS_MPEG, "minimum video quantizer scale");
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_H264_MAX_QP, avctx->qmax, V4L2_CTRL_CLASS_MPEG, "maximum video quantizer scale");
            break;
        case AV_CODEC_ID_MPEG4:
            val = v4l_mpeg4_profile_from_ff(avctx->profile);
            if (val >= 0) {
                 SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE, val, V4L2_CTRL_CLASS_MPEG, "mpeg4 profile");
            }
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP, avctx->qmin, V4L2_CTRL_CLASS_MPEG, "minimum video quantizer scale");
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP, avctx->qmax, V4L2_CTRL_CLASS_MPEG, "maximum video quantizer scale");
            if (avctx->flags & CODEC_FLAG_QPEL) {
                SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_MPEG4_QPEL, 1, V4L2_CTRL_CLASS_MPEG, "qpel");
            }
            break;
        case AV_CODEC_ID_H263:
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_H263_MIN_QP, avctx->qmin, V4L2_CTRL_CLASS_MPEG, "minimum video quantizer scale");
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_H263_MAX_QP, avctx->qmax, V4L2_CTRL_CLASS_MPEG, "maximum video quantizer scale");
            break;
        case AV_CODEC_ID_VP8:
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_VPX_MIN_QP, avctx->qmin, V4L2_CTRL_CLASS_MPEG, "minimum video quantizer scale");
            SET_V4L_EXT_CTRL(value, V4L2_CID_MPEG_VIDEO_VPX_MAX_QP, avctx->qmax, V4L2_CTRL_CLASS_MPEG, "maximum video quantizer scale");
            break;
    }
    return 0;
}

static int v4lm2m_encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                              const AVFrame *pict, int *got_packet)
{
    V4Lm2mContext *s = avctx->priv_data;
    int ret;

    if (pict) {
        ret = avpriv_v4l_enqueue_frame_or_pkt_or_buf(&s->output_pool, pict, NULL, NULL, 0);
        if (ret < 0)
            return ret;

        if (!s->output_pool.streamon && (ret = avpriv_set_stream_status(&s->output_pool, VIDIOC_STREAMON))) {
            av_log(avctx, AV_LOG_ERROR, "VIDIOC_STREAMON failed on output pool\n");
            return ret;
        }

        if (!s->capture_pool.streamon && (ret = avpriv_set_stream_status(&s->capture_pool, VIDIOC_STREAMON))) {
            av_log(avctx, AV_LOG_ERROR, "VIDIOC_STREAMON failed on capture pool\n");
            return ret;
        }

    } else if (s->output_pool.streamon) {
        /* last frame, stop encoder */
        struct v4l2_encoder_cmd cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = V4L2_ENC_CMD_STOP;

        ret = ioctl(s->fd, VIDIOC_ENCODER_CMD, &cmd);
        if (ret)
            av_log(avctx, AV_LOG_ERROR, "Failed to stop encoder (%s)\n", av_err2str(AVERROR(errno)));

        s->output_pool.streamon = 0;
    }

    if (s->output_pool.num_queued >= s->output_pool.num_buffers - 2 || !pict) {
        s->capture_pool.blocking_dequeue = 100;
    } else {
        s->capture_pool.blocking_dequeue = 0;
    }

    ret = avpriv_v4l_dequeue_frame_or_pkt(&(s->capture_pool), NULL, pkt);

    if (AVERROR(EAGAIN) == ret )
        return 0;

    if (!ret)
        *got_packet = 1;

    return ret;
}

#define OFFSET(x) offsetof(V4Lm2mContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption options[] = {
    V4L_M2M_DEFAULT_OPTS, { "num_capture_pool_buffers", "Number of buffers in the capture pool",
      OFFSET(capture_pool.num_buffers), AV_OPT_TYPE_INT, {.i64 = 4 }, 4, INT_MAX, FLAGS }, { NULL },
};

#define M2MENC(NAME, LONGNAME, CODEC) \
static const AVClass v4l2_m2m_ ## NAME ## _enc_class = {\
    .class_name = #NAME "_v4l2_m2m_encoder",\
    .item_name  = av_default_item_name,\
    .option     = options,\
    .version    = LIBAVUTIL_VERSION_INT,\
};\
\
AVCodec ff_ ## NAME ## _v4l2m2m_encoder = { \
    .name           = #NAME "_v4l2m2m" ,\
    .long_name      = NULL_IF_CONFIG_SMALL("V4L2 mem2mem " LONGNAME " encoder wrapper"),\
    .type           = AVMEDIA_TYPE_VIDEO,\
    .id             = CODEC ,\
    .priv_data_size = sizeof(V4Lm2mContext),\
    .priv_class     = &v4l2_m2m_ ## NAME ##_enc_class,\
    .init           = v4lm2m_encode_init,\
    .encode2        = v4lm2m_encode_frame,\
    .close          = ff_v4lm2m_codec_end,\
    .capabilities   = CODEC_CAP_DELAY,\
};

#if CONFIG_H263_V4L2M2M_ENCODER
M2MENC(h263 , "H.263" , AV_CODEC_ID_H263 );
#endif

#if CONFIG_H264_V4L2M2M_ENCODER
M2MENC(h264 , "H.264" , AV_CODEC_ID_H264 );
#endif

#if CONFIG_MPEG4_V4L2M2M_ENCODER
M2MENC(mpeg4, "MPEG4", AV_CODEC_ID_MPEG4);
#endif

#if CONFIG_VP8_V4L2M2M_ENCODER
M2MENC(vp8  , "VP8"  , AV_CODEC_ID_VP8  );
#endif

#if CONFIG_VP9_V4L2M2M_ENCODER
M2MENC(vp9  , "VP9"  , AV_CODEC_ID_VP9  );
#endif

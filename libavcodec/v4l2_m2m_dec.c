/*
 * V4L2 mem2mem decoders
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
#include "v4l2-common.h"
#include "v4l2-buffers.h"
#include "v4l2_m2m.h"
#include "decode.h"
#include "avcodec.h"

static int try_start(AVCodecContext *avctx)
{
    V4Lm2mContext *s = avctx->priv_data;
    struct v4l2_selection selection;
    struct v4l2_control ctrl;
    int ret;

    if (s->output_pool.streamon && s->capture_pool.streamon)
        return 0;

    /* this will report the size of the frame back (see a4lbuf_to_avframe) */
    s->capture_pool.height = avctx->coded_height;
    s->capture_pool.width = avctx->coded_width;

    /* start the output process */
    if (!s->output_pool.streamon) {
        ret = avpriv_set_stream_status(&s->output_pool, VIDIOC_STREAMON);
        if (ret < 0) {
            av_log(avctx, AV_LOG_DEBUG, "VIDIOC_STREAMON on output pool\n");
            return ret;
        }
    }

    /* get the capture format */
    s->capture_pool.format.type = s->capture_pool.type;
    ret = ioctl(s->fd, VIDIOC_G_FMT, &s->capture_pool.format);
    if (ret) {
        av_log(avctx, AV_LOG_DEBUG, "VIDIOC_G_FMT ioctl\n");
        return ret;
    }

    /* store what the decoder gives */
    avctx->pix_fmt = avpriv_v4l_fmt_v4l2ff(s->capture_pool.format.fmt.pix_mp.pixelformat, AV_CODEC_ID_RAWVIDEO);
    s->capture_pool.av_pix_fmt = avctx->pix_fmt;

    /* set the crop parameters */
    selection.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    selection.r.height = avctx->coded_height;
    selection.r.width = avctx->coded_width;
    ret = ioctl(s->fd, VIDIOC_S_SELECTION, &selection);
    if (!ret) {
        ret = ioctl(s->fd, VIDIOC_G_SELECTION, &selection);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "VIDIOC_G_SELECTION ioctl\n");
        } else {
            av_log(avctx, AV_LOG_DEBUG, "crop output %dx%d\n", selection.r.width, selection.r.height);
            /* update the size of the resulting frame */
            s->capture_pool.height = selection.r.height;
            s->capture_pool.width  = selection.r.width;
        }
    }

    /* get the minimum number of buffers required by capture */
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
    ret = ioctl(s->fd, VIDIOC_G_CTRL, &ctrl);
    if (ret) {
        s->capture_pool.min_queued_buffers = 6;
    } else {
        s->capture_pool.min_queued_buffers = ctrl.value;
    }

    /* init the capture pool */
    if (!s->capture_pool.buffers) {
        ret = avpriv_init_v4lbufpool(&s->capture_pool);
        if (ret) {
            av_log(avctx, AV_LOG_DEBUG, "can't request output buffers\n");
            return ret;
        }
    }

    /* start the capture process */
    ret = avpriv_set_stream_status(&s->capture_pool, VIDIOC_STREAMON);
    if (ret) {
        av_log(avctx, AV_LOG_DEBUG, "VIDIOC_STREAMON, on capture pool\n");
        return ret;
    }


    return 0;
}

static av_cold int v4lm2m_decode_init(AVCodecContext *avctx)
{
    V4Lm2mContext *s = avctx->priv_data;

    s->output_pool.cfg.format = avpriv_set_pool_format;
    s->output_pool.cfg.init = avpriv_init_v4lbufpool;

    s->output_pool.av_codec_tag = avctx->codec_tag;
    s->output_pool.av_codec_id = avctx->codec_id;
    s->output_pool.av_pix_fmt  = AV_PIX_FMT_NONE;
    s->output_pool.height = avctx->coded_height;
    s->output_pool.width = avctx->coded_width;
    s->output_pool.default_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

    s->capture_pool.cfg.format = avpriv_set_pool_format;
    s->capture_pool.cfg.init = NULL;

    s->capture_pool.av_codec_tag = 0;
    s->capture_pool.av_codec_id = AV_CODEC_ID_RAWVIDEO;
    s->capture_pool.av_pix_fmt = avctx->pix_fmt;
    s->capture_pool.height = avctx->coded_height;
    s->capture_pool.width = avctx->coded_width;
    s->capture_pool.default_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

    return ff_v4lm2m_codec_init(avctx);
}

static inline void set_dequeue_mode(V4Lm2mContext *s)
{
    s->capture_pool.blocking_dequeue = 0;

    if (s->output_pool.num_queued >= s->output_pool.num_buffers - 2)
        s->capture_pool.blocking_dequeue = 1000;
}

static int v4lm2m_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    V4Lm2mContext *s = avctx->priv_data;
    AVPacket avpkt = {0};
    int ret;

    ret = ff_decode_get_packet(avctx, &avpkt);
    if (ret < 0 && ret != AVERROR_EOF)
        return ret;

    if (avctx->extradata && avctx->extradata_size) {
            ret = avpriv_v4l_enqueue_frame_or_pkt_or_buf(&s->output_pool, NULL,
                                NULL, avctx->extradata, avctx->extradata_size);
            if (ret)
                return ret;

            (void) try_start(avctx);
    }

    ret = avpriv_v4l_enqueue_frame_or_pkt_or_buf(&s->output_pool, NULL, &avpkt, NULL, 0);
    if (ret < 0)
        return ret;

    ret = try_start(avctx);
    if (ret)
        return 0;

    set_dequeue_mode(s);

    return avpriv_v4l_dequeue_frame_or_pkt(&s->capture_pool, frame, NULL);
}

#define OFFSET(x) offsetof(V4Lm2mContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM

        static const AVOption options[] = {
        V4L_M2M_DEFAULT_OPTS,{ "num_capture_pool_extra_buffers",
        "Number of extra buffers in the capture pool",
        OFFSET(capture_pool.num_buffers), AV_OPT_TYPE_INT,{.i64 = 6}, 4, INT_MAX, FLAGS},
        { NULL},
        };

#define M2MDEC(NAME, LONGNAME, CODEC, bsf_name) \
static const AVClass v4l2_m2m_ ## NAME ## _dec_class = {\
    .class_name = #NAME "_v4l2_m2m_decoder",\
    .item_name  = av_default_item_name,\
    .option     = options,\
    .version    = LIBAVUTIL_VERSION_INT,\
};\
\
AVCodec ff_ ## NAME ## _v4l2m2m_decoder = { \
    .name           = #NAME "_v4l2m2m" ,\
    .long_name      = NULL_IF_CONFIG_SMALL("V4L2 mem2mem " LONGNAME " decoder wrapper"),\
    .type           = AVMEDIA_TYPE_VIDEO,\
    .id             = CODEC ,\
    .priv_data_size = sizeof(V4Lm2mContext),\
    .priv_class     = &v4l2_m2m_ ## NAME ## _dec_class,\
    .init           = v4lm2m_decode_init,\
    .receive_frame  = v4lm2m_receive_frame,\
    .close          = ff_v4lm2m_codec_end,\
    .capabilities   = CODEC_CAP_DELAY,\
    .bsfs           = bsf_name, \
};

#if CONFIG_H263_V4L2M2M_DECODER
        M2MDEC(h263 , "H.263" , AV_CODEC_ID_H263, NULL);
#endif

#if CONFIG_H264_V4L2M2M_DECODER
        M2MDEC(h264 , "H.264" , AV_CODEC_ID_H264, "h264_mp4toannexb");
#endif

#if CONFIG_MPEG1_V4L2M2M_DECODER
        M2MDEC(mpeg1, "MPEG1", AV_CODEC_ID_MPEG1VIDEO, NULL);
#endif

#if CONFIG_MPEG2_V4L2M2M_DECODER
        M2MDEC(mpeg2, "MPEG2", AV_CODEC_ID_MPEG2VIDEO, NULL);
#endif

#if CONFIG_MPEG4_V4L2M2M_DECODER
        M2MDEC(mpeg4, "MPEG4", AV_CODEC_ID_MPEG4, NULL);
#endif

#if CONFIG_VC1_V4L2M2M_DECODER
        M2MDEC(vc1  , "VC1"  , AV_CODEC_ID_VC1, NULL);
#endif

#if CONFIG_VP8_V4L2M2M_DECODER
        M2MDEC(vp8  , "VP8"  , AV_CODEC_ID_VP8, NULL);
#endif

#if CONFIG_VP9_V4L2M2M_DECODER
        M2MDEC(vp9  , "VP9"  , AV_CODEC_ID_VP9, NULL);
#endif

/*
 * V4L mem2mem wrapper
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
#include <sys/mman.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include "libavutil/imgutils.h"
#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "avcodec.h"
#include "v4l2_m2m_avcodec.h"
#include "v4l2-buffers.h"
#include "v4l2-common.h"
#include "v4l2_m2m.h"

#define V4L_MAX_STREAM_SIZE (3*1024*1024)

static inline int try_raw_format(V4LBufferPool* bp, enum AVPixelFormat pixfmt)
{
    struct v4l2_format *fmt = &bp->format;
    int ret, i, h;

    fmt->type  = bp->type;

    if (V4L2_TYPE_IS_MULTIPLANAR(bp->type)) {
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pixfmt);

        fmt->fmt.pix_mp.pixelformat = avpriv_v4l_fmt_ff2v4l(pixfmt, bp->av_codec_id, FF_V4L_PACK_AVFRAME);
        if (!fmt->fmt.pix_mp.pixelformat)
            return AVERROR(EINVAL);

        fmt->fmt.pix_mp.num_planes = av_pix_fmt_count_planes(pixfmt);
        for (i = 0; i < fmt->fmt.pix_mp.num_planes; i++) {
            fmt->fmt.pix_mp.plane_fmt[i].bytesperline = av_image_get_linesize(pixfmt, bp->width, i);
            h = (i == 1 || i == 2) ? FF_CEIL_RSHIFT(bp->height, desc->log2_chroma_h) : bp->height;
            fmt->fmt.pix_mp.plane_fmt[i].sizeimage = fmt->fmt.pix_mp.plane_fmt[i].bytesperline * h;
        }
    } else {

        fmt->fmt.pix.pixelformat  = avpriv_v4l_fmt_ff2v4l(pixfmt, bp->av_codec_id, FF_V4L_PACK_AVFRAME);
        if (!fmt->fmt.pix.pixelformat)
            return AVERROR(EINVAL);

        fmt->fmt.pix.bytesperline = av_image_get_linesize(pixfmt, bp->width, 0);
        fmt->fmt.pix.sizeimage = fmt->fmt.pix.bytesperline * bp->height;
    }

    ret = ioctl(bp->fd, VIDIOC_TRY_FMT, fmt);
    if (ret)
        return AVERROR(EINVAL);

    if (V4L2_TYPE_IS_MULTIPLANAR(bp->type)) {
        fmt->fmt.pix_mp.height = bp->height;
        fmt->fmt.pix_mp.width = bp->width;
    } else {
        fmt->fmt.pix.height = bp->height;
        fmt->fmt.pix.width = bp->width;
    }

    return 0;
}

static int set_raw_format(V4LBufferPool* bp, int set)
{
    enum AVPixelFormat pixfmt = bp->av_pix_fmt;
    struct v4l2_format *fmt = &bp->format;
    struct v4l2_fmtdesc fmtdesc = { 0 };
    int ret;

    fmtdesc.type = bp->type;
    if (pixfmt != AV_PIX_FMT_NONE) {
        ret = try_raw_format(bp, pixfmt);
        if (ret)
            pixfmt = AV_PIX_FMT_NONE;
    }

    while (AV_PIX_FMT_NONE == pixfmt && !ioctl(bp->fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        pixfmt = avpriv_v4l_fmt_v4l2ff(fmtdesc.pixelformat, AV_CODEC_ID_RAWVIDEO);

        ret = try_raw_format(bp, pixfmt);
        if (ret)
            pixfmt = AV_PIX_FMT_NONE;

        if (pixfmt != AV_PIX_FMT_NONE && set) {
            bp->av_pix_fmt = pixfmt;
        }

        fmtdesc.index++;
    }

    if (pixfmt == AV_PIX_FMT_NONE)
        return AVERROR(EINVAL);

    if (set)
        return ioctl(bp->fd, VIDIOC_S_FMT, fmt);

    return 0;
}

static int set_coded_format(V4LBufferPool* bp, int set)
{
    struct v4l2_format *fmt = &bp->format;
    struct v4l2_fmtdesc fdesc;
    uint32_t v4l2_fmt;
    int found = 0;

    v4l2_fmt = avpriv_v4l_fmt_ff2v4l(bp->av_pix_fmt, bp->av_codec_id, FF_V4L_PACK_AVPACKET);
    memset(&fdesc, 0, sizeof(fdesc));
    fdesc.type = bp->type;

    while (!ioctl(bp->fd, VIDIOC_ENUM_FMT, &fdesc)) {
        if (v4l2_fmt == fdesc.pixelformat) {
            found = 1;
            break;
        }
        fdesc.index++;
    }

    if (!found)
        return AVERROR(EINVAL);

    fmt->type = bp->type;

    if (V4L2_TYPE_IS_MULTIPLANAR(bp->type)) {
        fmt->fmt.pix_mp.num_planes = VIDEO_MAX_PLANES;
        fmt->fmt.pix_mp.pixelformat = v4l2_fmt;
        if (!fmt->fmt.pix_mp.pixelformat) {
            av_log(bp->log_ctx, AV_LOG_ERROR, "no V4L codec for id %i\n", bp->av_codec_id);
            return AVERROR(EINVAL);
        }
        fmt->fmt.pix_mp.plane_fmt[0].sizeimage = V4L_MAX_STREAM_SIZE;
        fmt->fmt.pix_mp.height = bp->height;
        fmt->fmt.pix_mp.width = bp->width;

    } else {
        fmt->fmt.pix.pixelformat = v4l2_fmt;
        if (!fmt->fmt.pix.pixelformat) {
            av_log(bp->log_ctx, AV_LOG_ERROR, "no V4L codec for id %i\n", bp->av_codec_id);
            return AVERROR(EINVAL);
        }
        fmt->fmt.pix.sizeimage = V4L_MAX_STREAM_SIZE;
        fmt->fmt.pix.height = bp->height;
        fmt->fmt.pix.width = bp->width;
    }

    if (set)
        return ioctl(bp->fd, VIDIOC_S_FMT, fmt);

    return ioctl(bp->fd, VIDIOC_TRY_FMT, fmt);
}

int avpriv_set_pool_format(V4LBufferPool* bp, int set)
{
    if (bp->av_codec_id == AV_CODEC_ID_RAWVIDEO)
        return set_raw_format(bp, set);

    return set_coded_format(bp, set);
}

static inline int splane_video(struct v4l2_capability *cap)
{
    if (cap->capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING))
        return 1;

    if (cap->capabilities & (V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING))
        return 1;

    return 0;
}

static inline int mplane_video(struct v4l2_capability *cap)
{
    if (cap->capabilities & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING))
        return 1;

    if (cap->capabilities & (V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING))
        return 1;

    return 0;
}

static int prepare_pools(V4Lm2mContext* s, void *log_ctx)
{
    int ret;

    s->capture_pool.log_ctx = s->output_pool.log_ctx = log_ctx;
    s->capture_pool.broken = s->output_pool.broken = 0;
    s->capture_pool.fd = s->output_pool.fd = s->fd;
    s->capture_pool.name = "capture pool";
    s->output_pool.name = "output pool";

    memset(&s->cap, 0, sizeof(s->cap));
    ret = ioctl(s->fd, VIDIOC_QUERYCAP, &s->cap);
    if (ret < 0) {
        return ret;
    }

    if (mplane_video(&s->cap)) {
        s->capture_pool.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        s->output_pool.type  = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        return 0;
    }

    if (splane_video(&s->cap)) {
        s->capture_pool.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        s->output_pool.type  = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        return 0;
    }

    return AVERROR(EINVAL);
}

static int probe_and_set(V4Lm2mContext* s, void *log_ctx, int set)
{
    int fail_log_level = ( set ? AV_LOG_ERROR : AV_LOG_DEBUG);
    int ret;

    s->fd = open(s->devname, O_RDWR | O_NONBLOCK, 0);
    if (s->fd < 0)
        return AVERROR(errno);

    ret = prepare_pools(s, log_ctx);
    if (ret < 0)
        goto error;

    if (s->output_pool.cfg.format) {
        ret = s->output_pool.cfg.format(&s->output_pool, set);
        if (ret) {
            av_log(log_ctx, fail_log_level, "can't set input format\n");
            goto error;
        }
    }

    if (s->capture_pool.cfg.format) {
        ret = s->capture_pool.cfg.format(&s->capture_pool, set);
        if (ret) {
            av_log(log_ctx, fail_log_level, "can't to set output format\n");
            goto error;
        }
    }

    if (s->output_pool.cfg.init && set) {
        ret = s->output_pool.cfg.init(&s->output_pool);
        if (ret) {
            av_log(log_ctx, fail_log_level, "no output pool's buffers\n");
            goto error;
        }
    }

    if (s->capture_pool.cfg.init && set) {
        ret = s->capture_pool.cfg.init(&s->capture_pool);
        if (ret) {
            av_log(log_ctx, fail_log_level, "no capture pool's buffers\n");
            goto error;
        }
    }

    av_log(log_ctx, AV_LOG_INFO, "using driver '%s' on card '%s'\n", s->cap.driver, s->cap.card);

error:
    if (!set || ret) {
        close(s->fd);
        s->fd = 0;
    }

    return ret;
}

int avpriv_v4lm2m_init(V4Lm2mContext* s, void* log_ctx)
{
    char *devname_save = s->devname;
    int ret = AVERROR(EINVAL);
    char tmpbuf[PATH_MAX];
    struct dirent *dp;
    DIR *dirp;

    if (s->devname && *s->devname)
        return probe_and_set(s, log_ctx, 1);

    if (!(dirp = opendir("/dev")))
        return AVERROR(errno);

    for (dp = readdir(dirp); dp; dp = readdir(dirp)) {

        if (!strncmp(dp->d_name, "video", sizeof("video") - 1)) {
            snprintf(tmpbuf, sizeof(tmpbuf) - 1, "/dev/%s", dp->d_name);
            av_log(log_ctx, AV_LOG_DEBUG, "probing %s\n", tmpbuf);

            s->devname = tmpbuf;
            ret = probe_and_set(s, log_ctx, 0);
            if (!ret)
                break;
        }
    }
    closedir(dirp);

    if (ret) {
        av_log(log_ctx, AV_LOG_ERROR, "Could not find a valid device\n");
        s->devname = devname_save;

        return ret;
    }

    av_log(log_ctx, AV_LOG_INFO, "Using device %s\n", tmpbuf);
    ret = probe_and_set(s, log_ctx, 1);
    s->devname = devname_save;

    return ret;
}

int ff_v4lm2m_codec_init(AVCodecContext *avctx)
{
    V4Lm2mContext *s = avctx->priv_data;

    return avpriv_v4lm2m_init(s, avctx);
}

int avpriv_v4lm2m_end(V4Lm2mContext* s)
{
    avpriv_release_buffer_pool(&s->output_pool);
    avpriv_release_buffer_pool(&s->capture_pool);
    avpriv_set_stream_status(&s->output_pool, VIDIOC_STREAMOFF);
    avpriv_set_stream_status(&s->capture_pool, VIDIOC_STREAMOFF);
    close(s->fd);

    return 0;
}

int ff_v4lm2m_codec_end(AVCodecContext *avctx)
{
    V4Lm2mContext *s = avctx->priv_data;

    av_log(avctx, AV_LOG_DEBUG, "Closing context\n");

    return avpriv_v4lm2m_end(s);
}

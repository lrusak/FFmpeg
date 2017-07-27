/*
 * V4L2 buffer{,pool} helper functions.
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
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include "avcodec.h"
#include "internal.h"
#include "v4l2-buffers.h"
#include "v4l2-common.h"

#if 0
#define V4L_BUFFER_DEBUG */
#endif

#define IS_BP_SUPPORTED(bp) ((bp->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) || \
                             (bp->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)  || \
                             (bp->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)        || \
                             (bp->type == V4L2_BUF_TYPE_VIDEO_OUTPUT))

enum V4LBuffer_status {
    V4LBUF_AVAILABLE,
    V4LBUF_IN_DRIVER,
    V4LBUF_RET_USER,
};

struct V4LBuffer {
    AVBufferRef *bufrefs[VIDEO_MAX_PLANES];
    struct V4LBufferPool *pool;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    struct v4l2_buffer buf;

    void * mm_addr[VIDEO_MAX_PLANES];
    size_t lengths[VIDEO_MAX_PLANES];
    enum V4LBuffer_status status;
    int bytesperline[4];
    int num_planes;
    int num_lines;
    int index;
    int flags;
    struct timeval timestamp;
    int ref_cnt;
};

static int enqueue_v4lbuf(V4LBuffer* avbuf)
{
    int ret;

    memset(&avbuf->buf, 0, sizeof(avbuf->buf));
    avbuf->buf.memory = avbuf->pool->memory;
    avbuf->buf.type = avbuf->pool->type;
    avbuf->buf.index = avbuf->index;

    if (V4L2_TYPE_IS_MULTIPLANAR(avbuf->pool->type)) {
        avbuf->buf.length   = avbuf->num_planes;
        avbuf->buf.m.planes = avbuf->planes;
    } else {
        avbuf->buf.bytesused = avbuf->planes[avbuf->index].bytesused;
        avbuf->buf.m.userptr = avbuf->planes[avbuf->index].m.userptr;
        avbuf->buf.length    = avbuf->planes[avbuf->index].length;
    }

    avbuf->buf.flags = avbuf->pool->default_flags | avbuf->flags;
    avbuf->buf.timestamp = avbuf->timestamp;

    ret = ioctl(avbuf->pool->fd, VIDIOC_QBUF, &avbuf->buf);
    if (ret < 0)
        return AVERROR(errno);

    avbuf->status = V4LBUF_IN_DRIVER;
    avbuf->pool->num_queued++;

#ifdef V4L_BUFFER_DEBUG
    av_log(avbuf->pool->log_ctx, AV_LOG_DEBUG, " buffer enqueued on %s\n", avbuf->pool->name);
#endif

    return 0;
}

static V4LBuffer* dequeue_v4lbuf(V4LBufferPool *bp)
{
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    struct v4l2_buffer buf = { 0 };
    V4LBuffer* avbuf = NULL;
    struct pollfd pfd;
    int ret;
    int i;

    if (bp->num_queued < bp->min_queued_buffers) {
        return NULL;
    }

    if (bp->blocking_dequeue) {
        pfd.fd = bp->fd;
        switch (bp->type) {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
            pfd.events = POLLIN | POLLERR;
            break;
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
            pfd.events = POLLOUT | POLLERR | POLLWRNORM;
            break;
        default:
            pfd.events = POLLIN | POLLERR | POLLRDNORM;
        }

        ret = poll(&pfd, 1, bp->blocking_dequeue);
        if (ret<= 0) {
            av_log(bp->log_ctx, AV_LOG_WARNING, "%s: timeout (%d ms)\n", bp->name, bp->blocking_dequeue);
            return NULL;
        }
    }

    memset(&buf, 0, sizeof(buf));
    buf.memory = bp->memory;
    buf.type = bp->type;
    if (V4L2_TYPE_IS_MULTIPLANAR(bp->type)) {
        memset(planes, 0, sizeof(planes));
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;
    }

    ret = ioctl(bp->fd, VIDIOC_DQBUF, &buf);
    if (ret) {
        if (errno != EAGAIN) {
            av_log(bp->log_ctx, AV_LOG_DEBUG, "%s: VIDIOC_DQBUF, errno (%d)\n", bp->name, errno);
            bp->broken = errno;
        }
        return NULL;
    }

    avbuf = &(bp->buffers[buf.index]);
    if (V4L2_TYPE_IS_MULTIPLANAR(bp->type)) {
        memcpy(avbuf->planes, planes, sizeof(planes));
        avbuf->buf.m.planes = avbuf->planes;
    }
    avbuf->status = V4LBUF_AVAILABLE;
    avbuf->pool->num_queued--;
    avbuf->buf = buf;

    if (V4L2_TYPE_IS_OUTPUT(avbuf->pool->type)) {
        for (i = 0; i < avbuf->num_planes; i++) {
            if (avbuf->bufrefs[i]) {
                av_buffer_unref(&avbuf->bufrefs[i]);
            }
        }
    }

#ifdef V4L_BUFFER_DEBUG
    av_log(bp->log_ctx, AV_LOG_DEBUG, "dequeued buffer on %s\n", bp->name);
#endif

    return avbuf;
}

static void buffer_callback(void *opaque, uint8_t *unused)
{
    V4LBuffer* avbuf = opaque;

    if (--avbuf->ref_cnt <= 0) {
        if (V4LBUF_IN_DRIVER != avbuf->status) {
            if (!V4L2_TYPE_IS_OUTPUT(avbuf->pool->type)) {
                enqueue_v4lbuf(avbuf);
            } else {
                avbuf->status = V4LBUF_AVAILABLE;
            }
        }
    }
}

static inline int init_buffer(V4LBuffer* avbuf)
{
    int ret, i;

    avbuf->buf.memory = avbuf->pool->memory;
    avbuf->buf.type = avbuf->pool->type;
    avbuf->buf.index = avbuf->index;

    if (V4L2_TYPE_IS_MULTIPLANAR(avbuf->pool->type)) {
        avbuf->buf.length = VIDEO_MAX_PLANES;
        avbuf->buf.m.planes = avbuf->planes;
    }

    ret = ioctl(avbuf->pool->fd, VIDIOC_QUERYBUF, &avbuf->buf);
    if (ret < 0)
        return AVERROR(errno);

    if (V4L2_TYPE_IS_MULTIPLANAR(avbuf->pool->type)) {
        avbuf->num_planes = 0;
        for (;;) {
            if (avbuf->num_planes < avbuf->buf.length) {
                if (avbuf->buf.m.planes[avbuf->num_planes].length) {
                    avbuf->num_planes++;
                    continue;
                }
            }
            break;
        }
    } else {
        avbuf->num_planes = 1;
    }

    avbuf->num_lines = avbuf->pool->format.fmt.pix_mp.height;
    for (i = 0; i < avbuf->num_planes; i++) {
        if (V4L2_TYPE_IS_MULTIPLANAR(avbuf->pool->type)) {
            avbuf->bytesperline[i] = avbuf->pool->format.fmt.pix_mp.plane_fmt[i].bytesperline;
        } else {
            avbuf->bytesperline[i] = avbuf->pool->format.fmt.pix.bytesperline;
        }

        switch (avbuf->pool->memory) {
        case V4L2_MEMORY_MMAP:
            if (V4L2_TYPE_IS_MULTIPLANAR(avbuf->pool->type)) {
                avbuf->lengths[i] = avbuf->buf.m.planes[i].length;
                avbuf->mm_addr[i] = mmap(NULL, avbuf->buf.m.planes[i].length,
                                         PROT_READ | PROT_WRITE, MAP_SHARED,
                                         avbuf->pool->fd, avbuf->buf.m.planes[i].m.mem_offset);
            } else {
                avbuf->lengths[i] = avbuf->buf.length;
                avbuf->mm_addr[i] = mmap(NULL, avbuf->buf.length,
                                         PROT_READ | PROT_WRITE, MAP_SHARED,
                                         avbuf->pool->fd, avbuf->buf.m.offset);
            }
            if (avbuf->mm_addr[i] == MAP_FAILED) {
                return AVERROR(ENOMEM);
            }
            break;
        case V4L2_MEMORY_USERPTR:
            /* Nothing to do */
            break;
        default:
            av_log(avbuf->pool->log_ctx, AV_LOG_ERROR, "memory type %i not supported\n", avbuf->pool->memory);
            return AVERROR_PATCHWELCOME;
        }
    }
    avbuf->status = V4LBUF_AVAILABLE;

    if (!V4L2_TYPE_IS_OUTPUT(avbuf->pool->type)) {
        if (avbuf->pool->memory != V4L2_MEMORY_USERPTR) {
            return enqueue_v4lbuf(avbuf);
        }
    }

    return 0;
}

int avpriv_init_v4lbufpool(V4LBufferPool* bufs)
{
    struct v4l2_requestbuffers req;
    int ret, i;

    if (!IS_BP_SUPPORTED(bufs)) {
        av_log(bufs->log_ctx, AV_LOG_ERROR, "%type %i not supported\n", bufs->type);
        return AVERROR_PATCHWELCOME;
    }

    memset(&req, 0, sizeof(req));
    req.count = bufs->num_buffers + bufs->min_queued_buffers;
    req.memory = bufs->memory;
    req.type = bufs->type;

    ret = ioctl(bufs->fd, VIDIOC_REQBUFS, &req);
    if (ret< 0)
        return AVERROR(errno);

    bufs->num_buffers = req.count;
    bufs->num_queued  = 0;
    bufs->buffers = av_mallocz(bufs->num_buffers * sizeof(V4LBuffer));

    for (i = 0; i < req.count; i++) {
        V4LBuffer *avbuf = &bufs->buffers[i];

        avbuf->pool = bufs;
        avbuf->index = i;
        ret = init_buffer(avbuf);
        if (ret < 0) {
            av_log(bufs->log_ctx, AV_LOG_ERROR, "%s buffer initialization (%s)\n", bufs->name, av_err2str(ret));
            return ret;
        }
    }

    return 0;
}

static void release_buf(V4LBuffer* b)
{
    int i;

    for (i = 0; i < b->num_planes; i++) {
        if (b->mm_addr[i] && b->lengths[i]) {
            munmap(b->mm_addr[i], b->lengths[i]);
        }
    }
}

void avpriv_release_buffer_pool(V4LBufferPool* bp)
{
    if (bp->buffers) {
        int i;

        for (i = 0; i < bp->num_buffers; i++)
            release_buf(&bp->buffers[i]);

        av_free(bp->buffers);
    }
}

static inline void set_pts(V4LBuffer *out, int64_t pts)
{
    out->timestamp.tv_sec  = pts / INT64_C(1000000);
    out->timestamp.tv_usec = pts % INT64_C(1000000);
}

static inline uint64_t get_pts(V4LBuffer *avbuf)
{
    if (avbuf->buf.timestamp.tv_sec || avbuf->buf.timestamp.tv_usec)
        return (avbuf->buf.timestamp.tv_sec * INT64_C(1000000) + avbuf->buf.timestamp.tv_usec);

    return AV_NOPTS_VALUE;
}

static int buf2v4l(V4LBuffer *out, int plane, const uint8_t* data, int size, AVBufferRef* bref)
{
    if (plane >= out->num_planes)
        return AVERROR(EINVAL);

    switch (out->pool->memory) {
    case V4L2_MEMORY_MMAP:
            memcpy(out->mm_addr[plane], data, FFMIN(size, out->lengths[plane]));
            break;
    case V4L2_MEMORY_USERPTR:
        if (!bref) {
            av_log(out->pool->log_ctx, AV_LOG_ERROR,
                   "needs to be set with an AVBufferRef for USERPTR memory type\n");
            return AVERROR_PATCHWELCOME;
        }

        if (out->bufrefs[plane]) {
            av_log(out->pool->log_ctx, AV_LOG_WARNING,
                   "V4L buffer already had a buffer referenced\n");
            av_buffer_unref(&out->bufrefs[plane]);
        }

        out->bufrefs[plane] = av_buffer_ref(bref);
        if (!out->bufrefs[plane])
            return AVERROR(ENOMEM);

        out->planes[plane].m.userptr = (unsigned long)out->bufrefs[plane]->data;
        out->lengths[plane] = out->bufrefs[plane]->size;

        break;
    default:
        av_log(out->pool->log_ctx, AV_LOG_ERROR,
               "memory type %i not supported", out->pool->memory);
        return AVERROR_PATCHWELCOME;
    }

    out->planes[plane].bytesused = FFMIN(size ? size : 1, out->lengths[plane]);
    out->planes[plane].length    = out->lengths[plane];

    return 0;
}

static int avframe_to_v4lbuf(const AVFrame *pict, V4LBuffer* out) {
    int i, ret;

    for (i = 0; i < out->num_planes; i++) {
        ret = buf2v4l(out, i, pict->buf[i]->data, pict->buf[i]->size, pict->buf[i]);
        if (ret)
            return ret;
    }
    set_pts(out, pict->pts);

    return 0;
}

static int avpkt_to_v4lbuf(const AVPacket *pkt, V4LBuffer *out) {
    int ret;

    ret = buf2v4l(out, 0, pkt->data, pkt->size, pkt->buf);
    if (ret)
        return ret;

    if (pkt->pts != AV_NOPTS_VALUE)
        set_pts(out, pkt->pts);

    if (pkt->flags & AV_PKT_FLAG_KEY)
        out->flags = V4L2_BUF_FLAG_KEYFRAME;

    if (!pkt->size)
        out->flags = V4L2_BUF_FLAG_LAST;

    return 0;
}

static inline int v4l2bufref(V4LBuffer *in, int plane, AVBufferRef **buf)
{
#ifdef V4L_BUFFER_DEBUG
    av_log(in->pool->log_ctx, AV_LOG_DEBUG,
           "Making an avbuffer from V4L buffer %i[%i] on %s (%i,%i)\n",
           in->index, plane, in->pool->name, in->pool->type, in->pool->memory);
#endif

    if (plane >= in->num_planes) {
        return AVERROR(EINVAL);
    }

    switch (in->pool->memory) {
    case V4L2_MEMORY_MMAP:
        *buf = av_buffer_create(in->mm_addr[plane], in->lengths[plane], buffer_callback, in, 0);
        if (!*buf)
            return AVERROR(ENOMEM);

        in->status = V4LBUF_RET_USER;
        in->ref_cnt++;
        break;
    case V4L2_MEMORY_USERPTR:
        if (!in->bufrefs[plane]) {
            av_log(in->pool->log_ctx, AV_LOG_ERROR, "AVBufferRef not found\n");
            return AVERROR(EINVAL);
        }

        *buf = av_buffer_ref(in->bufrefs[plane]);
        if (!*buf)
            return AVERROR(ENOMEM);

        av_buffer_unref(&in->bufrefs[plane]);
        in->status = V4LBUF_AVAILABLE;
        break;
    default:
        av_log(in->pool->log_ctx, AV_LOG_ERROR, "memory type %i not supported", in->pool->memory);
        return AVERROR_PATCHWELCOME;
    }
    return 0;
}

static int v4lbuf_to_avpkt(AVPacket *pkt, V4LBuffer *avbuf)
{
    int ret;

    av_free_packet(pkt);
    if (ret = v4l2bufref(avbuf, 0, &pkt->buf))
        return ret;

    pkt->data = pkt->buf->data;
    if (V4L2_TYPE_IS_MULTIPLANAR(avbuf->pool->type)) {
        pkt->size = avbuf->buf.m.planes[0].bytesused;
    } else {
        pkt->size = avbuf->buf.bytesused;
    }

    if (avbuf->buf.flags & V4L2_BUF_FLAG_KEYFRAME) {
        pkt->flags |= AV_PKT_FLAG_KEY;
    }

    pkt->pts = get_pts(avbuf);

    return 0;
}

static int v4lbuf_to_avframe(AVFrame *frame, V4LBuffer *avbuf)
{
    int i, ret;

    av_frame_unref(frame);

    for (i = 0; i < avbuf->num_planes; i++) {
        ret = v4l2bufref(avbuf, i, &frame->buf[i]);
        if (ret)
            return ret;

        frame->linesize[i] = avbuf->bytesperline[i];
        frame->data[i] = frame->buf[i]->data;

        if (avbuf->num_planes == 1) {
            if (avbuf->pool->av_pix_fmt == AV_PIX_FMT_NV12) {
                frame->linesize[1] = avbuf->bytesperline[0];
                frame->data[1] = frame->buf[0]->data + avbuf->bytesperline[0] * avbuf->num_lines;
            }
        }
    }

    frame->key_frame = !!(avbuf->buf.flags & V4L2_BUF_FLAG_KEYFRAME);
    frame->format = avbuf->pool->av_pix_fmt;
    frame->height = avbuf->pool->height;
    frame->width = avbuf->pool->width;
    frame->pts = get_pts(avbuf);

    return 0;
}

static V4LBuffer* v4lbufpool_get_from_avframe(const AVFrame* frame, V4LBufferPool *p)
{
    int i;

    for (i = 0; i < p->num_buffers; i++) {
        if (V4LBUF_RET_USER == p->buffers[i].status) {
            if (p->memory == V4L2_MEMORY_MMAP) {
                if (p->buffers[i].mm_addr[0] == frame->buf[0]->data)
                    return &p->buffers[i];
                continue;
            }
            av_log(p->log_ctx, AV_LOG_ERROR, "memory type %i not supported\n", p->memory);
            return NULL;
        }
    }

    return NULL;
}

V4LBuffer* avpriv_v4lbufpool_getfreebuf(V4LBufferPool *p, const AVFrame *f, const AVPacket* pkt)
{
    V4LBuffer* ret;
    int i;

#ifdef V4L_BUFFER_DEBUG
    av_log(p->log_ctx, AV_LOG_DEBUG, "Polling for a free buffer on %s\n", p->name);
#endif

    if (V4L2_TYPE_IS_OUTPUT(p->type)) {
        for (;;) {
            if (!dequeue_v4lbuf(p)) {
                break;
            }
        }
    }

    if (f) {
        ret = v4lbufpool_get_from_avframe(f, p);
        if (ret)
            return ret;
    }

    for (i = 0; i < p->num_buffers; i++) {
        if (p->buffers[i].status == V4LBUF_AVAILABLE)
            return &p->buffers[i];
    }

    return NULL;
}

int avpriv_set_stream_status(V4LBufferPool* bp, int cmd)
{
    int type = bp->type;
    int ret;

    ret = ioctl(bp->fd, cmd, &type);
    if (ret < 0)
        return AVERROR(errno);

    bp->streamon = (cmd == VIDIOC_STREAMON);

    return 0;
}

int avpriv_v4l_enqueue_frame_or_pkt_or_buf(V4LBufferPool* bp, const AVFrame* f, const AVPacket* pkt, const uint8_t* buf, int buf_size)
{
    V4LBuffer* avbuf;
    int ret;

    if (!f && !pkt && !buf) {
        av_log(bp->log_ctx, AV_LOG_ERROR, "either AVFrame*, AVPacket* or buf must valid\n");
        return AVERROR_BUG;
    }

    avbuf = avpriv_v4lbufpool_getfreebuf(bp, f, pkt);
    if (!avbuf)
        return AVERROR(ENOMEM);

    if (f && (ret = avframe_to_v4lbuf(f, avbuf)))
        return ret;

    if (pkt && (ret = avpkt_to_v4lbuf(pkt, avbuf)))
        return ret;

    if (buf && (ret = buf2v4l(avbuf, 0, buf, buf_size, NULL)))
        return ret;

    if (ret = enqueue_v4lbuf(avbuf))
        return ret;

    return 0;
}

int avpriv_v4l_dequeue_frame_or_pkt(V4LBufferPool* bp, AVFrame* f, AVPacket* pkt)
{
    V4LBuffer* avbuf = NULL;

    if ((!f && !pkt) || (f && pkt)) {
        av_log(bp->log_ctx, AV_LOG_ERROR, "either AVFrame* or AVPacket* must be valid\n");
        return AVERROR_BUG;
    }

    if (!(avbuf = dequeue_v4lbuf(bp))) {
        if (bp->broken) {
            return AVERROR_EOF;
        }
        return AVERROR(EAGAIN);
    }

    if (f)
        return v4lbuf_to_avframe(f, avbuf);

    if (pkt)
        return v4lbuf_to_avpkt(pkt, avbuf);

    return AVERROR_BUG;
}



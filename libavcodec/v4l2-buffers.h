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

#ifndef AVCODEC_V4L2_BUFFERS_H
#define AVCODEC_V4L2_BUFFERS_H

#include "v4l2-common.h"
#include "avcodec.h"
#include "libavutil/pixfmt.h"
#include "libavutil/frame.h"

struct V4LBuffer;
typedef struct V4LBuffer V4LBuffer;

struct V4LBufferPool;
typedef int (*format_f)(struct V4LBufferPool *, int set);
typedef int (*init_f)(struct V4LBufferPool *);

typedef struct V4LBufferPoolCfg {
    format_f format;
    init_f init;
} V4LBufferPoolCfg;

typedef struct V4LBufferPool {
    /**
     * Buffer pool initial configuration.
     */
    V4LBufferPoolCfg cfg;

    /**
     * Log context (for av_log()). Can be NULL.
     */
    void *log_ctx;

    /**
     * Pool's name. Must be set before calling avpriv_init_v4lbufpool().
     */
    const char* name;

    /**
     * File descriptor obtained from opening the associated device.
     * Must be set before calling avpriv_init_v4lbufpool().
     * Readonly after init.
     */
    int fd;

    /**
     * Type of this buffer pool.
     * See V4L2_BUF_TYPE_VIDEO_* in videodev2.h
     * Must be set before calling avpriv_init_v4lbufpool().
     * Readonly after init.
     */
    enum v4l2_buf_type type;

    /**
     * Memory type this buffer pool uses.
     * See V4L2_MEMORY_* in videodev2.h
     * Must be set before calling avpriv_init_v4lbufpool().
     * Readonly after init.
     */
    enum v4l2_memory memory;

    /**
     * AVPixelFormat corresponding to this buffer pool.
     * AV_PIX_FMT_NONE means this is an encoded stream.
     */
    enum AVPixelFormat av_pix_fmt;

    /**
     * AVCodecID corresponding to this buffer pool.
     * AV_CODEC_ID_RAWVIDEO means this is a raw stream and av_pix_fmt must be set to a valid value.
     */
    enum AVCodecID   av_codec_id;

    /**
     * fourcc (LSB first, so "ABCD" -> ('D'<<24) + ('C'<<16) + ('B'<<8) + 'A').
     * This is used to work around some encoder bugs.
     * A demuxer should set this to what is stored in the field used to identify the codec.
     * If there are multiple such fields in a container then the demuxer should choose the one
     * which maximizes the information about the used codec.
     * If the codec tag field in a container is larger than 32 bits then the demuxer should
     * remap the longer ID to 32 bits with a table or other structure. Alternatively a new
     * extra_codec_tag + size could be added but for this a clear advantage must be demonstrated
     * first.
     * - encoding: Set by user, if not then the default based on codec_id will be used.
     * - decoding: Set by user, will be converted to uppercase by libavcodec during init.
     */
    unsigned int av_codec_tag;

    /**
     * Format returned by the driver after initializing the buffer pool.
     * Must be set before calling avpriv_init_v4lbufpool().
     * avpriv_set_pool_format() can set it.
     * Readonly after init.
     */
    struct v4l2_format format;

    /**
     * Width and height of the frames it produces (in case of a capture pool, e.g. when decoding)
     * or accepts (in case of an output pool, e.g. when encoding).
     *
     * For output pools, this must must be set before calling avpriv_init_v4lbufpool().
     * For capture pools, it will be set after having received the information from the driver.
     */
    int width, height;

    /**
     * Default flags to set on buffers to enqueue.
     * See V4L2_BUF_FLAG_*.
     */
    int default_flags;

    /**
     * Whether the stream has been started (VIDIOC_STREAMON has been sent).
     */
    int streamon;

    /**
     * Number of queued buffers.
     */
    int num_queued;

    /**
     * Time (in ms) we can wait for a buffer before considering it a failure.
     */
    int blocking_dequeue;

    /**
     * Minimum number of buffers that must be kept queued in this queue.
     *
     * E.g. for decoders, the drivers might have such requirements to produce proper output.
     */
    int min_queued_buffers;

    /**
     * The actual number of buffers.
     *
     * Before calling avpriv_init_v4lbufpool() this is the number of buffers we would like to have available.
     * avpriv_init_v4lbufpool() asks for (min_buffers + num_buffers) and sets this value to the actual number
     * of buffers the driver gave us.
     * Readonly after init.
     */
    int num_buffers;

    /**
     * Opaque pointers to the actual buffers representations.
     * After initialization, it is an array of size num_buffers.
     */
    V4LBuffer *buffers;

    /**
     * Pool in unrecoverable error notified by the V4L2 kernel api
     */
    int broken;

} V4LBufferPool;

/**
 * Initializes a V4LBufferPool.
 *
 * @param[in] bp A pointer to a V4LBufferPool. See V4LBufferPool description for required variables.
 * @return 0 in case of success, a negative value representing the error otherwise.
 */
int avpriv_init_v4lbufpool(V4LBufferPool* bp);

/**
 * Releases a V4LBufferPool.
 *
 * @param[in] bp A pointer to a V4LBufferPool.
 *               The caller is reponsible for freeing it.
 *               It must not be used after calling this function.
 */
void avpriv_release_buffer_pool(V4LBufferPool* bp);

/**
 * Sets the status of a V4LBufferPool.
 *
 * @param[in] bp A pointer to a V4LBufferPool.
 * @param[in] cmd The status to set (VIDIOC_STREAMON or VIDIOC_STREAMOFF).
 *                Warning: If VIDIOC_STREAMOFF is sent to a buffer pool that still has some frames buffered,
 *                those frames will be dropped.
 * @return 0 in case of success, a negative value representing the error otherwise.
 */
int avpriv_set_stream_status(V4LBufferPool* bp, int cmd);

/**
 * Dequeues a buffer from a V4LBufferPool to either an AVFrame or an AVPacket.
 *
 * Exactly one of f or pkt must be non NULL.
 * @param[in] bp The V4LBufferPool to dequeue from.
 * @param[inout] f The AVFrame to dequeue to.
 * @param[inout] pkt The AVPacket to dequeue to.
 * @return 0 in case of success, AVERROR(EAGAIN) if no buffer was ready, another negative error in case of error.
 */
int avpriv_v4l_dequeue_frame_or_pkt(V4LBufferPool* bp, AVFrame* f, AVPacket* pkt);

/**
 * Enqueues a buffer to a V4LBufferPool from either an AVFrame, an AVPacket or a raw buffer.
 * Exactly one of f, pkt or buf must be non NULL.
 *
 * @param[in] bp The V4LBufferPool to enqueue to.
 * @param[in] f A pointer to an AVFrame to enqueue.
 * @param[in] pkt A pointer to an AVPacket to enqueue.
 * @param[in] buf A pointer to a buffer to enqueue.
 * @param[in] buf_size The size of the buffer pointed by buf.
 * @return 0 in case of success, a negative error otherwise.
 */
int avpriv_v4l_enqueue_frame_or_pkt_or_buf(V4LBufferPool* bp, const AVFrame* f, const AVPacket* pkt, const uint8_t* buf, int buf_size);

/**
 * Gets a free V4LBuffer from a V4LBufferPool.
 *
 * If no matching buffer is found (see below), it tries to dequeue a buffer first
 * in order to minimize the size of the V4L queue.e
 *
 * @param[in] p Pointer to a V4LBufferPool where to get the buffer from.
 * @param[in] f A pointer to an existing AVFrame:
 *              If the AVFrame's buffers match a V4LBuffer, this V4LBuffer will be returned.
 *              Can be NULL.
 * @param[in] pkt A pointer to an existing AVPacket:
 *                If the AVPacket's buffers match a V4LBuffer, this V4LBuffer will be returned.
 *                Can be NULL.
 * @return A pointer to the V4LBuffer or NULL in case of error.
 */
V4LBuffer* avpriv_v4lbufpool_getfreebuf(V4LBufferPool *p, const AVFrame *f, const AVPacket* pkt);

#endif // AVCODEC_V4L2_BUFFERS_H

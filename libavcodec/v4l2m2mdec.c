#include <linux/videodev2.h>
#include <libudev.h>

#include <drm_fourcc.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>


#include "avcodec.h"
#include "decode.h"
#include "hwaccel.h"

#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"

typedef struct {
    int index;
    int fd;
    uint8_t *addr;
    uint32_t width;
    uint32_t height;
    uint32_t size;
    uint32_t used;
    struct v4l2_buffer buffer;
} V4L2Buffer;

typedef struct {
    V4L2Buffer *output_buffers;
    int num_output_buffers;
    V4L2Buffer *capture_buffers;
    int num_capture_buffers;
    AVBufferRef *frames_ref;
    AVBufferRef *device_ref;
} V4L2Decoder;

typedef struct {
    AVClass *class;
    enum v4l2_buf_type output_type;
    struct v4l2_format format;
    int video_fd;
    AVClass *av_class;
    AVBufferRef *decoder_ref;
} V4L2DecodeContext;

typedef struct {
    int fd;
    AVBufferRef *decoder_ref;
} V4L2FrameContext;

const uint32_t v4l2_capture_pixelformats[] = {
    V4L2_PIX_FMT_YUV420,
    V4L2_PIX_FMT_NV12,
};

static uint32_t v4l2_format_avcodec_to_v4l2(AVCodecContext *avctx)
{
    switch (avctx->codec_id) {
    case AV_CODEC_ID_H264:          return V4L2_PIX_FMT_H264;
    default:                        return 0;
    }
}

static int v4l2_close_decoder(AVCodecContext *avctx)
{
    V4L2DecodeContext *ctx = avctx->priv_data;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p\n", __func__, avctx, ctx);

    av_buffer_unref(&ctx->decoder_ref);

    if (ctx->video_fd >= 0) {
        close(ctx->video_fd);
    }

    return 0;
}

static void v4l2_release_decoder(void *opaque, uint8_t *data)
{
    V4L2Decoder *decoder = (V4L2Decoder *)data;

    av_buffer_unref(&decoder->frames_ref);
    av_buffer_unref(&decoder->device_ref);

    av_free(decoder);
}

static int v4l2_buffer_alloc(AVCodecContext *avctx, V4L2Buffer *buf, enum v4l2_buf_type type)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    struct v4l2_create_buffers buffers = {0};
    int ret;
    unsigned int n;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p buf=%p type=%u\n", __func__, avctx, buf, type);

    buffers.format.type = type;
    buffers.memory = V4L2_MEMORY_MMAP;
    buffers.count = 10;

    ret = ioctl(ctx->video_fd, VIDIOC_G_FMT, &buffers.format);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get format failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
        return ret;
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: pixelformat=%d width=%u height=%u bytesperline=%u sizeimage=%u\n", __func__, buffers.format.fmt.pix.pixelformat, buffers.format.fmt.pix.width, buffers.format.fmt.pix.height, buffers.format.fmt.pix.bytesperline, buffers.format.fmt.pix.sizeimage);

    ret = ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: create buffers failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
        return ret;
    }

    buf = av_mallocz(sizeof(V4L2Buffer) * buffers.count);
    if (!buf) {
        return AVERROR(ENOMEM);
    }

    for (n = 0; n < buffers.count; ++n) {
        buf[n].index = buffers.index;
        buf[n].width = buffers.format.fmt.pix.width;
        buf[n].height = buffers.format.fmt.pix.height;
        buf[n].size = buffers.format.fmt.pix.sizeimage;
        buf[n].used = 0;

        buf[n].buffer.type = type;
        buf[n].buffer.memory = V4L2_MEMORY_MMAP;
        buf[n].buffer.index = n;

        ret = ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &buf[n].buffer);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: query buffer %d failed, %s (%d)\n", __func__, buf->index, strerror(errno), errno);
            return ret;
        }

        if (V4L2_TYPE_IS_OUTPUT(type)) {
            void *addr = mmap(NULL, buf[n].size, PROT_READ | PROT_WRITE, MAP_SHARED, ctx->video_fd, buf[n].buffer.m.offset);
            if (addr == MAP_FAILED) {
                av_log(avctx, AV_LOG_ERROR, "%s: mmap failed, %s (%d)\n", __func__, strerror(errno), errno);
                return -1;
            }

            buf[n].addr = (uint8_t*)addr;
        } else {
            struct v4l2_exportbuffer exportbuffer = {0};
            exportbuffer.type = type;
            exportbuffer.index = buf->index;
            exportbuffer.flags = O_RDONLY;

            ret = ioctl(ctx->video_fd, VIDIOC_EXPBUF, &exportbuffer);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "%s: export buffer %d failed, %s (%d)\n", __func__, buf->index, strerror(errno), errno);
                return ret;
            }

            buf[n].fd = exportbuffer.fd;
        }

        av_log(avctx, AV_LOG_DEBUG, "%s: buf=%p index=%d fd=%d addr=%p width=%u height=%u size=%u\n", __func__, &buf[n], buf[n].index, buf[n].fd, buf[n].addr, buf[n].width, buf[n].height, buf[n].size);
    }

    return buffers.count;
}

static int v4l2_init_context(AVCodecContext *avctx)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder = NULL;
    int ret;

    avctx->pix_fmt = AV_PIX_FMT_DRM_PRIME;

    // create a decoder and a ref to it
    decoder = av_mallocz(sizeof(V4L2Decoder));
    if (!decoder) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    ctx->decoder_ref = av_buffer_create((uint8_t *)decoder, sizeof(*decoder), v4l2_release_decoder, NULL, AV_BUFFER_FLAG_READONLY);
    if (!ctx->decoder_ref) {
        av_free(decoder);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    decoder->num_output_buffers = v4l2_buffer_alloc(avctx, decoder->output_buffers, V4L2_BUF_TYPE_VIDEO_OUTPUT);
    if (decoder->num_output_buffers < 0) {
        goto fail;
    }

    decoder->num_capture_buffers = v4l2_buffer_alloc(avctx, decoder->capture_buffers, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (decoder->num_capture_buffers < 0) {
        goto fail;
    }

    decoder->device_ref = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
    if (!decoder->device_ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    ret = av_hwdevice_ctx_init(decoder->device_ref);
    if (ret < 0) {
        goto fail;
    }

    return 0;

fail:
    v4l2_close_decoder(avctx);
    return ret;
}

static int v4l2_try_format(AVCodecContext *avctx, enum v4l2_buf_type type, uint32_t pixelformat)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };

    while (ioctl(ctx->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        if (fmtdesc.pixelformat == pixelformat)
            return 0;

        fmtdesc.index++;
    }

    av_log(avctx, AV_LOG_INFO, "%s: pixelformat %u not supported for type %u\n", __func__, pixelformat, type);
    return -1;
}

static int v4l2_set_format(AVCodecContext *avctx, enum v4l2_buf_type type, uint32_t pixelformat)
{
    int ret;
    V4L2DecodeContext *ctx = avctx->priv_data;
    struct v4l2_format format = {
        .type = type,
    };

    format.fmt.pix.width = avctx->coded_width;
    format.fmt.pix.height = avctx->coded_height;
    format.fmt.pix.pixelformat = pixelformat;

    if (!V4L2_TYPE_IS_OUTPUT(type)) {
        struct v4l2_selection selection = {
            .type = type,
            .r.height = avctx->height,
            .r.width = avctx->width,
            .r.left = 0,
            .r.top = 0,
            .target = V4L2_SEL_TGT_COMPOSE,
        };

        ret = ioctl(ctx->video_fd, VIDIOC_S_SELECTION, &selection);
        if (!ret) {
            ret = ioctl(ctx->video_fd, VIDIOC_G_SELECTION, &selection);
            if (ret) {
                av_log(avctx, AV_LOG_WARNING, "VIDIOC_G_SELECTION ioctl\n");
            } else {
                av_log(avctx, AV_LOG_DEBUG, "crop output %dx%d\n", selection.r.width, selection.r.height);
                /* update the size of the resulting frame */
                format.fmt.pix.width  = selection.r.width;
                format.fmt.pix.height = selection.r.height;
            }
        }
    }

    return ioctl(ctx->video_fd, VIDIOC_S_FMT, &format);
}

static int v4l2_select_capture_format(AVCodecContext *avctx)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    enum v4l2_buf_type type = ctx->format.type;

    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_capture_pixelformats); i++) {
        uint32_t pixelformat = v4l2_capture_pixelformats[i];
        if (!v4l2_try_format(avctx, type, pixelformat))
            return v4l2_set_format(avctx, type, pixelformat);
    }

    return -1;
}

static int v4l2_probe_video_device(struct udev_device *device, AVCodecContext *avctx)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    int ret = AVERROR(EINVAL);
    struct v4l2_capability capability = {0};
    unsigned int capabilities = 0;

    const char *path = udev_device_get_devnode(device);
    if (!path) {
        av_log(avctx, AV_LOG_ERROR, "%s: get video device devnode failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ctx->video_fd = open(path, O_RDWR | O_NONBLOCK, 0);
    if (ctx->video_fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, %s (%d)\n", __func__, path, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_QUERYCAP, &capability);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get video capability failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
        capabilities = capability.device_caps;
    else
        capabilities = capability.capabilities;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p path=%s capabilities=%u\n", __func__, avctx, ctx, path, capabilities);

    if ((capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING) {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required streaming capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if ((capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) == V4L2_CAP_VIDEO_M2M_MPLANE) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if ((capabilities & V4L2_CAP_VIDEO_M2M) == V4L2_CAP_VIDEO_M2M) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required mem2mem capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_try_format(avctx, ctx->output_type, v4l2_format_avcodec_to_v4l2(avctx));
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: try output format failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_set_format(avctx, ctx->output_type, v4l2_format_avcodec_to_v4l2(avctx));
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set output format failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_select_capture_format(avctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_WARNING, "%s: select capture format failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    return 0;

fail:
    if (ctx->video_fd >= 0) {
        close(ctx->video_fd);
        ctx->video_fd = -1;
    }

    return ret;
}

static int v4l2_init_decoder(AVCodecContext *avctx)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    int ret = AVERROR(EINVAL);
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices;
    struct udev_list_entry *entry;
    struct udev_device *device;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p hw_device_ctx=%p hw_frames_ctx=%p\n", __func__, avctx, avctx->hw_device_ctx, avctx->hw_frames_ctx);

    ctx->video_fd = -1;

    udev = udev_new();
    if (!udev) {
        av_log(avctx, AV_LOG_ERROR, "%s: allocating udev context failed\n", __func__);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    enumerate = udev_enumerate_new(udev);
    if (!enumerate) {
        av_log(avctx, AV_LOG_ERROR, "%s: allocating udev enumerator failed\n", __func__);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);

    devices = udev_enumerate_get_list_entry(enumerate);
    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        if (!path)
            continue;

        device = udev_device_new_from_syspath(udev, path);
        if (!device)
            continue;

        ret = v4l2_probe_video_device(device, avctx);
        udev_device_unref(device);

        if (!ret)
            break;
    }

    udev_enumerate_unref(enumerate);
    if (ret < 0)
        goto fail;

    ret = v4l2_init_context(avctx);

fail:
    udev_unref(udev);
    return ret;
}

static int v4l2_queue_buffer(V4L2DecodeContext *ctx, V4L2Buffer *buf)
{
    struct v4l2_buffer buffer = {
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
        .index = buf->index,
        .flags = 0,
    };

    if (V4L2_TYPE_IS_OUTPUT(buf->buffer.type)) {
        buffer.bytesused = buf->used;
    }

    return ioctl(ctx->video_fd, VIDIOC_QBUF, &buffer);
}

static int v4l2_dequeue_buffer(V4L2DecodeContext *ctx, V4L2Buffer *buf)
{
    struct v4l2_buffer buffer = {
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
    };
    int ret;

    ret =  ioctl(ctx->video_fd, VIDIOC_DQBUF, &buffer);
    if (ret < 0) {
        return ret;
    }

    return buffer.index;
}

static int v4l2_queue_packet(AVCodecContext *avctx, const AVPacket* pkt)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder = (V4L2Decoder *)ctx->decoder_ref->data;
    V4L2Buffer *buf = decoder->output_buffers;
    unsigned int bytesused;
    int ret;
    int i;

    for (i = 0; i < decoder->num_output_buffers; ++i) {
        ret = ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &buf[i].buffer);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: query buffer %d failed, %s (%d)\n", __func__, buf[i].index, strerror(errno), errno);
            return ret;
        }

        if (buf[i].buffer.flags == V4L2_BUF_FLAG_MAPPED) {
            bytesused = FFMIN(pkt->size, buf[i].size);

            memcpy(buf[i].addr, pkt->data, bytesused);

            buf[i].used = bytesused;

            return v4l2_queue_buffer(ctx, &buf[i]);
        }
    }

    return -1;
}

static int v4l2_try_start(AVCodecContext *avctx)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder = (V4L2Decoder *)ctx->decoder_ref->data;
    V4L2Buffer *output = decoder->output_buffers;
    V4L2Buffer *capture = decoder->capture_buffers;
    int ret;
    int i;

    ret = ioctl(ctx->video_fd, VIDIOC_STREAMON, &output[0].buffer.type);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: streamon output failed, %s (%d)\n", __func__, strerror(errno), errno);
        return ret;
    }

    for (i = 0; i < decoder->num_capture_buffers; ++i) {
        ret = v4l2_queue_buffer(ctx, &capture[i]);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: queue capture buffer %d failed, %s (%d)\n", __func__, capture[i].index, strerror(errno), errno);
            return ret;
        }
    }

    ret = ioctl(ctx->video_fd, VIDIOC_STREAMON, &capture[0].buffer.type);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: streamon capture failed, %s (%d)\n", __func__, strerror(errno), errno);
        return ret;
    }

    return 0;
}

static inline uint64_t v4l2_get_pts(AVCodecContext *avctx, V4L2Buffer *buf)
{
    AVRational v4l2_timebase = { 1, 1000000 };
    int64_t v4l2_pts;

    v4l2_pts = (int64_t)buf->buffer.timestamp.tv_sec * 1000000 + buf->buffer.timestamp.tv_usec;

    return av_rescale_q(v4l2_pts, v4l2_timebase, avctx->time_base);
}

static enum AVColorPrimaries v4l2_get_color_primaries(struct v4l2_format format)
{
    enum v4l2_ycbcr_encoding ycbcr;
    enum v4l2_colorspace cs;

    cs = format.fmt.pix.colorspace;
    ycbcr = format.fmt.pix.ycbcr_enc;

    switch(ycbcr) {
    case V4L2_YCBCR_ENC_XV709:
    case V4L2_YCBCR_ENC_709: return AVCOL_PRI_BT709;
    case V4L2_YCBCR_ENC_XV601:
    case V4L2_YCBCR_ENC_601:return AVCOL_PRI_BT470M;
    default:
        break;
    }

    switch(cs) {
    case V4L2_COLORSPACE_470_SYSTEM_BG: return AVCOL_PRI_BT470BG;
    case V4L2_COLORSPACE_SMPTE170M: return AVCOL_PRI_SMPTE170M;
    case V4L2_COLORSPACE_SMPTE240M: return AVCOL_PRI_SMPTE240M;
    case V4L2_COLORSPACE_BT2020: return AVCOL_PRI_BT2020;
    default:
        break;
    }

    return AVCOL_PRI_UNSPECIFIED;
}

static enum AVColorRange v4l2_get_color_range(struct v4l2_format format)
{
    enum v4l2_quantization qt;

    qt = format.fmt.pix.quantization;

    switch (qt) {
    case V4L2_QUANTIZATION_LIM_RANGE: return AVCOL_RANGE_MPEG;
    case V4L2_QUANTIZATION_FULL_RANGE: return AVCOL_RANGE_JPEG;
    default:
        break;
    }

     return AVCOL_RANGE_UNSPECIFIED;
}

static enum AVColorSpace v4l2_get_color_space(struct v4l2_format format)
{
    enum v4l2_ycbcr_encoding ycbcr;
    enum v4l2_colorspace cs;

    cs = format.fmt.pix.colorspace;
    ycbcr = format.fmt.pix.ycbcr_enc;

    switch(cs) {
    case V4L2_COLORSPACE_SRGB: return AVCOL_SPC_RGB;
    case V4L2_COLORSPACE_REC709: return AVCOL_SPC_BT709;
    case V4L2_COLORSPACE_470_SYSTEM_M: return AVCOL_SPC_FCC;
    case V4L2_COLORSPACE_470_SYSTEM_BG: return AVCOL_SPC_BT470BG;
    case V4L2_COLORSPACE_SMPTE170M: return AVCOL_SPC_SMPTE170M;
    case V4L2_COLORSPACE_SMPTE240M: return AVCOL_SPC_SMPTE240M;
    case V4L2_COLORSPACE_BT2020:
        if (ycbcr == V4L2_YCBCR_ENC_BT2020_CONST_LUM)
            return AVCOL_SPC_BT2020_CL;
        else
             return AVCOL_SPC_BT2020_NCL;
    default:
        break;
    }

    return AVCOL_SPC_UNSPECIFIED;
}

static enum AVColorTransferCharacteristic v4l2_get_color_trc(struct v4l2_format format)
{
    enum v4l2_ycbcr_encoding ycbcr;
    enum v4l2_xfer_func xfer;
    enum v4l2_colorspace cs;

    cs = format.fmt.pix.colorspace;
    ycbcr = format.fmt.pix.ycbcr_enc;
    xfer = format.fmt.pix.xfer_func;

    switch (xfer) {
    case V4L2_XFER_FUNC_709: return AVCOL_TRC_BT709;
    case V4L2_XFER_FUNC_SRGB: return AVCOL_TRC_IEC61966_2_1;
    default:
        break;
    }

    switch (cs) {
    case V4L2_COLORSPACE_470_SYSTEM_M: return AVCOL_TRC_GAMMA22;
    case V4L2_COLORSPACE_470_SYSTEM_BG: return AVCOL_TRC_GAMMA28;
    case V4L2_COLORSPACE_SMPTE170M: return AVCOL_TRC_SMPTE170M;
    case V4L2_COLORSPACE_SMPTE240M: return AVCOL_TRC_SMPTE240M;
    default:
        break;
    }

    switch (ycbcr) {
    case V4L2_YCBCR_ENC_XV709:
    case V4L2_YCBCR_ENC_XV601: return AVCOL_TRC_BT1361_ECG;
    default:
        break;
    }

    return AVCOL_TRC_UNSPECIFIED;
}

static void v4l2_release_frame(void *opaque, uint8_t *data)
{
    AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)data;
    AVBufferRef *framecontextref = (AVBufferRef *)opaque;
    V4L2FrameContext *framecontext = (V4L2FrameContext *)framecontextref->data;

    close(framecontext->fd);
    av_buffer_unref(&framecontext->decoder_ref);
    av_buffer_unref(&framecontextref);

    av_free(desc);
}

static int v4l2_set_drm_descriptor(AVCodecContext *avctx, AVFrame *frame, int index)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder = (V4L2Decoder *)ctx->decoder_ref->data;
    V4L2FrameContext *framecontext = NULL;
    AVBufferRef *framecontextref = NULL;
    AVDRMFrameDescriptor *desc = NULL;
    AVDRMLayerDescriptor *layer = NULL;
    AVHWFramesContext *hwframes = NULL;
    struct v4l2_format format;
    V4L2Buffer *buffer = &decoder->capture_buffers[index];
    int ret;

    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(ctx->video_fd, VIDIOC_G_FMT, &format);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get format failed for type %u, %s (%d)\n", __func__, format.type, strerror(errno), errno);
        goto fail;
    }

    av_buffer_unref(&decoder->frames_ref);

    decoder->frames_ref = av_hwframe_ctx_alloc(decoder->device_ref);
    if (!decoder->frames_ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    avctx->width = format.fmt.pix.width;
    avctx->height = format.fmt.pix.height;

    hwframes = (AVHWFramesContext*)decoder->frames_ref->data;
    hwframes->format = format.fmt.pix.pixelformat;
    hwframes->sw_format = v4l2_capture_pixelformats[0]; // todo
    hwframes->width = avctx->width;
    hwframes->height = avctx->height;
    ret = av_hwframe_ctx_init(decoder->frames_ref);
    if (ret < 0)
        goto fail;

    // setup general frame fields
    frame->format = AV_PIX_FMT_DRM_PRIME;
    frame->width = buffer->width;
    frame->height = buffer->height;

    frame->color_primaries = v4l2_get_color_primaries(format);
    frame->colorspace = v4l2_get_color_space(format);
    frame->color_range = v4l2_get_color_range(format);
    frame->color_trc = v4l2_get_color_trc(format);
    frame->pts = v4l2_get_pts(avctx, buffer);

    desc = av_mallocz(sizeof(AVDRMFrameDescriptor));
    if (!desc) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    desc->nb_objects = 1;
    desc->objects[0].fd = buffer->fd;
    desc->objects[0].size = buffer->size;

    desc->nb_layers = 1;
    layer = &desc->layers[0];

    switch (format.fmt.pix.pixelformat) {
    case V4L2_PIX_FMT_NV12:
        layer->format = DRM_FORMAT_NV12;
        desc->objects[0].format_modifier = DRM_FORMAT_MOD_LINEAR;

        layer->nb_planes = 2;

        layer->planes[0].object_index = 0;
        layer->planes[0].offset = 0;
        layer->planes[0].pitch = format.fmt.pix.bytesperline;

        layer->planes[1].object_index = 0;
        layer->planes[1].offset = layer->planes[0].pitch * format.fmt.pix.height;
        layer->planes[1].pitch = layer->planes[0].pitch;

        break;
    case V4L2_PIX_FMT_YUV420:
        layer->format = DRM_FORMAT_YUV420;
        desc->objects[0].format_modifier = DRM_FORMAT_MOD_LINEAR;

        layer->nb_planes = 3;

        layer->planes[0].object_index = 0;
        layer->planes[0].offset = 0;
        layer->planes[0].pitch = format.fmt.pix.bytesperline;

        layer->planes[1].object_index = 0;
        layer->planes[1].offset = layer->planes[0].pitch * format.fmt.pix.height;
        layer->planes[1].pitch = layer->planes[0].pitch >> 1;

        layer->planes[2].object_index = 0;
        layer->planes[2].offset = layer->planes[1].offset + ((layer->planes[0].pitch * format.fmt.pix.height) >> 2);
        layer->planes[2].pitch = layer->planes[0].pitch >> 1;

        break;
    default:
        goto fail;
    }

    framecontextref = av_buffer_allocz(sizeof(*framecontext));
    if (!framecontextref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    framecontext = (V4L2FrameContext *)framecontextref->data;
    framecontext->decoder_ref = av_buffer_ref(ctx->decoder_ref);
    framecontext->fd = buffer->fd;

    frame->data[0] = (uint8_t *)desc;
    frame->buf[0] = av_buffer_create((uint8_t *)desc, sizeof(*desc), v4l2_release_frame,
                                     framecontextref, AV_BUFFER_FLAG_READONLY);

    if (!frame->buf[0]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    frame->hw_frames_ctx = av_buffer_ref(decoder->frames_ref);
    if (!frame->hw_frames_ctx) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    return 0;

fail:
    if (framecontext)
        av_buffer_unref(&framecontext->decoder_ref);

    if (framecontextref)
        av_buffer_unref(&framecontextref);

    if (desc)
        av_free(desc);

    return ret;
}

static int v4l2_dequeue_frame(AVCodecContext *avctx, AVFrame *frame)
{
    V4L2DecodeContext *ctx = avctx->priv_data;
    V4L2Decoder *decoder = (V4L2Decoder *)ctx->decoder_ref->data;
    V4L2Buffer *output = decoder->output_buffers;
    V4L2Buffer *capture = decoder->capture_buffers;

    for (;;) {
        fd_set fds[3];
        fd_set *read_fds = &fds[0]; /* for capture */
        fd_set *except_fds = &fds[1]; /* for exceptions */
        fd_set *write_fds = &fds[2]; /* for output */
        struct timeval tv;
        int index;
        int ret;

        if (read_fds) {
            FD_ZERO(read_fds);
            FD_SET(ctx->video_fd, read_fds);
        }

        if (except_fds) {
            FD_ZERO(except_fds);
            FD_SET(ctx->video_fd, except_fds);
        }

        if (write_fds) {
            FD_ZERO(write_fds);
            FD_SET(ctx->video_fd, write_fds);
        }

        /* Timeout. */
        tv.tv_sec = 0;
        tv.tv_usec = 500000;

        ret = select(ctx->video_fd + 1, read_fds, write_fds, except_fds, &tv);
        if (ret < 0) {
            if (EINTR == errno)
                continue;

            return -1;
        }

        if (ret == 0) {
            return -1;
        }

        if (read_fds && FD_ISSET(ctx->video_fd, read_fds)) {
            index = v4l2_dequeue_buffer(ctx, capture);
            if (index >= 0) {
                ret = v4l2_set_drm_descriptor(avctx, frame, index);
                if (ret < 0) {
                    return ret;
                }

                break;
            }
        }

        if (write_fds && FD_ISSET(ctx->video_fd, write_fds)) {
            index = v4l2_dequeue_buffer(ctx, output);
            if (index >= 0) {
                break;
            }
        }

        // if (except_fds && FD_ISSET(ctx->video_fd, except_fds)) {
        //     handle_event();
        // }
    }

    return AVERROR(EAGAIN);
}

static int v4l2_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    AVPacket avpkt = {0};
    int ret;

    ret = ff_decode_get_packet(avctx, &avpkt);
    if (ret < 0 && ret != AVERROR_EOF) {
        return ret;
    }

    ret = v4l2_queue_packet(avctx, &avpkt);
    if (ret < 0) {
        if (ret != AVERROR(ENOMEM))
           return ret;
        /* no input buffers available, continue dequeing */
    }

    if (avpkt.size) {
        ret = v4l2_try_start(avctx);
        if (ret)
            return 0;
    }

    return v4l2_dequeue_frame(avctx, frame);
}

static const AVCodecHWConfigInternal *v4l2_hw_configs[] = {
    HW_CONFIG_INTERNAL(DRM_PRIME),
    NULL
};

#define V4L2_DEC_CLASS(NAME) \
    static const AVClass v4l2_prime_##NAME##_dec_class = { \
        .class_name = "v4l2_prime_" #NAME "_decoder", \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define V4L2_DEC(NAME, ID, BSFS) \
    V4L2_DEC_CLASS(NAME) \
    AVCodec ff_##NAME##_v4l2_prime_decoder = { \
        .name           = #NAME "_v4l2", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (v4l2)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .priv_data_size = sizeof(V4L2DecodeContext), \
        .init           = v4l2_init_decoder, \
        .close          = v4l2_close_decoder, \
        .receive_frame  = v4l2_receive_frame, \
        .priv_class     = &v4l2_prime_##NAME##_dec_class, \
        .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_HARDWARE, \
        .pix_fmts       = (const enum AVPixelFormat[]) { AV_PIX_FMT_DRM_PRIME, \
                                                         AV_PIX_FMT_NONE}, \
        .hw_configs     = v4l2_hw_configs, \
        .bsfs           = BSFS, \
        .wrapper_name   = "v4l2", \
    };

V4L2_DEC(h264,  AV_CODEC_ID_H264,          "h264_mp4toannexb")
//V4L2_DEC(hevc,  AV_CODEC_ID_HEVC,          "hevc_mp4toannexb")
//V4L2_DEC(vp8,   AV_CODEC_ID_VP8,           NULL)
//V4L2_DEC(vp9,   AV_CODEC_ID_VP9,           NULL)
//V4L2_DEC(mpeg4, AV_CODEC_ID_MPEG4,         NULL)

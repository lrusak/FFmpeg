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

#include "v4l2-common.h"

const struct v4l_fmt_map avpriv_v4l_fmt_conversion_table[] = {
    /* ff_fmt             codec_id              v4l2_fmt                  pack_flags */
    { AV_PIX_FMT_YUV420P, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_YUV420     , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_YUV422P, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_YUV422P    , FF_V4L_PACK_AVPACKET                       },
    { AV_PIX_FMT_YUYV422, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_YUYV       , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_UYVY422, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_UYVY       , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_YUV411P, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_YUV411P    , FF_V4L_PACK_AVPACKET                       },
    { AV_PIX_FMT_YUV410P, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_YUV410     , FF_V4L_PACK_AVPACKET                       },
    { AV_PIX_FMT_YUV410P, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_YVU410     , FF_V4L_PACK_AVPACKET                       },
    { AV_PIX_FMT_RGB555LE,AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_RGB555     , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_RGB555BE,AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_RGB555X    , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_RGB565LE,AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_RGB565     , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_RGB565BE,AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_RGB565X    , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_BGR24,   AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_BGR24      , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_RGB24,   AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_RGB24      , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_BGR0,    AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_BGR32      , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_0RGB,    AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_RGB32      , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_GRAY8,   AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_GREY       , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
#ifdef V4L2_PIX_FMT_Y16
    { AV_PIX_FMT_GRAY16LE,AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_Y16        , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
#endif
    { AV_PIX_FMT_NV12,    AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_NV12       , FF_V4L_PACK_AVPACKET                       },
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_MJPEG,    V4L2_PIX_FMT_MJPEG      , FF_V4L_PACK_AVPACKET                       },
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_MJPEG,    V4L2_PIX_FMT_JPEG       , FF_V4L_PACK_AVPACKET                       },
#ifdef V4L2_PIX_FMT_H264
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_H264,     V4L2_PIX_FMT_H264       , FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_MPEG4
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_MPEG4,    V4L2_PIX_FMT_MPEG4      , FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_CPIA1
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_CPIA,     V4L2_PIX_FMT_CPIA1      , FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_SRGGB8
    { AV_PIX_FMT_BAYER_BGGR8, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_SBGGR8 , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_BAYER_GBRG8, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_SGBRG8 , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_BAYER_GRBG8, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_SGRBG8 , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
    { AV_PIX_FMT_BAYER_RGGB8, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_SRGGB8 , FF_V4L_PACK_AVPACKET | FF_V4L_PACK_AVFRAME },
#endif
#ifdef V4L2_PIX_FMT_NV12M
    { AV_PIX_FMT_NV12,    AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_NV12M      , FF_V4L_PACK_AVFRAME                        },
#endif
#ifdef V4L2_PIX_FMT_NV21M
    { AV_PIX_FMT_NV21,    AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_NV21M      , FF_V4L_PACK_AVFRAME                        },
#endif
#ifdef V4L2_PIX_FMT_YUV420M
    { AV_PIX_FMT_YUV420P, AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_YUV420M    , FF_V4L_PACK_AVFRAME                        },
#endif
#ifdef V4L2_PIX_FMT_NV16M
    { AV_PIX_FMT_NV16,    AV_CODEC_ID_RAWVIDEO, V4L2_PIX_FMT_NV16M      , FF_V4L_PACK_AVFRAME                        },
#endif
#ifdef V4L2_PIX_FMT_DV
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_DVVIDEO,  V4L2_PIX_FMT_DV         , FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_H263
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_H263,     V4L2_PIX_FMT_H263       , FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_MPEG1
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_MPEG1VIDEO, V4L2_PIX_FMT_MPEG1    , FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_MPEG2
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_MPEG2VIDEO, V4L2_PIX_FMT_MPEG2    , FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_VC1_ANNEX_G
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_VC1,      V4L2_PIX_FMT_VC1_ANNEX_G, FF_V4L_PACK_AVPACKET                       },
#endif
#ifdef V4L2_PIX_FMT_VP8
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_VP8,      V4L2_PIX_FMT_VP8        , FF_V4L_PACK_AVPACKET                       },
#endif
    { AV_PIX_FMT_NONE,    AV_CODEC_ID_NONE,     0                       , 0                                          },
};

uint32_t avpriv_v4l_fmt_ff2v4l(enum AVPixelFormat pix_fmt, enum AVCodecID codec_id, int pack_flags)
{
    int i;

    for (i = 0; avpriv_v4l_fmt_conversion_table[i].codec_id != AV_CODEC_ID_NONE; i++) {
        if ((codec_id == AV_CODEC_ID_NONE ||
             avpriv_v4l_fmt_conversion_table[i].codec_id == codec_id) &&
            (pix_fmt == AV_PIX_FMT_NONE ||
             avpriv_v4l_fmt_conversion_table[i].ff_fmt == pix_fmt) &&
             (avpriv_v4l_fmt_conversion_table[i].pack_flags & pack_flags)) {

            return avpriv_v4l_fmt_conversion_table[i].v4l2_fmt;
        }
    }

    return 0;
}

enum AVPixelFormat avpriv_v4l_fmt_v4l2ff(uint32_t v4l2_fmt, enum AVCodecID codec_id)
{
    int i;

    for (i = 0; avpriv_v4l_fmt_conversion_table[i].codec_id != AV_CODEC_ID_NONE; i++) {
        if (avpriv_v4l_fmt_conversion_table[i].v4l2_fmt == v4l2_fmt &&
            avpriv_v4l_fmt_conversion_table[i].codec_id == codec_id) {
            return avpriv_v4l_fmt_conversion_table[i].ff_fmt;
        }
    }

    return AV_PIX_FMT_NONE;
}

enum AVCodecID avpriv_v4l_fmt_v4l2codec(uint32_t v4l2_fmt)
{
    int i;

    for (i = 0; avpriv_v4l_fmt_conversion_table[i].codec_id != AV_CODEC_ID_NONE; i++) {
        if (avpriv_v4l_fmt_conversion_table[i].v4l2_fmt == v4l2_fmt) {
            return avpriv_v4l_fmt_conversion_table[i].codec_id;
        }
    }

    return AV_CODEC_ID_NONE;
}

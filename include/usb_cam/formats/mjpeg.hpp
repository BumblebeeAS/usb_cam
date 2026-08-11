// Copyright 2023 Evan Flynn
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Evan Flynn nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef USB_CAM__FORMATS__MJPEG_HPP_
#define USB_CAM__FORMATS__MJPEG_HPP_
///
/// @file Most of this code follows the example provided by ffmpeg:
///
/// https://www.ffmpeg.org/doxygen/5.1/decode__video_8c_source.html
/// https://www.ffmpeg.org/doxygen/4.0/decode__video_8c_source.html
///

#include <iostream>


extern "C" {
#define __STDC_CONSTANT_MACROS  // Required for libavutil
#include "libavutil/imgutils.h"
#include "libavformat/avformat.h"
#include "libavutil/error.h"
#include "libavutil/log.h"
#include "linux/videodev2.h"
#include "libswscale/swscale.h"
}

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/utils.hpp"
#include "usb_cam/formats/av_pixel_format_helper.hpp"


namespace usb_cam
{
namespace formats
{

// Map the decoder's full-range JPEG YUV formats (YUVJ*) to their plain YUV
// equivalents so swscale accepts them; full range is signalled separately.
static inline AVPixelFormat dejpeg_pixel_format(AVPixelFormat f)
{
  switch (f) {
    case AV_PIX_FMT_YUVJ420P: return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P: return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P: return AV_PIX_FMT_YUV444P;
    case AV_PIX_FMT_YUVJ440P: return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ411P: return AV_PIX_FMT_YUV411P;
    default:                  return f;
  }
}


class RAW_MJPEG : public pixel_format_base
{
public:
  explicit RAW_MJPEG(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "raw_mjpeg",
      V4L2_PIX_FMT_MJPEG,
      get_ros_pixel_format_from_av_format(args.av_device_format_str),
      get_channels_from_av_format(args.av_device_format_str),
      get_bit_depth_from_av_format(args.av_device_format_str),
      false)
  {}
};

class MJPEG2RGB : public pixel_format_base
{
public:
  explicit MJPEG2RGB(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "mjpeg2rgb",
      V4L2_PIX_FMT_MJPEG,
      usb_cam::constants::RGB8,
      3,
      8,
      true),
    m_avcodec(avcodec_find_decoder(AVCodecID::AV_CODEC_ID_MJPEG)),
    m_avframe_device(av_frame_alloc()),
    m_avpacket(av_packet_alloc()),
    m_averror_str(reinterpret_cast<char *>(malloc(AV_ERROR_MAX_STRING_SIZE))),
    m_width(args.width),
    m_height(args.height),
    m_rgb_size(static_cast<size_t>(args.width) * static_cast<size_t>(args.height) * 3)
  {
    if (!m_avcodec) {
      throw std::runtime_error("Could not find MJPEG decoder");
    }
    if (!m_avframe_device || !m_avpacket) {
      throw std::runtime_error("Could not allocate MJPEG decode frame/packet");
    }

    m_avcodec_context = avcodec_alloc_context3(m_avcodec);
    if (!m_avcodec_context) {
      throw std::runtime_error("Could not allocate MJPEG codec context");
    }

    // Hints only; the decoder determines the true output format/linesize at runtime.
    m_avcodec_context->width = args.width;
    m_avcodec_context->height = args.height;
    m_avcodec_context->codec_type = AVMEDIA_TYPE_VIDEO;
    // Single-threaded is lowest-latency here: at this resolution the cost is the
    // colorspace convert, which slice threading does not accelerate (profiled).
    // FRAME threading MUST NOT be used (it adds multi-frame pipeline latency).
    m_avcodec_context->thread_count = 1;
    m_avcodec_context->thread_type = FF_THREAD_SLICE;

    // Suppress warnings from ffmpeg libraries to avoid spamming the console
    av_log_set_level(AV_LOG_FATAL);
    av_log_set_flags(AV_LOG_SKIP_REPEATED);

    if (avcodec_open2(m_avcodec_context, m_avcodec, NULL) < 0) {
      throw std::runtime_error("Could not open decoder");
    }
    // NOTE: deliberately NO av_frame_get_buffer / sws_getContext here.
    // avcodec_receive_frame owns the decode buffer; sws is built lazily in
    // convert() once the decoder's real output format is known.
  }

  ~MJPEG2RGB()
  {
    if (m_averror_str) {free(m_averror_str);}
    if (m_avcodec_context) {
      avcodec_close(m_avcodec_context);
      avcodec_free_context(&m_avcodec_context);
    }
    if (m_avframe_device) {av_frame_free(&m_avframe_device);}
    if (m_avpacket) {av_packet_free(&m_avpacket);}
    if (m_sws_context) {sws_freeContext(m_sws_context);}
  }

  void convert(const char * & src, char * & dest, const int & bytes_used) override
  {
    // Zero-copy wrap of the borrowed V4L2 buffer. MJPEG decode is synchronous
    // within send_packet, so borrowing src for the duration of this call is safe.
    av_packet_unref(m_avpacket);
    m_avpacket->data = const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(src));
    m_avpacket->size = bytes_used;

    m_result = avcodec_send_packet(m_avcodec_context, m_avpacket);
    if (m_result < 0) {
      print_av_error_string(m_result);
      return;  // do not publish garbage; keep previous buffer contents
    }

    m_result = avcodec_receive_frame(m_avcodec_context, m_avframe_device);
    if (m_result == AVERROR(EAGAIN) || m_result == AVERROR_EOF) {
      return;
    } else if (m_result < 0) {
      print_av_error_string(m_result);
      return;
    }

    // Read the REAL decoded geometry/format (set by the decoder, not the ctor).
    const AVPixelFormat src_fmt =
      dejpeg_pixel_format(static_cast<AVPixelFormat>(m_avframe_device->format));
    const int src_w = m_avframe_device->width;
    const int src_h = m_avframe_device->height;
    const bool full_range =
      (m_avframe_device->color_range == AVCOL_RANGE_JPEG) ||
      (m_avframe_device->format == AV_PIX_FMT_YUVJ420P ||
      m_avframe_device->format == AV_PIX_FMT_YUVJ422P ||
      m_avframe_device->format == AV_PIX_FMT_YUVJ444P ||
      m_avframe_device->format == AV_PIX_FMT_YUVJ440P);

    // (Re)build sws only when the decoded description changes (once for a fixed cam).
    if (!m_sws_context || src_fmt != m_sws_src_fmt ||
      src_w != m_sws_src_w || src_h != m_sws_src_h)
    {
      m_sws_context = sws_getCachedContext(
        m_sws_context,
        src_w, src_h, src_fmt,
        m_width, m_height, AV_PIX_FMT_RGB24,
        (src_w == m_width && src_h == m_height) ? SWS_POINT : SWS_FAST_BILINEAR,
        NULL, NULL, NULL);
      if (!m_sws_context) {return;}

      if (full_range) {
        int * inv_table = nullptr;
        int * table = nullptr;
        int srcRange = 0, dstRange = 0, brightness = 0, contrast = 0, saturation = 0;
        sws_getColorspaceDetails(
          m_sws_context, &inv_table, &srcRange, &table, &dstRange,
          &brightness, &contrast, &saturation);
        sws_setColorspaceDetails(
          m_sws_context,
          sws_getCoefficients(SWS_CS_ITU601), 1 /*src full range*/,
          sws_getCoefficients(SWS_CS_ITU601), 1 /*dst full range (RGB)*/,
          brightness, contrast, saturation);
      }
      m_sws_src_fmt = src_fmt;
      m_sws_src_w = src_w;
      m_sws_src_h = src_h;
    }

    // Convert straight into the ROS RGB buffer (unpadded step == width*3).
    uint8_t * dst_data[4] = {reinterpret_cast<uint8_t *>(dest), nullptr, nullptr, nullptr};
    int dst_stride[4] = {m_width * 3, 0, 0, 0};
    sws_scale(
      m_sws_context,
      m_avframe_device->data, m_avframe_device->linesize, 0, src_h,
      dst_data, dst_stride);
  }

private:
  void print_av_error_string(int & err_code)
  {
    av_make_error_string(m_averror_str, AV_ERROR_MAX_STRING_SIZE, err_code);
    std::cerr << m_averror_str << std::endl;
  }

  const AVCodec * m_avcodec;
  AVCodecContext * m_avcodec_context = nullptr;
  AVFrame * m_avframe_device = nullptr;
  AVPacket * m_avpacket = nullptr;
  SwsContext * m_sws_context = nullptr;
  char * m_averror_str = nullptr;
  int m_result = 0;

  const int m_width;
  const int m_height;
  const size_t m_rgb_size;  // width*height*3; available if a defensive clear is wanted

  // sws cache keys (last decoded source description)
  int m_sws_src_fmt = AV_PIX_FMT_NONE;
  int m_sws_src_w = 0;
  int m_sws_src_h = 0;
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__MJPEG_HPP_

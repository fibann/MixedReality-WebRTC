/*
*  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/

#ifndef THIRD_PARTY_H264_WINUWP_H264ENCODER_H264ENCODER_H_
#define THIRD_PARTY_H264_WINUWP_H264ENCODER_H264ENCODER_H_

#include <mfapi.h>
#include <mfidl.h>
#include <Mfreadwrite.h>
#include <mferror.h>
#include <vector>
#include "H264MediaSink.h"
#include "IH264EncodingCallback.h"
#include "../Utils/SampleAttributeQueue.h"
#include "../Utils/Utils.h"
#include <fstream>
#include <functional>

#pragma comment(lib, "mfreadwrite")
#pragma comment(lib, "mfplat")
#pragma comment(lib, "mfuuid")

namespace rtc {
int64_t TimeMillis();
}

namespace webrtc {
namespace H264 {

enum Profile {
  kProfileConstrainedBaseline,
  kProfileBaseline,
  kProfileMain,
  kProfileConstrainedHigh,
  kProfileHigh,
};

}  // namespace H264

	class VideoFrame {
 public:

  // Get frame width.
  int width() const { return width_; }
  // Get frame height.
  int height() const { return height_; }
	  // Get frame size in pixels.
  uint32_t size() const;

  // System monotonic clock, same timebase as rtc::TimeMicros().
  int64_t timestamp_us() const { return timestamp_us_; }


  // Get frame timestamp (90kHz).
  uint32_t timestamp() const { return timestamp_rtp_; }

  const uint8_t* DataY() const { return video_frame_buffer_; }
  const uint8_t* DataU() const {
    return video_frame_buffer_ + StrideY() * height_;
  }
  const uint8_t* DataV() const {
    return video_frame_buffer_ + StrideY() * height_ + StrideU() * ((height_ + 1) / 2);
  }

  int StrideY() const { return width_; }
  int StrideU() const { return width_; }
  int StrideV() const { return width_; }

  // An opaque reference counted handle that stores the pixel data.
  const uint8_t* video_frame_buffer_;
  int width_;
  int height_;
  uint32_t timestamp_rtp_;
  int64_t ntp_time_ms_;
  int64_t timestamp_us_;
};

	class VideoCodec {
 public:
          VideoCodec() = default;

  // Public variables. TODO(hta): Make them private with accessors.
  unsigned char plType;

  // TODO(nisse): Change to int, for consistency.
  uint16_t width;
  uint16_t height;

  unsigned int startBitrate;   // kilobits/sec.
  unsigned int maxBitrate;     // kilobits/sec.
  unsigned int minBitrate;     // kilobits/sec.
  unsigned int targetBitrate;  // kilobits/sec.

  uint32_t maxFramerate;

  // This enables/disables encoding and sending when there aren't multiple
  // simulcast streams,by allocating 0 bitrate if inactive.
  bool active;

  unsigned int qpMax;
  unsigned char numberOfSimulcastStreams;

  bool expect_encode_from_texture;
  bool operator==(const VideoCodec& other) const = delete;
  bool operator!=(const VideoCodec& other) const = delete;
};


class H264MediaSink;

using EncodedImageCallback = std::function<void(const uint8_t*, int64_t, int32_t, bool)>;

class WinUWPH264EncoderImpl : public IH264EncodingCallback {
 public:
  WinUWPH264EncoderImpl();

  ~WinUWPH264EncoderImpl();

  // === VideoEncoder overrides ===
  int InitEncode(const VideoCodec* codec_settings,
    int number_of_cores, size_t max_payload_size);
  int RegisterEncodeCompleteCallback(EncodedImageCallback* callback);
  int Release();
  int Encode(const VideoFrame& input_image, bool keyframe);
  int SetRates(uint32_t new_bitrate_kbit, uint32_t frame_rate);

  // === IH264EncodingCallback overrides ===
  void OnH264Encoded(ComPtr<IMFSample> sample) override;

 private:
  ComPtr<IMFSample> FromVideoFrame(const VideoFrame& frame);
  int InitWriter();
  int ReleaseWriter();
  LONGLONG GetFrameTimestampHns(const VideoFrame& frame) const;
  int ReconfigureSinkWriter(UINT32 new_width,
                            UINT32 new_height,
                            UINT32 new_target_bps,
                            UINT32 new_frame_rate);

 private:
  CritSec crit_;
  CritSec callbackCrit_;
  bool inited_ {};
  ComPtr<IMFSinkWriter> sinkWriter_;
  ComPtr<H264MediaSink> mediaSink_;
  EncodedImageCallback* encodedCompleteCallback_ {};
  DWORD streamIndex_ {};
  LONGLONG startTime_ {};
  LONGLONG lastTimestampHns_ {};
  bool firstFrame_ {true};
  int framePendingCount_ {};
  DWORD frameCount_ {};
  bool lastFrameDropped_ {};

  int last_keyframe_req_frame_count = 0;
  int last_keyframe_produced_frame_count = 0;

  //These fields are never used
  /*
  UINT32 currentWidth_ {};
  UINT32 currentHeight_ {};
  UINT32 currentBitrateBps_ {};
  UINT32 currentFps_ {};
  */
  UINT32 max_bitrate_;

  UINT32 width_;
  UINT32 height_;
  UINT32 encoded_height_;
  UINT32 frame_rate_;
  UINT32 target_bps_;
  // H.264 specifc parameters
  bool frame_dropping_on_;
  int key_frame_interval_;

  int64_t last_rate_change_time_rtc_ms {};
  bool rate_change_requested_ {};

  // Values to use as soon as the min interval between rate changes has passed
  UINT32 next_frame_rate_;
  UINT32 next_target_bps_;

  struct CachedFrameAttributes {
    uint32_t timestamp;
    uint32_t duration;
    uint64_t ntpTime;
    uint64_t captureRenderTime;
    uint32_t frameWidth;
    uint32_t frameHeight;
  };
  SampleAttributeQueue<CachedFrameAttributes> _sampleAttributeQueue;

  class RateWindow {
   public:
    void AddSample(unsigned long sample, int64_t ts) {
      samples_.push_back({sample, ts});
    }

    int GetSumUpTo(int64_t ts) {
      unsigned long res = 0;
      while (samples_.front().ts < ts) {
        res += samples_.front().value;
        samples_.pop_front();
      }
      return res;
    }

   private:
    struct Sample {
      unsigned long value;
      int64_t ts;
    };

    std::deque<Sample> samples_;
  };
  std::ofstream compressed_file_out_;
  RateWindow bitrate_window_;
  RateWindow framerate_window_;
  int64_t last_stats_time_;
  int max_qp_;
  H264::Profile profile_;
};  // end of WinUWPH264EncoderImpl class

}  // namespace webrtc
#endif  // THIRD_PARTY_H264_WINUWP_H264ENCODER_H264ENCODER_H_


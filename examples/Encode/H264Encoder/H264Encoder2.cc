/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "H264Encoder.h"

#include <Windows.h>
#include <stdlib.h>
#include <ppltasks.h>
#include <mfapi.h>
#include <robuffer.h>
#include <wrl.h>
#include <mfidl.h>
#include <codecapi.h>
#include <mfreadwrite.h>
#include <wrl\implements.h>
#include <sstream>
#include <vector>
#include <iomanip>
#include <windows.graphics.holographic.h>

#include "H264StreamSink.h"
#include "H264MediaSink.h"
#include "../Utils/Utils.h"
#include <libyuv.h>

DebugLog debugLog;
#if defined(WINAPI_FAMILY) && (WINAPI_FAMILY == WINAPI_FAMILY_APP)
#define WINUWP
#endif
#define WEBRTC_WIN

#if defined(WEBRTC_WIN)
static const uint64_t kFileTimeToUnixTimeEpochOffset = 116444736000000000ULL;
static const uint64_t kNTPTimeToUnixTimeEpochOffset = 2208988800000L;
#endif

int64_t gAppStartTime = -1;      // Record app start time
int64_t gTimeSinceOsStart = -1;  // when app start,
int64_t gOsTicksPerSecond = -1;
static const int64_t kNumMillisecsPerSec = INT64_C(1000);
static const int64_t kNumMicrosecsPerSec = INT64_C(1000000);
static const int64_t kNumNanosecsPerSec = INT64_C(1000000000);
static const int64_t kNumMicrosecsPerMillisec =
    kNumMicrosecsPerSec / kNumMillisecsPerSec;
static const int64_t kNumNanosecsPerMillisec =
    kNumNanosecsPerSec / kNumMillisecsPerSec;
static const int64_t kNumNanosecsPerMicrosec =
    kNumNanosecsPerSec / kNumMicrosecsPerSec;

#if defined(WEBRTC_WIN)
// Warning, right now, the gAppStartTime and gTimeSinceOsStart are not protected
// with mutex. we only call this function to sync the clock of testing device
// with ntp when the app starts. if we want to call this function periodically
// in the runtime,then, suggest to use mutex
void SyncWithNtp(int64_t timeFromNtpServer /*in ms*/) {
  TIME_ZONE_INFORMATION timeZone;
  GetTimeZoneInformation(&timeZone);
  int64_t timeZoneBias =
      (int64_t)timeZone.Bias * 60 * 1000 * 1000 * 1000;  // ns

  gAppStartTime =
      (timeFromNtpServer - kNTPTimeToUnixTimeEpochOffset) * 1000000 -
      timeZoneBias;

  // Since we just update the app reference time, need to update the reference
  // point as well.

  LARGE_INTEGER qpfreq;
  QueryPerformanceFrequency(&qpfreq);
  gOsTicksPerSecond = qpfreq.QuadPart;

  LARGE_INTEGER qpcnt;
  QueryPerformanceCounter(&qpcnt);
  gTimeSinceOsStart = (int64_t)(
      (((uint64_t)qpcnt.QuadPart) * 100000ull / ((uint64_t)gOsTicksPerSecond)) *
      10000ull);  // ns
}

inline void InitializeAppStartTimestamp() {
  if (gTimeSinceOsStart != -1)  // already initialized
    return;

  TIME_ZONE_INFORMATION timeZone;
  GetTimeZoneInformation(&timeZone);
  int64_t timeZoneBias =
      (int64_t)timeZone.Bias * 60 * 1000 * 1000 * 1000;  // ns
  FILETIME ft;                                           // In hns.
  GetSystemTimeAsFileTime(&ft);  // this will give us system file in UTC format
  LARGE_INTEGER li;
  li.HighPart = ft.dwHighDateTime;
  li.LowPart = ft.dwLowDateTime;

  gAppStartTime = (li.QuadPart - kFileTimeToUnixTimeEpochOffset) * 100  // ns
                  - timeZoneBias;

  LARGE_INTEGER qpfreq;
  QueryPerformanceFrequency(&qpfreq);
  gOsTicksPerSecond = qpfreq.QuadPart;

  LARGE_INTEGER qpcnt;
  QueryPerformanceCounter(&qpcnt);
  gTimeSinceOsStart = (int64_t)(
      (((uint64_t)qpcnt.QuadPart) * 100000ull / ((uint64_t)gOsTicksPerSecond)) *
      10000ull);  // ns
}
#else
void SyncWithNtp(int64_t timeFromNtpServer /*in ms*/) {}
#endif  // WEBRTC_WIN
inline int64_t SystemTimeNanos() {
  int64_t ticks;
#if defined(WINUWP)
  InitializeAppStartTimestamp();
  LARGE_INTEGER qpcnt;
  QueryPerformanceCounter(&qpcnt);
  ticks = (int64_t)(
      (((uint64_t)qpcnt.QuadPart) * 100000ull / ((uint64_t)gOsTicksPerSecond)) *
      10000ull);  // ns
  ticks = gAppStartTime + ticks - gTimeSinceOsStart;
#elif defined(WEBRTC_WIN)
  static volatile LONG last_timegettime = 0;
  static volatile int64_t num_wrap_timegettime = 0;
  volatile LONG* last_timegettime_ptr = &last_timegettime;
  DWORD now = timeGetTime();
  // Atomically update the last gotten time
  DWORD old = InterlockedExchange(last_timegettime_ptr, now);
  if (now < old) {
    // If now is earlier than old, there may have been a race between threads.
    // 0x0fffffff ~3.1 days, the code will not take that long to execute
    // so it must have been a wrap around.
    if (old > 0xf0000000 && now < 0x0fffffff) {
      num_wrap_timegettime++;
    }
  }
  ticks = now + (num_wrap_timegettime << 32);
  // TODO(deadbeef): Calculate with nanosecond precision. Otherwise, we're
  // just wasting a multiply and divide when doing Time() on Windows.
  ticks = ticks * kNumNanosecsPerMillisec;
#else
#error Unsupported platform.
#endif
  return ticks;
}

namespace rtc {

int64_t SystemTimeMillis() {
  return static_cast<int64_t>(SystemTimeNanos() / kNumNanosecsPerMillisec);
}

int64_t TimeNanos() {

  return SystemTimeNanos();
}

uint32_t Time32() {
  return static_cast<uint32_t>(TimeNanos() / kNumNanosecsPerMillisec);
}

int64_t TimeMillis() {
  return TimeNanos() / kNumNanosecsPerMillisec;
}
}  // namespace rtc

#pragma comment(lib, "mfreadwrite")
#pragma comment(lib, "mfplat")
#pragma comment(lib, "mfuuid.lib")

// If true, pad frames whose height is not multiple of 16 on Hololens.
// Otherwise, crop the frames.
// User code should extern-declare and set this. There seems to be no easy way
// to pass arbitrary parameters to the encoder so this is the best (easy)
// option.
// Determines which H.264 profile to use for encoding.
// Note: by default we should use what's passed by WebRTC on codec
// initialization (which seems to be always ConstrainedBaseline), but we use
// Baseline to avoid changing behavior compared to earlier versions.
std::atomic<webrtc::H264::Profile> webrtc__WinUWPH264EncoderImpl__profile =
    webrtc::H264::kProfileBaseline;

int webrtc__WinUWPH264EncoderImpl__maxQp = -1;
int webrtc__WinUWPH264EncoderImpl__quality = -1;

namespace webrtc {

// QP scaling thresholds.
static constexpr int kLowH264QpThreshold = 24;
static constexpr int kHighH264QpThreshold = 37;

static constexpr unsigned kMaxH264Qp = 51;

// On some encoders (e.g. Hololens) changing rates is slow and will cause
// visible stuttering, so we don't want to do it too often.
// todo(fibann): we are ignoring small variations which means the rates might
// end up being off the requested value by a small amount in the long term. We
// should not ignore small variations but possibly use a longer min interval so
// they are eventually applied.
static constexpr int kMinIntervalBetweenRateChangesMs = 5000;
static constexpr float kMinRateVariation = 0.1f;

//////////////////////////////////////////
// H264 WinUWP Encoder Implementation
//////////////////////////////////////////

WinUWPH264EncoderImpl::WinUWPH264EncoderImpl()
{
  HRESULT hr = S_OK;
  ON_SUCCEEDED(MFStartup(MF_VERSION));
}

WinUWPH264EncoderImpl::~WinUWPH264EncoderImpl() {
  Release();
  HRESULT hr = S_OK;
  ON_SUCCEEDED(MFShutdown());
}

namespace {
using namespace Microsoft::WRL::Wrappers;
using namespace Windows::Foundation;
using namespace ABI::Windows::Graphics::Holographic;

bool CheckIfHololens() {
  // The best way to check if we are running on Hololens is checking if this is
  // a x86 Windows device with a transparent holographic display (AR).


#if defined(_M_IX86) /* x86 */ && \
    defined(WINAPI_FAMILY) && \
    (WINAPI_FAMILY == WINAPI_FAMILY_APP) /* UWP app */ && \
    defined(_WIN32_WINNT_WIN10) && \
    _WIN32_WINNT >= _WIN32_WINNT_WIN10 /* Win10 */

#define RETURN_IF_ERROR(...) \
  if (FAILED(__VA_ARGS__)) { \
    return false;            \
  }

  // HolographicSpace.IsAvailable
  ComPtr<IHolographicSpaceStatics2> holo_space_statics;
  RETURN_IF_ERROR(GetActivationFactory(
      HStringReference(
          RuntimeClass_Windows_Graphics_Holographic_HolographicSpace)
          .Get(),
      &holo_space_statics));
  boolean is_holo_space_available;
  RETURN_IF_ERROR(
      holo_space_statics->get_IsAvailable(&is_holo_space_available));
  if (!is_holo_space_available) {
	// Not a holographic device.
    return false;
  }

  // HolographicDisplay.GetDefault().IsOpaque
  ComPtr<IHolographicDisplayStatics> holo_display_statics;
  RETURN_IF_ERROR(GetActivationFactory(
      HStringReference(
          RuntimeClass_Windows_Graphics_Holographic_HolographicDisplay)
          .Get(),
      &holo_display_statics));
  ComPtr<IHolographicDisplay> holo_display;
  RETURN_IF_ERROR(holo_display_statics->GetDefault(&holo_display));
  boolean is_opaque;
  RETURN_IF_ERROR(holo_display->get_IsOpaque(&is_opaque));
  // Hololens if not opaque (otherwise VR).
  return !is_opaque;
#undef RETURN_IF_ERROR
#else
  return false;
#endif
}

bool IsHololens() {
  static const bool is_hololens = CheckIfHololens();
  return is_hololens;
}
}

int WinUWPH264EncoderImpl::InitEncode(const VideoCodec* codec_settings,
  int /*number_of_cores*/,
  size_t /*maxPayloadSize */) {

  if (!codec_settings) {
	  debugLog << "H264 UWP Encoder not registered as H264 codec";
	  return -4;
  }
  if (codec_settings->maxFramerate == 0) {
	  debugLog << "H264 UWP Encoder has no framerate defined";
	  return -4;
  }
  if (codec_settings->width < 1 || codec_settings->height < 1) {
	  debugLog << "H264 UWP Encoder has no valid frame size defined";
	  return -4;
  }

  width_ = codec_settings->width;
  height_ = codec_settings->height;

  // WebRTC only passes the max frame rate so use it as the initial value for
  // the desired frame rate too.
  frame_rate_ = codec_settings->maxFramerate;

  max_qp_ = 51;
  //std::min(codec_settings->qpMax, kMaxH264Qp);


  //frame_dropping_on_ = codec_settings->H264().frameDroppingOn;
  //key_frame_interval_ = codec_settings->H264().keyFrameInterval;

  // Codec_settings uses kbits/second; encoder uses bits/second.
  max_bitrate_ = codec_settings->maxBitrate * 1000;

  if (codec_settings->targetBitrate > 0) {
    target_bps_ = codec_settings->targetBitrate * 1000;
  } else if (codec_settings->startBitrate > 0) {
    target_bps_ = codec_settings->startBitrate * 1000;
  } else {
    // Weight*Height*2 kbit represents a good balance between video quality and
    // the bandwidth that a 620 Windows phone can handle.
    target_bps_ = width_ * height_ * 2;
  }

  // Initialize the profile for the track encoded by this object.
  profile_ = webrtc__WinUWPH264EncoderImpl__profile.load();

  // Configure the encoder.
  HRESULT hr = S_OK;

  ON_SUCCEEDED(InitWriter());

  return hr;
}

int WinUWPH264EncoderImpl::InitWriter() {
  HRESULT hr = S_OK;

  bool is_hololens = IsHololens();

  AutoLock lock(&crit_);
  encoded_height_ = height_;

  // output media type (h264)
  ComPtr<IMFMediaType> mediaTypeOut;
  ON_SUCCEEDED(MFCreateMediaType(&mediaTypeOut));
  ON_SUCCEEDED(mediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
  ON_SUCCEEDED(mediaTypeOut->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264));

  eAVEncH264VProfile mf_profile;
  switch (profile_) {
    case H264::kProfileConstrainedBaseline:
      mf_profile = eAVEncH264VProfile_ConstrainedBase;
      //RTC_LOG_F(LS_INFO) << "Using Constrained Baseline profile";
      break;
    case H264::kProfileBaseline:
      mf_profile = eAVEncH264VProfile_Base;
      //RTC_LOG_F(LS_INFO) << "Using Baseline profile";
      break;
    case H264::kProfileMain:
      mf_profile = eAVEncH264VProfile_Main;
      //RTC_LOG_F(LS_INFO) << "Using Main profile";
      break;
    case H264::kProfileConstrainedHigh:
      mf_profile = eAVEncH264VProfile_ConstrainedHigh;
      //RTC_LOG_F(LS_INFO) << "Using Constrained High profile";
      break;
    case H264::kProfileHigh:
      mf_profile = eAVEncH264VProfile_High;
      //RTC_LOG_F(LS_INFO) << "Using High profile";
      break;
    default:
      return -1;
      break;
  }
  ON_SUCCEEDED(mediaTypeOut->SetUINT32(MF_MT_MPEG2_PROFILE, mf_profile));


  ON_SUCCEEDED(mediaTypeOut->SetUINT32(
    MF_MT_AVG_BITRATE, target_bps_));
  ON_SUCCEEDED(mediaTypeOut->SetUINT32(
    MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
  ON_SUCCEEDED(MFSetAttributeSize(mediaTypeOut.Get(), MF_MT_FRAME_SIZE, width_,
                                  encoded_height_));
  ON_SUCCEEDED(MFSetAttributeRatio(mediaTypeOut.Get(), MF_MT_FRAME_RATE,
                                   frame_rate_, 1));

  // input media type (nv12)
  ComPtr<IMFMediaType> mediaTypeIn;
  ON_SUCCEEDED(MFCreateMediaType(&mediaTypeIn));
  ON_SUCCEEDED(mediaTypeIn->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
  ON_SUCCEEDED(mediaTypeIn->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_NV12));
  ON_SUCCEEDED(mediaTypeIn->SetUINT32(
    MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
  ON_SUCCEEDED(mediaTypeIn->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE));
  ON_SUCCEEDED(MFSetAttributeSize(mediaTypeIn.Get(), MF_MT_FRAME_SIZE, width_,
                                  encoded_height_));

  ON_SUCCEEDED(MFSetAttributeRatio(mediaTypeIn.Get(),
    MF_MT_FRAME_RATE, frame_rate_, 1));

  // Create the media sink
   ON_SUCCEEDED(Microsoft::WRL::MakeAndInitialize<H264MediaSink>(&mediaSink_));

  // SinkWriter creation attributes
  ComPtr<IMFAttributes> sinkWriterCreationAttributes;
  ON_SUCCEEDED(MFCreateAttributes(&sinkWriterCreationAttributes, 1));
  ON_SUCCEEDED(sinkWriterCreationAttributes->SetUINT32(
    MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE));
  ON_SUCCEEDED(sinkWriterCreationAttributes->SetUINT32(
    MF_SINK_WRITER_DISABLE_THROTTLING, TRUE));
  ON_SUCCEEDED(sinkWriterCreationAttributes->SetUINT32(
    MF_LOW_LATENCY, TRUE));

  // Create the sink writer
  ON_SUCCEEDED(MFCreateSinkWriterFromMediaSink(mediaSink_.Get(),
    sinkWriterCreationAttributes.Get(), &sinkWriter_));

  // Add the h264 output stream to the writer
  ON_SUCCEEDED(sinkWriter_->AddStream(mediaTypeOut.Get(), &streamIndex_));

  // SinkWriter encoder properties
  ComPtr<IMFAttributes> encodingAttributes;
  ON_SUCCEEDED(MFCreateAttributes(&encodingAttributes, 1));
  ON_SUCCEEDED(
      encodingAttributes->SetUINT32(CODECAPI_AVEncH264CABACEnable, VARIANT_TRUE));

  if (webrtc__WinUWPH264EncoderImpl__maxQp >= 26) {
    max_qp_ = webrtc__WinUWPH264EncoderImpl__maxQp;
  }

  // kMaxH264Qp is the default.
  if (max_qp_ < kMaxH264Qp) {
    //RTC_LOG(LS_INFO) << "Set max QP to " << max_qp_;
    ON_SUCCEEDED(
        encodingAttributes->SetUINT32(CODECAPI_AVEncVideoMaxQP, max_qp_));
  }

  if (webrtc__WinUWPH264EncoderImpl__quality >= 0) {
    encodingAttributes->SetUINT32(CODECAPI_AVEncCommonQuality,
                                  webrtc__WinUWPH264EncoderImpl__quality);
  }

  ON_SUCCEEDED(sinkWriter_->SetInputMediaType(streamIndex_, mediaTypeIn.Get(),
                                              encodingAttributes.Get()));

  // Register this as the callback for encoded samples.
  ON_SUCCEEDED(mediaSink_->RegisterEncodingCallback(this));

  ON_SUCCEEDED(sinkWriter_->BeginWriting());
  if (SUCCEEDED(hr)) {
    inited_ = true;
    last_rate_change_time_rtc_ms = rtc::TimeMillis();
    last_stats_time_ = rtc::TimeMillis();
    return 0;
  } else {
    return hr;
  }
}

int WinUWPH264EncoderImpl::RegisterEncodeCompleteCallback(
  EncodedImageCallback* callback) {
  AutoLock lock(&callbackCrit_);
  encodedCompleteCallback_ = callback;
  return 0;
}

int WinUWPH264EncoderImpl::ReleaseWriter() {
  // Use a temporary sink variable to prevent lock inversion
  // between the shutdown call and OnH264Encoded() callback.
  ComPtr<H264MediaSink> tmpMediaSink;

  {
    AutoLock lock(&crit_);
    sinkWriter_->Finalize();
    sinkWriter_.Reset();
    if (mediaSink_ != nullptr) {
      tmpMediaSink = mediaSink_;
    }
    mediaSink_.Reset();
    startTime_ = 0;
    lastTimestampHns_ = 0;
    firstFrame_ = true;
    inited_ = false;
    framePendingCount_ = 0;
    _sampleAttributeQueue.clear();
    AutoLock callbackLock(&callbackCrit_);
    encodedCompleteCallback_ = nullptr;
  }

  if (tmpMediaSink != nullptr) {
    tmpMediaSink->Shutdown();
  }
  return 0;
}

int WinUWPH264EncoderImpl::Release() {
  ReleaseWriter();

  return 0;
}

int64_t first_frame_ts = -1;

ComPtr<IMFSample> WinUWPH264EncoderImpl::FromVideoFrame(const VideoFrame& frame) {
  HRESULT hr = S_OK;
  ComPtr<IMFSample> sample;
  ON_SUCCEEDED(MFCreateSample(sample.GetAddressOf()));

  ComPtr<IMFAttributes> sampleAttributes;
  ON_SUCCEEDED(sample.As(&sampleAttributes));

  int dst_height_uv = (height_ + 1) / 2;
  int dst_stride_y = frame.width_;
  int dst_stride_uv = dst_stride_y;

  auto totalSize =
      dst_stride_y * encoded_height_ + dst_stride_uv * encoded_height_ / 2;

  //if (SUCCEEDED(hr)) {
    ComPtr<IMFMediaBuffer> mediaBuffer;
    ON_SUCCEEDED(MFCreateMemoryBuffer(totalSize, mediaBuffer.GetAddressOf()));

    BYTE* destBuffer = nullptr;
    if (SUCCEEDED(hr)) {
      DWORD cbMaxLength;
      DWORD cbCurrentLength;
      ON_SUCCEEDED(
          mediaBuffer->Lock(&destBuffer, &cbMaxLength, &cbCurrentLength));
      std::memcpy(destBuffer, frame.video_frame_buffer_, totalSize);
    }

    if (firstFrame_) {
      firstFrame_ = false;
      startTime_ = frame.timestamp();
    }

    auto timestampHns = GetFrameTimestampHns(frame);
    ON_SUCCEEDED(sample->SetSampleTime(timestampHns));
    auto durationHns = timestampHns - lastTimestampHns_;

    if (SUCCEEDED(hr)) {
      hr = sample->SetSampleDuration(durationHns);
    }

    if (SUCCEEDED(hr)) {
      lastTimestampHns_ = timestampHns;

      // Cache the frame attributes to get them back after the encoding.
      CachedFrameAttributes frameAttributes;
      frameAttributes.timestamp = frame.timestamp();
      frameAttributes.frameWidth = frame.width();
      frameAttributes.frameHeight = encoded_height_;
      _sampleAttributeQueue.push(timestampHns, frameAttributes);
    }

    ON_SUCCEEDED(
        mediaBuffer->SetCurrentLength(totalSize));

    if (destBuffer != nullptr) {
      mediaBuffer->Unlock();
    }

    ON_SUCCEEDED(sample->AddBuffer(mediaBuffer.Get()));

    if (lastFrameDropped_) {
      lastFrameDropped_ = false;
      sampleAttributes->SetUINT32(MFSampleExtension_Discontinuity, TRUE);
    }
  //}

  auto now = rtc::TimeMillis();
  if (first_frame_ts < 0) {
    first_frame_ts = now;
  }
  return sample;
}

// Returns the timestamp in hundreds of nanoseconds (Media Foundation unit)
LONGLONG WinUWPH264EncoderImpl::GetFrameTimestampHns(const VideoFrame& frame) const {
  // H.264 clock rate is 90kHz (https://tools.ietf.org/html/rfc6184#page-11).
  // timestamp_100ns = timestamp_90kHz / {90'000 Hz} * {10'000'000 hns/sec}
  return (frame.timestamp() - startTime_) * 10'000 / 90;
}

int WinUWPH264EncoderImpl::Encode(
  const VideoFrame& frame, bool keyframe) {
  if (!inited_) {
    return -1;
  }

  {
    auto* frame_buffer = &frame;
    int cur_width = frame_buffer->width();
    int cur_height = frame_buffer->height();
    int64_t now = rtc::TimeMillis();

    AutoLock lock(&crit_);
    // Reset the encoder configuration if necessary.
    bool res_changed = cur_width != (int)width_ || cur_height != (int)height_;
    bool should_change_rate_now =
        rate_change_requested_ &&
        (now - last_rate_change_time_rtc_ms) > kMinIntervalBetweenRateChangesMs;
    if (res_changed || should_change_rate_now) {
      int res = ReconfigureSinkWriter(cur_width, cur_height, next_target_bps_,
                                      next_frame_rate_);
      if (FAILED(res)) {
        return res;
      }
      rate_change_requested_ = false;
    }
  }
  //if (frame_types != nullptr) {
  //  for (auto frameType : *frame_types) {
  //    if (frameType == kVideoFrameKey) {
  //      RTC_LOG(LS_INFO) << "Key frame requested in H264 encoder.";
  //      std::stringstream str;
  //      str << "Keyframe requested - last was "
  //          << (frameCount_ - last_keyframe_req_frame_count) << " frames ago\n";
  //      OutputDebugStringA(str.str().c_str());
  //      last_keyframe_req_frame_count = frameCount_;
  //      ComPtr<IMFSinkWriterEncoderConfig> encoderConfig;
  //      sinkWriter_.As(&encoderConfig);
  //      ComPtr<IMFAttributes> encoderAttributes;
  //      MFCreateAttributes(&encoderAttributes, 1);
  //      encoderAttributes->SetUINT32(CODECAPI_AVEncVideoForceKeyFrame, TRUE);
  //      encoderConfig->PlaceEncodingParameters(streamIndex_, encoderAttributes.Get());
  //      break;
  //    }
  //  }
  //}

  HRESULT hr = S_OK;

  ComPtr<IMFSample> sample;
  {
    AutoLock lock(&crit_);
    // Only encode the frame if the encoder pipeline is not full.
    //if (_sampleAttributeQueue.size() <= 2) {
      sample = FromVideoFrame(frame);
    //}
  }

  if (!sample) {
    // Drop the frame. Send a tick to keep the encoder going.
    lastFrameDropped_ = true;
    auto timestampHns = GetFrameTimestampHns(frame);
    ON_SUCCEEDED(sinkWriter_->SendStreamTick(streamIndex_, timestampHns));
    lastTimestampHns_ = timestampHns;
    return 0;
  }

  ON_SUCCEEDED(sinkWriter_->WriteSample(streamIndex_, sample.Get()));

  AutoLock lock(&crit_);
  //// Some threads online mention this is useful to do regularly.
  ++frameCount_;
  //if (frameCount_ % 30 == 0) {
  //  ON_SUCCEEDED(sinkWriter_->NotifyEndOfSegment(streamIndex_));
  //}

  ++framePendingCount_;
  return 0;
}

int64_t first_decoded_ts = -1;

class RTPFragmentationHeader {
 public:
  RTPFragmentationHeader();
  RTPFragmentationHeader(const RTPFragmentationHeader&) = delete;
  RTPFragmentationHeader(RTPFragmentationHeader&& other);
  RTPFragmentationHeader& operator=(const RTPFragmentationHeader& other) =
      delete;
  RTPFragmentationHeader& operator=(RTPFragmentationHeader&& other);
  ~RTPFragmentationHeader();

  friend void swap(RTPFragmentationHeader& a, RTPFragmentationHeader& b);

  void CopyFrom(const RTPFragmentationHeader& src);
  void VerifyAndAllocateFragmentationHeader(size_t size) { Resize(size); }

  void Resize(size_t size);
  size_t Size() const { return fragmentationVectorSize; }

  size_t Offset(size_t index) const { return fragmentationOffset[index]; }
  size_t Length(size_t index) const { return fragmentationLength[index]; }
  uint16_t TimeDiff(size_t index) const { return fragmentationTimeDiff[index]; }
  int PayloadType(size_t index) const { return fragmentationPlType[index]; }

  // TODO(danilchap): Move all members to private section,
  // simplify by replacing 4 raw arrays with single std::vector<Fragment>
  uint16_t fragmentationVectorSize;  // Number of fragmentations
  size_t* fragmentationOffset;       // Offset of pointer to data for each
                                     // fragmentation
  size_t* fragmentationLength;       // Data size for each fragmentation
  uint16_t* fragmentationTimeDiff;   // Timestamp difference relative "now" for
                                     // each fragmentation
  uint8_t* fragmentationPlType;      // Payload type of each fragmentation
};
void RTPFragmentationHeader::Resize(size_t size) {
  const uint16_t size16 = static_cast<uint16_t>(size);
  if (fragmentationVectorSize < size16) {
    uint16_t oldVectorSize = fragmentationVectorSize;
    {
      // offset
      size_t* oldOffsets = fragmentationOffset;
      fragmentationOffset = new size_t[size16];
      memset(fragmentationOffset + oldVectorSize, 0,
             sizeof(size_t) * (size16 - oldVectorSize));
      // copy old values
      memcpy(fragmentationOffset, oldOffsets, sizeof(size_t) * oldVectorSize);
      delete[] oldOffsets;
    }
    // length
    {
      size_t* oldLengths = fragmentationLength;
      fragmentationLength = new size_t[size16];
      memset(fragmentationLength + oldVectorSize, 0,
             sizeof(size_t) * (size16 - oldVectorSize));
      memcpy(fragmentationLength, oldLengths, sizeof(size_t) * oldVectorSize);
      delete[] oldLengths;
    }
    // time diff
    {
      uint16_t* oldTimeDiffs = fragmentationTimeDiff;
      fragmentationTimeDiff = new uint16_t[size16];
      memset(fragmentationTimeDiff + oldVectorSize, 0,
             sizeof(uint16_t) * (size16 - oldVectorSize));
      memcpy(fragmentationTimeDiff, oldTimeDiffs,
             sizeof(uint16_t) * oldVectorSize);
      delete[] oldTimeDiffs;
    }
    // payload type
    {
      uint8_t* oldTimePlTypes = fragmentationPlType;
      fragmentationPlType = new uint8_t[size16];
      memset(fragmentationPlType + oldVectorSize, 0,
             sizeof(uint8_t) * (size16 - oldVectorSize));
      memcpy(fragmentationPlType, oldTimePlTypes,
             sizeof(uint8_t) * oldVectorSize);
      delete[] oldTimePlTypes;
    }
    fragmentationVectorSize = size16;
  }
}
RTPFragmentationHeader::RTPFragmentationHeader()
    : fragmentationVectorSize(0),
      fragmentationOffset(nullptr),
      fragmentationLength(nullptr),
      fragmentationTimeDiff(nullptr),
      fragmentationPlType(nullptr) {}


RTPFragmentationHeader::~RTPFragmentationHeader() {
  delete[] fragmentationOffset;
  delete[] fragmentationLength;
  delete[] fragmentationTimeDiff;
  delete[] fragmentationPlType;
}


std::vector<byte> sendBuffer;

void WinUWPH264EncoderImpl::OnH264Encoded(ComPtr<IMFSample> sample) {
  DWORD totalLength;
  HRESULT hr = S_OK;
  int64_t now = rtc::TimeMillis();
  ON_SUCCEEDED(sample->GetTotalLength(&totalLength));

  LONGLONG sampleTimestamp = 0;
  ON_SUCCEEDED(sample->GetSampleTime(&sampleTimestamp));

  // Pop the attributes for this frame. This must be done even if the
  // frame is discarded later, or the queue will clog.
  CachedFrameAttributes frameAttributes;
  if (!_sampleAttributeQueue.pop(sampleTimestamp, frameAttributes)) {
    // No point in processing a frame that doesn't have correct attributes.
    return;
  }

  ComPtr<IMFMediaBuffer> buffer;
  hr = sample->GetBufferByIndex(0, &buffer);


  if (SUCCEEDED(hr)) {
    BYTE* byteBuffer;
    DWORD maxLength;
    DWORD curLength;
    hr = buffer->Lock(&byteBuffer, &maxLength, &curLength);
    if (FAILED(hr)) {
      return;
    }

    //int64_t now = rtc::TimeMillis();
    //bitrate_window_.AddSample(curLength * 8, now);
    //framerate_window_.AddSample(1, now);
    //if (now - last_stats_time_ > 1000) {
    //  int bps = bitrate_window_.GetSumUpTo(now);
    //  int frames = framerate_window_.GetSumUpTo(now);
    //  std::stringstream str;
    //  str << "RATES: " << frames << " fps - " << bps / 1000 << " kbps";
    //  str << "\n";
    //  OutputDebugStringA(str.str().c_str());
    //  last_stats_time_ = now;
    //}


    if (curLength == 0) {
      debugLog << "WARNING Got empty sample.\n";
      buffer->Unlock();
      (*encodedCompleteCallback_)(nullptr, sampleTimestamp,
                                  0, false);
      return;
    } else {
      sendBuffer.resize(curLength);
      memcpy(sendBuffer.data(), byteBuffer, curLength);
    }


    hr = buffer->Unlock();
    if (FAILED(hr)) {
      return;
    }

    bool keyframe = false;

    ComPtr<IMFAttributes> sampleAttributes;
    hr = sample.As(&sampleAttributes);
    if (SUCCEEDED(hr)) {
      UINT32 cleanPoint;
      hr = sampleAttributes->GetUINT32(MFSampleExtension_CleanPoint,
                                       &cleanPoint);
      if (SUCCEEDED(hr) && cleanPoint) {
        keyframe = true;
      }
    }

    // Scan for and create mark all fragments.
    RTPFragmentationHeader fragmentationHeader;
    uint32_t fragIdx = 0;
    for (uint32_t i = 0; i < sendBuffer.size() - 5; ++i) {
      byte* ptr = sendBuffer.data() + i;
      int prefixLengthFound = 0;
      if (ptr[0] == 0x00 && ptr[1] == 0x00 && ptr[2] == 0x00 &&
          ptr[3] == 0x01 &&
          ((ptr[4] & 0x1f) != 0x09 /* ignore access unit delimiters */)) {
        prefixLengthFound = 4;
      } else if (ptr[0] == 0x00 && ptr[1] == 0x00 && ptr[2] == 0x01 &&
                 ((ptr[3] & 0x1f) !=
                  0x09 /* ignore access unit delimiters */)) {
        prefixLengthFound = 3;
      }

      // Found a key frame, mark is as such in case
      // MFSampleExtension_CleanPoint wasn't set on the sample.
      if (prefixLengthFound > 0 && (ptr[prefixLengthFound] & 0x1f) == 0x05) {
        keyframe = true;
      }

      if (prefixLengthFound > 0) {
        fragmentationHeader.VerifyAndAllocateFragmentationHeader(fragIdx + 1);
        fragmentationHeader.fragmentationOffset[fragIdx] =
            i + prefixLengthFound;
        fragmentationHeader.fragmentationLength[fragIdx] =
            0;  // We'll set that later
        // Set the length of the previous fragment.
        if (fragIdx > 0) {
          fragmentationHeader.fragmentationLength[fragIdx - 1] =
              i - fragmentationHeader.fragmentationOffset[fragIdx - 1];
        }
        fragmentationHeader.fragmentationPlType[fragIdx] = 0;
        fragmentationHeader.fragmentationTimeDiff[fragIdx] = 0;
        ++fragIdx;
        i += 5;
      }
    }

    //if (keyframe) {
    //  OutputDebugStringA("Keyframe\n");
    //} else {
    //  OutputDebugStringA("Delta frame\n");
    //}

    {
      AutoLock lock(&callbackCrit_);
      --framePendingCount_;

	  if (encodedCompleteCallback_ != nullptr) {
        (*encodedCompleteCallback_)(sendBuffer.data(),
                                                 sampleTimestamp, curLength, keyframe);
      }
    }
  }
}



#define DYNAMIC_FPS
#define DYNAMIC_BITRATE

int WinUWPH264EncoderImpl::SetRates(uint32_t new_bitrate_kbit,
                                    uint32_t new_framerate) {
  if (sinkWriter_ == nullptr) {
    return -1;
  }
  debugLog << "WinUWPH264EncoderImpl::SetRates(" << new_bitrate_kbit
                   << "kbit " << new_framerate << "fps)";

  // This may happen. Ignore it.
  if (new_framerate == 0) {
    return 0;
  }

  int64_t now = rtc::TimeMillis();
  AutoLock lock(&crit_);
  int64_t time_to_wait_before_rate_change =
      kMinIntervalBetweenRateChangesMs - (now - last_rate_change_time_rtc_ms);
  if (time_to_wait_before_rate_change > 0) {
    // Delay rate update until the interval has passed.
    debugLog << "Postponing this SetRates() in "
                     << time_to_wait_before_rate_change << " ms.\n";
    next_target_bps_ = new_bitrate_kbit * 1000;
    next_frame_rate_ = new_framerate;
    rate_change_requested_ = true;
    return 0;
  }
  // Update the configuration.
  return ReconfigureSinkWriter(width_, height_, new_bitrate_kbit * 1000,
                               new_framerate);
}

int WinUWPH264EncoderImpl::ReconfigureSinkWriter(UINT32 new_width,
                                                 UINT32 new_height,
                                                 UINT32 new_target_bps,
                                                 UINT32 new_frame_rate) {
    #if 0
  // NOTE: must be called under crit_ lock.
  debugLog << "WinUWPH264EncoderImpl::ResetSinkWriter() " << new_width
                   << "x" << new_height << "@" << new_frame_rate << " "
                   << new_target_bps / 1000 << "kbps";
  bool resUpdated = false;
  bool bitrateUpdated = false;
  bool fpsUpdated = false;

  if (width_ != new_width || height_ != new_height) {
    resUpdated = true;
    width_ = new_width;
    height_ = new_height;
  }

#ifdef DYNAMIC_BITRATE
  // Ignore small changes.
  if (std::abs((int)target_bps_ - (int)new_target_bps) >
      target_bps_ * kMinRateVariation) {
    bitrateUpdated = true;
    target_bps_ = new_target_bps;
  }
#endif

#ifdef DYNAMIC_FPS
  // Ignore small changes.
  if (std::abs((int)frame_rate_ - (int)new_frame_rate) >
      frame_rate_ * kMinRateVariation) {
    fpsUpdated = true;
    frame_rate_ = new_frame_rate;
  }
#endif

  if (resUpdated || bitrateUpdated || fpsUpdated) {
    ReleaseWriter();
    InitWriter();

    last_rate_change_time_rtc_ms = rtc::TimeMillis();
  }
  #endif

  return 0;
}

}  // namespace webrtc

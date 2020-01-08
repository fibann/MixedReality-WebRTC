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
bool webrtc__WinUWPH264EncoderImpl__add_padding = false;

namespace webrtc {

// QP scaling thresholds.
static const int kLowH264QpThreshold = 24;
static const int kHighH264QpThreshold = 37;

// On some encoders (e.g. Hololens) changing rates is slow and will cause
// visible stuttering, so we don't want to do it too often.
// todo(fibann): we are ignoring small variations which means the rates might
// end up being off the requested value by a small amount in the long term. We
// should not ignore small variations but possibly use a longer min interval so
// they are eventually applied.
static const int kMinIntervalBetweenRateChangesMs = 5000;
static const float kMinRateVariation = 0.1f;

//////////////////////////////////////////
// H264 WinUWP Encoder Implementation
//////////////////////////////////////////

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

WinUWPH264EncoderImpl::WinUWPH264EncoderImpl() = default;
WinUWPH264EncoderImpl::~WinUWPH264EncoderImpl() = default;

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

  // Configure the encoder.
  HRESULT hr = S_OK;
  ON_SUCCEEDED(MFStartup(MF_VERSION));

  ComPtr<ABI::Windows::Storage::IApplicationDataStatics> appDataStatics;
  ON_SUCCEEDED(Windows::Foundation::GetActivationFactory(
      Microsoft::WRL::Wrappers::HStringReference(
          RuntimeClass_Windows_Storage_ApplicationData)
          .Get(),
      &appDataStatics));
  ComPtr<ABI::Windows::Storage::IApplicationData> appData;
  ON_SUCCEEDED(appDataStatics->get_Current(&appData));
  ComPtr<ABI::Windows::Storage::IStorageFolder> folder;
  ON_SUCCEEDED(appData->get_LocalFolder(&folder));
  ComPtr<ABI::Windows::Storage::IStorageItem> asItem;
  ON_SUCCEEDED(folder.As(&asItem));
  Microsoft::WRL::Wrappers::HString path;
  ON_SUCCEEDED(asItem->get_Path(path.GetAddressOf()));

  // ComPtr<ABI::Windows::Storage::IStorageFile> compressed_file_out;
  // ComPtr<ABI::Windows::Storage::IStorageFile> uncompressed_file_out;
  // ComPtr<ABI::Windows::Foundation::IAsyncOperation<ABI::Windows::Storage::StorageFile*>>
  //    compressed_file_op;
  // ComPtr<ABI::Windows::Foundation::IAsyncOperation<
  //    ABI::Windows::Storage::StorageFile*>>
  //    uncompressed_file_op;

  // ON_SUCCEEDED(folder->CreateFileAsync(
  //    Microsoft::WRL::Wrappers::HStringReference(L"compressed.dat").Get(),
  //    ABI::Windows::Storage::CreationCollisionOption_ReplaceExisting,
  //    &compressed_file_op));
  // ON_SUCCEEDED(folder->CreateFileAsync(
  //    Microsoft::WRL::Wrappers::HStringReference(L"uncompressed.dat").Get(),
  //    ABI::Windows::Storage::CreationCollisionOption_ReplaceExisting,
  //    &uncompressed_file_op));
  // compressed_file_op->GetResults(&compressed_file_out);
  // uncompressed_file_op->GetResults(&uncompressed_file_out);
  unsigned length;
  auto ptr = path.GetRawBuffer(&length);
  std::string folderNameA(ptr, ptr + length);

  compressed_file_out_.open(folderNameA + "\\compressed_out.dat",
                            std::ifstream::binary);

  bool is_ok = compressed_file_out_.good();
  (void)is_ok;

  ON_SUCCEEDED(InitWriter());

  return hr;
}

int WinUWPH264EncoderImpl::InitWriter() {
  HRESULT hr = S_OK;

  bool is_hololens = IsHololens();

  AutoLock lock(&crit_);

  // The hardware encoder on Hololens 1 produces severe artifacts at low
  // bitrates when processing frames whose height is not multiple of 16. As a
  // workaround we pad or crop those resolutions before passing them to the
  // encoder.
  encoded_height_ = height_;
  if (is_hololens) {
    encoded_height_ = webrtc__WinUWPH264EncoderImpl__add_padding
                          ? (height_ + 15) & ~15
                          : height_ & ~15;
  }

  // output media type (h264)
  ComPtr<IMFMediaType> mediaTypeOut;
  ON_SUCCEEDED(MFCreateMediaType(&mediaTypeOut));
  ON_SUCCEEDED(mediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
  ON_SUCCEEDED(mediaTypeOut->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264));
  // Lumia 635 and Lumia 1520 Windows phones don't work well
  // with constrained baseline profile.
  //ON_SUCCEEDED(mediaTypeOut->SetUINT32(MF_MT_MPEG2_PROFILE, eAVEncH264VProfile_ConstrainedBase));

  ON_SUCCEEDED(mediaTypeOut->SetUINT32(
    MF_MT_AVG_BITRATE, target_bps_));
  ON_SUCCEEDED(mediaTypeOut->SetUINT32(
    MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
  ON_SUCCEEDED(MFSetAttributeSize(mediaTypeOut.Get(),
    MF_MT_FRAME_SIZE, width_, encoded_height_));
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
  //ON_SUCCEEDED(
  //    encodingAttributes->SetUINT32(CODECAPI_AVEncMPVGOPSize, 3 * frame_rate_));
  ON_SUCCEEDED(
      encodingAttributes->SetUINT32(CODECAPI_AVEncVideoMaxQP, 40));
  ON_SUCCEEDED(
      sinkWriter_->SetInputMediaType(streamIndex_, mediaTypeIn.Get(), encodingAttributes.Get()));

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

int WinUWPH264EncoderImpl::ReleaseWriter() {
  // Use a temporary sink variable to prevent lock inversion
  // between the shutdown call and OnH264Encoded() callback.
  ComPtr<H264MediaSink> tmpMediaSink;

  {
    AutoLock lock(&crit_);
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
  }

  if (tmpMediaSink != nullptr) {
    tmpMediaSink->Shutdown();
  }
  return 0;
}

int WinUWPH264EncoderImpl::Release() {
  ReleaseWriter();
  HRESULT hr = S_OK;
  ON_SUCCEEDED(MFShutdown());
  return 0;
}

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

  if (SUCCEEDED(hr)) {
    ComPtr<IMFMediaBuffer> mediaBuffer;
    ON_SUCCEEDED(MFCreateMemoryBuffer(totalSize, mediaBuffer.GetAddressOf()));

    BYTE* destBuffer = nullptr;
    if (SUCCEEDED(hr)) {
      DWORD cbMaxLength;
      DWORD cbCurrentLength;
      ON_SUCCEEDED(mediaBuffer->Lock(&destBuffer, &cbMaxLength, &cbCurrentLength));
      std::memcpy(destBuffer, frame.video_frame_buffer_, totalSize);
    }



    if (firstFrame_) {
      firstFrame_ = false;
      startTime_ = 0;  // frame.timestamp();
    }

    auto timestampHns = GetFrameTimestampHns(frame);
    ON_SUCCEEDED(sample->SetSampleTime(timestampHns));

    if (SUCCEEDED(hr)) {
      auto durationHns = timestampHns - lastTimestampHns_;
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

      if (keyframe) {
        debugLog << "Key frame requested in H264 encoder.";
        std::stringstream str;
        str << "Keyframe requested - last was "
            << (frameCount_ - last_keyframe_req_frame_count) << " frames ago\n";
        OutputDebugStringA(str.str().c_str());
        last_keyframe_req_frame_count = frameCount_;
        ComPtr<IMFSinkWriterEncoderConfig> encoderConfig;
        sinkWriter_.As(&encoderConfig);
        ComPtr<IMFAttributes> encoderAttributes;
        MFCreateAttributes(&encoderAttributes, 1);
        encoderAttributes->SetUINT32(CODECAPI_AVEncVideoForceKeyFrame, TRUE);
        encoderConfig->PlaceEncodingParameters(streamIndex_, encoderAttributes.Get());
      }

  HRESULT hr = S_OK;

  ComPtr<IMFSample> sample;
  {
    AutoLock lock(&crit_);
    // Only encode the frame if the encoder pipeline is not full.
    if (_sampleAttributeQueue.size() <= 2) {
      sample = FromVideoFrame(frame);
    }
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

void WinUWPH264EncoderImpl::OnH264Encoded(ComPtr<IMFSample> sample) {
  DWORD totalLength;
  HRESULT hr = S_OK;
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

    int64_t now = rtc::TimeMillis();
    bitrate_window_.AddSample(curLength * 8, now);
    framerate_window_.AddSample(1, now);
    if (now - last_stats_time_ > 1000) {
      int bps = bitrate_window_.GetSumUpTo(now);
	  int frames = framerate_window_.GetSumUpTo(now);
      std::stringstream str;
      str << "RATES: " << frames << " fps - " << bps / 1000 << " kbps";
      str << "\n";
      OutputDebugStringA(str.str().c_str());
      last_stats_time_ = now;
    }


    if (curLength == 0) {
      debugLog << "Got empty sample.";
      buffer->Unlock();
      return;
    }
    std::vector<byte> sendBuffer;
    sendBuffer.resize(curLength);
    memcpy(sendBuffer.data(), byteBuffer, curLength);

    LONGLONG zero = 0;
    compressed_file_out_.write((char*)&sampleTimestamp,
                               sizeof(sampleTimestamp));
    compressed_file_out_.write((char*)&zero, sizeof(zero));
    compressed_file_out_.write((char*)&curLength, sizeof(curLength));
    compressed_file_out_.write((char*)byteBuffer, curLength);

    hr = buffer->Unlock();
    if (FAILED(hr)) {
      return;
    }

    //if (keyframe) {
    //  std::stringstream str;
    //  str << "Keyframe produced. Last was "
    //      << (frameCount_ - last_keyframe_produced_frame_count)
    //      << " frames ago\n";
    //  OutputDebugStringA(str.str().c_str());
    //  last_keyframe_produced_frame_count = frameCount_;
    //}


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

  return 0;
}

}  // namespace webrtc

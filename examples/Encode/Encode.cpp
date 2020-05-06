//
// MainPage.xaml.cpp
// Implementation of the MainPage class.
//
#define NOMINMAX

#include <assert.h>

#include <algorithm>
#include <codecvt>
#include <fstream>

#include "H264Encoder/H264Encoder.h"
#include <atomic>
#include <cstddef>
#include <algorithm>


void ReadFile(std::ifstream& file, int frame_size, int num_frames = 100) {
  auto destBuffer = new char[frame_size];

  int i = 0;
  LONGLONG last_frame_timestamp = 0;
  int last_delivery_timestamp = 0;
  int64_t last_second = 0;
  int frames_at_last_second = 0;
  int bits_at_last_second = 0;
  int bits = 0;
  char log[1024];
  while (i < num_frames && !file.eof()) {
    int64_t timestampHns, durationHns;
    int32_t totalSize, delivery_ts{};

    file.read((char*)&timestampHns, sizeof(timestampHns));
    file.read((char*)&durationHns, sizeof(durationHns));
    //file.read((char*)&delivery_ts, sizeof(delivery_ts));
    file.read((char*)&totalSize, sizeof(totalSize));
    assert(totalSize <= frame_size);
    file.read((char*)destBuffer, totalSize);

    if (last_second < timestampHns / 10'000'000) {
      last_second = timestampHns / 10'000'000;
      auto bitrate = (bits - bits_at_last_second) / 1000.0f;
      //if (bitrate > 1500) {
      //  OutputDebugStringA("WARNING - ");
      //}
      sprintf(log, "%d fps %f kbps\n", i - frames_at_last_second,
              bitrate);
      frames_at_last_second = i;
      bits_at_last_second = bits;
      OutputDebugStringA(log);
    }

    auto actualDurationHns = timestampHns - last_frame_timestamp;
    auto actualSpaceBetweenFrames = delivery_ts - last_delivery_timestamp;
    last_delivery_timestamp = delivery_ts;
    last_frame_timestamp = timestampHns;
    //sprintf(log, "%d TS %lld ATS %d D %lld AD %lld SBF %d S %d\n", i++, timestampHns / 10'000, delivery_ts, durationHns / 10'000,
    //        actualDurationHns / 10'000, actualSpaceBetweenFrames, totalSize);
    sprintf(log, "%d Timestamp %lld Duration %lld Size %d\n", i++,
            timestampHns / 10'000, durationHns / 10'000, totalSize);
    bits += totalSize * 8;
    OutputDebugStringA(log);
  }

  sprintf(log, "Total %f fps %f kbps\n",
          ((float)i) / (last_frame_timestamp / 10'000'000),
          bits / 1000.0f / (last_frame_timestamp / 10'000'000));
  OutputDebugStringA(log);
}

int64_t time_to_sleep;

void DoEncode(std::ifstream& uncompressed_file,
              std::ofstream& compressed_file,
              int width,
              int height,
              int framerate,
    int bitrate,
    int num_frames = 100) {
  webrtc::VideoCodec codec;
  codec.width = width;
  codec.height = height;
  codec.maxFramerate = framerate;
  codec.maxBitrate = 2500;
  codec.targetBitrate = bitrate;
  codec.startBitrate = bitrate;
  int frame_size = codec.width * codec.height * 3 / 2;

  LONGLONG last_frame_timestamp = 0;
  LONGLONG last_timestamp = rtc::TimeMillis();
  LONGLONG begin_timestamp = rtc::TimeMillis();
  {
    auto destBuffer = std::make_unique<uint8_t[]>(frame_size);

    auto encoder = std::make_unique<webrtc::WinUWPH264EncoderImpl>();
    encoder->InitEncode(&codec, 0, 0);

    std::atomic<bool> arrived = false;

    webrtc::EncodedImageCallback cb = [&](const uint8_t* byteBuffer, int64_t sampleTimestamp,
                                  int32_t curLength) {
       LONGLONG zero = 0;
      // if (first_decoded_ts < 0) {
      //  first_decoded_ts = now;
      //}
      // int32_t now_delta = (int32_t)(now - first_decoded_ts);
       compressed_file.write((char*)&sampleTimestamp,
                                 sizeof(sampleTimestamp));
       compressed_file.write((char*)&zero, sizeof(zero));
      //compressed_file.write((char*)&now_delta, sizeof(now_delta));
       compressed_file.write((char*)&curLength, sizeof(curLength));
       compressed_file.write((char*)byteBuffer, curLength);
      arrived.store(true, std::memory_order::memory_order_release);
    };
    encoder->RegisterEncodeCompleteCallback(&cb);
    int i = 0;
    char log[1024];
    int64_t first_delivery_ts = -1;
    while (i < num_frames && !uncompressed_file.eof()) {
      int64_t timestampHns, durationHns;
      int32_t totalSize;
      int32_t delivery_ts;

      uncompressed_file.read((char*)&timestampHns, sizeof(timestampHns));
      uncompressed_file.read((char*)&durationHns, sizeof(durationHns));
      //uncompressed_file.read((char*)&delivery_ts, sizeof(delivery_ts));
      uncompressed_file.read((char*)&totalSize, sizeof(totalSize));
      assert(totalSize == frame_size);
      uncompressed_file.read((char*)destBuffer.get(), totalSize);

      // auto actualDurationHns = timestampHns - last_frame_timestamp;

      // sprintf(log, "%d TS %lld D %lld AD %lld S %d\n", i++, timestampHns /
      // 10'000, durationHns / 10'000, actualDurationHns / 10'000,
      //        totalSize);
      // OutputDebugStringA(log);

      webrtc::VideoFrame frame;
      frame.width_ = codec.width;
      frame.height_ = codec.height;
      frame.video_frame_buffer_ = destBuffer.get();
      frame.timestamp_rtp_ = timestampHns * 90 / 10'000;

      auto now = rtc::TimeMillis();
      if (first_delivery_ts < 0) {
        first_delivery_ts = now;
      }

      //auto now_delta = now - first_delivery_ts;
      //time_to_sleep = max(0, delivery_ts - now_delta);
      //auto now_delta = now - begin_timestamp;
      //time_to_sleep = max(0, timestampHns / 10'000 - now_delta);
      //if (time_to_sleep) {
      //  Sleep(time_to_sleep);
      //}
      //Sleep(33);

      last_timestamp = now;
      last_frame_timestamp = timestampHns;

      encoder->Encode(frame, false);

      while (!arrived.load(std::memory_order::memory_order_acquire)) {
        Sleep(1);
      }
      arrived.store(false, std::memory_order_relaxed);
    }
  }
}

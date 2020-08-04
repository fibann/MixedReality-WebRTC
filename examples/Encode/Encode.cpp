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
#include "Encode.h"
#include <atomic>
#include <cstddef>
#include <iostream>

void Log(const char* msg) {
  std::cout << msg << std::endl;
  //OutputDebugStringA(msg);
}

void ReadFile(std::ifstream& file, int frame_size) {
  std::vector<char> destBuffer;

  int i = 0;
  LONGLONG last_frame_timestamp = 0;
  int last_delivery_timestamp = 0;
  int64_t last_second = 0;
  int frames_at_last_second = 0;
  int bits_at_last_second = 0;
  int bits = 0;
  char log[1024];
  while (true) {
    int64_t timestampHns, durationHns;
    int32_t totalSize, delivery_ts{};

    file.read((char*)&timestampHns, sizeof(timestampHns));
    if (file.eof()) {
      break;
    }
    file.read((char*)&durationHns, sizeof(durationHns));
    //file.read((char*)&delivery_ts, sizeof(delivery_ts));
    file.read((char*)&totalSize, sizeof(totalSize));
    assert(totalSize <= frame_size);
    destBuffer.resize(totalSize);
    file.read(destBuffer.data(), totalSize);

    if (last_second < timestampHns / 10'000'000) {
      last_second = timestampHns / 10'000'000;
      auto bitrate = (bits - bits_at_last_second) / 1000.0f;

      sprintf(log, "%d fps %f kbps", i - frames_at_last_second,
              bitrate);
      frames_at_last_second = i;
      bits_at_last_second = bits;
      Log(log);

    }

    auto actualDurationHns = timestampHns - last_frame_timestamp;
    auto actualSpaceBetweenFrames = delivery_ts - last_delivery_timestamp;
    last_delivery_timestamp = delivery_ts;
    last_frame_timestamp = timestampHns;
    //sprintf(log, "%d TS %lld ATS %d D %lld AD %lld SBF %d S %d\n", i, timestampHns / 10'000, delivery_ts, durationHns / 10'000,
    //        actualDurationHns / 10'000, actualSpaceBetweenFrames, totalSize);
    sprintf(log, "%d Timestamp %lld Duration %lld Size %d", i,
            timestampHns / 10'000, durationHns / 10'000, totalSize);
    bits += totalSize * 8;
    Log(log);
    ++i;
  }

  sprintf(log, "Total %f fps %f kbps",
          ((float)i) / (last_frame_timestamp / 10'000'000),
          bits / 1000.0f / (last_frame_timestamp / 10'000'000));
  Log(log);
}

void StripHeaders(std::istream& file, std::ostream& out, int num_frames, int start_frame) {
  std::vector<char> buffer;

  int i = 0;
  LONGLONG last_frame_timestamp = 0;
  int last_delivery_timestamp = 0;
  int64_t last_second = 0;
  int frames_at_last_second = 0;
  int bits_at_last_second = 0;
  int bits = 0;
  char log[1024];
  while (i < num_frames) {
    int64_t timestampHns, durationHns;
    int32_t totalSize, delivery_ts{};

    file.read((char*)&timestampHns, sizeof(timestampHns));
    if (file.eof()) {
      break;
    }
    file.read((char*)&durationHns, sizeof(durationHns));
    // file.read((char*)&delivery_ts, sizeof(delivery_ts));
    file.read((char*)&totalSize, sizeof(totalSize));
    buffer.resize(std::max((int)buffer.size(), totalSize));
    file.read(buffer.data(), totalSize);

    if (i >= start_frame) {
      out.write(buffer.data(), totalSize);
    }
    ++i;
  }
}

void Trim(std::istream& file,
                  std::ostream& out,
                  int num_frames,
                  int start_frame) {
  std::vector<char> buffer;

  int i = 0;
  LONGLONG last_frame_timestamp = 0;
  int last_delivery_timestamp = 0;
  int64_t last_second = 0;
  int frames_at_last_second = 0;
  int bits_at_last_second = 0;
  int bits = 0;
  char log[1024];
  while (i < num_frames) {
    int64_t timestampHns, durationHns;
    int32_t totalSize, delivery_ts{};

    file.read((char*)&timestampHns, sizeof(timestampHns));
    if (file.eof()) {
      break;
    }
    file.read((char*)&durationHns, sizeof(durationHns));
    // file.read((char*)&delivery_ts, sizeof(delivery_ts));
    file.read((char*)&totalSize, sizeof(totalSize));
    buffer.resize(std::max((int)buffer.size(), totalSize));
    file.read(buffer.data(), totalSize);

    if (i >= start_frame) {
      out.write((char*)&timestampHns, sizeof(timestampHns));
      out.write((char*)&durationHns, sizeof(durationHns));
      out.write((char*)&totalSize, sizeof(totalSize));
      out.write(buffer.data(), totalSize);

      sprintf(log, "%d TS %lld D %lld S %d\n", i, timestampHns / 10'000,
              durationHns / 10'000, totalSize);
      OutputDebugStringA(log);
    }
    ++i;
  }
}


struct Frame {
  int64_t timestampHns, durationHns;
  int32_t totalSize, delivery_ts{};
  std::vector<char> buffer;
};

void Read(std::istream& file, Frame& frame) {
  file.read((char*)&frame.timestampHns, sizeof(frame.timestampHns));
  file.read((char*)&frame.durationHns, sizeof(frame.durationHns));
  // file.read((char*)&delivery_ts, sizeof(delivery_ts));
  file.read((char*)&frame.totalSize, sizeof(frame.totalSize));
  frame.buffer.resize(std::max((int)frame.buffer.size(), frame.totalSize));
  file.read(frame.buffer.data(), frame.totalSize);

}

void RemoveMissingFrames(std::istream& uncompressed,
                        std::istream& compressed, std::ostream& out) {
  std::vector<char> buffer;

  int i = 0;
  LONGLONG last_frame_timestamp = 0;
  int last_delivery_timestamp = 0;
  int64_t last_second = 0;
  int frames_at_last_second = 0;
  int bits_at_last_second = 0;
  int bits = 0;
  char log[1024];
  while (true) {
    Frame frame_u;
    Frame frame_c;
    Read(compressed, frame_c);
    if (compressed.eof()) {
      break;
    }
    Read(uncompressed, frame_u);
    if (uncompressed.eof()) {
      break;
    }
    std::cout << i << ": " << frame_u.timestampHns << " - " << frame_c.timestampHns << std::endl;
    while (frame_u.timestampHns / 10000 < frame_c.timestampHns / 10000) {
      Read(uncompressed, frame_u);
      if (uncompressed.eof()) {
        break;
      }
      std::cout << frame_u.timestampHns << std::endl;
    }


    //assert(frame_u.timestampHns == frame_c.timestampHns);

    out.write((char*)&frame_u.timestampHns, sizeof(frame_u.timestampHns));
    out.write((char*)&frame_u.durationHns, sizeof(frame_u.durationHns));
    out.write((char*)&frame_u.totalSize, sizeof(frame_u.totalSize));
    out.write((char*)frame_u.buffer.data(), frame_u.totalSize);
    ++i;
  }
}

int64_t time_to_sleep;

void DoEncode(std::ifstream& uncompressed_file,
              std::ofstream& compressed_file,
              int width,
              int height,
              int framerate,
              int bitrate,
              Mode mode,
              webrtc::H264::Profile profile,
              int maxQp,
              int quality) {
  webrtc::VideoCodec codec;
  codec.width = width;
  codec.height = height;
  codec.maxFramerate = framerate;
  codec.maxBitrate = 2500;
  codec.targetBitrate = bitrate;
  codec.startBitrate = bitrate;
  int frame_size = codec.width * codec.height * 3 / 2;

  webrtc__WinUWPH264EncoderImpl__profile = profile;
  webrtc__WinUWPH264EncoderImpl__mode = mode;
  webrtc__WinUWPH264EncoderImpl__maxQp = maxQp;
  webrtc__WinUWPH264EncoderImpl__quality = quality;


  LONGLONG last_frame_timestamp = 0;
  LONGLONG last_timestamp = rtc::TimeMillis();
  LONGLONG begin_timestamp = rtc::TimeMillis();
  {
    auto destBuffer = std::make_unique<uint8_t[]>(frame_size);

    auto encoder = std::make_unique<webrtc::WinUWPH264EncoderImpl>();
    encoder->InitEncode(&codec, 0, 0);

    std::atomic<bool> arrived = false;

    int i = 0;
    webrtc::EncodedImageCallback cb = [&](const uint8_t* byteBuffer, int64_t sampleTimestamp,
                                  int32_t curLength, bool keyframe) {
       LONGLONG zero = 0;
      if (curLength) {
        // if (first_decoded_ts < 0) {
        //  first_decoded_ts = now;
        //}
        // int32_t now_delta = (int32_t)(now - first_decoded_ts);
        compressed_file.write((char*)&sampleTimestamp, sizeof(sampleTimestamp));
        compressed_file.write((char*)&zero, sizeof(zero));
        // compressed_file.write((char*)&now_delta, sizeof(now_delta));
        compressed_file.write((char*)&curLength, sizeof(curLength));
        compressed_file.write((char*)byteBuffer, curLength);

        if (keyframe) {
          std::stringstream str;
          str << i << " is a keyframe " << std::endl;
          OutputDebugStringA(str.str().c_str());
        }
      }
      arrived.store(true);
    };
    encoder->RegisterEncodeCompleteCallback(&cb);
    char log[1024];
    int64_t first_delivery_ts = -1;
    while (true) {
      int64_t timestampHns, durationHns;
      int32_t totalSize;
      int32_t delivery_ts;

      uncompressed_file.read((char*)&timestampHns, sizeof(timestampHns));
      if (uncompressed_file.eof()) {
        break;
      }
      uncompressed_file.read((char*)&durationHns, sizeof(durationHns));
      //uncompressed_file.read((char*)&delivery_ts, sizeof(delivery_ts));
      uncompressed_file.read((char*)&totalSize, sizeof(totalSize));
      assert(totalSize == frame_size);
      uncompressed_file.read((char*)destBuffer.get(), totalSize);

       auto actualDurationHns = timestampHns - last_frame_timestamp;

       //sprintf(log, "%d TS %lld D %lld AD %lld S %d\n", i, timestampHns /
       //10'000, durationHns / 10'000, actualDurationHns / 10'000,
       //       totalSize);
       //OutputDebugStringA(log);

      webrtc::VideoFrame frame;
      frame.width_ = codec.width;
      frame.height_ = codec.height;
      frame.video_frame_buffer_ = destBuffer.get();
      frame.timestamp_rtp_ = timestampHns * 90 / 10'000;

      //auto now = rtc::TimeMillis();
      //if (first_delivery_ts < 0) {
      //  first_delivery_ts = now;
      //}

      // auto now_delta = now - first_delivery_ts;
      // time_to_sleep = max(0, delivery_ts - now_delta);
      // auto now_delta = now - begin_timestamp;
      // time_to_sleep = max(0, timestampHns / 10'000 - now_delta);
      // if (time_to_sleep) {
      //  Sleep(time_to_sleep);
      //}
      // Sleep(33);

      //last_timestamp = now;
      last_frame_timestamp = timestampHns;

      encoder->Encode(frame, false);

      while (!arrived.load()) {
        Sleep(33);
      }
      arrived.store(false);
      ++i;
    }
  }
}

//
//
//HRESULT InitializeSinkWriter(IMFSinkWriter** ppWriter, DWORD* pStreamIndex) {
//  *ppWriter = NULL;
//  *pStreamIndex = NULL;
//
//  IMFSinkWriter* pSinkWriter = NULL;
//  IMFMediaType* pMediaTypeOut = NULL;
//  IMFMediaType* pMediaTypeIn = NULL;
//  DWORD streamIndex;
//
//  HRESULT hr =
//      MFCreateSinkWriterFromURL(L"output.wmv", NULL, NULL, &pSinkWriter);
//
//  // Set the output media type.
//  if (SUCCEEDED(hr)) {
//    hr = MFCreateMediaType(&pMediaTypeOut);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pMediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pMediaTypeOut->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_WMV3);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pMediaTypeOut->SetUINT32(MF_MT_AVG_BITRATE, VIDEO_BIT_RATE);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pMediaTypeOut->SetUINT32(MF_MT_INTERLACE_MODE,
//                                  MFVideoInterlace_Progressive);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = MFSetAttributeSize(pMediaTypeOut, MF_MT_FRAME_SIZE, VIDEO_WIDTH,
//                            VIDEO_HEIGHT);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_FRAME_RATE, VIDEO_FPS, 1);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pSinkWriter->AddStream(pMediaTypeOut, &streamIndex);
//  }
//
//  // Set the input media type.
//  if (SUCCEEDED(hr)) {
//    hr = MFCreateMediaType(&pMediaTypeIn);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pMediaTypeIn->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pMediaTypeIn->SetGUID(MF_MT_SUBTYPE, VIDEO_INPUT_FORMAT);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pMediaTypeIn->SetUINT32(MF_MT_INTERLACE_MODE,
//                                 MFVideoInterlace_Progressive);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = MFSetAttributeSize(pMediaTypeIn, MF_MT_FRAME_SIZE, VIDEO_WIDTH,
//                            VIDEO_HEIGHT);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = MFSetAttributeRatio(pMediaTypeIn, MF_MT_FRAME_RATE, VIDEO_FPS, 1);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = MFSetAttributeRatio(pMediaTypeIn, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pSinkWriter->SetInputMediaType(streamIndex, pMediaTypeIn, NULL);
//  }
//
//  // Tell the sink writer to start accepting data.
//  if (SUCCEEDED(hr)) {
//    hr = pSinkWriter->BeginWriting();
//  }
//
//  // Return the pointer to the caller.
//  if (SUCCEEDED(hr)) {
//    *ppWriter = pSinkWriter;
//    (*ppWriter)->AddRef();
//    *pStreamIndex = streamIndex;
//  }
//
//  SafeRelease(&pSinkWriter);
//  SafeRelease(&pMediaTypeOut);
//  SafeRelease(&pMediaTypeIn);
//  return hr;
//}
//
//HRESULT WriteFrame(IMFSinkWriter* pWriter,
//                   DWORD streamIndex,
//                   const LONGLONG& rtStart  // Time stamp.
//) {
//  IMFSample* pSample = NULL;
//  IMFMediaBuffer* pBuffer = NULL;
//
//  const LONG cbWidth = 4 * VIDEO_WIDTH;
//  const DWORD cbBuffer = cbWidth * VIDEO_HEIGHT;
//
//  BYTE* pData = NULL;
//
//  // Create a new memory buffer.
//  HRESULT hr = MFCreateMemoryBuffer(cbBuffer, &pBuffer);
//
//  // Lock the buffer and copy the video frame to the buffer.
//  if (SUCCEEDED(hr)) {
//    hr = pBuffer->Lock(&pData, NULL, NULL);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = MFCopyImage(pData,                    // Destination buffer.
//                     cbWidth,                  // Destination stride.
//                     (BYTE*)videoFrameBuffer,  // First row in source image.
//                     cbWidth,                  // Source stride.
//                     cbWidth,                  // Image width in bytes.
//                     VIDEO_HEIGHT              // Image height in pixels.
//    );
//  }
//  if (pBuffer) {
//    pBuffer->Unlock();
//  }
//
//  // Set the data length of the buffer.
//  if (SUCCEEDED(hr)) {
//    hr = pBuffer->SetCurrentLength(cbBuffer);
//  }
//
//  // Create a media sample and add the buffer to the sample.
//  if (SUCCEEDED(hr)) {
//    hr = MFCreateSample(&pSample);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pSample->AddBuffer(pBuffer);
//  }
//
//  // Set the time stamp and the duration.
//  if (SUCCEEDED(hr)) {
//    hr = pSample->SetSampleTime(rtStart);
//  }
//  if (SUCCEEDED(hr)) {
//    hr = pSample->SetSampleDuration(VIDEO_FRAME_DURATION);
//  }
//
//  // Send the sample to the Sink Writer.
//  if (SUCCEEDED(hr)) {
//    hr = pWriter->WriteSample(streamIndex, pSample);
//  }
//
//  SafeRelease(&pSample);
//  SafeRelease(&pBuffer);
//  return hr;
//}


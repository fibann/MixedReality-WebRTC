//
// MainPage.xaml.cpp
// Implementation of the MainPage class.
//

#include "MainPage.xaml.h"

#include <assert.h>

#include <algorithm>
#include <codecvt>
#include <fstream>

#include "H264Encoder/H264Encoder.h"
#include "pch.h"

using namespace Encode;

using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::Foundation::Collections;
using namespace Windows::UI::Xaml;
using namespace Windows::UI::Xaml::Controls;
using namespace Windows::UI::Xaml::Controls::Primitives;
using namespace Windows::UI::Xaml::Data;
using namespace Windows::UI::Xaml::Input;
using namespace Windows::UI::Xaml::Media;
using namespace Windows::UI::Xaml::Navigation;

// The Blank Page item template is documented at
// https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

void ReadFile(const char* filename, int frame_size) {
  std::ifstream compressed_file(filename, std::ifstream::binary);
  assert(compressed_file.good());
  auto destBuffer = new char[frame_size];

  int i = 0;
  LONGLONG last_frame_timestamp = 0;
  int last_second = 0;
  int frames_at_last_second = 0;
  int bits_at_last_second = 0;
  int bits = 0;
  char log[1024];
  while (!compressed_file.eof()) {
    int64_t timestampHns, durationHns;
    int32_t totalSize;

    compressed_file.read((char*)&timestampHns, sizeof(timestampHns));
    compressed_file.read((char*)&durationHns, sizeof(durationHns));
    compressed_file.read((char*)&totalSize, sizeof(totalSize));
    assert(totalSize <= frame_size);
    compressed_file.read((char*)destBuffer, totalSize);

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
    last_frame_timestamp = timestampHns;
    sprintf(log, "%d TS %lld D %lld AD %lld S %d\n", i++, timestampHns / 10'000,
            durationHns / 10'000, actualDurationHns / 10'000, totalSize);
    bits += totalSize * 8;
    OutputDebugStringA(log);
  }

  sprintf(log, "%f fps %f kbps\n",
          ((float)i) / (last_frame_timestamp / 10'000'000),
          bits / 1000.0f / (last_frame_timestamp / 10'000'000));
  OutputDebugStringA(log);
}

void Encode::MainPage::Button_Click(Platform::Object ^ sender,
                                    Windows::UI::Xaml::RoutedEventArgs ^ e) {}

const int WIDTH = 1280;
const int HEIGHT = 720;

void DoEncode(const std::string& folderNameA) {
  webrtc::VideoCodec codec;
  // uncompressed_file.read((char*)&codec.width, sizeof(codec.width));
  // uncompressed_file.read((char*)&codec.height, sizeof(codec.height));
  // uncompressed_file.read((char*)&codec.maxFramerate,
  // sizeof(codec.maxFramerate));
  codec.width = WIDTH;
  codec.height = HEIGHT;
  codec.maxFramerate = 30;
  codec.maxBitrate = 2500;
  codec.targetBitrate = 0;
  codec.startBitrate = 1000;
  int frame_size = codec.width * codec.height * 3 / 2;

  LONGLONG last_frame_timestamp = 0;
  LONGLONG last_timestamp = rtc::TimeMillis();
  {
    std::ifstream uncompressed_file(folderNameA + "\\uncompressed.dat",
                                    std::ifstream::binary);
    assert(uncompressed_file.good());

    auto destBuffer = std::make_unique<uint8_t[]>(frame_size);

    auto encoder = std::make_unique<webrtc::WinUWPH264EncoderImpl>();
    encoder->InitEncode(&codec, 0, 0);
    int i = 0;
    char log[1024];
    while (!uncompressed_file.eof()) {
      int64_t timestampHns, durationHns;
      int32_t totalSize;

      uncompressed_file.read((char*)&timestampHns, sizeof(timestampHns));
      uncompressed_file.read((char*)&durationHns, sizeof(durationHns));
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
      auto diff = timestampHns - last_frame_timestamp;

      auto time_to_sleep = max(0, diff / 10'000 - (now - last_timestamp));
      if (time_to_sleep) {
        Sleep(time_to_sleep);
      }
      last_timestamp = now;
      last_frame_timestamp = timestampHns;

      encoder->Encode(frame, false);
    }
  }
}

MainPage::MainPage() {
  InitializeComponent();
  auto current = Windows::Storage::ApplicationData::Current;
  auto ppath = current->LocalFolder->Path;
  std::wstring wpath(ppath->Data());

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
  using convert_type = std::codecvt_utf8<wchar_t>;
  std::wstring_convert<convert_type, wchar_t> converter;

  // use converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
  std::string folderNameA = converter.to_bytes(wpath);

  int frame_size = WIDTH * HEIGHT * 3 / 2;

    DoEncode(folderNameA);
  ReadFile((folderNameA + "\\uncompressed.dat").c_str(), frame_size);
  ReadFile((folderNameA + "\\compressed_out.dat").c_str(), frame_size);
  // ReadFile((folderNameA + "\\uncompressed.dat").c_str(), frame_size);
}

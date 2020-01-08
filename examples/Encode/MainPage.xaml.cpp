//
// MainPage.xaml.cpp
// Implementation of the MainPage class.
//

#include "MainPage.xaml.h"
#include <assert.h>
#include <codecvt>
#include <fstream>
#include "H264Encoder/H264Encoder.h"
#include "pch.h"
#include <algorithm>

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
  while (!compressed_file.eof()) {
    int64_t timestampHns, durationHns;
    int32_t totalSize;

    compressed_file.read((char*)&timestampHns, sizeof(timestampHns));
    compressed_file.read((char*)&durationHns, sizeof(durationHns));
    compressed_file.read((char*)&totalSize, sizeof(totalSize));
    assert(totalSize <= frame_size);
    compressed_file.read((char*)destBuffer, totalSize);

    char log[1024];
    sprintf(log, "%d TS %lld D %lld S %d\n", i++, timestampHns / 10'000,
            durationHns / 10'000, totalSize);
    OutputDebugStringA(log);
  }
}

void Encode::MainPage::Button_Click(Platform::Object ^ sender,
                                    Windows::UI::Xaml::RoutedEventArgs ^ e) {}

void DoEncode() {
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

  webrtc::VideoCodec codec;
  // uncompressed_file.read((char*)&codec.width, sizeof(codec.width));
  // uncompressed_file.read((char*)&codec.height, sizeof(codec.height));
  // uncompressed_file.read((char*)&codec.maxFramerate,
  // sizeof(codec.maxFramerate));
  codec.width = 1280;
  codec.height = 720;
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
    while (!uncompressed_file.eof()) {
      int64_t timestampHns, durationHns;
      int32_t totalSize;

      uncompressed_file.read((char*)&timestampHns, sizeof(timestampHns));
      uncompressed_file.read((char*)&durationHns, sizeof(durationHns));
      uncompressed_file.read((char*)&totalSize, sizeof(totalSize));
      assert(totalSize == frame_size);
      uncompressed_file.read((char*)destBuffer.get(), totalSize);

      char log[1024];
      sprintf(log, "%d TS %lld D %lld S %d\n", i++, timestampHns / 10'000,
              durationHns / 10'000, totalSize);
      OutputDebugStringA(log);

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

  ReadFile((folderNameA + "\\compressed.dat").c_str(), frame_size);
  ReadFile((folderNameA + "\\compressed_out.dat").c_str(), frame_size);
}

MainPage::MainPage() {
  InitializeComponent();
  DoEncode();
}

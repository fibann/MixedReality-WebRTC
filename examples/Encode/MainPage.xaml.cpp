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

void ReadFile(std::ifstream& file, int frame_size, int num_frames = 100);
void DoEncode(std::ifstream& uncompressed_file,
              std::ofstream& compressed_file,
              int width,
              int height,
              int framerate,
              int bitrate,
              int num_frames = 100);
void StripHeaders(std::istream& file, std::ostream& out, int num_frames = 100);
void RemoveMissingFrames(std::istream& uncompressed,
                         std::istream& compressed,
                         std::ostream& out,
                         int num_frames = 100);


  const int WIDTH = 1280;
const int HEIGHT = 720;

int frame_size = WIDTH * HEIGHT * 3 / 2;

MainPage::MainPage() {
  InitializeComponent();

  const auto uncompressed = "uncompressed_full";
  const auto compressed = "compressed_HL2-vbr-maxQP45-1600";

  auto current = Windows::Storage::ApplicationData::Current;
  auto ppath = current->LocalFolder->Path;
  std::wstring wpath(ppath->Data());
  using convert_type = std::codecvt_utf8<wchar_t>;
  std::wstring_convert<convert_type, wchar_t> converter;
  std::string folderNameA = converter.to_bytes(wpath);
  {
    std::ifstream uncompressed_file(
        (folderNameA + "\\" + uncompressed + ".dat").c_str(),
        std::ios_base::binary);
    std::ofstream compressed_file(
        (folderNameA + "\\" + compressed + ".dat").c_str(),
        std::ios_base::binary);

    DoEncode(uncompressed_file, compressed_file, WIDTH, HEIGHT, 30, 1600,
             INT_MAX);
  }
  {
    std::ifstream compressed_file(
        (folderNameA + "\\" + compressed + ".dat").c_str(),
        std::ios_base::binary);
    std::ofstream no_headers(
        (folderNameA + "\\" + compressed + ".h264").c_str(),
        std::ios_base::binary);

    StripHeaders(compressed_file, no_headers, 500);
  }

  //std::ifstream uncompressed_file(
  //    (folderNameA + "\\capture_2020-04-24\\uncompressed.dat").c_str(),
  //    std::ios_base::binary);
  //std::ifstream compressed_file(
  //    (folderNameA + "\\capture_2020-04-24\\compressed.dat").c_str(),
  //    std::ios_base::binary);
  //std::ifstream uncompressed_file(
  //    (folderNameA + "\\uncompressed_full.dat").c_str(), std::ios_base::binary);
  //std::ifstream compressed_file(
  //    (folderNameA + "\\compressed_HL2_1280x720-30-1000-vbr-gop4.dat").c_str(),
  //    std::ios_base::binary);
  //ReadFile(uncompressed_file, frame_size, INT_MAX);
  //ReadFile(compressed_file, frame_size, INT_MAX);
  OutputDebugStringA("--- DONE ---\n");
}

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

void ReadFile(const char* filename, int frame_size);

void DoEncode(const std::string& folderNameA, int width, int height);

MainPage::MainPage() {
  InitializeComponent();
  auto current = Windows::Storage::ApplicationData::Current;
  auto ppath = current->LocalFolder->Path;
  std::wstring wpath(ppath->Data());
  using convert_type = std::codecvt_utf8<wchar_t>;
  std::wstring_convert<convert_type, wchar_t> converter;
  std::string folderNameA = converter.to_bytes(wpath);

  const int WIDTH = 1280;
  const int HEIGHT = 720;

  int frame_size = WIDTH * HEIGHT * 3 / 2;

    DoEncode(folderNameA, WIDTH, HEIGHT);
  ReadFile((folderNameA + "\\uncompressed.dat").c_str(), frame_size);
  ReadFile((folderNameA + "\\compressed_out.dat").c_str(), frame_size);
   ReadFile((folderNameA + "\\compressed.dat").c_str(), frame_size);
}

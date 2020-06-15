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
              webrtc::H264::Profile profile = webrtc::H264::kProfileBaseline,
              int maxQp = -1,
              int quality = -1);
  void StripHeaders(std::istream& file,
                  std::ostream& out,
                  int num_frames = INT_MAX,
                  int start_frame = 0);
void RemoveMissingFrames(std::istream& uncompressed,
                         std::istream& compressed,
                         std::ostream& out,
                         int num_frames = INT_MAX);



MainPage::MainPage() {
  InitializeComponent();
}

int ToInt(TextBox ^ box, const wchar_t* name, TextBlock ^ log) {
  try {
    auto wstr = std::wstring(box->Text->Data());
    return std::stoi(wstr);
  } catch (std::exception e) {
    std::wstring msg(L"Invalid ");
    msg += name;
    using convert_type = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_type> converter;
    std::wstring w = converter.from_bytes(e.what());
    msg += L":" + w;
    log->Text = ref new Platform::String(msg.c_str());
    return -1;
  }
}

void MainPage::Button_Click(Platform::Object ^ sender,
                       Windows::UI::Xaml::RoutedEventArgs ^ e) {
  auto input_val = this->input->Text;
  auto output_val = this->output->Text;



  auto current = Windows::Storage::ApplicationData::Current;
  auto ppath = current->LocalFolder->Path;
  using convert_type = std::codecvt_utf8<wchar_t>;
  std::wstring_convert<convert_type, wchar_t> converter;
  auto input_abs = ppath + L"\\" + input_val;
  auto output_abs = ppath + L"\\" + output_val;
  {
    std::ifstream uncompressed_file(converter.to_bytes(input_abs->Data()),
        std::ios_base::binary);

    if (!uncompressed_file.good()) {
      log->Text = L"Invalid input: " + input_abs;
      return;
    }

    std::ofstream compressed_file(converter.to_bytes(output_abs->Data()),
        std::ios_base::binary);

    if (!compressed_file.good()) {
      log->Text = L"Invalid output: " + output_abs;
      return;
    }

    int w = ToInt(width, L"width", log);
    if (w <= 0) {
      return;
    }

    int h = ToInt(height, L"height", log);
    if (h <= 0) {
      return;
    }

    int br = ToInt(bitrate, L"bitrate", log);
    if (br <= 0) {
      return;
    }

    int fr = ToInt(fps, L"fps", log);
    if (fr <= 0) {
      return;
    }

    auto p = (webrtc::H264::Profile)profile->SelectedIndex;

    int qp = ToInt(max_qp, L"MaxQP", log);
    if (qp <= 0) {
      return;
    }
    int q = ToInt(quality, L"quality", log);
    if (q <= 0) {
      return;
    }

    DoEncode(uncompressed_file, compressed_file, w, h, fr, br, p, qp, q);
    log->Text = L"Encoding done";
  }
  OutputDebugStringA("--- DONE ---\n");
}

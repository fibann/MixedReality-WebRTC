#pragma once

#include "H264Encoder/H264Encoder.h"
#include <iosfwd>

void Log(const char* msg);

void ReadFile(std::ifstream& file, int frame_size);

void StripHeaders(std::istream& file,
                  std::ostream& out,
                  int num_frames,
                  int start_frame);

void Trim(std::istream& file,
          std::ostream& out,
          int num_frames,
          int start_frame);


void RemoveMissingFrames(std::istream& uncompressed,
                         std::istream& compressed,
                         std::ostream& out);


void DoEncode(std::ifstream& uncompressed_file,
              std::ofstream& compressed_file,
              int width,
              int height,
              int framerate,
              int bitrate,
              Mode mode = Mode::CBR,
              webrtc::H264::Profile profile = webrtc::H264::kProfileBaseline,
              int maxQp = -1,
              int quality = -1);

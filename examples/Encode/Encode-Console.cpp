// Encode-Console.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <string>
#include <iostream>
#include <fstream>

void ReadFile(std::ifstream& file, int frame_size, int num_frames = 100);
void DoEncode(std::ifstream& uncompressed_file,
              std::ofstream& compressed_file,
              int width,
              int height,
    int framerate,
              int bitrate,
              int num_frames = INT_MAX,
              int start_frame = 0);
void StripHeaders(std::istream& file,
                  std::ostream& out,
                  int num_frames = INT_MAX,
                  int start_frame = 0);
void Trim(std::istream& file,
                  std::ostream& out,
                  int num_frames = INT_MAX,
                  int start_frame = 0);
void RemoveMissingFrames(std::istream& uncompressed,
                         std::istream& compressed,
                         std::ostream& out,
                         int num_frames = 100);
int main(int argc, char* argv[]) {


  //std::ifstream compressed_file(
  //    R"(C:\Users\fiban\Downloads\capture_2020-04-24\compressed.dat)",
  //    std::ios_base::binary);
  //std::ofstream out(
  //    R"(C:\Users\fiban\Downloads\capture_2020-04-24\compressed.264)",
  //    std::ios_base::binary);

  //StripHeaders(compressed_file, out);

  //if (argc != 7) {
  //  std::cout
  //      << "Usage: Encode-Console <width> <height> <fps> <target_bitrate_kbps> <input_file> <output_file>"
  //      << std::endl;
  //  return 1;
  //}

   //int width = std::atoi(argv[1]);
   //int heigth = std::atoi(argv[2]);
   //int frame_rate = std::atoi(argv[3]);
   //int bitrate = std::atoi(argv[4]);
   //const char* input_file = argv[5];
   //const char* output_file = argv[6];

   int width = 1280;
   int heigth = 720;
   int frame_rate = 30;
   int bitrate = 1000;

   const char* dir =
       R"(C:\Users\fiban\Downloads\capture_2020-06-10\)";


   const char* input_file_name = "uncompressed_complex";
   const char* output_file_name = "compressed_complex-high-maxQP45";

   auto input_file = std::string(dir) + input_file_name + ".dat";
   auto output_file = std::string(dir) + output_file_name + ".dat";

  if (width <= 0 || heigth <= 0) {
    std::cout << "Invalid res: " << width << "," << heigth << std::endl;
    return 1;
  }

  if (frame_rate <= 0) {
    std::cout << "Invalid frame rate: " << frame_rate << std::endl;
    return 1;
  }
  if (bitrate <= 0) {
    std::cout << "Invalid bitrate: " << bitrate << std::endl;
    return 1;
  }

  //{
  //  std::ifstream uncompressed_file(input_file, std::ios_base::binary);
  //  if (!uncompressed_file.good()) {
  //    std::cout << "Invalid input: " << input_file << std::endl;
  //    return 1;
  //  }
  //  std::ofstream compressed_file(output_file, std::ios_base::binary);
  //  if (!compressed_file.good()) {
  //    std::cout << "Invalid output: " << output_file << std::endl;
  //    return 1;
  //  }

  //  DoEncode(uncompressed_file, compressed_file, width, heigth, frame_rate,
  //           bitrate, INT_MAX);
  //}

  std::ifstream uncompressed_file(input_file, std::ios_base::binary);
  std::ifstream compressed_file(output_file, std::ios_base::binary);
   if (!uncompressed_file.good()) {
    std::cout << "Invalid input: " << input_file << std::endl;
    return 1;
  }
   if (!compressed_file.good()) {
    std::cout << "Invalid output: " << output_file << std::endl;
    return 1;
  }

  //{
  //  std::ofstream stripped_file(unc_stripped, std::ios_base::binary);
  //  RemoveMissingFrames(uncompressed_file, compressed_file, stripped_file,
  //                      INT_MAX);
  //}

   //auto unc_raw = std::string(dir) + input_file_name + ".raw";
  auto unc_raw = std::string(dir) + input_file_name + "_trim.dat";
  std::ifstream stripped_file(input_file, std::ios_base::binary);
  std::ofstream raw_file(unc_raw, std::ios_base::binary);
  Trim(stripped_file, raw_file, 720 - 180, 180);

   //auto compr_raw = std::string(dir) + output_file_name + ".h264";
  auto compr_raw = std::string(dir) + output_file_name + "_trim.dat";
  std::ifstream cfile(output_file, std::ios_base::binary);
  std::ofstream crawfile(compr_raw, std::ios_base::binary);
  Trim(cfile, crawfile, 720 - 180, 180);

  int frame_size = width * heigth * 3 / 2;

  //ReadFile(uncompressed_file, frame_size, INT_MAX);
  //ReadFile(compressed_file, frame_size, INT_MAX);
}

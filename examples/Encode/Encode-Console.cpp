// Encode-Console.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <string>
#include <iostream>
#include <fstream>

void ReadFile(std::ifstream& file, int frame_size);
void DoEncode(std::ifstream& uncompressed_file,
              std::ofstream& compressed_file,
              int width,
              int height,
    int framerate,
    int bitrate);

int main(int argc, char* argv[]) {

  if (argc != 7) {
    std::cout
        << "Usage: Encode-Console <width> <height> <fps> <target_bitrate_kbps> <input_file> <output_file>"
        << std::endl;
    return 1;
  }

  int width = std::atoi(argv[1]);
  int heigth = std::atoi(argv[2]);
  int frame_rate = std::atoi(argv[3]);
  int bitrate = std::atoi(argv[4]);
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

  {
    std::ifstream uncompressed_file(argv[5], std::ios_base::binary);
    if (!uncompressed_file.good()) {
      std::cout << "Invalid input: " << argv[5] << std::endl;
      return 1;
    }
    std::ofstream compressed_file(argv[6], std::ios_base::binary);
    if (!compressed_file.good()) {
      std::cout << "Invalid output: " << argv[6] << std::endl;
      return 1;
    }

    DoEncode(uncompressed_file, compressed_file, width, heigth, frame_rate, bitrate);
  }

  std::ifstream uncompressed_file(argv[5], std::ios_base::binary);
  std::ifstream compressed_file(argv[6], std::ios_base::binary);
  int frame_size = width * heigth * 3 / 2;

  ReadFile(uncompressed_file, frame_size);
  ReadFile(compressed_file, frame_size);
}

// Encode-Console.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <string>
#include <iostream>
#include <fstream>

void ReadFile(std::ifstream& file, int frame_size);
void DoEncode(std::ifstream& uncompressed_file,
              std::ofstream& compressed_file,
              int width,
              int height);

int main(int argc, char* argv[]) {

  if (argc != 5) {
    std::cout
        << "Usage: Encode-Console <width> <height> <input_file> <output_file>"
        << std::endl;
    return 1;
  }

  int width = std::atoi(argv[1]);
  int heigth = std::atoi(argv[2]);
  if (width < 0 || heigth < 0) {
    std::cout << "Invalid res: " << width << "," << heigth << std::endl;
    return 1;
  }

  {
    std::ifstream uncompressed_file(argv[3], std::ios_base::binary);
    if (!uncompressed_file.good()) {
      std::cout << "Invalid input: " << argv[3] << std::endl;
      return 1;
    }
    std::ofstream compressed_file(argv[4], std::ios_base::binary);
    if (!compressed_file.good()) {
      std::cout << "Invalid output: " << argv[4] << std::endl;
      return 1;
    }

    DoEncode(uncompressed_file, compressed_file, width, heigth);
  }


  std::ifstream uncompressed_file(argv[3], std::ios_base::binary);
  std::ifstream compressed_file(argv[4], std::ios_base::binary);
  int frame_size = width * heigth * 3 / 2;

  ReadFile(uncompressed_file, frame_size);
  ReadFile(compressed_file, frame_size);
}

Usage: Encode-Console.exe <width> <height> <frame_rate> <target_bitrate_kbps> <input_file> <output_file>

E.g. Usage: Encode-Console.exe 1280 720 30 1000 C:\input\uncompressedXYZ.dat C:\output\compressedXYZ.dat

Encodes a captured NV12 stream using the Media Foundation H.264 encoder
configured with the passed target bitrate in Kbps. 

The input stream must be a sequential dump of independent frames with the
resolution and framerate passed to the command line. Every frame must have a
20 bytes header:

Timestamp in hundreds of ns - LONGLONG (8)
Duration in hundreds of ns - LONGLONG (8)
Frame size in bytes - DWORD (4)

followed by the frame data.

The output file will be a dump with the same format. Output frames have variable
length and might not correspond 1:1 to the source frames (some frames might be dropped).
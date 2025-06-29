COMPILE:

g++ -o batch_3fr_to_exr batch_3fr_to_exr.cpp $(pkg-config --cflags --libs OpenEXR Imath) -lraw -lz -pthread

USE:
./batch_3fr_to_exr /path/to/3fr/files

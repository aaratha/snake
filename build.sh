# Create build directory if it doesn't exist
mkdir -p build

# Run CMake to configure and build
cmake -B build \
  -DCMAKE_PREFIX_PATH=/run/current-system/sw \
  -DFREETYPE_INCLUDE_DIRS=$(pkg-config --variable=includedir freetype2) \
  -DFREETYPE_LIBRARY=$(pkg-config --variable=libdir freetype2)/libfreetype.so
cmake --build build

./build/snake

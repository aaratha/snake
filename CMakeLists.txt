cmake_minimum_required(VERSION 3.20)
project(snake LANGUAGES CXX)

if(UNIX AND NOT APPLE)
  list(APPEND CMAKE_PREFIX_PATH "/run/current-system/sw")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(SDL_WAYLAND ON CACHE BOOL "" FORCE)
set(SDL_X11 ON CACHE BOOL "" FORCE)
# NixOS: disable dynamic loading so SDL links directly to wayland/X11 instead of dlopen-ing them
set(SDL_WAYLAND_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_X11_SHARED OFF CACHE BOOL "" FORCE)

# Add SDL3 submodule
add_subdirectory(external/sdl)
add_subdirectory(external/sdl_ttf)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Gather source files
file(GLOB_RECURSE SLINGER_SOURCES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

# Create executable
add_executable(snake ${SLINGER_SOURCES})

# Link SDL3
target_link_libraries(snake PRIVATE SDL3::SDL3 SDL3_ttf::SDL3_ttf)


# Optionally: if you want to use SDL_main (for Win/macOS entry point)
# target_link_libraries(snake PRIVATE SDL3::SDL3main)

# If you want compiler warnings
target_compile_options(snake PRIVATE -Wall -Wextra -Wpedantic)

execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
)

cmake_minimum_required(VERSION 3.16)
cmake_policy(VERSION "3.16")

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake
  CACHE STRING "Vcpkg toolchain file")

project(fast-euclidean-clustering)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL CONFIG REQUIRED)

add_subdirectory(examples)

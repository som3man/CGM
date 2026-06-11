cmake_minimum_required(VERSION 3.28)

project("CGM")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(CGM)

target_compile_features(CGM PUBLIC cxx_std_20)

target_sources(CGM
    PUBLIC
        FILE_SET CXX_MODULES FILES
            src/CGM.ixx
            src/CGM-Angle.ixx
            src/CGM-Matrix.ixx
            src/CGM-Rotation.ixx
            src/CGM-Vector.ixx
            src/CGM-Defines.ixx
)
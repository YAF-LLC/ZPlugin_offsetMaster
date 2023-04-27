#
# Copyright (C) 2023 Kazutaka Nakashima (kazutaka.nakashima@n-taka.info)
#
# GPLv3
#
# This file is part of offsetMaster.
#
# offsetMaster is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
# offsetMaster is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along with offsetMaster. If not, see <https://www.gnu.org/licenses/>.
#

cmake_minimum_required(VERSION 3.15)

if(WIN32)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif ()

set(PROJECT resampleWithMarchingCubes)
project(${PROJECT})

##################
##### target #####
##################
# Target for this build
add_library(${PROJECT} SHARED)

target_sources(
    ${PROJECT}
    PRIVATE
    src/resample.cpp
    src/readGoZAndTriangulate.cpp
)
########################
##### target (end) #####
########################


#####################
##### Libraries #####
#####################
###
# json
find_package(nlohmann_json CONFIG REQUIRED)

###
# eigen3
find_package(Eigen3 CONFIG REQUIRED)

# Add external libraries
target_link_libraries(
    ${PROJECT}
    PRIVATE
    nlohmann_json
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)
###########################
##### Libraries (end) #####
###########################


###################
##### Include #####
###################
# Add a include files
target_include_directories(
    ${PROJECT}
    PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../submodule/libigl/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../submodule/eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/../submodule/FromZ/src
)
#########################
##### Include (end) #####
#########################


###########################
##### compile options #####
###########################
# because we want to use std::filesystem, we require C++17
target_compile_features(${PROJECT} PRIVATE cxx_std_17)
target_compile_definitions(${PROJECT} PRIVATE _USE_MATH_DEFINES)

if (WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4244 /wd4819")
    add_definitions(-D_WIN32_WINNT=0x0A00)
endif ()
#################################
##### compile options (end) #####
#################################
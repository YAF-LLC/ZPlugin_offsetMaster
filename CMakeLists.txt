cmake_minimum_required(VERSION 3.15)

if(WIN32)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif ()

set(PROJECT ZOffset_Master)
project(${PROJECT})

# version
set(Version "1.0")
# basically, we use Release
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
endif()

##################
##### target #####
##################
# Target for this build
add_executable(${PROJECT})

target_sources(
    ${PROJECT}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
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

add_subdirectory(plugin)

# Add external libraries
target_link_libraries(
    ${PROJECT}
    PRIVATE
    nlohmann_json
    nlohmann_json::nlohmann_json
    resampleWithMarchingCubes
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
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin/include
    ${CMAKE_CURRENT_SOURCE_DIR}/submodule/libigl/include
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

#####################
##### Resources #####
#####################
if (WIN32)
    add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/build/plugin/Release/resampleWithMarchingCubes.dll" "$<TARGET_FILE_DIR:${PROJECT}>")
    add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/build/plugin/Release/resampleWithMarchingCubes.dll" "${CMAKE_CURRENT_SOURCE_DIR}/ZPlugs64/offsetMaster_2022/resampleWithMarchingCubes.dll")
elseif (APPLE)
    add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/build/plugin/libresampleWithMarchingCubes.dylib" "$<TARGET_FILE_DIR:${PROJECT}>/resampleWithMarchingCubes.lib")
    add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/build/plugin/libresampleWithMarchingCubes.dylib" "${CMAKE_CURRENT_SOURCE_DIR}/ZPlugs64/offsetMaster_2022/resampleWithMarchingCubes.lib")
endif ()
###########################
##### Resources (end) #####
###########################

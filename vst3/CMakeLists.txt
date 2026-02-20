cmake_minimum_required(VERSION 3.20)
project(PinchFXVST3 VERSION 0.9.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ENV{VST3_SDK_ROOT} AND NOT VST3_SDK_ROOT)
  message(FATAL_ERROR "Set VST3_SDK_ROOT to the Steinberg VST3 SDK path.")
endif()

if(NOT VST3_SDK_ROOT)
  set(VST3_SDK_ROOT $ENV{VST3_SDK_ROOT})
endif()

# Normalize path separators for CMake.
file(TO_CMAKE_PATH "${VST3_SDK_ROOT}" VST3_SDK_ROOT)

list(APPEND CMAKE_MODULE_PATH "${VST3_SDK_ROOT}/cmake/modules")
include("${VST3_SDK_ROOT}/cmake/modules/SMTG_VST3_SDK.cmake")
include("${VST3_SDK_ROOT}/cmake/modules/SMTG_AddVST3Library.cmake")

# Avoid building SDK samples/tools we don't need.
set(SMTG_ENABLE_VST3_PLUGIN_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SMTG_ENABLE_VST3_HOSTING_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SMTG_ADD_VST3_UTILITIES OFF CACHE BOOL "" FORCE)
set(SMTG_ENABLE_VSTGUI_SUPPORT ON CACHE BOOL "" FORCE)
set(SMTG_RUN_VST_VALIDATOR OFF CACHE BOOL "" FORCE)
set(SMTG_CREATE_MODULE_INFO OFF CACHE BOOL "" FORCE)
set(SMTG_CREATE_PLUGIN_LINK OFF CACHE BOOL "" FORCE)

# Bring in SDK targets (validator, moduleinfotool, etc.).
add_subdirectory(${VST3_SDK_ROOT} ${CMAKE_BINARY_DIR}/vst3sdk)

set(PinchFX_SOURCES
    src/PinchFxProcessor.cpp
    src/PinchFxController.cpp
    src/PinchFxPluginFactory.cpp
    ${VST3_SDK_ROOT}/public.sdk/source/main/dllmain.cpp
)

set(PinchFX_HEADERS
    src/PinchFxProcessor.h
    src/PinchFxController.h
    src/PinchFxIDs.h
    src/dsp/PitchTrackerACF.h
    src/dsp/PitchTrackerAlgo.h
    src/dsp/TwoPoleResonator.h
    src/dsp/OnePoleLP.h
    src/dsp/Oversampler2x.h
    src/dsp/TubeStage.h
    src/dsp/PeakLimiter.h
    src/dsp/PinchFxAlgorithm.h
)

smtg_add_vst3plugin(PinchFX
    PACKAGE_NAME "PinchFX"
    SOURCES_LIST ${PinchFX_SOURCES} ${PinchFX_HEADERS}
)

target_include_directories(PinchFX PRIVATE
    ${VST3_SDK_ROOT}
)

target_link_libraries(PinchFX PRIVATE
    sdk
    sdk_common
    base
    pluginterfaces
    vstgui_support
)

target_compile_definitions(PinchFX PRIVATE
    $<$<CONFIG:Debug>:VSTGUI_LIVE_EDITING=1>
    $<$<CONFIG:Debug>:PINCHFX_DEBUG_NAME>
    $<$<CONFIG:Debug>:PINCHFX_DEBUG_UIDS>
)

set_target_properties(PinchFX PROPERTIES
    OUTPUT_NAME "PinchFX"
    OUTPUT_NAME_DEBUG "DebugPinchFX"
    OUTPUT_NAME_RELEASE "PinchFX"
    OUTPUT_NAME_RELWITHDEBINFO "PinchFX"
    OUTPUT_NAME_MINSIZEREL "PinchFX"
)

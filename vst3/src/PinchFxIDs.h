// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include "pluginterfaces/base/funknown.h"
#include "pluginterfaces/vst/vsttypes.h"

namespace pinchfx {

#ifdef PINCHFX_DEBUG_UIDS
// {CF09A8AA-FA40-40E1-AA07-3D13FD6D1189}
static const Steinberg::FUID kPinchFxProcessorUID(0xCF09A8AA, 0xFA4040E1, 0xAA073D13, 0xFD6D1189);
// {EB8C6E5B-06A3-4714-99D4-1B9D97548FC3}
static const Steinberg::FUID kPinchFxControllerUID(0xEB8C6E5B, 0x06A34714, 0x99D41B9D, 0x97548FC3);
#else
// {CF09A8AA-FA40-40E1-AA07-3D13FD6D1189}
static const Steinberg::FUID kPinchFxProcessorUID(0xCF09A8AA, 0xFA4040E1, 0xAA073D13, 0xFD6D1189);
// {EB8C6E5B-06A3-4714-99D4-1B9D97548FC3}
static const Steinberg::FUID kPinchFxControllerUID(0xEB8C6E5B, 0x06A34714, 0x99D41B9D, 0x97548FC3);
#endif

#ifdef PINCHFX_DEBUG_NAME
constexpr Steinberg::FIDString kPinchFxVst3Name = "Debug PinchFX";
#else
constexpr Steinberg::FIDString kPinchFxVst3Name = "PinchFX";
#endif
constexpr Steinberg::FIDString kPinchFxVst3Vendor = "VirtualRobot";
constexpr Steinberg::FIDString kPinchFxVst3Version = "0.9.1 (" __DATE__ " " __TIME__ ")";
constexpr const char kPinchFxVst3Build[] = __DATE__ " " __TIME__;
constexpr Steinberg::FIDString kPinchFxVst3Url = "https://ableplugs.local/pinchfx";
constexpr Steinberg::FIDString kPinchFxVst3Email = "support@ableplugs.local";

enum ParamIDs : Steinberg::Vst::ParamID {
    kParamTrig = 0,
    kParamPosition,
    kParamSqueal,
    kParamLock,
    kParamGlide,
    kParamTone,
    kParamMix,
    kParamMonitor,
    kParamMode,
    kParamHeat,
    kParamSens,
    kParamGain1,
    kParamPosition2,
    kParamFeedback2,
    kParamLock2,
    kParamGain2,
    kParamPosition3,
    kParamFeedback3,
    kParamLock3,
    kParamGain3
};

} // namespace pinchfx

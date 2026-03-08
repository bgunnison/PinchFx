// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include "pluginterfaces/base/funknown.h"
#include "pluginterfaces/vst/vsttypes.h"

namespace pinchfx {

#ifdef PINCHFX_DEBUG_UIDS
// {B1D3856B-3BD6-4DC7-83BC-4D74AAAE75D5}
static const Steinberg::FUID kPinchFxProcessorUID(0xB1D3856B, 0x3BD64DC7, 0x83BC4D74, 0xAAAE75D5);
// {37F4E9FE-4DCE-4D32-A47B-6201BC78511C}
static const Steinberg::FUID kPinchFxControllerUID(0x37F4E9FE, 0x4DCE4D32, 0xA47B6201, 0xBC78511C);
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
    // 0 was kParamTrig (removed).
    kParamPosition = 1,
    kParamSqueal = 2,
    kParamLock = 3,
    kParamGlide = 4,
    kParamTone = 5,
    kParamMix = 6,
    // 7 was kParamMonitor (removed).
    // 8 was kParamMode (removed).
    kParamHeat = 9,
    kParamSens = 10,
    kParamGain1 = 11,
    kParamPosition2 = 12,
    kParamFeedback2 = 13,
    kParamLock2 = 14,
    kParamGain2 = 15,
    kParamPosition3 = 16,
    kParamFeedback3 = 17,
    kParamLock3 = 18,
    kParamGain3 = 19
};

} // namespace pinchfx

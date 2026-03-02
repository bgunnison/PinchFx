// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#include "PinchFxController.h"
#include "PinchFxPartials.h"

#include "base/source/fstreamer.h"
#include "pluginterfaces/base/ustring.h"
#include "pluginterfaces/vst/ivstparameterchanges.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace pinchfx {
using namespace Steinberg;
using namespace Steinberg::Vst;

namespace {

const char* monitorLabel(int index) {
    switch (index) {
        case 0: return "Mix";
        case 1: return "Input";
        case 2: return "Detect";
        case 3: return "Resonator";
        case 4: return "Squeal";
        case 5: return "Tone";
        case 6: return "Tube";
        case 7: return "Limiter";
        default: return "";
    }
}

bool equalsAscii(const TChar* text, const char* ascii) {
    if (!text || !ascii) return false;
    UString128 ustr(text);
    char buffer[128]{};
    ustr.toAscii(buffer, sizeof(buffer));
    return std::strcmp(buffer, ascii) == 0;
}

} // namespace

//------------------------------------------------------------------------

Steinberg::tresult PLUGIN_API PinchFxController::initialize(FUnknown* context) {
    tresult result = EditControllerEx1::initialize(context);
    if (result != kResultOk) return result;

    buildParamOrder();

    auto addParam = [&](const char* title, ParamID id, ParamValue def, int32 flags, int32 stepCount = 0) {
        UString128 utitle;
        UString128 uunits;
        utitle.fromAscii(title);
        uunits.fromAscii("");
        parameters.addParameter(utitle, uunits, stepCount, def, flags, id);
    };

    addParam("INPUT", kParamSens, defaultNormalized(kParamSens), ParameterInfo::kCanAutomate);
    addParam("TRACK DELAY", kParamGlide, defaultNormalized(kParamGlide), ParameterInfo::kCanAutomate);
    addParam("A PARTIAL", kParamPosition, defaultNormalized(kParamPosition), ParameterInfo::kCanAutomate, kPartialStepCount);
    addParam("A RES", kParamLock, defaultNormalized(kParamLock), ParameterInfo::kCanAutomate);
    addParam("A FEEDBACK", kParamSqueal, defaultNormalized(kParamSqueal), ParameterInfo::kCanAutomate);
    addParam("A GAIN", kParamGain1, defaultNormalized(kParamGain1), ParameterInfo::kCanAutomate);
    addParam("B PARTIAL", kParamPosition2, defaultNormalized(kParamPosition2), ParameterInfo::kCanAutomate, kPartialStepCount);
    addParam("B RES", kParamLock2, defaultNormalized(kParamLock2), ParameterInfo::kCanAutomate);
    addParam("B FEEDBACK", kParamFeedback2, defaultNormalized(kParamFeedback2), ParameterInfo::kCanAutomate);
    addParam("B GAIN", kParamGain2, defaultNormalized(kParamGain2), ParameterInfo::kCanAutomate);
    addParam("C PARTIAL", kParamPosition3, defaultNormalized(kParamPosition3), ParameterInfo::kCanAutomate, kPartialStepCount);
    addParam("C RES", kParamLock3, defaultNormalized(kParamLock3), ParameterInfo::kCanAutomate);
    addParam("C FEEDBACK", kParamFeedback3, defaultNormalized(kParamFeedback3), ParameterInfo::kCanAutomate);
    addParam("C GAIN", kParamGain3, defaultNormalized(kParamGain3), ParameterInfo::kCanAutomate);
    addParam("HEAT", kParamHeat, defaultNormalized(kParamHeat), ParameterInfo::kCanAutomate);
    addParam("TONE", kParamTone, defaultNormalized(kParamTone), ParameterInfo::kCanAutomate);
    addParam("WET/DRY", kParamMix, defaultNormalized(kParamMix), ParameterInfo::kCanAutomate);
    for (auto pid : paramOrder_) {
        paramState_[pid] = defaultNormalized(pid);
    }

    return kResultOk;
}

IPlugView* PLUGIN_API PinchFxController::createView(FIDString name) {
    if (name && std::strcmp(name, ViewType::kEditor) == 0) {
        return nullptr;
    }
    return nullptr;
}

Steinberg::tresult PLUGIN_API PinchFxController::setComponentState(IBStream* state) {
    if (!state) return kResultFalse;
    IBStreamer streamer(state, kLittleEndian);
    for (auto pid : paramOrder_) {
        double value = defaultNormalized(pid);
        if (!streamer.readDouble(value)) {
            value = defaultNormalized(pid);
        }
        setParamNormalized(pid, value);
    }
    return kResultOk;
}

Steinberg::tresult PLUGIN_API PinchFxController::getState(IBStream* state) {
    if (!state) return kResultFalse;
    IBStreamer streamer(state, kLittleEndian);
    for (auto pid : paramOrder_) {
        ParamValue value = defaultNormalized(pid);
        auto it = paramState_.find(pid);
        if (it != paramState_.end()) value = it->second;
        streamer.writeDouble(value);
    }
    return kResultOk;
}

Steinberg::tresult PLUGIN_API PinchFxController::setState(IBStream* state) {
    return setComponentState(state);
}

Steinberg::tresult PLUGIN_API PinchFxController::setParamNormalized(ParamID pid, ParamValue value) {
    paramState_[pid] = value;
    return EditControllerEx1::setParamNormalized(pid, value);
}

Steinberg::tresult PLUGIN_API PinchFxController::getParamStringByValue(ParamID pid, ParamValue valueNormalized, String128 string) {
    UString128 result(string);
    if (pid == kParamPosition || pid == kParamPosition2 || pid == kParamPosition3) {
        const int partial = pinchfx::partialFromNormalized(valueNormalized);
        char text[16]{};
        std::snprintf(text, sizeof(text), "%d", partial);
        result.fromAscii(text);
        return kResultOk;
    }
    if (pid == kParamMonitor) {
        int idx = static_cast<int>(std::round(valueNormalized * 7.0));
        result.fromAscii(monitorLabel(std::clamp(idx, 0, 7)));
        return kResultOk;
    }
    return EditControllerEx1::getParamStringByValue(pid, valueNormalized, string);
}

Steinberg::tresult PLUGIN_API PinchFxController::getParamValueByString(ParamID pid, TChar* string, ParamValue& valueNormalized) {
    if (pid == kParamPosition || pid == kParamPosition2 || pid == kParamPosition3) {
        UString128 ustr(string);
        char ascii[64]{};
        ustr.toAscii(ascii, sizeof(ascii));
        char* endPtr = nullptr;
        const long parsed = std::strtol(ascii, &endPtr, 10);
        if (endPtr != ascii) {
            for (int i = 0; i < kPartialChoiceCount; ++i) {
                if (kPartialChoices[static_cast<size_t>(i)] == parsed) {
                    valueNormalized = static_cast<ParamValue>(pinchfx::normalizedFromPartialIndex(i));
                    return kResultOk;
                }
            }
        }
    }
    if (pid == kParamMonitor) {
        for (int i = 0; i <= 7; ++i) {
            if (equalsAscii(string, monitorLabel(i))) {
                valueNormalized = static_cast<ParamValue>(i) / 7.0;
                return kResultOk;
            }
        }
    }
    return EditControllerEx1::getParamValueByString(pid, string, valueNormalized);
}

void PinchFxController::buildParamOrder() {
    paramOrder_.clear();
    paramOrder_.push_back(kParamTrig);
    paramOrder_.push_back(kParamPosition);
    paramOrder_.push_back(kParamSqueal); // FEEDBACK control (reuses legacy slot).
    paramOrder_.push_back(kParamLock);
    paramOrder_.push_back(kParamGlide); // Tracker time-constant control.
    paramOrder_.push_back(kParamTone);
    paramOrder_.push_back(kParamMix);
    paramOrder_.push_back(kParamMonitor);
    paramOrder_.push_back(kParamMode); // Hidden legacy slot for state compatibility.
    paramOrder_.push_back(kParamHeat);
    paramOrder_.push_back(kParamSens); // INPUT control.
    paramOrder_.push_back(kParamGain1);
    paramOrder_.push_back(kParamPosition2);
    paramOrder_.push_back(kParamFeedback2);
    paramOrder_.push_back(kParamLock2);
    paramOrder_.push_back(kParamGain2);
    paramOrder_.push_back(kParamPosition3);
    paramOrder_.push_back(kParamFeedback3);
    paramOrder_.push_back(kParamLock3);
    paramOrder_.push_back(kParamGain3);
}

ParamValue PinchFxController::defaultNormalized(ParamID pid) const {
    switch (pid) {
        case kParamTrig: return 0.0;
        case kParamPosition: return 0.5;
        case kParamSqueal: return 0.0;
        case kParamLock: return 0.1111111111111111;
        case kParamGlide: return 0.25;
        case kParamTone: return 1.0;
        case kParamMix: return 0.35;
        case kParamMonitor: return 0.0;
        case kParamMode: return 0.0;
        case kParamHeat: return 0.0;
        case kParamSens: return 0.5;
        case kParamGain1: return 1.0;
        case kParamPosition2: return 0.5;
        case kParamFeedback2: return 0.0;
        case kParamLock2: return 0.1111111111111111;
        case kParamGain2: return 0.0;
        case kParamPosition3: return 0.5;
        case kParamFeedback3: return 0.0;
        case kParamLock3: return 0.1111111111111111;
        case kParamGain3: return 0.0;
        default: break;
    }
    return 0.0;
}

} // namespace pinchfx

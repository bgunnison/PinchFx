// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#include "PinchFxController.h"
#include "PinchFxDefaults.h"
#include "PinchFxPartials.h"

#include "base/source/fstreamer.h"
#include "pluginterfaces/base/ustring.h"
#include "pluginterfaces/vst/ivstparameterchanges.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace pinchfx {
using namespace Steinberg;
using namespace Steinberg::Vst;

namespace {
constexpr size_t kLegacyStateValueCount = 20;
}

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

    std::vector<double> values{};
    values.reserve(kLegacyStateValueCount);
    double value = 0.0;
    while (streamer.readDouble(value)) values.push_back(value);

    auto loadParam = [&](size_t index, ParamID pid) {
        const double loaded = (index < values.size()) ? values[index] : defaultNormalized(pid);
        setParamNormalized(pid, loaded);
    };

    if (values.size() >= kLegacyStateValueCount) {
        // Legacy state layout includes removed TRIGGER (index 0), MONITOR (index 7), and MODE (index 8).
        loadParam(1, kParamPosition);
        loadParam(2, kParamSqueal);
        loadParam(3, kParamLock);
        loadParam(4, kParamGlide);
        loadParam(5, kParamTone);
        loadParam(6, kParamMix);
        loadParam(9, kParamHeat);
        loadParam(10, kParamSens);
        loadParam(11, kParamGain1);
        loadParam(12, kParamPosition2);
        loadParam(13, kParamFeedback2);
        loadParam(14, kParamLock2);
        loadParam(15, kParamGain2);
        loadParam(16, kParamPosition3);
        loadParam(17, kParamFeedback3);
        loadParam(18, kParamLock3);
        loadParam(19, kParamGain3);
    } else {
        // Current compact layout (TRIGGER/MONITOR/MODE removed).
        loadParam(0, kParamPosition);
        loadParam(1, kParamSqueal);
        loadParam(2, kParamLock);
        loadParam(3, kParamGlide);
        loadParam(4, kParamTone);
        loadParam(5, kParamMix);
        loadParam(6, kParamHeat);
        loadParam(7, kParamSens);
        loadParam(8, kParamGain1);
        loadParam(9, kParamPosition2);
        loadParam(10, kParamFeedback2);
        loadParam(11, kParamLock2);
        loadParam(12, kParamGain2);
        loadParam(13, kParamPosition3);
        loadParam(14, kParamFeedback3);
        loadParam(15, kParamLock3);
        loadParam(16, kParamGain3);
    }
    return kResultOk;
}

Steinberg::tresult PLUGIN_API PinchFxController::getState(IBStream* state) {
    if (!state) return kResultFalse;
    IBStreamer streamer(state, kLittleEndian);

    auto getParam = [&](ParamID pid) -> double {
        double current = defaultNormalized(pid);
        auto it = paramState_.find(pid);
        if (it != paramState_.end()) current = it->second;
        return current;
    };

    // Write legacy-compatible layout so older builds can still parse state.
    streamer.writeDouble(0.0); // Removed TRIGGER slot.
    streamer.writeDouble(getParam(kParamPosition));
    streamer.writeDouble(getParam(kParamSqueal));
    streamer.writeDouble(getParam(kParamLock));
    streamer.writeDouble(getParam(kParamGlide));
    streamer.writeDouble(getParam(kParamTone));
    streamer.writeDouble(getParam(kParamMix));
    streamer.writeDouble(0.0); // Removed MONITOR slot.
    streamer.writeDouble(0.0); // Removed MODE slot.
    streamer.writeDouble(getParam(kParamHeat));
    streamer.writeDouble(getParam(kParamSens));
    streamer.writeDouble(getParam(kParamGain1));
    streamer.writeDouble(getParam(kParamPosition2));
    streamer.writeDouble(getParam(kParamFeedback2));
    streamer.writeDouble(getParam(kParamLock2));
    streamer.writeDouble(getParam(kParamGain2));
    streamer.writeDouble(getParam(kParamPosition3));
    streamer.writeDouble(getParam(kParamFeedback3));
    streamer.writeDouble(getParam(kParamLock3));
    streamer.writeDouble(getParam(kParamGain3));

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
    return EditControllerEx1::getParamValueByString(pid, string, valueNormalized);
}

void PinchFxController::buildParamOrder() {
    paramOrder_.clear();
    paramOrder_.push_back(kParamPosition);
    paramOrder_.push_back(kParamSqueal); // FEEDBACK control (reuses legacy slot).
    paramOrder_.push_back(kParamLock);
    paramOrder_.push_back(kParamGlide); // Tracker time-constant control.
    paramOrder_.push_back(kParamTone);
    paramOrder_.push_back(kParamMix);
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
        case kParamPosition: return kDefaultPartialA;
        case kParamSqueal: return kDefaultFeedbackA;
        case kParamLock: return kDefaultResonanceA;
        case kParamGlide: return kDefaultTrackDelay;
        case kParamTone: return kDefaultTone;
        case kParamMix: return kDefaultWetDry;
        case kParamHeat: return kDefaultHeat;
        case kParamSens: return kDefaultInput;
        case kParamGain1: return kDefaultGainA;
        case kParamPosition2: return kDefaultPartialB;
        case kParamFeedback2: return kDefaultFeedbackB;
        case kParamLock2: return kDefaultResonanceB;
        case kParamGain2: return kDefaultGainB;
        case kParamPosition3: return kDefaultPartialC;
        case kParamFeedback3: return kDefaultFeedbackC;
        case kParamLock3: return kDefaultResonanceC;
        case kParamGain3: return kDefaultGainC;
        default: break;
    }
    return 0.0;
}

} // namespace pinchfx

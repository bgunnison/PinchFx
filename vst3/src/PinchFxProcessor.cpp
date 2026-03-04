// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#include "PinchFxProcessor.h"

#include "PinchFxController.h"
#include "PinchFxDefaults.h"

#include "base/source/fstreamer.h"
#include "pluginterfaces/vst/ivstparameterchanges.h"

#include <algorithm>
#include <cmath>
#include <vector>
#ifdef PINCHFX_DEBUG_NAME
#include <fstream>
#endif

namespace pinchfx {
using namespace Steinberg;
using namespace Steinberg::Vst;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWetMakeup = 1.0; // Wet level now auto-matched in DSP via output AGC.
constexpr size_t kLegacyStateValueCount = 20;

void writeOutputSilence(ProcessData& data) {
    for (int32 bus = 0; bus < data.numOutputs; ++bus) {
        auto& out = data.outputs[bus];
        out.silenceFlags = 0;
        if (out.channelBuffers32 && data.symbolicSampleSize == kSample32) {
            for (uint32 c = 0; c < out.numChannels; ++c) {
                std::fill_n(out.channelBuffers32[c], data.numSamples, 0.0f);
            }
        }
        if (out.channelBuffers64 && data.symbolicSampleSize == kSample64) {
            for (uint32 c = 0; c < out.numChannels; ++c) {
                std::fill_n(out.channelBuffers64[c], data.numSamples, 0.0);
            }
        }
    }
}

} // namespace

PinchFxProcessor::PinchFxProcessor() {
    setControllerClass(kPinchFxControllerUID);
    setProcessing(true);
    buildParamOrder();
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::initialize(FUnknown* context) {
    tresult result = AudioEffect::initialize(context);
    if (result != kResultOk) return result;

    addAudioInput(STR16("Main In"), SpeakerArr::kStereo);
    addAudioOutput(STR16("Main Out"), SpeakerArr::kStereo);

    return kResultOk;
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::terminate() {
    return AudioEffect::terminate();
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::setBusArrangements(SpeakerArrangement* inputs, int32 numIns,
                                                                   SpeakerArrangement* outputs, int32 numOuts) {
    if (numIns != 1) return kResultFalse;
    if (numOuts != 1) return kResultFalse;

    if (inputs[0] != SpeakerArr::kMono && inputs[0] != SpeakerArr::kStereo) return kResultFalse;
    if (outputs[0] != inputs[0]) return kResultFalse;

    return AudioEffect::setBusArrangements(inputs, numIns, outputs, numOuts);
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::setupProcessing(ProcessSetup& setup) {
    sampleRate_ = setup.sampleRate > 0.0 ? setup.sampleRate : 44100.0;
    maxBlockSize_ = setup.maxSamplesPerBlock;

    algorithm_.prepare(sampleRate_, maxBlockSize_);
    syncAlgorithm();

    return AudioEffect::setupProcessing(setup);
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::setActive(TBool state) {
    if (state) {
        algorithm_.reset();
    }
    return AudioEffect::setActive(state);
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::canProcessSampleSize(int32 symbolicSampleSize) {
    if (symbolicSampleSize == kSample32 || symbolicSampleSize == kSample64) return kResultOk;
    return kResultFalse;
}

void PinchFxProcessor::buildParamOrder() {
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

    paramState_.clear();
    for (auto pid : paramOrder_) {
        paramState_[pid] = defaultNormalized(pid);
    }
}

ParamValue PinchFxProcessor::defaultNormalized(ParamID pid) const {
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

void PinchFxProcessor::resetToDefaults() {
    for (auto pid : paramOrder_) {
        applyNormalizedParam(pid, defaultNormalized(pid));
    }
}

void PinchFxProcessor::handleParameterChanges(ProcessData& data) {
    if (!data.inputParameterChanges) return;
    const int32 count = data.inputParameterChanges->getParameterCount();
    for (int32 i = 0; i < count; ++i) {
        IParamValueQueue* queue = data.inputParameterChanges->getParameterData(i);
        if (!queue) continue;
        const ParamID pid = queue->getParameterId();
        const int32 points = queue->getPointCount();
        if (points <= 0) continue;
        ParamValue value = 0.0;
        int32 sampleOffset = 0;
        queue->getPoint(points - 1, sampleOffset, value);
        applyNormalizedParam(pid, value);
    }
}

void PinchFxProcessor::applyNormalizedParam(ParamID pid, ParamValue value) {
    paramState_[pid] = value;
    switch (pid) {
        case kParamPosition: params_.position = value; break;
        case kParamSqueal: params_.feedback = value; break;
        case kParamGain1: params_.gain1 = value; break;
        case kParamPosition2: params_.position2 = value; break;
        case kParamFeedback2: params_.feedback2 = value; break;
        case kParamLock2: params_.lock2 = value; break;
        case kParamGain2: params_.gain2 = value; break;
        case kParamPosition3: params_.position3 = value; break;
        case kParamFeedback3: params_.feedback3 = value; break;
        case kParamLock3: params_.lock3 = value; break;
        case kParamGain3: params_.gain3 = value; break;
        case kParamLock: params_.lock = value; break;
        case kParamGlide: params_.glide = value; break;
        case kParamTone: params_.tone = value; break;
        case kParamMix: params_.mix = value; break;
        case kParamHeat: params_.heat = value; break;
        case kParamSens: params_.sens = value; break;
        default: break;
    }
}

void PinchFxProcessor::syncAlgorithm() {
    algorithm_.setParams(params_);
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::process(ProcessData& data) {
    if (data.numOutputs == 0 || !data.outputs) return kResultOk;

    for (int32 bus = 0; bus < data.numOutputs; ++bus) {
        data.outputs[bus].silenceFlags = 0;
    }

    handleParameterChanges(data);
    syncAlgorithm();


    if (data.numSamples <= 0) return kResultOk;

    auto* mainIn = data.numInputs > 0 ? &data.inputs[0] : nullptr;

    if (!mainIn || mainIn->numChannels == 0) {
        writeOutputSilence(data);
        return kResultOk;
    }

    auto& outBus = data.outputs[0];
    if (outBus.numChannels != mainIn->numChannels) {
        writeOutputSilence(data);
        return kResultOk;
    }

    const int32 numChannels = static_cast<int32>(outBus.numChannels);

    if (data.symbolicSampleSize == kSample32) {
        processAudio<float>(data,
                            mainIn->channelBuffers32,
                            outBus.channelBuffers32,
                            numChannels,
                            data.numSamples);
    } else if (data.symbolicSampleSize == kSample64) {
        processAudio<double>(data,
                             mainIn->channelBuffers64,
                             outBus.channelBuffers64,
                             numChannels,
                             data.numSamples);
    }

    return kResultOk;
}

template <typename SampleType>
void PinchFxProcessor::processAudio(ProcessData& data, SampleType** in, SampleType** out,
                                    int32_t numChannels, int32_t numSamples) {
    const double mix = std::clamp(params_.mix, 0.0, 1.0);
    const double mixShaped = std::sqrt(mix); // Bias knob response toward wet so mid settings are more audible.
    const double dryMix = std::cos(0.5 * kPi * mixShaped);
    const double wetMix = std::sin(0.5 * kPi * mixShaped);
    blockCounter_ += 1;
#ifdef PINCHFX_DEBUG_NAME
    int nonFiniteCount = 0;
    double lastNonFinite = 0.0;
#endif

    for (int32_t i = 0; i < numSamples; ++i) {
        const double inL = static_cast<double>(in[0][i]);
        const double inR = (numChannels > 1) ? static_cast<double>(in[1][i]) : inL;
        const double monoIn = 0.5 * (inL + inR);

        const auto algOut = algorithm_.processSample(monoIn);

        const double wet = kWetMakeup * algOut.xlim;
        const double outL = dryMix * inL + wetMix * wet;
        const double outR = dryMix * inR + wetMix * wet;

#ifdef PINCHFX_DEBUG_NAME
        if (!std::isfinite(algOut.xlim) || !std::isfinite(outL) || !std::isfinite(outR)) {
            nonFiniteCount += 1;
            lastNonFinite = algOut.xlim;
        }
#endif

        out[0][i] = static_cast<SampleType>(outL);
        if (numChannels > 1) {
            out[1][i] = static_cast<SampleType>(outR);
        }
    }

#ifdef PINCHFX_DEBUG_NAME
    if (nonFiniteCount > 0) {
        std::ofstream log("C:\\projects\\ableplugs\\pinchfx\\pinchfx_nan.log", std::ios::app);
        log << "[PinchFX] block=" << blockCounter_
            << " nonfinite=" << nonFiniteCount
            << " lastXlim=" << lastNonFinite
            << " sr=" << sampleRate_
            << " mix=" << mix
            << std::endl;
    }
#endif
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::setState(IBStream* state) {
    if (!state) return kResultFalse;
    IBStreamer streamer(state, kLittleEndian);
    std::vector<double> values{};
    values.reserve(kLegacyStateValueCount);
    double v = 0.0;
    while (streamer.readDouble(v)) values.push_back(v);

    auto loadParam = [&](size_t index, ParamID pid) {
        const double loaded = (index < values.size()) ? values[index] : defaultNormalized(pid);
        applyNormalizedParam(pid, loaded);
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
    syncAlgorithm();
    return kResultOk;
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::getState(IBStream* state) {
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

} // namespace pinchfx

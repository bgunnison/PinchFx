// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#include "PinchFxProcessor.h"

#include "PinchFxController.h"

#include "base/source/fstreamer.h"
#include "pluginterfaces/vst/ivstparameterchanges.h"

#include <algorithm>
#include <cmath>
#ifdef PINCHFX_DEBUG_NAME
#include <fstream>
#endif

namespace pinchfx {
using namespace Steinberg;
using namespace Steinberg::Vst;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWetMakeup = 1.8; // Compensate lower wet-path level so WET/DRY balance is usable.

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
        lastTrigValue_ = 0.0;
    }
    return AudioEffect::setActive(state);
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::canProcessSampleSize(int32 symbolicSampleSize) {
    if (symbolicSampleSize == kSample32 || symbolicSampleSize == kSample64) return kResultOk;
    return kResultFalse;
}

void PinchFxProcessor::buildParamOrder() {
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
    paramOrder_.push_back(kParamSens); // Hidden legacy slot for state compatibility.

    paramState_.clear();
    for (auto pid : paramOrder_) {
        paramState_[pid] = defaultNormalized(pid);
    }
}

ParamValue PinchFxProcessor::defaultNormalized(ParamID pid) const {
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
        ParamValue value = 0.0;
        int32 sampleOffset = 0;
    if (pid == kParamTrig) {
        bool trigNow = false;
        for (int32 p = 0; p < points; ++p) {
            ParamValue v = 0.0;
            int32 o = 0;
            queue->getPoint(p, o, v);
            if (v > 0.5 && lastTrigValue_ <= 0.5) {
                trigNow = true;
                break;
            }
        }
        queue->getPoint(points - 1, sampleOffset, value);
        applyNormalizedParam(pid, value);
        if (trigNow) {
            algorithm_.triggerManual();
        }
        lastTrigValue_ = value;
        } else {
            queue->getPoint(points - 1, sampleOffset, value);
            applyNormalizedParam(pid, value);
        }
    }
}

void PinchFxProcessor::applyNormalizedParam(ParamID pid, ParamValue value) {
    paramState_[pid] = value;
    switch (pid) {
        case kParamTrig: params_.trig = value; break;
        case kParamPosition: params_.position = value; break;
        case kParamSqueal: params_.feedback = value; break;
        case kParamLock: params_.lock = value; break;
        case kParamGlide: params_.glide = value; break;
        case kParamTone: params_.tone = value; break;
        case kParamMix: params_.mix = value; break;
        case kParamMonitor: params_.monitor = value; break;
        case kParamMode: break; // Legacy no-op.
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
    const int monitor = std::clamp(static_cast<int>(std::round(params_.monitor * 7.0)), 0, 7);
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
        const double xbpMon = algOut.xbp;

        double outL = 0.0;
        double outR = 0.0;
        if (monitor == 0) {
            const double wet = kWetMakeup * algOut.xlim;
            outL = dryMix * inL + wetMix * wet;
            outR = dryMix * inR + wetMix * wet;
        } else {
            double tap = 0.0;
            switch (monitor) {
                case 1: tap = monoIn; break;
                case 2: tap = monoIn; break;
                case 3: tap = xbpMon; break;
                case 4: tap = algOut.xr; break;
                case 5: tap = algOut.xtone; break;
                case 6: tap = algOut.xtube; break;
                case 7: tap = algOut.xlim; break;
                default: tap = algOut.xlim; break;
            }
            outL = tap;
            outR = tap;
        }

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
            << " monitor=" << monitor
            << std::endl;
    }
#endif
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::setState(IBStream* state) {
    if (!state) return kResultFalse;
    IBStreamer streamer(state, kLittleEndian);
    for (auto pid : paramOrder_) {
        double v = defaultNormalized(pid);
        if (!streamer.readDouble(v)) v = defaultNormalized(pid);
        applyNormalizedParam(pid, v);
    }
    syncAlgorithm();
    return kResultOk;
}

Steinberg::tresult PLUGIN_API PinchFxProcessor::getState(IBStream* state) {
    if (!state) return kResultFalse;
    IBStreamer streamer(state, kLittleEndian);
    for (auto pid : paramOrder_) {
        double v = defaultNormalized(pid);
        auto it = paramState_.find(pid);
        if (it != paramState_.end()) v = it->second;
        streamer.writeDouble(v);
    }
    return kResultOk;
}

} // namespace pinchfx

// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include "PinchFxIDs.h"
#include "dsp/PinchFxAlgorithm.h"

#include "public.sdk/source/vst/vstaudioeffect.h"

#include <array>
#include <unordered_map>
#include <vector>

namespace pinchfx {

class PinchFxProcessor : public Steinberg::Vst::AudioEffect {
public:
    PinchFxProcessor();

    static Steinberg::FUnknown* createInstance(void*) {
        return static_cast<Steinberg::Vst::IAudioProcessor*>(new PinchFxProcessor());
    }

    Steinberg::tresult PLUGIN_API initialize(Steinberg::FUnknown* context) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API terminate() SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API setState(Steinberg::IBStream* state) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API getState(Steinberg::IBStream* state) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API setBusArrangements(Steinberg::Vst::SpeakerArrangement* inputs, Steinberg::int32 numIns,
                                                     Steinberg::Vst::SpeakerArrangement* outputs, Steinberg::int32 numOuts) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API process(Steinberg::Vst::ProcessData& data) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API setupProcessing(Steinberg::Vst::ProcessSetup& setup) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API setActive(Steinberg::TBool state) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API canProcessSampleSize(Steinberg::int32 symbolicSampleSize) SMTG_OVERRIDE;

private:
    using Params = dsp::PinchFxAlgorithm::Params;

    void handleParameterChanges(Steinberg::Vst::ProcessData& data);
    void applyNormalizedParam(Steinberg::Vst::ParamID pid, Steinberg::Vst::ParamValue value);
    void buildParamOrder();
    void resetToDefaults();
    Steinberg::Vst::ParamValue defaultNormalized(Steinberg::Vst::ParamID pid) const;

    template <typename SampleType>
    void processAudio(Steinberg::Vst::ProcessData& data, SampleType** in, SampleType** out, int32_t numChannels, int32_t numSamples);

    void syncAlgorithm();

    Params params_{};
    std::vector<Steinberg::Vst::ParamID> paramOrder_{};
    std::unordered_map<Steinberg::Vst::ParamID, double> paramState_{};

    dsp::PinchFxAlgorithm algorithm_{};

    double sampleRate_{44100.0};
    int32_t maxBlockSize_{0};

    int64_t blockCounter_{0};

};

} // namespace pinchfx

// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include "PinchFxIDs.h"

#include "public.sdk/source/vst/vsteditcontroller.h"

#include <unordered_map>
#include <vector>

namespace pinchfx {

class PinchFxController : public Steinberg::Vst::EditControllerEx1 {
public:
    PinchFxController() = default;
    static Steinberg::FUnknown* createInstance(void*) {
        return static_cast<Steinberg::Vst::IEditController*>(new PinchFxController());
    }

    Steinberg::tresult PLUGIN_API initialize(Steinberg::FUnknown* context) SMTG_OVERRIDE;
    Steinberg::IPlugView* PLUGIN_API createView(Steinberg::FIDString name) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API setComponentState(Steinberg::IBStream* state) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API getState(Steinberg::IBStream* state) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API setState(Steinberg::IBStream* state) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API setParamNormalized(Steinberg::Vst::ParamID pid, Steinberg::Vst::ParamValue value) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API getParamStringByValue(Steinberg::Vst::ParamID pid,
                                                        Steinberg::Vst::ParamValue valueNormalized,
                                                        Steinberg::Vst::String128 string) SMTG_OVERRIDE;
    Steinberg::tresult PLUGIN_API getParamValueByString(Steinberg::Vst::ParamID pid,
                                                        Steinberg::Vst::TChar* string,
                                                        Steinberg::Vst::ParamValue& valueNormalized) SMTG_OVERRIDE;

private:
    void buildParamOrder();
    Steinberg::Vst::ParamValue defaultNormalized(Steinberg::Vst::ParamID pid) const;

    std::vector<Steinberg::Vst::ParamID> paramOrder_{};
    std::unordered_map<Steinberg::Vst::ParamID, Steinberg::Vst::ParamValue> paramState_{};
};

} // namespace pinchfx

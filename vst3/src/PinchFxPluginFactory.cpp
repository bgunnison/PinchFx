// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#include "PinchFxController.h"
#include "PinchFxIDs.h"
#include "PinchFxProcessor.h"

#include "public.sdk/source/main/pluginfactory.h"

#ifdef PINCHFX_DEBUG_NAME
#define stringPluginName "Debug PinchFX"
#else
#define stringPluginName "PinchFX"
#endif

using namespace pinchfx;
using namespace Steinberg;
using namespace Steinberg::Vst;

BEGIN_FACTORY_DEF(kPinchFxVst3Vendor, kPinchFxVst3Url, kPinchFxVst3Email)

    DEF_CLASS2(INLINE_UID_FROM_FUID(kPinchFxProcessorUID),
               PClassInfo::kManyInstances,
               kVstAudioEffectClass,
               stringPluginName,
               Vst::kDistributable,
               PlugType::kFx,
               kPinchFxVst3Version,
               kVstVersionString,
               PinchFxProcessor::createInstance)

    DEF_CLASS2(INLINE_UID_FROM_FUID(kPinchFxControllerUID),
               PClassInfo::kManyInstances,
               kVstComponentControllerClass,
               stringPluginName " Controller",
               0,
               "",
               kPinchFxVst3Version,
               kVstVersionString,
               PinchFxController::createInstance)

END_FACTORY

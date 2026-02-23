// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#include "SimIO.h"
#include "PinchFxSimProcessor.h"
#include "config.h"

//#define SAMPLEWAV "C:\\projects\\ableplugs\\pinchfx\\sim\\pluck.wav"
//#define SAMPLE_WAV "C:\\projects\\ableplugs\\pinchfx\\sim\\cleanchords.wav"
#define SAMPLE_WAV "C:\\projects\\ableplugs\\pinchfx\\sim\\elecnotesup.wav"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#include <commctrl.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using pinchfx::sim::AudioBuffer;
using pinchfx::sim::PinchFxSimProcessor;

namespace {

struct SharedParams {
    std::atomic<float> inputGain{0.5f};
    std::atomic<float> position{0.5f};
    std::atomic<float> lock{0.1111111111111111f};
    std::atomic<float> glide{0.25f};
    std::atomic<float> tone{1.0f};
    std::atomic<float> mix{0.35f};
    std::atomic<float> heat{0.0f};
    std::atomic<bool> mute{false};
#if PINCHFX_SIM_ENABLE_SCOPE
    std::atomic<float> scopeTime{0.5f}; // 0..1 mapped to 0.25x..4x
    std::atomic<float> scopeLevel{0.5f}; // 0..1 mapped to 0.25x..4x
#endif
};

#if PINCHFX_SIM_ENABLE_SCOPE
enum class ScopeTap : int {
    Input = 0,
    Agc,
    F0,
    ResonatorFreq,
    Resonator,
    Tone,
    Tube,
    Limiter,
    Output
};
#endif

struct SliderSpec {
    const char* label;
    int id;
    int y;
    int minPos;
    int maxPos;
    float defaultValue;
};

constexpr int kSliderWidth = 260;
constexpr int kSliderHeight = 24;
constexpr int kLabelWidth = 110;
constexpr int kLeftMargin = 16;
constexpr int kTopMargin = 16;
constexpr int kRowHeight = 36;
#if PINCHFX_SIM_ENABLE_SCOPE
constexpr int kScopeTopBar = 64;
#endif

constexpr int kIdPosition = 1001;
constexpr int kIdLock = 1003;
constexpr int kIdTrack = 1004;
constexpr int kIdTone = 1005;
constexpr int kIdMix = 1006;
constexpr int kIdHeat = 1008;
constexpr int kIdInputGain = 1011;
constexpr int kIdEnable = 1012;
#if PINCHFX_SIM_ENABLE_SCOPE
constexpr int kIdScopeTap = 1013;
constexpr int kIdScopeTime = 1015;
constexpr int kIdScopeFreeze = 1016;
constexpr int kIdScopeLevel = 1017;
#endif

constexpr float kResDefault = 0.1111111111111111f;
constexpr float kQMin = 0.5f;
constexpr float kQMax = 5.0f;
constexpr float kQRange = kQMax - kQMin;
constexpr float kInputGainBoost = 10.0f;

constexpr SliderSpec kSliders[] = {
    {"INPUT", kIdInputGain, kTopMargin + 0 * kRowHeight, 0, 1000, 0.5f},
    {"PARTIAL", kIdPosition, kTopMargin + 1 * kRowHeight, 0, 1000, 0.5f},
    {"RES", kIdLock, kTopMargin + 2 * kRowHeight, 0, 1000, kResDefault},
    {"TRACK", kIdTrack, kTopMargin + 3 * kRowHeight, 0, 1000, 0.25f},
    {"HEAT", kIdHeat, kTopMargin + 4 * kRowHeight, 0, 1000, 0.0f},
    {"TONE", kIdTone, kTopMargin + 5 * kRowHeight, 0, 1000, 1.0f},
    {"WET/DRY", kIdMix, kTopMargin + 6 * kRowHeight, 0, 1000, 0.35f},
};

struct ScopeBuffer {
    std::unique_ptr<std::atomic<float>[]> samples;
    int size{0};
    std::atomic<int> writeIndex{0};
};

struct AudioState {
    WAVEFORMATEX format{};
    HWAVEOUT waveOut{};
    HANDLE eventHandle{};
    std::vector<WAVEHDR> headers;
    std::vector<std::vector<int16_t>> buffers;
};

std::atomic<bool> gRunning{true};
SharedParams gParams{};
AudioBuffer gInput{};
size_t gReadIndex = 0;
#if PINCHFX_SIM_ENABLE_SCOPE
std::atomic<int> gScopeTap{static_cast<int>(ScopeTap::Output)};
ScopeBuffer gScope{};
HWND gScopeCombo = nullptr;
HWND gScopeHwnd = nullptr;
HWND gScopeTimeSlider = nullptr;
HWND gScopeFreezeButton = nullptr;
HWND gScopeLevelSlider = nullptr;
std::atomic<bool> gScopeFrozen{false};
#endif
HWND gLockSlider = nullptr;
HWND gResLabel = nullptr;
HWND gTrackLabel = nullptr;
HWND gPositionLabel = nullptr;
std::atomic<float> gPitchF0{0.0f};
std::atomic<float> gPitchFh{0.0f};
std::atomic<float> gPitchConf{0.0f};

float sliderPosToNorm(int pos) {
    return static_cast<float>(pos) / 1000.0f;
}

int normToSliderPos(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return static_cast<int>(v * 1000.0f);
}

void setSliderValue(HWND slider, int id, float value) {
    SendMessage(slider, TBM_SETPOS, TRUE, normToSliderPos(value));
}

void updateResLabel(float resValue) {
    if (!gResLabel) return;
    const float qValue = kQMin + kQRange * std::min(1.0f, std::max(0.0f, resValue));
    char text[32]{};
    std::snprintf(text, sizeof(text), "RES %.2f", qValue);
    SetWindowText(gResLabel, text);
}

void updateTrackLabel() {
    if (!gTrackLabel) return;
    const float resonatorHz = gPitchFh.load();
    char text[48]{};
    if (std::isfinite(resonatorHz) && resonatorHz > 0.0f) {
        std::snprintf(text, sizeof(text), "TRACK: %.0f", resonatorHz);
    } else {
        std::snprintf(text, sizeof(text), "TRACK: --");
    }
    SetWindowText(gTrackLabel, text);
}

void updatePositionLabel(float positionValue) {
    if (!gPositionLabel) return;
    static constexpr int kPartials[6] = {2, 5, 7, 9, 12, 15};
    const float clamped = std::min(1.0f, std::max(0.0f, positionValue));
    const int index = static_cast<int>(std::round(clamped * 5.0f));
    const int partial = kPartials[std::clamp(index, 0, 5)];
    char text[48]{};
    std::snprintf(text, sizeof(text), "PARTIAL: %d", partial);
    SetWindowText(gPositionLabel, text);
}

void updateParamsFromSlider(int id, int pos) {
    const float norm = sliderPosToNorm(pos);
    switch (id) {
        case kIdInputGain: gParams.inputGain.store(norm); break;
        case kIdPosition:
            gParams.position.store(norm);
            updatePositionLabel(norm);
            break;
        case kIdLock: {
            gParams.lock.store(norm);
            updateResLabel(norm);
            break;
        }
        case kIdTrack: gParams.glide.store(norm); break;
        case kIdTone: gParams.tone.store(norm); break;
        case kIdMix: gParams.mix.store(norm); break;
        case kIdHeat: gParams.heat.store(norm); break;
#if PINCHFX_SIM_ENABLE_SCOPE
        case kIdScopeTime: gParams.scopeTime.store(norm); break;
        case kIdScopeLevel: gParams.scopeLevel.store(norm); break;
#endif
        default: break;
    }
}

#if PINCHFX_SIM_ENABLE_SCOPE
const char* scopeTapLabel(ScopeTap tap) {
    switch (tap) {
        case ScopeTap::Input: return "Input";
        case ScopeTap::Agc: return "AGC";
        case ScopeTap::F0: return "F0 Hz";
        case ScopeTap::ResonatorFreq: return "Res Freq";
        case ScopeTap::Resonator: return "Resonator";
        case ScopeTap::Tone: return "Tone";
        case ScopeTap::Tube: return "Tube";
        case ScopeTap::Limiter: return "Limiter";
        case ScopeTap::Output: return "Output";
        default: return "Output";
    }
}
#endif

bool initAudioOutput(AudioState& audio, int sampleRate, int channels, int framesPerBuffer) {
    audio.format.wFormatTag = WAVE_FORMAT_PCM;
    audio.format.nChannels = static_cast<WORD>(channels);
    audio.format.nSamplesPerSec = static_cast<DWORD>(sampleRate);
    audio.format.wBitsPerSample = 16;
    audio.format.nBlockAlign = static_cast<WORD>(channels * (audio.format.wBitsPerSample / 8));
    audio.format.nAvgBytesPerSec = audio.format.nSamplesPerSec * audio.format.nBlockAlign;

    audio.eventHandle = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (!audio.eventHandle) return false;

    if (waveOutOpen(&audio.waveOut, WAVE_MAPPER, &audio.format, reinterpret_cast<DWORD_PTR>(audio.eventHandle), 0, CALLBACK_EVENT) != MMSYSERR_NOERROR) {
        return false;
    }

    // Use larger buffering so Debug builds (and the pitch tracker) don't underrun and create audible gaps.
    const int bufferCount = 8;
    audio.headers.resize(bufferCount);
    audio.buffers.resize(bufferCount);
    for (int i = 0; i < bufferCount; ++i) {
        audio.buffers[i].resize(static_cast<size_t>(framesPerBuffer * channels));
        WAVEHDR& hdr = audio.headers[i];
        std::memset(&hdr, 0, sizeof(WAVEHDR));
        hdr.lpData = reinterpret_cast<LPSTR>(audio.buffers[i].data());
        hdr.dwBufferLength = static_cast<DWORD>(audio.buffers[i].size() * sizeof(int16_t));
        waveOutPrepareHeader(audio.waveOut, &hdr, sizeof(WAVEHDR));
    }

    return true;
}

void shutdownAudio(AudioState& audio) {
    if (audio.waveOut) {
        waveOutReset(audio.waveOut);
        for (auto& hdr : audio.headers) {
            waveOutUnprepareHeader(audio.waveOut, &hdr, sizeof(WAVEHDR));
        }
        waveOutClose(audio.waveOut);
        audio.waveOut = nullptr;
    }
    if (audio.eventHandle) {
        CloseHandle(audio.eventHandle);
        audio.eventHandle = nullptr;
    }
}

void fillBuffer(std::vector<int16_t>& buffer, PinchFxSimProcessor& processor, int frames, int channels) {
    PinchFxSimProcessor::Params params{};
    params.position = gParams.position.load();
    params.lock = gParams.lock.load();
    params.glide = gParams.glide.load();
    params.tone = gParams.tone.load();
    params.mix = gParams.mix.load();
    params.heat = gParams.heat.load();
    params.sens = gParams.inputGain.load(); // INPUT also drives AGC excitation trim in shared DSP.
    const bool mute = gParams.mute.load();
    processor.setParams(params);

    const int totalFrames = static_cast<int>(gInput.samples.size() / gInput.channels);

#if PINCHFX_SIM_ENABLE_SCOPE
    const ScopeTap tap = static_cast<ScopeTap>(gScopeTap.load());
#endif
    for (int i = 0; i < frames; ++i) {
        const int readFrame = static_cast<int>(gReadIndex);
        const int readBase = readFrame * gInput.channels;
        const float gain = gParams.inputGain.load() * kInputGainBoost;
        const float inL = gInput.samples[readBase] * gain;
        const float inR = (gInput.channels > 1) ? gInput.samples[readBase + 1] * gain : inL;

        float tempIn[2]{inL, inR};
        float tempOut[2]{};
        processor.processBlock(tempIn, tempOut, channels, 1);

        float outL = tempOut[0];
        float outR = (channels > 1) ? tempOut[1] : outL;
        if (mute) {
            outL = 0.0f;
            outR = 0.0f;
        }

        gPitchF0.store(static_cast<float>(processor.lastF0()));
        gPitchFh.store(static_cast<float>(processor.lastFh()));
        gPitchConf.store(static_cast<float>(processor.lastConf()));

        int16_t sL = static_cast<int16_t>(std::max(-1.0f, std::min(1.0f, outL)) * 32767.0f);
        int16_t sR = static_cast<int16_t>(std::max(-1.0f, std::min(1.0f, outR)) * 32767.0f);

        buffer[static_cast<size_t>(i * channels)] = sL;
        if (channels > 1) buffer[static_cast<size_t>(i * channels + 1)] = sR;

#if PINCHFX_SIM_ENABLE_SCOPE
        float tapValue = 0.0f;
        switch (tap) {
            case ScopeTap::Input: tapValue = 0.5f * (inL + inR); break;
            case ScopeTap::Agc: tapValue = static_cast<float>(processor.lastAgc()); break;
            case ScopeTap::F0: {
                constexpr float kF0Min = 50.0f;
                constexpr float kF0Max = 1200.0f;
                const float f0 = gPitchF0.load();
                const float norm = (std::min(kF0Max, std::max(kF0Min, f0)) - kF0Min) / (kF0Max - kF0Min);
                tapValue = norm * 2.0f - 1.0f;
                break;
            }
            case ScopeTap::ResonatorFreq: {
                constexpr float kFhMin = 50.0f;
                const float fhMax = std::max(200.0f, static_cast<float>(0.45 * static_cast<double>(gInput.sampleRate)));
                const float fh = std::min(fhMax, std::max(kFhMin, gPitchFh.load()));
                const float logMin = std::log2(kFhMin);
                const float logMax = std::log2(fhMax);
                const float norm = (std::log2(fh) - logMin) / std::max(1e-6f, (logMax - logMin));
                tapValue = norm * 2.0f - 1.0f;
                break;
            }
            case ScopeTap::Resonator: {
                tapValue = static_cast<float>(processor.lastResonator());
                break;
            }
            case ScopeTap::Tone: tapValue = static_cast<float>(processor.lastTone()); break;
            case ScopeTap::Tube: tapValue = static_cast<float>(processor.lastTube()); break;
            case ScopeTap::Limiter: tapValue = static_cast<float>(processor.lastLimiter()); break;
            case ScopeTap::Output: tapValue = 0.5f * (outL + outR); break;
            default: tapValue = 0.0f; break;
        }

        if (!gScopeFrozen.load()) {
            const int idx = gScope.writeIndex.fetch_add(1, std::memory_order_relaxed);
            int slot = 0;
            if (gScope.size > 0) {
                slot = idx % gScope.size;
                if (slot < 0) slot += gScope.size;
            }
            if (gScope.size > 0) {
                gScope.samples[slot].store(tapValue, std::memory_order_relaxed);
            }
        }
#endif

        gReadIndex = (gReadIndex + 1) % static_cast<size_t>(totalFrames);
    }
}

DWORD WINAPI audioThreadProc(LPVOID) {
    PinchFxSimProcessor processor{};
    processor.prepare(static_cast<double>(gInput.sampleRate), 512);
    processor.reset();

    AudioState audio{};
    const int channels = gInput.channels;
    // 2048 @ 48kHz ~= 42.7ms. Higher latency is fine for an offline/diagnostic simulator and avoids dropouts.
    const int framesPerBuffer = 2048;
    if (!initAudioOutput(audio, gInput.sampleRate, channels, framesPerBuffer)) {
        gRunning = false;
        return 1;
    }
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

    for (size_t i = 0; i < audio.headers.size(); ++i) {
        fillBuffer(audio.buffers[i], processor, framesPerBuffer, channels);
        waveOutWrite(audio.waveOut, &audio.headers[i], sizeof(WAVEHDR));
    }

    while (gRunning.load()) {
        WaitForSingleObject(audio.eventHandle, INFINITE);
        for (size_t i = 0; i < audio.headers.size(); ++i) {
            auto& hdr = audio.headers[i];
            if (hdr.dwFlags & WHDR_DONE) {
                fillBuffer(audio.buffers[i], processor, framesPerBuffer, channels);
                waveOutWrite(audio.waveOut, &hdr, sizeof(WAVEHDR));
            }
        }
    }

    shutdownAudio(audio);
    return 0;
}

#if PINCHFX_SIM_ENABLE_SCOPE
LRESULT CALLBACK scopeWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_HSCROLL: {
            const HWND slider = reinterpret_cast<HWND>(lParam);
            if (slider) {
                const int id = GetDlgCtrlID(slider);
                const int pos = static_cast<int>(SendMessage(slider, TBM_GETPOS, 0, 0));
                updateParamsFromSlider(id, pos);
            }
            return 0;
        }
        case WM_COMMAND: {
            const int id = LOWORD(wParam);
            if (id == kIdScopeTap && HIWORD(wParam) == CBN_SELCHANGE) {
                const int sel = static_cast<int>(SendMessage(gScopeCombo, CB_GETCURSEL, 0, 0));
                gScopeTap.store(sel);
                for (int i = 0; i < gScope.size; ++i) {
                    gScope.samples[i].store(0.0f, std::memory_order_relaxed);
                }
                gScope.writeIndex.store(0, std::memory_order_relaxed);
            }
            if (id == kIdScopeFreeze) {
                const bool enabled = (SendMessage(reinterpret_cast<HWND>(lParam), BM_GETCHECK, 0, 0) == BST_CHECKED);
                gScopeFrozen.store(enabled);
            }
            return 0;
        }
        case WM_TIMER:
            InvalidateRect(hwnd, nullptr, FALSE);
            return 0;
        case WM_SIZE:
            InvalidateRect(hwnd, nullptr, TRUE);
            return 0;
        case WM_PAINT: {
            PAINTSTRUCT ps{};
            HDC hdc = BeginPaint(hwnd, &ps);
            RECT rect{};
            GetClientRect(hwnd, &rect);
            HBRUSH bg = CreateSolidBrush(RGB(20, 20, 20));
            FillRect(hdc, &rect, bg);
            DeleteObject(bg);

            HPEN pen = CreatePen(PS_SOLID, 1, RGB(0, 220, 180));
            HPEN oldPen = static_cast<HPEN>(SelectObject(hdc, pen));

            RECT plotRect = rect;
            plotRect.top += kScopeTopBar;
            const int width = plotRect.right - plotRect.left;
            const int height = plotRect.bottom - plotRect.top;
            const int sampleCount = gScope.size;
            if (sampleCount > 0 && width > 1 && height > 1) {
                std::vector<float> snapshot(sampleCount);
                for (int i = 0; i < sampleCount; ++i) {
                    snapshot[static_cast<size_t>(i)] = gScope.samples[i].load(std::memory_order_relaxed);
                }
                const int writeIndex = gScope.writeIndex.load(std::memory_order_relaxed);

                const float norm = gParams.scopeTime.load();
                constexpr float kScopeMinSeconds = 0.25f;
                constexpr float kScopeMaxSeconds = 30.0f;
                const float timeSeconds = kScopeMinSeconds + (kScopeMaxSeconds - kScopeMinSeconds) * std::min(1.0f, std::max(0.0f, norm));
                const int span = std::max(64, std::min(sampleCount, static_cast<int>(timeSeconds * gInput.sampleRate)));

                const float levelNorm = std::min(1.0f, std::max(0.0f, gParams.scopeLevel.load()));
                constexpr float kScopeLevelMinDb = -80.0f;
                constexpr float kScopeLevelMaxDb = 12.0f;
                const float levelDb = kScopeLevelMinDb + (kScopeLevelMaxDb - kScopeLevelMinDb) * levelNorm;
                const float scale = std::pow(10.0f, levelDb / 20.0f);
                float peak = 0.0f;
                for (float v : snapshot) {
                    const float a = std::abs(v);
                    if (a > peak) peak = a;
                }

                int start = writeIndex % sampleCount;
                if (start < 0) start += sampleCount;
                int yMid = plotRect.top + height / 2;
                float v0 = snapshot[static_cast<size_t>(start)] * scale;
                if (v0 > 1.0f) v0 = 1.0f;
                if (v0 < -1.0f) v0 = -1.0f;
                int y0 = yMid - static_cast<int>(v0 * (height / 2 - 2));
                MoveToEx(hdc, plotRect.left, y0, nullptr);
                for (int x = 1; x < width; ++x) {
                    const int64_t offset = (static_cast<int64_t>(x) * static_cast<int64_t>(span)) / static_cast<int64_t>(width);
                    int idx = static_cast<int>((static_cast<int64_t>(start) + offset) % static_cast<int64_t>(sampleCount));
                    if (idx < 0) idx += sampleCount;
                    float v = snapshot[static_cast<size_t>(idx)] * scale;
                    if (v > 1.0f) v = 1.0f;
                    if (v < -1.0f) v = -1.0f;
                    int y = yMid - static_cast<int>(v * (height / 2 - 2));
                    LineTo(hdc, plotRect.left + x, y);
                }

            }

            SelectObject(hdc, oldPen);
            DeleteObject(pen);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_DESTROY:
            gRunning = false;
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProc(hwnd, msg, wParam, lParam);
    }
}
#endif

LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_HSCROLL: {
            const HWND slider = reinterpret_cast<HWND>(lParam);
            if (slider) {
                const int id = GetDlgCtrlID(slider);
                const int pos = static_cast<int>(SendMessage(slider, TBM_GETPOS, 0, 0));
                updateParamsFromSlider(id, pos);
            }
            return 0;
        }
        case WM_COMMAND: {
            const int id = LOWORD(wParam);
            if (id == kIdEnable) {
                const bool mute = (SendMessage(reinterpret_cast<HWND>(lParam), BM_GETCHECK, 0, 0) == BST_CHECKED);
                gParams.mute.store(mute);
            }
#if PINCHFX_SIM_ENABLE_SCOPE
            if (id == kIdScopeTap && HIWORD(wParam) == CBN_SELCHANGE) {
                const int sel = static_cast<int>(SendMessage(gScopeCombo, CB_GETCURSEL, 0, 0));
                gScopeTap.store(sel);
                for (int i = 0; i < gScope.size; ++i) {
                    gScope.samples[i].store(0.0f, std::memory_order_relaxed);
                }
                gScope.writeIndex.store(0, std::memory_order_relaxed);
            }
            if (id == kIdScopeFreeze) {
                const bool enabled = (SendMessage(reinterpret_cast<HWND>(lParam), BM_GETCHECK, 0, 0) == BST_CHECKED);
                gScopeFrozen.store(enabled);
            }
#endif
            return 0;
        }
        case WM_TIMER:
            updateTrackLabel();
            return 0;
        case WM_DESTROY:
            gRunning = false;
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProc(hwnd, msg, wParam, lParam);
    }
}

HWND createSlider(HWND parent, const SliderSpec& spec) {
    HWND slider = CreateWindowEx(
        0,
        TRACKBAR_CLASS,
        "",
        WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_AUTOTICKS,
        kLeftMargin + kLabelWidth,
        spec.y,
        kSliderWidth,
        kSliderHeight,
        parent,
        reinterpret_cast<HMENU>(static_cast<intptr_t>(spec.id)),
        nullptr,
        nullptr);

    SendMessage(slider, TBM_SETRANGE, TRUE, MAKELONG(spec.minPos, spec.maxPos));
    setSliderValue(slider, spec.id, spec.defaultValue);
    updateParamsFromSlider(spec.id, normToSliderPos(spec.defaultValue));
    if (spec.id == kIdLock) gLockSlider = slider;
    return slider;
}

void createLabel(HWND parent, const SliderSpec& spec) {
    HWND label = CreateWindowEx(
        0,
        "STATIC",
        spec.label,
        WS_CHILD | WS_VISIBLE,
        kLeftMargin,
        spec.y + 4,
        kLabelWidth,
        kSliderHeight,
        parent,
        nullptr,
        nullptr,
        nullptr);
    if (spec.id == kIdLock) {
        gResLabel = label;
        updateResLabel(spec.defaultValue);
    } else if (spec.id == kIdPosition) {
        gPositionLabel = label;
        updatePositionLabel(spec.defaultValue);
    } else if (spec.id == kIdTrack) {
        gTrackLabel = label;
        updateTrackLabel();
    }
}

} // namespace

int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int cmdShow) {
    INITCOMMONCONTROLSEX icc{};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_BAR_CLASSES;
    InitCommonControlsEx(&icc);
    timeBeginPeriod(1);

    if (!pinchfx::sim::readWav(SAMPLE_WAV, gInput)) {
        MessageBox(nullptr, "Failed to read in.wav", "PinchFxSim", MB_OK | MB_ICONERROR);
        return 1;
    }

    const char* className = "PinchFxSimWindow";
    char mainTitle[128]{};
    std::snprintf(mainTitle, sizeof(mainTitle), "PinchFxSim (%s %s)", __DATE__, __TIME__);
    WNDCLASS wc{};
    wc.lpfnWndProc = wndProc;
    wc.hInstance = instance;
    wc.lpszClassName = className;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClass(&wc);

#if PINCHFX_SIM_ENABLE_SCOPE
    const char* scopeClassName = "PinchFxScopeWindow";
    char scopeTitle[128]{};
    std::snprintf(scopeTitle, sizeof(scopeTitle), "PinchFx Scope (%s %s)", __DATE__, __TIME__);
    WNDCLASS swc{};
    swc.lpfnWndProc = scopeWndProc;
    swc.hInstance = instance;
    swc.lpszClassName = scopeClassName;
    swc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClass(&swc);
#endif

    const int width = 420;
    const int height = kTopMargin + static_cast<int>(std::size(kSliders)) * kRowHeight + 120;
    HWND hwnd = CreateWindowEx(
        0,
        className,
        mainTitle,
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        width,
        height,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (!hwnd) return 1;

    for (const auto& spec : kSliders) {
        createLabel(hwnd, spec);
        createSlider(hwnd, spec);
    }

    HWND enable = CreateWindowEx(
        0,
        "BUTTON",
        "MUTE",
        WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
        kLeftMargin,
        kTopMargin + static_cast<int>(std::size(kSliders)) * kRowHeight + 10,
        120,
        24,
        hwnd,
        reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdEnable)),
        instance,
        nullptr);
    SendMessage(enable, BM_SETCHECK, BST_UNCHECKED, 0);

#if PINCHFX_SIM_ENABLE_SCOPE
    constexpr float kScopeMaxSeconds = 30.0f;
    const int scopeSamples = std::max(2048, static_cast<int>(gInput.sampleRate * kScopeMaxSeconds));
    gScope.size = scopeSamples;
    gScope.samples = std::make_unique<std::atomic<float>[]>(static_cast<size_t>(scopeSamples));
    for (int i = 0; i < gScope.size; ++i) {
        gScope.samples[i].store(0.0f, std::memory_order_relaxed);
    }

        gScopeHwnd = CreateWindowEx(
            0,
        scopeClassName,
        scopeTitle,
            WS_OVERLAPPEDWINDOW | WS_SIZEBOX | WS_CLIPCHILDREN,
            CW_USEDEFAULT,
        CW_USEDEFAULT,
        640,
        320,
        hwnd,
        nullptr,
        instance,
        nullptr);
    if (gScopeHwnd) {
        ShowWindow(gScopeHwnd, cmdShow);
        UpdateWindow(gScopeHwnd);
        SetTimer(gScopeHwnd, 1, 33, nullptr);

        CreateWindowEx(
            0,
            "STATIC",
            "Tap",
            WS_CHILD | WS_VISIBLE,
            12,
            10,
            40,
            20,
            gScopeHwnd,
            nullptr,
            instance,
            nullptr);

        gScopeCombo = CreateWindowEx(
            0,
            "COMBOBOX",
            "",
            WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
            56,
            8,
            160,
            200,
            gScopeHwnd,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeTap)),
            instance,
            nullptr);

        for (int i = 0; i <= static_cast<int>(ScopeTap::Output); ++i) {
            SendMessage(gScopeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(scopeTapLabel(static_cast<ScopeTap>(i))));
        }
        SendMessage(gScopeCombo, CB_SETCURSEL, static_cast<WPARAM>(gScopeTap.load()), 0);

        CreateWindowEx(
            0,
            "STATIC",
            "Time",
            WS_CHILD | WS_VISIBLE,
            230,
            10,
            40,
            20,
            gScopeHwnd,
            nullptr,
            instance,
            nullptr);

        gScopeTimeSlider = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            "",
            WS_CHILD | WS_VISIBLE | TBS_HORZ,
            274,
            8,
            160,
            24,
            gScopeHwnd,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeTime)),
            instance,
            nullptr);
        SendMessage(gScopeTimeSlider, TBM_SETRANGE, TRUE, MAKELONG(0, 1000));
        SendMessage(gScopeTimeSlider, TBM_SETPOS, TRUE, normToSliderPos(gParams.scopeTime.load()));

        CreateWindowEx(
            0,
            "STATIC",
            "Level",
            WS_CHILD | WS_VISIBLE,
            12,
            36,
            40,
            20,
            gScopeHwnd,
            nullptr,
            instance,
            nullptr);

        gScopeLevelSlider = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            "",
            WS_CHILD | WS_VISIBLE | TBS_HORZ,
            56,
            32,
            180,
            24,
            gScopeHwnd,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeLevel)),
            instance,
            nullptr);
        SendMessage(gScopeLevelSlider, TBM_SETRANGE, TRUE, MAKELONG(0, 1000));
        SendMessage(gScopeLevelSlider, TBM_SETPOS, TRUE, normToSliderPos(gParams.scopeLevel.load()));

        gScopeFreezeButton = CreateWindowEx(
            0,
            "BUTTON",
            "Freeze",
            WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
            260,
            34,
            80,
            22,
            gScopeHwnd,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeFreeze)),
            instance,
            nullptr);
    }
#endif

    ShowWindow(hwnd, cmdShow);
    UpdateWindow(hwnd);
    SetTimer(hwnd, 2, 100, nullptr);

    HANDLE audioThread = CreateThread(nullptr, 0, audioThreadProc, nullptr, 0, nullptr);

    MSG msg{};
    while (GetMessage(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    gRunning = false;
    if (audioThread) {
        WaitForSingleObject(audioThread, INFINITE);
        CloseHandle(audioThread);
    }
    timeEndPeriod(1);

    return 0;
}

// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include "config.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commctrl.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

namespace pinchfx::sim {

class SimScope {
public:
    enum class Tap : int {
        Input = 0,
        Agc,
        F0,
        ResonatorFreq,
        Resonator,
        Tone,
        Tube,
        Limiter,
        Output,
        Count
    };

    struct SampleFrame {
        float input{0.0f};
        float agc{0.0f};
        float f0{0.0f};
        float resonatorFreq{0.0f};
        float resonator{0.0f};
        float tone{0.0f};
        float tube{0.0f};
        float limiter{0.0f};
        float output{0.0f};
    };

    bool initialize(HINSTANCE instance, HWND owner, int cmdShow, int sampleRate) {
#if PINCHFX_SIM_ENABLE_SCOPE
        if (hwnd_) return true;
        sampleRate_ = std::max(1, sampleRate);

        const int scopeSamples = std::max(2048, static_cast<int>(sampleRate_ * kScopeMaxSeconds));
        bufferSize_ = scopeSamples;
        samples_ = std::make_unique<std::atomic<float>[]>(static_cast<size_t>(bufferSize_));
        clearBuffer_();

        if (!registerClass_(instance)) return false;

        char title[128]{};
        std::snprintf(title, sizeof(title), "PinchFx Scope (%s %s)", __DATE__, __TIME__);
        hwnd_ = CreateWindowEx(
            0,
            kWindowClassName,
            title,
            WS_OVERLAPPEDWINDOW | WS_SIZEBOX | WS_CLIPCHILDREN,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            640,
            320,
            owner,
            nullptr,
            instance,
            this);
        if (!hwnd_) return false;

        createControls_(instance);
        SetTimer(hwnd_, kScopeTimerId, 33, nullptr);
        ShowWindow(hwnd_, cmdShow);
        UpdateWindow(hwnd_);
#else
        (void)instance;
        (void)owner;
        (void)cmdShow;
        (void)sampleRate;
#endif
        return true;
    }

    void shutdown() {
#if PINCHFX_SIM_ENABLE_SCOPE
        if (!hwnd_) return;
        KillTimer(hwnd_, kScopeTimerId);
        DestroyWindow(hwnd_);
        hwnd_ = nullptr;
        comboTap_ = nullptr;
        sliderTime_ = nullptr;
        sliderLevel_ = nullptr;
        buttonFreeze_ = nullptr;
        samples_.reset();
        bufferSize_ = 0;
#endif
    }

    void pushSample(const SampleFrame& frame) {
#if PINCHFX_SIM_ENABLE_SCOPE
        if (!samples_ || bufferSize_ <= 0 || frozen_.load(std::memory_order_relaxed)) return;
        const float mapped = mapTapSample_(frame, static_cast<Tap>(tap_.load(std::memory_order_relaxed)));
        const int idx = writeIndex_.fetch_add(1, std::memory_order_relaxed);
        int slot = idx % bufferSize_;
        if (slot < 0) slot += bufferSize_;
        samples_[slot].store(mapped, std::memory_order_relaxed);
#else
        (void)frame;
#endif
    }

private:
#if PINCHFX_SIM_ENABLE_SCOPE
    static constexpr const char* kWindowClassName = "PinchFxScopeWindow";
    static constexpr int kScopeTimerId = 1;
    static constexpr int kScopeTopBar = 64;
    static constexpr float kScopeMinSeconds = 0.25f;
    static constexpr float kScopeMaxSeconds = 30.0f;
    static constexpr float kScopeLevelMinDb = -80.0f;
    static constexpr float kScopeLevelMaxDb = 12.0f;
    static constexpr int kIdScopeTap = 5001;
    static constexpr int kIdScopeTime = 5002;
    static constexpr int kIdScopeFreeze = 5003;
    static constexpr int kIdScopeLevel = 5004;

    static float clamp01_(float v) {
        return std::min(1.0f, std::max(0.0f, v));
    }

    static const char* tapLabel_(Tap tap) {
        switch (tap) {
            case Tap::Input: return "Input";
            case Tap::Agc: return "AGC";
            case Tap::F0: return "F0 Hz";
            case Tap::ResonatorFreq: return "Res Freq";
            case Tap::Resonator: return "Resonator";
            case Tap::Tone: return "Tone";
            case Tap::Tube: return "Tube";
            case Tap::Limiter: return "Limiter";
            case Tap::Output: return "Output";
            default: return "Output";
        }
    }

    static LRESULT CALLBACK staticWndProc_(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
        SimScope* self = nullptr;
        if (msg == WM_NCCREATE) {
            auto* create = reinterpret_cast<CREATESTRUCT*>(lParam);
            self = static_cast<SimScope*>(create->lpCreateParams);
            SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        } else {
            self = reinterpret_cast<SimScope*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
        }

        if (self) {
            return self->wndProc_(hwnd, msg, wParam, lParam);
        }
        return DefWindowProc(hwnd, msg, wParam, lParam);
    }

    bool registerClass_(HINSTANCE instance) {
        WNDCLASS existing{};
        if (GetClassInfo(instance, kWindowClassName, &existing) != 0) return true;

        WNDCLASS wc{};
        wc.lpfnWndProc = staticWndProc_;
        wc.hInstance = instance;
        wc.lpszClassName = kWindowClassName;
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        return RegisterClass(&wc) != 0;
    }

    void createControls_(HINSTANCE instance) {
        CreateWindowEx(
            0,
            "STATIC",
            "Tap",
            WS_CHILD | WS_VISIBLE,
            12,
            10,
            40,
            20,
            hwnd_,
            nullptr,
            instance,
            nullptr);

        comboTap_ = CreateWindowEx(
            0,
            "COMBOBOX",
            "",
            WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
            56,
            8,
            160,
            200,
            hwnd_,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeTap)),
            instance,
            nullptr);
        for (int i = 0; i < static_cast<int>(Tap::Count); ++i) {
            SendMessage(comboTap_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(tapLabel_(static_cast<Tap>(i))));
        }
        SendMessage(comboTap_, CB_SETCURSEL, static_cast<WPARAM>(tap_.load(std::memory_order_relaxed)), 0);

        CreateWindowEx(
            0,
            "STATIC",
            "Time",
            WS_CHILD | WS_VISIBLE,
            230,
            10,
            40,
            20,
            hwnd_,
            nullptr,
            instance,
            nullptr);

        sliderTime_ = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            "",
            WS_CHILD | WS_VISIBLE | TBS_HORZ,
            274,
            8,
            160,
            24,
            hwnd_,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeTime)),
            instance,
            nullptr);
        SendMessage(sliderTime_, TBM_SETRANGE, TRUE, MAKELONG(0, 1000));
        SendMessage(sliderTime_, TBM_SETPOS, TRUE, static_cast<LPARAM>(timeNorm_.load(std::memory_order_relaxed) * 1000.0f));

        CreateWindowEx(
            0,
            "STATIC",
            "Level",
            WS_CHILD | WS_VISIBLE,
            12,
            36,
            40,
            20,
            hwnd_,
            nullptr,
            instance,
            nullptr);

        sliderLevel_ = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            "",
            WS_CHILD | WS_VISIBLE | TBS_HORZ,
            56,
            32,
            180,
            24,
            hwnd_,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeLevel)),
            instance,
            nullptr);
        SendMessage(sliderLevel_, TBM_SETRANGE, TRUE, MAKELONG(0, 1000));
        SendMessage(sliderLevel_, TBM_SETPOS, TRUE, static_cast<LPARAM>(levelNorm_.load(std::memory_order_relaxed) * 1000.0f));

        buttonFreeze_ = CreateWindowEx(
            0,
            "BUTTON",
            "Freeze",
            WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
            260,
            34,
            80,
            22,
            hwnd_,
            reinterpret_cast<HMENU>(static_cast<intptr_t>(kIdScopeFreeze)),
            instance,
            nullptr);
    }

    void clearBuffer_() {
        if (!samples_ || bufferSize_ <= 0) return;
        for (int i = 0; i < bufferSize_; ++i) {
            samples_[i].store(0.0f, std::memory_order_relaxed);
        }
        writeIndex_.store(0, std::memory_order_relaxed);
    }

    float mapTapSample_(const SampleFrame& frame, Tap tap) const {
        switch (tap) {
            case Tap::Input: return frame.input;
            case Tap::Agc: return frame.agc;
            case Tap::F0: {
                static constexpr float kF0Min = 50.0f;
                static constexpr float kF0Max = 1200.0f;
                const float f0 = std::min(kF0Max, std::max(kF0Min, frame.f0));
                const float norm = (f0 - kF0Min) / (kF0Max - kF0Min);
                return norm * 2.0f - 1.0f;
            }
            case Tap::ResonatorFreq: {
                static constexpr float kFhMin = 50.0f;
                const float fhMax = std::max(200.0f, 0.45f * static_cast<float>(sampleRate_));
                const float fh = std::min(fhMax, std::max(kFhMin, frame.resonatorFreq));
                const float logMin = std::log2(kFhMin);
                const float logMax = std::log2(fhMax);
                const float norm = (std::log2(fh) - logMin) / std::max(1e-6f, (logMax - logMin));
                return norm * 2.0f - 1.0f;
            }
            case Tap::Resonator: return frame.resonator;
            case Tap::Tone: return frame.tone;
            case Tap::Tube: return frame.tube;
            case Tap::Limiter: return frame.limiter;
            case Tap::Output: return frame.output;
            default: return 0.0f;
        }
    }

    void paint_(HWND hwnd) {
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
        if (bufferSize_ > 0 && width > 1 && height > 1) {
            std::vector<float> snapshot(static_cast<size_t>(bufferSize_));
            for (int i = 0; i < bufferSize_; ++i) {
                snapshot[static_cast<size_t>(i)] = samples_[i].load(std::memory_order_relaxed);
            }
            const int writeIndex = writeIndex_.load(std::memory_order_relaxed);

            const float timeSeconds = kScopeMinSeconds + (kScopeMaxSeconds - kScopeMinSeconds) * clamp01_(timeNorm_.load(std::memory_order_relaxed));
            const int span = std::max(64, std::min(bufferSize_, static_cast<int>(timeSeconds * static_cast<float>(sampleRate_))));

            const float levelDb = kScopeLevelMinDb + (kScopeLevelMaxDb - kScopeLevelMinDb) * clamp01_(levelNorm_.load(std::memory_order_relaxed));
            const float scale = std::pow(10.0f, levelDb / 20.0f);

            int start = writeIndex % bufferSize_;
            if (start < 0) start += bufferSize_;
            const int yMid = plotRect.top + height / 2;
            float v0 = snapshot[static_cast<size_t>(start)] * scale;
            if (v0 > 1.0f) v0 = 1.0f;
            if (v0 < -1.0f) v0 = -1.0f;
            const int y0 = yMid - static_cast<int>(v0 * (height / 2 - 2));
            MoveToEx(hdc, plotRect.left, y0, nullptr);

            for (int x = 1; x < width; ++x) {
                const int64_t offset = (static_cast<int64_t>(x) * static_cast<int64_t>(span)) / static_cast<int64_t>(width);
                int idx = static_cast<int>((static_cast<int64_t>(start) + offset) % static_cast<int64_t>(bufferSize_));
                if (idx < 0) idx += bufferSize_;
                float v = snapshot[static_cast<size_t>(idx)] * scale;
                if (v > 1.0f) v = 1.0f;
                if (v < -1.0f) v = -1.0f;
                const int y = yMid - static_cast<int>(v * (height / 2 - 2));
                LineTo(hdc, plotRect.left + x, y);
            }
        }

        SelectObject(hdc, oldPen);
        DeleteObject(pen);
        EndPaint(hwnd, &ps);
    }

    void handleScroll_(HWND slider) {
        if (!slider) return;
        const int id = GetDlgCtrlID(slider);
        const int pos = static_cast<int>(SendMessage(slider, TBM_GETPOS, 0, 0));
        const float norm = clamp01_(static_cast<float>(pos) / 1000.0f);
        if (id == kIdScopeTime) {
            timeNorm_.store(norm, std::memory_order_relaxed);
        } else if (id == kIdScopeLevel) {
            levelNorm_.store(norm, std::memory_order_relaxed);
        }
    }

    LRESULT wndProc_(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
        switch (msg) {
            case WM_HSCROLL:
                handleScroll_(reinterpret_cast<HWND>(lParam));
                return 0;
            case WM_COMMAND: {
                const int id = LOWORD(wParam);
                if (id == kIdScopeTap && HIWORD(wParam) == CBN_SELCHANGE && comboTap_) {
                    const int sel = static_cast<int>(SendMessage(comboTap_, CB_GETCURSEL, 0, 0));
                    tap_.store(sel, std::memory_order_relaxed);
                    clearBuffer_();
                } else if (id == kIdScopeFreeze && buttonFreeze_) {
                    const bool enabled = (SendMessage(buttonFreeze_, BM_GETCHECK, 0, 0) == BST_CHECKED);
                    frozen_.store(enabled, std::memory_order_relaxed);
                }
                return 0;
            }
            case WM_TIMER:
            case WM_SIZE:
                InvalidateRect(hwnd, nullptr, FALSE);
                return 0;
            case WM_PAINT:
                paint_(hwnd);
                return 0;
            case WM_DESTROY:
                KillTimer(hwnd, kScopeTimerId);
                return 0;
            case WM_NCDESTROY:
                hwnd_ = nullptr;
                comboTap_ = nullptr;
                sliderTime_ = nullptr;
                sliderLevel_ = nullptr;
                buttonFreeze_ = nullptr;
                return DefWindowProc(hwnd, msg, wParam, lParam);
            default:
                return DefWindowProc(hwnd, msg, wParam, lParam);
        }
    }

    HWND hwnd_{nullptr};
    HWND comboTap_{nullptr};
    HWND sliderTime_{nullptr};
    HWND sliderLevel_{nullptr};
    HWND buttonFreeze_{nullptr};

    int sampleRate_{48000};
    int bufferSize_{0};
    std::unique_ptr<std::atomic<float>[]> samples_{};
    std::atomic<int> writeIndex_{0};
    std::atomic<int> tap_{static_cast<int>(Tap::Output)};
    std::atomic<float> timeNorm_{0.5f};
    std::atomic<float> levelNorm_{0.5f};
    std::atomic<bool> frozen_{false};
#endif
};

} // namespace pinchfx::sim

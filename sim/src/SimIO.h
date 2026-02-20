// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace pinchfx::sim {

struct AudioBuffer {
    int sampleRate{0};
    int channels{0};
    std::vector<float> samples; // interleaved
};

inline uint32_t readU32LE(std::FILE* f) {
    uint8_t b[4]{};
    if (std::fread(b, 1, 4, f) != 4) return 0;
    return static_cast<uint32_t>(b[0]) |
           (static_cast<uint32_t>(b[1]) << 8) |
           (static_cast<uint32_t>(b[2]) << 16) |
           (static_cast<uint32_t>(b[3]) << 24);
}

inline uint16_t readU16LE(std::FILE* f) {
    uint8_t b[2]{};
    if (std::fread(b, 1, 2, f) != 2) return 0;
    return static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
}

inline bool readWav(const std::string& path, AudioBuffer& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;

    char riff[4]{};
    if (std::fread(riff, 1, 4, f) != 4 || std::string(riff, 4) != "RIFF") {
        std::fclose(f);
        return false;
    }
    readU32LE(f); // file size
    char wave[4]{};
    if (std::fread(wave, 1, 4, f) != 4 || std::string(wave, 4) != "WAVE") {
        std::fclose(f);
        return false;
    }

    uint16_t audioFormat = 0;
    uint16_t numChannels = 0;
    uint32_t sampleRate = 0;
    uint16_t bitsPerSample = 0;
    uint32_t dataSize = 0;
    long dataOffset = 0;

    while (!std::feof(f)) {
        char id[4]{};
        if (std::fread(id, 1, 4, f) != 4) break;
        uint32_t size = readU32LE(f);
        if (std::string(id, 4) == "fmt ") {
            audioFormat = readU16LE(f);
            numChannels = readU16LE(f);
            sampleRate = readU32LE(f);
            readU32LE(f); // byte rate
            readU16LE(f); // block align
            bitsPerSample = readU16LE(f);
            std::fseek(f, static_cast<long>(size) - 16, SEEK_CUR);
        } else if (std::string(id, 4) == "data") {
            dataSize = size;
            dataOffset = std::ftell(f);
            std::fseek(f, static_cast<long>(size), SEEK_CUR);
        } else {
            std::fseek(f, static_cast<long>(size), SEEK_CUR);
        }
    }

    if (audioFormat == 0 || numChannels == 0 || sampleRate == 0 || bitsPerSample == 0 || dataSize == 0) {
        std::fclose(f);
        return false;
    }

    if (audioFormat != 1 && audioFormat != 3) {
        std::fclose(f);
        return false;
    }

    std::fseek(f, dataOffset, SEEK_SET);
    const size_t frameCount = dataSize / (numChannels * (bitsPerSample / 8));
    out.sampleRate = static_cast<int>(sampleRate);
    out.channels = static_cast<int>(numChannels);
    out.samples.assign(frameCount * numChannels, 0.0f);

    for (size_t i = 0; i < frameCount * numChannels; ++i) {
        float v = 0.0f;
        if (audioFormat == 3 && bitsPerSample == 32) {
            float f32 = 0.0f;
            std::fread(&f32, sizeof(float), 1, f);
            v = f32;
        } else if (audioFormat == 1 && bitsPerSample == 16) {
            int16_t s = 0;
            std::fread(&s, sizeof(int16_t), 1, f);
            v = static_cast<float>(s) / 32768.0f;
        } else if (audioFormat == 1 && bitsPerSample == 24) {
            uint8_t b[3]{};
            std::fread(b, 1, 3, f);
            int32_t s = (b[0] | (b[1] << 8) | (b[2] << 16));
            if (s & 0x800000) s |= ~0xFFFFFF;
            v = static_cast<float>(s) / 8388608.0f;
        } else if (audioFormat == 1 && bitsPerSample == 32) {
            int32_t s = 0;
            std::fread(&s, sizeof(int32_t), 1, f);
            v = static_cast<float>(s) / 2147483648.0f;
        } else {
            std::fclose(f);
            return false;
        }
        out.samples[i] = v;
    }

    std::fclose(f);
    return true;
}

inline bool writeWav(const std::string& path, const AudioBuffer& in) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    const uint16_t numChannels = static_cast<uint16_t>(in.channels);
    const uint32_t sampleRate = static_cast<uint32_t>(in.sampleRate);
    const uint16_t bitsPerSample = 16;
    const uint16_t blockAlign = numChannels * (bitsPerSample / 8);
    const uint32_t byteRate = sampleRate * blockAlign;
    const uint32_t dataSize = static_cast<uint32_t>(in.samples.size() * sizeof(int16_t));

    std::fwrite("RIFF", 1, 4, f);
    const uint32_t riffSize = 36 + dataSize;
    std::fwrite(&riffSize, 4, 1, f);
    std::fwrite("WAVE", 1, 4, f);

    std::fwrite("fmt ", 1, 4, f);
    const uint32_t fmtSize = 16;
    std::fwrite(&fmtSize, 4, 1, f);
    const uint16_t audioFormat = 1;
    std::fwrite(&audioFormat, 2, 1, f);
    std::fwrite(&numChannels, 2, 1, f);
    std::fwrite(&sampleRate, 4, 1, f);
    std::fwrite(&byteRate, 4, 1, f);
    std::fwrite(&blockAlign, 2, 1, f);
    std::fwrite(&bitsPerSample, 2, 1, f);

    std::fwrite("data", 1, 4, f);
    std::fwrite(&dataSize, 4, 1, f);

    for (float v : in.samples) {
        float clamped = v;
        if (clamped > 1.0f) clamped = 1.0f;
        if (clamped < -1.0f) clamped = -1.0f;
        const int16_t s = static_cast<int16_t>(clamped * 32767.0f);
        std::fwrite(&s, sizeof(int16_t), 1, f);
    }

    std::fclose(f);
    return true;
}

} // namespace pinchfx::sim

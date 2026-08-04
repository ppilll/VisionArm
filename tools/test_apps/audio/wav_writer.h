#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>

class WavWriter {
public:
    WavWriter(const std::string& path,
              uint32_t sample_rate,
              uint16_t channels,
              uint16_t bits_per_sample);

    ~WavWriter();

    WavWriter(const WavWriter&) = delete;
    WavWriter& operator=(const WavWriter&) = delete;

    void write(const void* data, std::size_t bytes);
    void close();

private:
    std::fstream file_;
    uint32_t sample_rate_ = 0;
    uint16_t channels_ = 0;
    uint16_t bits_per_sample_ = 0;
    uint64_t data_bytes_ = 0;
    bool closed_ = false;

    void writeHeader(uint32_t data_size);

    static void writeLe16(std::ostream& output, uint16_t value);
    static void writeLe32(std::ostream& output, uint32_t value);
};
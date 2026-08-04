#include "wav_writer.h"

#include <filesystem>
#include <limits>
#include <stdexcept>

namespace {

void ensureParentDirectory(const std::string& path) {
    const std::filesystem::path file_path(path);
    const auto parent = file_path.parent_path();

    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
}

}  // namespace

WavWriter::WavWriter(const std::string& path,
                     uint32_t sample_rate,
                     uint16_t channels,
                     uint16_t bits_per_sample)
    : sample_rate_(sample_rate),
      channels_(channels),
      bits_per_sample_(bits_per_sample) {
    if (channels_ == 0 || sample_rate_ == 0) {
        throw std::invalid_argument(
            "Invalid WAV sample rate or channel count");
    }

    if (bits_per_sample_ != 16) {
        throw std::invalid_argument(
            "This V2A WAV writer currently supports 16-bit PCM only");
    }

    ensureParentDirectory(path);

    file_.open(path,
               std::ios::binary |
               std::ios::in |
               std::ios::out |
               std::ios::trunc);

    if (!file_) {
        throw std::runtime_error(
            "Cannot open WAV output: " + path);
    }

    writeHeader(0);
}

WavWriter::~WavWriter() {
    try {
        close();
    } catch (...) {
    }
}

void WavWriter::writeLe16(std::ostream& output,
                          uint16_t value) {
    const char bytes[2] = {
        static_cast<char>(value & 0xffU),
        static_cast<char>((value >> 8U) & 0xffU)
    };

    output.write(bytes, sizeof(bytes));
}

void WavWriter::writeLe32(std::ostream& output,
                          uint32_t value) {
    const char bytes[4] = {
        static_cast<char>(value & 0xffU),
        static_cast<char>((value >> 8U) & 0xffU),
        static_cast<char>((value >> 16U) & 0xffU),
        static_cast<char>((value >> 24U) & 0xffU)
    };

    output.write(bytes, sizeof(bytes));
}

void WavWriter::writeHeader(uint32_t data_size) {
    const uint16_t block_align =
        static_cast<uint16_t>(
            channels_ * bits_per_sample_ / 8U);

    const uint32_t byte_rate =
        sample_rate_ *
        static_cast<uint32_t>(block_align);

    file_.seekp(0, std::ios::beg);

    file_.write("RIFF", 4);
    writeLe32(file_, 36U + data_size);
    file_.write("WAVE", 4);

    file_.write("fmt ", 4);
    writeLe32(file_, 16U);
    writeLe16(file_, 1U);  // PCM
    writeLe16(file_, channels_);
    writeLe32(file_, sample_rate_);
    writeLe32(file_, byte_rate);
    writeLe16(file_, block_align);
    writeLe16(file_, bits_per_sample_);

    file_.write("data", 4);
    writeLe32(file_, data_size);

    if (!file_) {
        throw std::runtime_error(
            "Failed while writing WAV header");
    }
}

void WavWriter::write(const void* data,
                      std::size_t bytes) {
    if (closed_) {
        throw std::runtime_error(
            "Attempt to write a closed WAV file");
    }

    if (data_bytes_ + bytes >
        std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error(
            "WAV file exceeds 4 GiB RIFF limit");
    }

    file_.seekp(0, std::ios::end);
    file_.write(static_cast<const char*>(data),
                static_cast<std::streamsize>(bytes));

    if (!file_) {
        throw std::runtime_error(
            "Failed while writing WAV PCM data");
    }

    data_bytes_ += bytes;
}

void WavWriter::close() {
    if (closed_) {
        return;
    }

    if (file_.is_open()) {
        writeHeader(static_cast<uint32_t>(data_bytes_));
        file_.flush();

        if (!file_) {
            throw std::runtime_error(
                "Failed while finalizing WAV file");
        }

        file_.close();
    }

    closed_ = true;
}
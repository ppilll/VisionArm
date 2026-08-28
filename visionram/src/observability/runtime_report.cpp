#include "observability/runtime_report.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace visionarm::report {
namespace {

struct Entry {
    std::string key;
    std::string value;
};

void SetError(std::string* error, std::string message) noexcept {
    if (error == nullptr) return;
    try {
        *error = std::move(message);
    } catch (...) {
    }
}

std::string Lower(std::string_view text) {
    std::string result(text);
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    return result;
}

bool StartsWith(std::string_view value, std::string_view prefix) noexcept {
    return value.size() >= prefix.size() &&
           value.substr(0U, prefix.size()) == prefix;
}

bool EndsWith(std::string_view value, std::string_view suffix) noexcept {
    return value.size() >= suffix.size() &&
           value.substr(value.size() - suffix.size()) == suffix;
}

bool Contains(std::string_view value, std::string_view fragment) noexcept {
    return value.find(fragment) != std::string_view::npos;
}

bool IsValidKey(std::string_view key) noexcept {
    if (key.empty()) return false;
    return std::all_of(key.begin(), key.end(), [](unsigned char ch) {
        return std::isalnum(ch) || ch == '.' || ch == '_' || ch == '-';
    });
}

bool ParsePayload(std::string_view payload,
                  std::vector<Entry>* entries,
                  std::string* error) {
    std::unordered_set<std::string> keys{
        "schema", "schema_version", "report_level", "value_encoding"};
    std::istringstream input{std::string(payload)};
    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(input, line)) {
        ++line_number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        const std::size_t separator = line.find('=');
        if (separator == std::string::npos) {
            SetError(error, "report line " + std::to_string(line_number) +
                                " has no '=' separator");
            return false;
        }
        Entry entry{line.substr(0U, separator), line.substr(separator + 1U)};
        if (!IsValidKey(entry.key)) {
            SetError(error, "report line " + std::to_string(line_number) +
                                " has an invalid key");
            return false;
        }
        if (!keys.insert(entry.key).second) {
            SetError(error, "duplicate report key: " + entry.key);
            return false;
        }
        std::string decoded;
        std::string value_error;
        if (!UnescapeReportValue(entry.value, &decoded, &value_error)) {
            SetError(error, "invalid escaped value for " + entry.key +
                                ": " + value_error);
            return false;
        }
        entries->push_back(std::move(entry));
    }
    if (entries->empty()) {
        SetError(error, "diagnostic report payload is empty");
        return false;
    }
    return true;
}

bool Enabled(const std::unordered_map<std::string, std::string>& values,
             std::string_view key,
             bool default_value) {
    const auto found = values.find(std::string(key));
    if (found == values.end()) return default_value;
    return found->second == "1" || Lower(found->second) == "true";
}

bool IsDisabledModuleField(
    std::string_view key,
    const std::unordered_map<std::string, std::string>& values) {
    const bool audio = Enabled(values, "module.audio.enabled", false);
    const bool audio_encoder =
        Enabled(values, "module.audio_encoder.enabled", false);
    const bool recorder = Enabled(values, "module.recorder.enabled", false);
    const bool network = Enabled(values, "module.network.enabled", false);
    const bool telemetry = Enabled(values, "module.telemetry.enabled", false);
    const bool uart = Enabled(values, "module.uart.enabled", false);

    if (!audio &&
        ((StartsWith(key, "audio_") && key != "audio_enabled" &&
          key != "audio_path_ok" && key != "audio_encode_path_ok") ||
         StartsWith(key, "encoded_audio_") ||
         StartsWith(key, "media_audio_") ||
         StartsWith(key, "queue.audio"))) {
        return true;
    }
    if (!audio_encoder &&
        ((StartsWith(key, "audio_encode_") &&
          key != "audio_encode_path_ok") ||
         StartsWith(key, "audio_encoder_") ||
         StartsWith(key, "audio_encoded_") ||
         StartsWith(key, "encoded_audio_") ||
         StartsWith(key, "queue.audio_encoded"))) {
        return true;
    }
    if (!recorder && StartsWith(key, "local_av_") &&
        key != "local_av_mux_enabled" && key != "local_av_mux_ok") {
        return true;
    }
    if (!network && StartsWith(key, "network_") &&
        key != "network_mux_enabled" && key != "network_mux_ok") {
        return true;
    }
    if (!network && StartsWith(key, "queue.network")) return true;
    if (!telemetry && StartsWith(key, "telemetry_") &&
        key != "telemetry_enabled" && key != "telemetry_ok") {
        return true;
    }
    if (!uart && (StartsWith(key, "uart.") || StartsWith(key, "uart_"))) {
        return true;
    }
    const auto topology = values.find("topology");
    if (topology != values.end() &&
        (topology->second == "fused_npu_postprocess" ||
         topology->second == "fused") &&
        StartsWith(key, "queue.completed")) {
        return true;
    }
    return false;
}

bool IsSummaryField(std::string_view key) noexcept {
    if (StartsWith(key, "module.") || StartsWith(key, "log.")) return true;
    if (StartsWith(key, "queue.") &&
        (EndsWith(key, ".current_size") ||
         EndsWith(key, ".replaced_oldest") ||
         EndsWith(key, ".stopped"))) {
        return true;
    }
    if (key == "topology" || key == "requested_duration_seconds" ||
        key == "observed_duration_seconds" || key == "control_backend" ||
        key == "captured_frames" || key == "inference_successes" ||
        key == "postprocess_successes" || key == "video_frames_encoded" ||
        key == "h265_bytes_written" || key == "result") {
        return true;
    }
    return Contains(key, "completed") || Contains(key, "terminated") ||
           Contains(key, "fatal") || Contains(key, "failure") ||
           Contains(key, "failures") || Contains(key, "dropped") ||
           Contains(key, "outstanding") || Contains(key, "_ok") ||
           Contains(key, "errors") || Contains(key, "replaced_oldest") ||
           Contains(key, "overwritten") ||
           key == "auxiliary_media_runtime_fault";
}

bool IsDiagnosticOnlyField(std::string_view key) noexcept {
    static constexpr std::string_view prefixes[] = {
        "camera_requested_", "camera_configured_", "camera_isp_",
        "model_input_", "state_count.",
    };
    for (const std::string_view prefix : prefixes) {
        if (StartsWith(key, prefix)) return true;
    }
    return key == "camera_sensor_subdev" ||
           key == "media_epoch_monotonic_ns" || key == "audio_device" ||
           key == "audio_period_frames" || key == "audio_buffer_frames" ||
           key == "audio_timestamp_type" || key == "local_av_output" ||
           key == "network_url" || key == "network_io_timeout_ms" ||
           key == "network_burst_bits" || key == "network_packet_size" ||
           key == "network_send_buffer_bytes" || key == "telemetry_host" ||
           key == "telemetry_port" || key == "telemetry_interval_ms" ||
           key == "telemetry_send_buffer_bytes" || key == "uart_device" ||
           key == "uart_baud" || key == "uart_ready_timeout_ms" ||
           key == "input_slots" || key == "output_slots" ||
           key == "latest_frame_queue_capacity" || key == "acquire_hits" ||
           key == "lost_misses" || key == "max_result_age_ms" ||
           key == "uart.peer_boot_id";
}

bool ShouldEmit(std::string_view key, ReportLevel level) noexcept {
    if (level == ReportLevel::DIAGNOSTIC) return true;
    if (level == ReportLevel::SUMMARY) return IsSummaryField(key);
    return !IsDiagnosticOnlyField(key);
}

int HexDigit(unsigned char value) noexcept {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

}  // namespace

const char* ReportLevelName(ReportLevel level) noexcept {
    switch (level) {
        case ReportLevel::SUMMARY: return "summary";
        case ReportLevel::PERFORMANCE: return "performance";
        case ReportLevel::DIAGNOSTIC: return "diagnostic";
    }
    return "unknown";
}

bool ParseReportLevel(std::string_view text, ReportLevel* level) noexcept {
    if (level == nullptr) return false;
    try {
        const std::string lower = Lower(text);
        if (lower == "summary") *level = ReportLevel::SUMMARY;
        else if (lower == "performance") *level = ReportLevel::PERFORMANCE;
        else if (lower == "diagnostic") *level = ReportLevel::DIAGNOSTIC;
        else return false;
        return true;
    } catch (...) {
        return false;
    }
}

std::string EscapeReportValue(std::string_view value) {
    std::ostringstream output;
    output << std::hex << std::uppercase << std::setfill('0');
    for (const unsigned char ch : value) {
        switch (ch) {
            case '\\': output << "\\\\"; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            case '=': output << "\\="; break;
            default:
                if (ch < 0x20U || ch == 0x7FU) {
                    output << "\\x" << std::setw(2)
                           << static_cast<unsigned int>(ch);
                } else {
                    output << static_cast<char>(ch);
                }
                break;
        }
    }
    return output.str();
}

bool UnescapeReportValue(std::string_view encoded,
                         std::string* value,
                         std::string* error) noexcept {
    if (value == nullptr) {
        SetError(error, "unescaped report value is null");
        return false;
    }
    try {
        std::string decoded;
        decoded.reserve(encoded.size());
        for (std::size_t index = 0U; index < encoded.size(); ++index) {
            const char ch = encoded[index];
            if (ch != '\\') {
                decoded.push_back(ch);
                continue;
            }
            if (++index >= encoded.size()) {
                SetError(error, "trailing backslash in report value");
                return false;
            }
            const char escape = encoded[index];
            if (escape == '\\' || escape == '=') decoded.push_back(escape);
            else if (escape == 'n') decoded.push_back('\n');
            else if (escape == 'r') decoded.push_back('\r');
            else if (escape == 't') decoded.push_back('\t');
            else if (escape == 'x') {
                if (index + 2U >= encoded.size()) {
                    SetError(error, "short hexadecimal report escape");
                    return false;
                }
                const int high = HexDigit(
                    static_cast<unsigned char>(encoded[index + 1U]));
                const int low = HexDigit(
                    static_cast<unsigned char>(encoded[index + 2U]));
                if (high < 0 || low < 0) {
                    SetError(error, "invalid hexadecimal report escape");
                    return false;
                }
                decoded.push_back(static_cast<char>((high << 4) | low));
                index += 2U;
            } else {
                SetError(error, "unknown report value escape");
                return false;
            }
        }
        *value = std::move(decoded);
        return true;
    } catch (const std::exception& exception) {
        SetError(error, exception.what());
        return false;
    } catch (...) {
        SetError(error, "unknown report unescape failure");
        return false;
    }
}

bool WriteRuntimeReport(std::string_view diagnostic_payload,
                        ReportLevel level,
                        std::ostream& output,
                        std::string* error) noexcept {
    try {
        std::vector<Entry> entries;
        if (!ParsePayload(diagnostic_payload, &entries, error)) return false;

        std::unordered_map<std::string, std::string> values;
        values.reserve(entries.size());
        for (const Entry& entry : entries) values.emplace(entry.key, entry.value);

        output << "schema=visionarm.runtime_report.v1\n"
               << "schema_version=1\n"
               << "report_level=" << ReportLevelName(level) << '\n'
               << "value_encoding=backslash-v1\n";
        for (const Entry& entry : entries) {
            if (IsDisabledModuleField(entry.key, values) ||
                !ShouldEmit(entry.key, level)) {
                continue;
            }
            output << entry.key << '=' << entry.value << '\n';
        }
        output.flush();
        if (!output.good()) {
            SetError(error, "report stream write or flush failed");
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        SetError(error, exception.what());
        return false;
    } catch (...) {
        SetError(error, "unknown report serialization failure");
        return false;
    }
}

bool WriteRuntimeReportFile(const std::string& path,
                            std::string_view diagnostic_payload,
                            ReportLevel level,
                            std::string* error) noexcept {
    try {
        std::ofstream output(path, std::ios::out | std::ios::trunc |
                                      std::ios::binary);
        if (!output.is_open()) {
            SetError(error, "failed to open report file: " + path);
            return false;
        }
        if (!WriteRuntimeReport(diagnostic_payload, level, output, error)) {
            return false;
        }
        output.close();
        if (output.fail()) {
            SetError(error, "failed to close report file: " + path);
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        SetError(error, exception.what());
        return false;
    } catch (...) {
        SetError(error, "unknown report file failure");
        return false;
    }
}

}  // namespace visionarm::report

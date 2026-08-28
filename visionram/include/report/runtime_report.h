#pragma once

#include <iosfwd>
#include <string>
#include <string_view>

namespace visionarm::report {

enum class ReportLevel {
    SUMMARY = 0,
    PERFORMANCE = 1,
    DIAGNOSTIC = 2,
};

[[nodiscard]] const char* ReportLevelName(ReportLevel level) noexcept;
[[nodiscard]] bool ParseReportLevel(std::string_view text,
                                    ReportLevel* level) noexcept;

// key=value schema v1 escaping. Values are one physical line. Backslash,
// newline, carriage return, tab, '=' and other ASCII controls are escaped.
[[nodiscard]] std::string EscapeReportValue(std::string_view value);
[[nodiscard]] bool UnescapeReportValue(std::string_view encoded,
                                       std::string* value,
                                       std::string* error = nullptr) noexcept;

// The payload is the existing diagnostic key=value stream. This function
// validates unique keys, applies report-level and enabled-module filtering,
// prepends the schema header, flushes the stream, and verifies stream state.
[[nodiscard]] bool WriteRuntimeReport(
    std::string_view diagnostic_payload,
    ReportLevel level,
    std::ostream& output,
    std::string* error = nullptr) noexcept;

[[nodiscard]] bool WriteRuntimeReportFile(
    const std::string& path,
    std::string_view diagnostic_payload,
    ReportLevel level,
    std::string* error = nullptr) noexcept;

}  // namespace visionarm::report

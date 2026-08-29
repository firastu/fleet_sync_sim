#include "fleet/scenario/trace.hpp"

#include <format>
#include <ostream>
#include <variant>

namespace fleet::scenario {

namespace {

std::string format_trace_value_impl(const TraceValue& value) {
    if (const std::string* text = std::get_if<std::string>(&value)) {
        return *text;
    }
    if (const std::int64_t* number = std::get_if<std::int64_t>(&value)) {
        return std::format("{}", *number);
    }
    if (const double* real = std::get_if<double>(&value)) {
        return std::format("{:.2f}", *real);
    }
    return std::get<bool>(value) ? "true" : "false";
}

// Minimal JSON string escaping (quotes, backslash, control characters).
// Deterministic and dependency-free.
void write_json_string(std::ostream& out, const std::string& text) {
    out << '"';
    for (const char character : text) {
        switch (character) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(character) < 0x20) {
                    static constexpr char kHex[] = "0123456789abcdef";
                    out << "\\u00" << kHex[(character >> 4) & 0xF] << kHex[character & 0xF];
                } else {
                    out << character;
                }
        }
    }
    out << '"';
}

void write_json_value(std::ostream& out, const TraceValue& value) {
    if (const std::string* text = std::get_if<std::string>(&value)) {
        write_json_string(out, *text);
        return;
    }
    if (const std::int64_t* number = std::get_if<std::int64_t>(&value)) {
        out << *number;
        return;
    }
    if (const double* real = std::get_if<double>(&value)) {
        out << std::format("{:.2f}", *real);
        return;
    }
    out << (std::get<bool>(value) ? "true" : "false");
}

}  // namespace

std::string format_trace_value(const TraceValue& value) {
    return format_trace_value_impl(value);
}

ConsoleTraceSink::ConsoleTraceSink(std::ostream& out) : out_{out} {}

void ConsoleTraceSink::record(const TraceEvent& event) {
    out_ << std::format("[{}][{}] {}", event.at.value, event.source, event.type);
    for (const auto& [key, value] : event.fields) {
        out_ << ' ' << key << '=' << format_trace_value_impl(value);
    }
    out_ << '\n';
}

JsonlTraceSink::JsonlTraceSink(std::ostream& out) : out_{out} {}

void JsonlTraceSink::record(const TraceEvent& event) {
    out_ << "{\"t\":" << event.at.value << ",\"source\":";
    write_json_string(out_, event.source);
    out_ << ",\"type\":";
    write_json_string(out_, event.type);
    for (const auto& [key, value] : event.fields) {
        out_ << ',';
        write_json_string(out_, key);
        out_ << ':';
        write_json_value(out_, value);
    }
    out_ << "}\n";
}

}  // namespace fleet::scenario

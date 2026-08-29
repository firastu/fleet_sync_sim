#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "fleet/common/time.hpp"

namespace fleet::scenario {

// Payload value of a trace field. Kept small and closed: strings, integers,
// doubles and booleans cover every traceable fact without a generic JSON
// value type.
using TraceValue = std::variant<std::string, std::int64_t, double, bool>;

// One observable simulation event. THE observability artifact of
// FleetSyncSim (ADR-009): console output and machine-readable exports are
// both formatters over TraceEvent, never separate systems.
//
// Contract:
//   - `type` names are stable ("scenario", "seed", "route", "link_state",
//     "observation", "send", "delivery", "reconcile", "resynchronize");
//   - fields appear in insertion order — deterministic by construction;
//   - values are deterministic functions of (scenario, resolved seed):
//     no wall-clock timestamps, no pointer addresses, no unordered-container
//     iteration reaches the output.
struct TraceEvent {
    common::Tick at{};
    std::string source;  // "world" or a participant name
    std::string type;
    std::vector<std::pair<std::string, TraceValue>> fields;
};

// Observation only: sinks record events; they never influence behavior.
class TraceSink {
public:
    virtual ~TraceSink() = default;
    virtual void record(const TraceEvent& event) = 0;
};

// Human-readable formatter: [tick][source] type key=value ...
class ConsoleTraceSink : public TraceSink {
public:
    explicit ConsoleTraceSink(std::ostream& out);
    void record(const TraceEvent& event) override;

private:
    std::ostream& out_;
};

// Machine-readable formatter: one JSON object per line (JSONL).
class JsonlTraceSink : public TraceSink {
public:
    explicit JsonlTraceSink(std::ostream& out);
    void record(const TraceEvent& event) override;

private:
    std::ostream& out_;
};

// Serializes one field value the way the console sink prints it
// (strings raw, doubles with two decimals, bools true/false).
[[nodiscard]] std::string format_trace_value(const TraceValue& value);

}  // namespace fleet::scenario

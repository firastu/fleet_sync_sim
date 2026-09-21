#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "fleet/map/base_map.hpp"
#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/trace.hpp"

namespace fleet::isaac {

// Isaac stage 1 read-only replay export (ADR-020). APP-INTERNAL adapter
// code: nothing here belongs to the domain libraries, no simulator type
// appears in it, and the exporter runs on CPU only.
struct ExportOptions {
    std::uint64_t resolved_seed = 0;

    // Snapshot domain: ticks 0..duration_ms inclusive, every period_ms ticks
    // (the horizon must be a multiple of the period — misalignment is
    // rejected, never rounded).
    std::uint64_t period_ms = 100;

    // Geographic anchor recorded in the manifest. Defaults are the pinned
    // experiment origin (52.370, 9.730); changing them is adapter
    // configuration, not new domain behavior.
    double origin_latitude_deg = 52.370;
    double origin_longitude_deg = 9.730;

    // Every map node (therefore every exported pose) must lie within this
    // radius of the origin; the observed extent is recorded in the manifest.
    double footprint_radius_m = 50.0;

    std::filesystem::path output_dir;

    // The exact text the scenario was parsed from; hashed into the manifest.
    std::string scenario_json;

    // Normal trace sinks attached to the run, exactly like fleet_sim: the
    // export must not change what a plain run would emit (ADR-020).
    std::vector<scenario::TraceSink*> sinks;
};

struct ExportSummary {
    std::uint64_t first_tick_ms = 0;
    std::uint64_t last_tick_ms = 0;
    std::uint64_t snapshot_count = 0;
    std::uint64_t robot_count = 0;
    double observed_extent_m = 0.0;
    std::filesystem::path manifest_path;
    std::filesystem::path poses_path;
};

// Executes the scenario through the public ScenarioRunner API (begin once,
// then ordered run_until stepping — contractually identical to one-shot
// execution) and writes manifest.json + poses.jsonl into output_dir.
//
// Observation-only: consumes no randomness, mutates no participant state.
// Throws std::invalid_argument for missing map geometry, a missing node
// coordinate, footprint violations, or a misaligned snapshot period — a
// coordinate is never manufactured or zero-filled.
//
// Deterministic: the same (base, scenario, options) produce byte-identical
// output files on the same toolchain.
[[nodiscard]] ExportSummary export_replay_run(const map::BaseMap& base,
                                              const scenario::Scenario& scenario,
                                              const ExportOptions& options);

// FNV-1a 64-bit correlation hashes for the manifest (ADR-020): they identify
// inputs across tooling; they are not integrity checksums and not
// cryptographic. map_hash serializes the immutable topology + geographic
// side canonically (id order, hexfloat doubles); bytes_hash hashes the exact
// scenario-file bytes.
[[nodiscard]] std::uint64_t map_hash(const map::BaseMap& base);
[[nodiscard]] std::uint64_t bytes_hash(std::string_view bytes);

}  // namespace fleet::isaac

#pragma once

#include <compare>
#include <cstddef>
#include <functional>

namespace fleet::common {

// Tagged strong type: wraps a scalar representation so that distinct
// identifier domains cannot be mixed accidentally (e.g. a NodeId passed
// where an EdgeId is expected, or a version compared against a sequence
// number). Zero overhead over the representation; trivially copyable;
// ordered; hashable where the representation is.
//
// The default-constructed value is the zero value. Like raw indices, ids are
// only meaningful when obtained from the component that issued them.
template <typename Tag, typename Rep>
class StrongType {
public:
    constexpr StrongType() noexcept = default;

    constexpr explicit StrongType(Rep value) noexcept : value_{value} {}

    [[nodiscard]] constexpr Rep value() const noexcept { return value_; }

    constexpr auto operator<=>(const StrongType&) const noexcept = default;

private:
    Rep value_{0};
};

}  // namespace fleet::common

// Hash support so strongly-typed ids can key exact-lookup hash containers.
// Containers keyed by these types must use them for exact lookup only —
// iteration order must never influence behavior (ADR-002 determinism).
namespace std {
template <typename Tag, typename Rep>
struct hash<fleet::common::StrongType<Tag, Rep>> {
    std::size_t operator()(const fleet::common::StrongType<Tag, Rep>& id) const noexcept {
        return std::hash<Rep>{}(id.value());
    }
};
}  // namespace std

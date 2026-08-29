#pragma once

#include <compare>

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

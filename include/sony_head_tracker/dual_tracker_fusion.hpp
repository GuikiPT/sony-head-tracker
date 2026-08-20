// dual_tracker_fusion.hpp
// Optional two-stream orientation fusion for earbuds/headsets that expose two
// independent Android Head Tracker HID collections.
#pragma once

#include "sony_head_tracker/types.hpp"

#include <chrono>
#include <cstddef>
#include <mutex>
#include <optional>

namespace sony {

// Combines a primary orientation stream with a secondary stream without adding
// a wait/buffering stage. Secondary samples are cached; every primary sample is
// emitted immediately, fused only when the secondary sample is recent enough.
//
// The secondary sensor may be mounted at a different fixed angle (for example,
// left/right earbuds). The first synchronized pair estimates that fixed local
// mounting transform and subsequent secondary quaternions are aligned into the
// primary sensor frame before SLERP averaging.
class DualTrackerFusion {
public:
    explicit DualTrackerFusion(std::chrono::milliseconds maxSkew = std::chrono::milliseconds{35});

    // source 0 is the primary clock/output stream; source 1 is cached as the
    // secondary measurement. Invalid source indexes are ignored.
    std::optional<MotionSample> push(std::size_t source, MotionSample sample);

    void reset();
    [[nodiscard]] bool secondaryFresh() const;

private:
    std::chrono::milliseconds maxSkew_;
    mutable std::mutex mutex_;
    std::optional<MotionSample> secondary_;
    std::optional<Quaternion> secondaryAlignment_;
    std::optional<std::uint8_t> primaryResetCounter_;
    std::optional<std::uint8_t> secondaryResetCounter_;
    bool secondaryFresh_{};
};

} // namespace sony

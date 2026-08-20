// dual_tracker_fusion.cpp
#include "sony_head_tracker/dual_tracker_fusion.hpp"

#include "sony_head_tracker/math.hpp"

#include <chrono>
#include <utility>

namespace sony {

namespace {

std::chrono::steady_clock::duration absoluteDifference(
    std::chrono::steady_clock::time_point a,
    std::chrono::steady_clock::time_point b) {
    return a >= b ? a - b : b - a;
}

} // namespace

DualTrackerFusion::DualTrackerFusion(std::chrono::milliseconds maxSkew)
    : maxSkew_(maxSkew) {}

std::optional<MotionSample> DualTrackerFusion::push(std::size_t source, MotionSample sample) {
    std::scoped_lock lock(mutex_);

    if (source > 1) return std::nullopt;

    if (source == 1) {
        if (secondaryResetCounter_ && *secondaryResetCounter_ != sample.resetCounter) {
            secondaryAlignment_.reset();
        }
        secondaryResetCounter_ = sample.resetCounter;
        secondary_ = std::move(sample);
        return std::nullopt;
    }

    if (primaryResetCounter_ && *primaryResetCounter_ != sample.resetCounter) {
        // A device-side recenter/reset changes the primary reference frame, so
        // the fixed secondary->primary calibration must be learned again.
        secondaryAlignment_.reset();
    }
    primaryResetCounter_ = sample.resetCounter;
    secondaryFresh_ = false;

    if (!secondary_) return sample;

    const auto skew = absoluteDifference(sample.receivedAt, secondary_->receivedAt);
    if (skew > maxSkew_) return sample;

    const auto primaryQ = rotationVectorToQuaternion(sample.rotationVector);
    const auto secondaryQ = rotationVectorToQuaternion(secondary_->rotationVector);

    if (!secondaryAlignment_) {
        // If qPrimary = qHead * mountPrimary and qSecondary = qHead * mountSecondary,
        // then conjugate(qSecondary) * qPrimary is the constant local transform
        // from the secondary sensor frame into the primary sensor frame.
        secondaryAlignment_ = multiply(conjugate(secondaryQ), primaryQ);
    }

    const auto alignedSecondary = multiply(secondaryQ, *secondaryAlignment_);
    const auto fused = slerp(primaryQ, alignedSecondary, 0.5);
    sample.rotationVector = quaternionToRotationVector(fused);

    // Gyro/accelerometer vectors remain from the primary stream. They are
    // sensor-local vectors and should not be naively averaged until their exact
    // frame conventions are verified on hardware.
    secondaryFresh_ = true;
    return sample;
}

void DualTrackerFusion::reset() {
    std::scoped_lock lock(mutex_);
    secondary_.reset();
    secondaryAlignment_.reset();
    primaryResetCounter_.reset();
    secondaryResetCounter_.reset();
    secondaryFresh_ = false;
}

bool DualTrackerFusion::secondaryFresh() const {
    std::scoped_lock lock(mutex_);
    return secondaryFresh_;
}

} // namespace sony

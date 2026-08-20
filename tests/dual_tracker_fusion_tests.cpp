// dual_tracker_fusion_tests.cpp
#include "test_framework.hpp"

#include "sony_head_tracker/dual_tracker_fusion.hpp"
#include "sony_head_tracker/math.hpp"

#include <chrono>
#include <cmath>

using namespace sony;

namespace {

MotionSample sampleAt(Quaternion q, int milliseconds, std::uint8_t resetCounter = 0) {
    MotionSample s;
    s.rotationVector = quaternionToRotationVector(q);
    s.receivedAt = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds{milliseconds};
    s.resetCounter = resetCounter;
    return s;
}

MotionSample yawSample(double radians, int milliseconds, std::uint8_t resetCounter = 0) {
    return sampleAt(rotationVectorToQuaternion({0.0, 0.0, radians}), milliseconds, resetCounter);
}

} // namespace

TEST(dual_fusion_primary_passes_through_without_secondary) {
    DualTrackerFusion fusion;
    auto out = fusion.push(0, yawSample(0.25, 10));
    CHECK(out.has_value());
    CHECK_NEAR(out->rotationVector.z, 0.25, 1e-6);
    CHECK(!fusion.secondaryFresh());
}

TEST(dual_fusion_secondary_does_not_emit_by_itself) {
    DualTrackerFusion fusion;
    auto out = fusion.push(1, yawSample(0.25, 10));
    CHECK(!out.has_value());
}

TEST(dual_fusion_reduces_symmetric_orientation_noise) {
    DualTrackerFusion fusion;

    // Establish the common reference frame.
    fusion.push(1, yawSample(0.0, 0));
    auto first = fusion.push(0, yawSample(0.0, 0));
    CHECK(first.has_value());

    // Equal-and-opposite independent noise should mostly cancel.
    fusion.push(1, yawSample(0.02, 10));
    auto out = fusion.push(0, yawSample(-0.02, 10));
    CHECK(out.has_value());
    CHECK(std::fabs(out->rotationVector.z) < 1e-4);
    CHECK(fusion.secondaryFresh());
}

TEST(dual_fusion_compensates_fixed_secondary_mounting_offset) {
    DualTrackerFusion fusion;
    const auto mountOffset = rotationVectorToQuaternion({0.18, -0.11, 0.07});

    // At calibration time the secondary sensor is physically mounted at a fixed
    // angle relative to the primary.
    fusion.push(1, sampleAt(mountOffset, 0));
    auto first = fusion.push(0, sampleAt(Quaternion{}, 0));
    CHECK(first.has_value());
    CHECK_NEAR(first->rotationVector.x, 0.0, 1e-6);
    CHECK_NEAR(first->rotationVector.y, 0.0, 1e-6);
    CHECK_NEAR(first->rotationVector.z, 0.0, 1e-6);

    // Both sensors then undergo the same head motion. Secondary remains
    // head-motion * mounting-offset; fusion should align it back to primary.
    const auto head = rotationVectorToQuaternion({0.0, 0.0, 0.45});
    const auto secondary = multiply(head, mountOffset);
    fusion.push(1, sampleAt(secondary, 10));
    auto out = fusion.push(0, sampleAt(head, 10));
    CHECK(out.has_value());
    CHECK_NEAR(out->rotationVector.x, 0.0, 1e-5);
    CHECK_NEAR(out->rotationVector.y, 0.0, 1e-5);
    CHECK_NEAR(out->rotationVector.z, 0.45, 1e-5);
}

TEST(dual_fusion_ignores_stale_secondary_sample) {
    DualTrackerFusion fusion(std::chrono::milliseconds{20});
    fusion.push(1, yawSample(0.8, 0));
    auto out = fusion.push(0, yawSample(0.2, 100));
    CHECK(out.has_value());
    CHECK_NEAR(out->rotationVector.z, 0.2, 1e-6);
    CHECK(!fusion.secondaryFresh());
}

TEST(dual_fusion_relearns_alignment_after_primary_reset) {
    DualTrackerFusion fusion;
    const auto offset = rotationVectorToQuaternion({0.2, 0.0, 0.0});

    fusion.push(1, sampleAt(offset, 0, 1));
    fusion.push(0, sampleAt(Quaternion{}, 0, 1));

    const auto newPrimary = rotationVectorToQuaternion({0.0, 0.3, 0.0});
    const auto newSecondary = multiply(newPrimary, offset);
    fusion.push(1, sampleAt(newSecondary, 10, 1));
    auto beforeReset = fusion.push(0, sampleAt(newPrimary, 10, 1));
    CHECK(beforeReset.has_value());
    CHECK_NEAR(beforeReset->rotationVector.y, 0.3, 1e-5);

    // Changing the primary reset counter invalidates the previous frame mapping.
    const auto rebasedPrimary = rotationVectorToQuaternion({0.0, 0.0, -0.35});
    const auto rebasedSecondary = multiply(rebasedPrimary, offset);
    fusion.push(1, sampleAt(rebasedSecondary, 20, 1));
    auto afterReset = fusion.push(0, sampleAt(rebasedPrimary, 20, 2));
    CHECK(afterReset.has_value());
    CHECK_NEAR(afterReset->rotationVector.z, -0.35, 1e-5);
}

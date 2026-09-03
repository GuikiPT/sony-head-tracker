// orientation_tests.cpp
// OrientationFilter behaviour: recenter, smoothing attenuation, axis mapping, and
// graceful handling of samples with no angular velocity.
#include "test_framework.hpp"

#include "sony_head_tracker/math.hpp"
#include "sony_head_tracker/orientation.hpp"

#include <array>
#include <cmath>
#include <numbers>

using namespace sony;

static FilterConfig identityConfig(double smoothing) {
    FilterConfig cfg;
    cfg.axes = AxisMapping{{0, 1, 2}, {1.0, 1.0, 1.0}};   // no remap/inversion
    cfg.smoothing = smoothing;
    cfg.driftCorrectionPerSecond = 0.0;                    // isolate smoothing from drift
    return cfg;
}

static MotionSample rot(Vec3 v) { MotionSample s; s.rotationVector = v; return s; }

static Quaternion orientation(double yaw, double pitch, double roll) {
    return multiply(rotationVectorToQuaternion({0, 0, yaw}),
                    multiply(rotationVectorToQuaternion({0, pitch, 0}),
                             rotationVectorToQuaternion({roll, 0, 0})));
}

static MotionSample quaternionSample(Quaternion q, std::uint8_t resetCounter = 0) {
    MotionSample s;
    s.rotationVector = quaternionToRotationVector(q);
    s.resetCounter = resetCounter;
    return s;
}

static MotionSample relativeTo(Quaternion center, Quaternion motion, std::uint8_t resetCounter = 0) {
    // A physical movement around a head-local axis composes after the absolute pose.
    return quaternionSample(multiply(center, motion), resetCounter);
}

static void checkPurePitch(const MotionSample& sample) {
    CHECK(std::fabs(sample.euler.pitch) > 5.0);
    CHECK_NEAR(sample.euler.yaw, 0.0, 1e-6);
    CHECK_NEAR(sample.euler.roll, 0.0, 1e-6);
}

static void checkPureRoll(const MotionSample& sample) {
    CHECK(std::fabs(sample.euler.roll) > 5.0);
    CHECK_NEAR(sample.euler.yaw, 0.0, 1e-6);
    CHECK_NEAR(sample.euler.pitch, 0.0, 1e-6);
}

static void checkPureYaw(const MotionSample& sample) {
    CHECK(std::fabs(sample.euler.yaw) > 5.0);
    CHECK_NEAR(sample.euler.pitch, 0.0, 1e-6);
    CHECK_NEAR(sample.euler.roll, 0.0, 1e-6);
}

TEST(recenter_zeroes_output_for_the_held_pose) {
    OrientationFilter f(identityConfig(1.0));
    f.process(rot({0, 0, 0}));                     // first sample becomes the center
    const auto moved = f.process(rot({0.3, 0, 0}));
    CHECK(std::fabs(moved.euler.roll) > 1.0);      // rotation about X shows up as roll

    f.recenter();
    const auto after = f.process(rot({0.3, 0, 0}));   // recenter onto the current pose
    CHECK_NEAR(after.euler.yaw, 0.0, 1e-6);
    CHECK_NEAR(after.euler.pitch, 0.0, 1e-6);
    CHECK_NEAR(after.euler.roll, 0.0, 1e-6);
}

TEST(smoothing_attenuates_a_single_step) {
    OrientationFilter fast(identityConfig(1.0));   // follows instantly
    OrientationFilter slow(identityConfig(0.1));   // heavily smoothed
    fast.process(rot({0, 0, 0}));
    slow.process(rot({0, 0, 0}));

    const auto fastOut = fast.process(rot({0, 0, 0.5}));   // rotation about Z -> yaw
    const auto slowOut = slow.process(rot({0, 0, 0.5}));

    CHECK(std::fabs(slowOut.euler.yaw) < std::fabs(fastOut.euler.yaw));
    constexpr double deg = 180.0 / std::numbers::pi;
    CHECK_NEAR(fastOut.euler.yaw, 0.5 * deg, 0.5);         // ~28.6 degrees
}

TEST(axis_remap_applies_to_angular_velocity) {
    OrientationFilter f(identityConfig(1.0));
    MotionSample s = rot({0, 0, 0});
    s.angularVelocity = Vec3{1.0, 2.0, 3.0};
    const auto out = f.process(std::move(s));
    CHECK(out.angularVelocity.has_value());
    CHECK_NEAR(out.angularVelocity->x, 1.0, 1e-9);         // identity mapping preserves it
    CHECK_NEAR(out.angularVelocity->y, 2.0, 1e-9);
    CHECK_NEAR(out.angularVelocity->z, 3.0, 1e-9);
}

TEST(missing_angular_velocity_stays_missing_and_is_stable) {
    OrientationFilter f;   // default config
    const auto out = f.process(rot({0.1, 0.1, 0.1}));       // no angularVelocity set
    CHECK(!out.angularVelocity.has_value());                // the filter never fabricates a gyro
    const auto out2 = f.process(rot({0.1, 0.1, 0.1}));
    CHECK(std::isfinite(out2.euler.yaw));
    CHECK(std::isfinite(out2.euler.pitch));
    CHECK(std::isfinite(out2.euler.roll));
}

TEST(pure_nod_from_non_identity_reference_has_no_cross_axis_coupling) {
    OrientationFilter f(identityConfig(1.0));
    const auto center = orientation(40.0 * std::numbers::pi / 180.0,
                                    15.0 * std::numbers::pi / 180.0,
                                    -20.0 * std::numbers::pi / 180.0);
    f.process(quaternionSample(center));
    checkPurePitch(f.process(relativeTo(center, rotationVectorToQuaternion({0, 0.35, 0}))));
}

TEST(pure_tilt_from_non_identity_reference_has_no_cross_axis_coupling) {
    OrientationFilter f(identityConfig(1.0));
    const auto center = orientation(40.0 * std::numbers::pi / 180.0,
                                    15.0 * std::numbers::pi / 180.0,
                                    -20.0 * std::numbers::pi / 180.0);
    f.process(quaternionSample(center));
    checkPureRoll(f.process(relativeTo(center, rotationVectorToQuaternion({0.35, 0, 0}))));
}

TEST(pure_turn_from_non_identity_reference_has_no_cross_axis_coupling) {
    OrientationFilter f(identityConfig(1.0));
    const auto center = orientation(40.0 * std::numbers::pi / 180.0,
                                    15.0 * std::numbers::pi / 180.0,
                                    -20.0 * std::numbers::pi / 180.0);
    f.process(quaternionSample(center));
    checkPureYaw(f.process(relativeTo(center, rotationVectorToQuaternion({0, 0, 0.35}))));
}

TEST(centered_motion_is_independent_of_the_arbitrary_reference) {
    const std::array centers{
        Quaternion{},
        orientation(30.0 * std::numbers::pi / 180.0, -10.0 * std::numbers::pi / 180.0, 0),
        orientation(40.0 * std::numbers::pi / 180.0, 15.0 * std::numbers::pi / 180.0,
                    -20.0 * std::numbers::pi / 180.0),
    };
    const auto nod = rotationVectorToQuaternion({0, 0.35, 0});
    MotionSample expected{};
    for (const auto center : centers) {
        OrientationFilter f(identityConfig(1.0));
        f.process(quaternionSample(center));
        const auto out = f.process(relativeTo(center, nod));
        checkPurePitch(out);
        if (center.w == 1.0) expected = out;
        CHECK_NEAR(out.euler.yaw, expected.euler.yaw, 1e-6);
        CHECK_NEAR(out.euler.pitch, expected.euler.pitch, 1e-6);
        CHECK_NEAR(out.euler.roll, expected.euler.roll, 1e-6);
    }
}

TEST(recenter_at_an_arbitrary_pose_preserves_axis_independence) {
    OrientationFilter f(identityConfig(1.0));
    const auto initial = orientation(40.0 * std::numbers::pi / 180.0,
                                     15.0 * std::numbers::pi / 180.0,
                                     -20.0 * std::numbers::pi / 180.0);
    const auto newCenter = multiply(orientation(-20.0 * std::numbers::pi / 180.0,
                                                 25.0 * std::numbers::pi / 180.0,
                                                 30.0 * std::numbers::pi / 180.0),
                                     initial);
    f.process(quaternionSample(initial));
    f.process(quaternionSample(newCenter));
    f.recenter();
    const auto recentered = f.process(quaternionSample(newCenter));
    CHECK_NEAR(recentered.euler.yaw, 0.0, 1e-6);
    CHECK_NEAR(recentered.euler.pitch, 0.0, 1e-6);
    CHECK_NEAR(recentered.euler.roll, 0.0, 1e-6);
    checkPurePitch(f.process(relativeTo(newCenter, rotationVectorToQuaternion({0, 0.35, 0}))));
}

TEST(reset_counter_rebases_the_reference_and_clears_smoothing) {
    auto config = identityConfig(0.1);
    config.driftCorrectionPerSecond = 1.0;
    OrientationFilter f(config);
    const auto oldCenter = orientation(20.0 * std::numbers::pi / 180.0,
                                       -10.0 * std::numbers::pi / 180.0, 15.0 * std::numbers::pi / 180.0);
    const auto newCenter = orientation(-35.0 * std::numbers::pi / 180.0,
                                       25.0 * std::numbers::pi / 180.0, -30.0 * std::numbers::pi / 180.0);
    auto first = quaternionSample(oldCenter, 7);
    first.receivedAt = std::chrono::steady_clock::time_point{};
    f.process(first);
    auto moved = relativeTo(oldCenter, rotationVectorToQuaternion({0.4, 0, 0}), 7);
    moved.angularVelocity = Vec3{0.05, 0, 0};
    moved.receivedAt = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds{100};
    f.process(moved);
    for (int i = 2; i <= 101; ++i) {
        auto still = quaternionSample(oldCenter, 7);
        still.angularVelocity = Vec3{0.05, 0, 0};
        still.receivedAt = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds{i * 100};
        f.process(still);
    }
    auto reset = quaternionSample(newCenter, 8);
    reset.receivedAt = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds{10200};
    const auto rebased = f.process(reset);
    CHECK_NEAR(rebased.euler.yaw, 0.0, 1e-6);
    CHECK_NEAR(rebased.euler.pitch, 0.0, 1e-6);
    CHECK_NEAR(rebased.euler.roll, 0.0, 1e-6);

    OrientationFilter axisFilter(identityConfig(1.0));
    axisFilter.process(quaternionSample(oldCenter, 7));
    axisFilter.process(relativeTo(oldCenter, rotationVectorToQuaternion({0.4, 0, 0}), 7));
    axisFilter.process(quaternionSample(newCenter, 8));
    checkPurePitch(axisFilter.process(relativeTo(newCenter, rotationVectorToQuaternion({0, 0.35, 0}), 8)));
}

TEST(axis_mapping_is_independent_of_the_arbitrary_reference) {
    constexpr std::array<std::array<unsigned, 3>, 6> permutations{{
        {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}},
    }};
    const std::array identityAndComplexCenters{
        Quaternion{},
        orientation(40.0 * std::numbers::pi / 180.0, 15.0 * std::numbers::pi / 180.0,
                    -20.0 * std::numbers::pi / 180.0),
    };
    const Vec3 motion{0, 0.35, 0};
    const auto motionQuaternion = rotationVectorToQuaternion(motion);

    for (const auto& permutation : permutations) {
        for (const auto xSign : {-1.0, 1.0}) {
            for (const auto ySign : {-1.0, 1.0}) {
                for (const auto zSign : {-1.0, 1.0}) {
                    const AxisMapping mapping{permutation, {xSign, ySign, zSign}};
                    MotionSample expected{};
                    for (const auto center : identityAndComplexCenters) {
                        auto config = identityConfig(1.0);
                        config.axes = mapping;
                        OrientationFilter f(config);
                        f.process(quaternionSample(center));
                        const auto out = f.process(relativeTo(center, motionQuaternion));
                        if (center.w == 1.0) expected = out;
                        const auto mapped = remap(motion, mapping);
                        CHECK_NEAR(out.rotationVector.x, mapped.x, 1e-6);
                        CHECK_NEAR(out.rotationVector.y, mapped.y, 1e-6);
                        CHECK_NEAR(out.rotationVector.z, mapped.z, 1e-6);
                        CHECK_NEAR(out.rotationVector.x, expected.rotationVector.x, 1e-6);
                        CHECK_NEAR(out.rotationVector.y, expected.rotationVector.y, 1e-6);
                        CHECK_NEAR(out.rotationVector.z, expected.rotationVector.z, 1e-6);
                    }
                }
            }
        }
    }
}

#include <unity.h>

#define private public
#include "RPMESCController.hpp"
#undef private

RPMESCController esc(18, 0.0f, 12000.0f);

/**
 * @brief Tests if zero RPM maps to zero throttle.
 */
void test_zero_rpm_maps_to_zero_throttle() {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, esc.rpmToThrottle(0.0f));
}

/**
 * @brief Tests if half of the RPM range maps to 50% throttle.
 */
void test_half_rpm_maps_to_half_throttle() {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, esc.rpmToThrottle(6000.0f));
}

/**
 * @brief Tests if maximum RPM maps to 100% throttle.
 */
void test_max_rpm_maps_to_full_throttle() {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, esc.rpmToThrottle(12000.0f));
}

/**
 * @brief Tests if RPM values above the maximum are clamped.
 */
void test_rpm_above_max_is_clamped() {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, esc.rpmToThrottle(15000.0f));
}

/**
 * @brief Tests if RPM values below the minimum are clamped.
 */
void test_rpm_below_min_is_clamped() {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, esc.rpmToThrottle(-1000.0f));
}

/**
 * @brief Tests if zero throttle maps to the minimum pulse.
 */
void test_zero_throttle_maps_to_min_pulse() {
    TEST_ASSERT_EQUAL_UINT16(1000, esc.throttleToPulse(0.0f));
}

/**
 * @brief Tests if 50% throttle maps to the midpoint pulse.
 */
void test_half_throttle_maps_to_mid_pulse() {
    TEST_ASSERT_EQUAL_UINT16(1500, esc.throttleToPulse(50.0f));
}

/**
 * @brief Tests if full throttle maps to the maximum pulse.
 */
void test_full_throttle_maps_to_max_pulse() {
    TEST_ASSERT_EQUAL_UINT16(2000, esc.throttleToPulse(100.0f));
}

/**
 * @brief Tests if throttle values above 100% are clamped.
 */
void test_throttle_above_max_is_clamped() {
    TEST_ASSERT_EQUAL_UINT16(2000, esc.throttleToPulse(120.0f));
}

/**
 * @brief Tests if throttle values below 0% are clamped.
 */
void test_throttle_below_min_is_clamped() {
    TEST_ASSERT_EQUAL_UINT16(1000, esc.throttleToPulse(-20.0f));
}

/**
 * @brief Tests if 1000 us pulse maps to the expected duty cycle.
 */
void test_1000us_pulse_to_duty() {
    TEST_ASSERT_EQUAL_UINT32(3276, esc.pulseToDuty(1000));
}

/**
 * @brief Tests if 1500 us pulse maps to the expected duty cycle.
 */
void test_1500us_pulse_to_duty() {
    TEST_ASSERT_EQUAL_UINT32(4915, esc.pulseToDuty(1500));
}

/**
 * @brief Tests if 2000 us pulse maps to the expected duty cycle.
 */
void test_2000us_pulse_to_duty() {
    TEST_ASSERT_EQUAL_UINT32(6553, esc.pulseToDuty(2000));
}

/**
 * @brief Runs all unit tests.
 */
void setup() {
    UNITY_BEGIN();

    RUN_TEST(test_zero_rpm_maps_to_zero_throttle);
    RUN_TEST(test_half_rpm_maps_to_half_throttle);
    RUN_TEST(test_max_rpm_maps_to_full_throttle);
    RUN_TEST(test_rpm_above_max_is_clamped);
    RUN_TEST(test_rpm_below_min_is_clamped);

    RUN_TEST(test_zero_throttle_maps_to_min_pulse);
    RUN_TEST(test_half_throttle_maps_to_mid_pulse);
    RUN_TEST(test_full_throttle_maps_to_max_pulse);
    RUN_TEST(test_throttle_above_max_is_clamped);
    RUN_TEST(test_throttle_below_min_is_clamped);

    RUN_TEST(test_1000us_pulse_to_duty);
    RUN_TEST(test_1500us_pulse_to_duty);
    RUN_TEST(test_2000us_pulse_to_duty);

    UNITY_END();
}

void loop() {
}
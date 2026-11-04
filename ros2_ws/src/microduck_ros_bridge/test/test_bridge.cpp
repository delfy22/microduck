#include <limits>

#include <gtest/gtest.h>

#include "microduck_ros_bridge/helpers.hpp"

using microduck_ros_bridge::command_fresh;
using microduck_ros_bridge::monotonic_to_ros_ns;
using microduck_ros_bridge::tof_valid_return;
using microduck_ros_bridge::validated_command;

TEST(BridgeHelpers, ClampsForwardAndYawAndDropsLateralMotion) {
  const auto command = validated_command(2.0, 0.4, -3.0, 0.2, 1.0);
  ASSERT_TRUE(command);
  EXPECT_DOUBLE_EQ(command->vx, 0.2);
  EXPECT_DOUBLE_EQ(command->vyaw, -1.0);
}

TEST(BridgeHelpers, RejectsNonFiniteAndMalformedCommands) {
  EXPECT_FALSE(validated_command(std::numeric_limits<double>::quiet_NaN(), 0.0,
                                 0.0, 0.2, 1.0));
}

TEST(BridgeHelpers, FreshnessRejectsExpiredFutureAndInvalidStamps) {
  EXPECT_TRUE(command_fresh(0.2, 0.25));
  EXPECT_FALSE(command_fresh(0.26, 0.25));
  EXPECT_FALSE(command_fresh(-0.001, 0.25));
  EXPECT_FALSE(command_fresh(std::numeric_limits<double>::infinity(), 0.25));
}

TEST(BridgeHelpers, ToFStatusDistinguishesReturnsFromNoTargetAndFailures) {
  EXPECT_TRUE(tof_valid_return(5, 420));
  EXPECT_TRUE(tof_valid_return(9, 420));
  EXPECT_FALSE(tof_valid_return(5, -1));
  EXPECT_FALSE(tof_valid_return(255, 420));
  EXPECT_FALSE(tof_valid_return(4, 420));
}

TEST(BridgeHelpers, ConvertsMonotonicTimestampAndRejectsMissingOrOverflow) {
  EXPECT_EQ(monotonic_to_ros_ns(2'000'000'000ULL, 5'000'000'000LL),
            7'000'000'000LL);
  EXPECT_FALSE(monotonic_to_ros_ns(0, 5));
  EXPECT_FALSE(monotonic_to_ros_ns(2'000'000'000ULL, -3'000'000'000LL));
}

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "fake_naoqi/fake_almotion.hpp"

TEST(naoqi_driver, fake_almotion)
{
  naoqi::fake::FakeALMotion motion("nao");

  // Equivalent to the /joint_angles publish used by test_emulation.sh
  const std::vector<std::string> joint_names = {"HeadYaw", "HeadPitch"};
  const std::vector<float> joint_angles = {0.5f, 0.1f};
  const float speed = 0.1f;

  motion.setAngles(joint_names, joint_angles, speed);

  const auto body_names = motion.getBodyNames("Body");
  const auto body_angles = motion.getAngles("Body", true);

  ASSERT_EQ(body_names.size(), body_angles.size());
  ASSERT_GT(body_names.size(), 0u);

  const auto yaw_it = std::find(body_names.begin(), body_names.end(), "HeadYaw");
  const auto pitch_it = std::find(body_names.begin(), body_names.end(), "HeadPitch");

  ASSERT_NE(yaw_it, body_names.end());
  ASSERT_NE(pitch_it, body_names.end());

  const size_t yaw_idx = static_cast<size_t>(std::distance(body_names.begin(), yaw_it));
  const size_t pitch_idx = static_cast<size_t>(std::distance(body_names.begin(), pitch_it));

  EXPECT_NEAR(body_angles[yaw_idx], 0.5, 1e-6);
  EXPECT_NEAR(body_angles[pitch_idx], 0.1, 1e-6);
}

TEST(naoqi_driver, fake_almotion_duplicate_entries_last_wins)
{
  naoqi::fake::FakeALMotion motion("nao");

  // Data equivalent to the provided JointAnglesWithSpeed payload.
  // Note: duplicate entries exist; the last occurrence should take precedence.
  const std::vector<std::string> joint_names = {
      "LShoulderPitch",
      "HeadPitch",
      "RElbowYaw",
      "HeadYaw",
      "RWristYaw",
      "LElbowYaw",
      "LShoulderPitch",
      "RWristYaw",
      "RShoulderPitch",
      "LElbowYaw",
      "HeadPitch",
      "RElbowYaw",
      "HeadYaw",
      "RShoulderPitch",
  };

  const std::vector<float> joint_angles = {
      -0.8478882312774658f,
      0.5148720145225525f,
      -0.34572964906692505f,
      1.0476144552230835f,
      0.639968752861023f,
      0.7054705023765564f,
      -0.8478882312774658f,
      0.639968752861023f,
      -1.1386842727661133f,
      0.7054705023765564f,
      0.5148720145225525f,
      -0.34572964906692505f,
      1.0476144552230835f,
      -1.1386842727661133f,
  };

  ASSERT_EQ(joint_names.size(), joint_angles.size());

  const float speed = 0.30000001192092896f;
  motion.setAngles(joint_names, joint_angles, speed);

  // Build expected joint value map, enforcing "last occurrence wins".
  std::unordered_map<std::string, double> expected;
  expected.reserve(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    expected[joint_names[i]] = static_cast<double>(joint_angles[i]);
  }

  const auto body_names = motion.getBodyNames("Body");
  const auto body_angles = motion.getAngles("Body", true);
  ASSERT_EQ(body_names.size(), body_angles.size());
  ASSERT_GT(body_names.size(), 0u);

  for (const auto& kv : expected)
  {
    const auto it = std::find(body_names.begin(), body_names.end(), kv.first);
    ASSERT_NE(it, body_names.end()) << "Expected joint missing from Body: " << kv.first;
    const size_t idx = static_cast<size_t>(std::distance(body_names.begin(), it));
    EXPECT_NEAR(body_angles[idx], kv.second, 1e-6) << "Mismatch for joint: " << kv.first;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
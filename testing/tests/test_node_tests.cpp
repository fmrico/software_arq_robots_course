#include "testing/Duplicator.hpp"
#include <gtest/gtest.h>

class TestDuplicator : public testing::Duplicator
{
public:
  float duplicate(float value)
  {
    return Duplicator::duplicate(value);
  }
};


// Declare a test
TEST(Testing, tests_duplicate)
{
  TestDuplicator dup_test;

  ASSERT_NEAR(dup_test.duplicate(0.0f), 0.0f, 0.0000001f);
  ASSERT_NEAR(dup_test.duplicate(1.0f), 2.0f, 0.0000001f);
  ASSERT_NEAR(dup_test.duplicate(-4.0f), -8.0f, 0.0000001f);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
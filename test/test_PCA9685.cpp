#include <gtest/gtest.h>
#include <i2clib/PCA9685.hpp>

using namespace i2clib;

struct PCA9685Test : public ::testing::Test {
};

TEST_F(PCA9685Test, it_computes_prescaling_for_200Hz_as_described_in_the_documentation)
{
    ASSERT_EQ(PCA9685::periodToPrescale(5000000), 30);
}

TEST_F(PCA9685Test,
    it_computes_the_prescaling_for_the_maximum_frequency_as_described_in_the_documentation)
{
    ASSERT_EQ(PCA9685::periodToPrescale(655360), 0x03);
}

TEST_F(PCA9685Test,
    it_computes_the_prescaling_for_the_minimum_frequency_as_described_in_the_documentation)
{
    ASSERT_EQ(PCA9685::periodToPrescale(41666666), 0xfd);
}

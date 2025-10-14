#include <gtest/gtest.h>
#include <i2clib/MS5837.hpp>

using namespace i2clib;

struct MS5837Test : public ::testing::Test {
};

TEST_F(MS5837Test, it_performs_conversion_according_to_the_datasheet) {
    // The datasheet has an example conversion. This test makes sure our implementation
    // matches that example

    MS5837::PROM prom;
    prom.C[1] = 34982;
    prom.C[2] = 36352;
    prom.C[3] = 20328;
    prom.C[4] = 22354;
    prom.C[5] = 26646;
    prom.C[6] = 26146;

    int32_t raw_T = 6815414;
    int32_t raw_P = 4958179;

    auto [temperature, dT] = MS5837::compensateRawTemperature(raw_T, prom);
    auto pressure = MS5837::compensateRawPressure(raw_P, dT, prom);

    ASSERT_NEAR(19.81, temperature.getCelsius(), 1e-2);
    ASSERT_NEAR(3.9998, pressure.toBar(), 1e-4);
}
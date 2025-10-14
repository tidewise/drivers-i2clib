#include <gtest/gtest.h>
#include <i2clib/BMP280.hpp>

using namespace i2clib;

struct BMP280Test : public ::testing::Test {
};

TEST_F(BMP280Test, it_performs_conversion_according_to_the_datasheet) {
    // The datasheet has an example conversion. This test makes sure our implementation
    // matches that example

    BMP280::Calibration calibration;
    calibration.dig_T1 = 27504;
    calibration.dig_T2 = 26435;
    calibration.dig_T3 = -1000;
    calibration.dig_P1 = 36477;
    calibration.dig_P2 = -10685;
    calibration.dig_P3 = 3024;
    calibration.dig_P4 = 2855;
    calibration.dig_P5 = 140;
    calibration.dig_P6 = -7;
    calibration.dig_P7 = 15500;
    calibration.dig_P8 = -14600;
    calibration.dig_P9 = 6000;

    int32_t raw_T = 519888;
    int32_t raw_P = 415148;

    auto temperature = BMP280::compensate_T_int32(raw_T, calibration);
    auto pressure = BMP280::compensate_P_int32(raw_P, temperature.second, calibration);

    ASSERT_NEAR(25.08, temperature.first.getCelsius(), 1e-2);
    ASSERT_NEAR(100653, pressure.toPa(), 10);
}
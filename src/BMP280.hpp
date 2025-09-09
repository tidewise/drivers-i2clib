#ifndef I2CLIB_BMP280_HPP
#define I2CLIB_BMP280_HPP

#include <i2clib/BMP280Configuration.hpp>
#include <i2clib/BMP280Measurement.hpp>
#include <i2clib/I2CBus.hpp>

#include <cstdint>

namespace i2clib {
    /** Driver for Bosch's BMP280 i2c pressure sensor
     *
     * This is an _optionated_ driver. It does not implement all functions, only what
     * we deem relevant
     */
    class BMP280 {
    public:
        enum DeviceMode {
            MODE_SLEEP = 0,
            MODE_FORCED = 1,
            MODE_NORMAL = 3
        };

        using Configuration = BMP280Configuration;

        struct Calibration {
            uint16_t dig_P1 = 0;
            int16_t dig_P2 = 0;
            int16_t dig_P3 = 0;
            int16_t dig_P4 = 0;
            int16_t dig_P5 = 0;
            int16_t dig_P6 = 0;
            int16_t dig_P7 = 0;
            int16_t dig_P8 = 0;
            int16_t dig_P9 = 0;

            uint16_t dig_T1 = 0;
            int16_t dig_T2 = 0;
            int16_t dig_T3 = 0;
        };

    private:
        static constexpr std::uint8_t REGISTER_ID = 0xD0;
        static constexpr std::uint8_t REGISTER_STATUS = 0xF3;
        static constexpr std::uint8_t REGISTER_MEASUREMENT_CONTROL = 0xF4;
        static constexpr std::uint8_t REGISTER_CONFIG = 0xF5;
        static constexpr std::uint8_t REGISTER_PRESSURE_START = 0xF7;
        static constexpr std::uint8_t REGISTER_TEMPERATURE_START = 0xFA;
        static constexpr std::uint8_t REGISTER_COMPENSATION_PARAMETERS_START = 0x88;

        I2CBus& m_i2c;

        std::uint8_t m_address = 0;
        Configuration m_conf;
        Calibration m_calibration;

        Calibration readCalibration();
        void writeConfigurationRegisters(DeviceMode mode, Configuration const& conf);

    public:
        BMP280(I2CBus& bus, std::uint8_t address);

        /** Read the device ID
         *
         * Mostly useful as a sanity check
         */
        uint8_t readID();

        /** Change the device mode */
        void writeMode(DeviceMode mode);

        /** Change the device configuration
         *
         * The method puts the chip to sleep. Move back to measurement mode with writeMode
         * if needed afterwards
         */
        void sleepAndWriteConfiguration(Configuration const& conf);

        struct RawMeasurements {
            uint32_t pressure;
            uint32_t temperature;
        };

        /** Read the raw data from registers
         *
         * The BMP280 requires a complex compensation calculation to actually produce
         * temperature and pressure. This does not perform the calculation
         */
        RawMeasurements readRaw();

        /** Read data and calculate the actual measurements */
        BMP280Measurement read();
    };
}

#endif
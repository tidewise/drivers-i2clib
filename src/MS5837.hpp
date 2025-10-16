#ifndef I2CLIB_MS5837_HPP
#define I2CLIB_MS5837_HPP

#include <array>
#include <cstdint>

#include <i2clib/I2CBus.hpp>
#include <i2clib/MS5837Measurement.hpp>

namespace i2clib {
    /** TE Connectivity pressure sensor
     */
    class MS5837 {
    public:
        using Measurement = MS5837Measurement;

    private:
        static constexpr std::uint8_t CMD_RESET = 0x1e;
        static constexpr std::uint8_t CMD_CONVERT_D1_BASE = 0x40;
        static constexpr std::uint8_t CMD_CONVERT_D2_BASE = 0x50;
        static constexpr std::uint8_t CMD_ADC_READ = 0x00;
        static constexpr std::uint8_t CMD_PROM_READ_BASE = 0xA0;
        static constexpr std::uint8_t CMD_PROM_READ_COUNT = 7;

        static constexpr std::uint8_t CONVERT_OSR_256 = 0;
        static constexpr std::uint8_t CONVERT_OSR_8192 = 5;

    public:
        struct PROM {
            std::array<uint16_t, CMD_PROM_READ_COUNT> C;
        };

        enum Models {
            MODEL_30BA = 0
        };

    private:
        Models m_model;
        I2CBus& m_bus;
        uint8_t m_address;
        PROM m_prom;

        static uint8_t crc4(std::array<uint16_t, CMD_PROM_READ_COUNT> const& prom);

        /** Wait the time required to perform the conversion
         *
         * @param osr the oversampling parameter
         */
        static void waitConversion(int osr);

        /** Read ADC data */
        int32_t readADC();

    public:
        /**
         * @param model the exact model of the chip. Affects the conversion function
         */
        MS5837(Models model, I2CBus& bus, uint8_t address = 118);

        /** Reset the chip */
        void reset();

        /** Perform the complete measurement cycle
         *
         * @param temperature_osr oversampling parameter, see
         *   \c readRawTemperature for more details
         * @param pressure_osr oversampling parameter, see
         *   \c readRawPressure for more details
         */
        Measurement read(int temperature_osr, int pressure_osr);

        /** Compute the actual temperature from calibration and raw data */
        static std::pair<base::Temperature, int64_t> compensateRawTemperature(int32_t raw,
            PROM const& prom);

        /** Compute the actual pressure from calibration and raw data */
        static base::Pressure compensateRawPressure(int32_t raw,
            int64_t dT,
            PROM const& prom);

        /** Read calibration data
         *
         * This is public for debugging and testing purposes. It is called in the
         * class constructor
         */
        PROM readPROM();

        /** Read the raw ADC value for the pressure
         *
         * @param osr the oversampling factor, between 0 and 5 that corresponds to
         *   256 samples up to 8192. Acquisition time grows exponentially with this
         *   parameter, from 640us to 20.5ms
         */
        int32_t readRawPressure(int osr);

        /** Read the raw ADC value for the temperature
         *
         * @param osr the oversampling factor, between 0 and 5 that corresponds to
         *   256 samples up to 8192. Acquisition time grows exponentially with this
         *   parameter, from 640us to 20.5ms
         */
        int32_t readRawTemperature(int osr);
    };
}

#endif
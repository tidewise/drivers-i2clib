#ifndef I2CLIB_BMP280CONF_HPP
#define I2CLIB_BMP280CONF_HPP

#include <cstdint>

namespace i2clib {
    /** Configuration structure for \c BMP280
     */
    struct BMP280Configuration {
        enum Oversampling {
            NO_SAMPLING = 0,
            SAMPLING_1 = 1,
            OVERSAMPLING_2 = 2,
            OVERSAMPLING_4 = 3,
            OVERSAMPLING_8 = 4,
            OVERSAMPLING_16 = 5,
        };

        /** How many samples are integrated in a single pressure readout
         */
        Oversampling pressure_oversampling = SAMPLING_1;

        /** How many samples are integrated in a single temperature readout
         *
         * Set to zero to disable temperature reading altogether. The maximum is 16
         */
        Oversampling temperature_oversampling = SAMPLING_1;

        enum StandbyTimes {
            STANDBY_0_5_MS = 0,
            STANDBY_62_5_MS = 1,
            STANDBY_125_MS = 2,
            STANDBY_250_MS = 3,
            STANDBY_500_MS = 4,
            STANDBY_1000_MS = 5,
            STANDBY_2000_MS = 6,
            STANDBY_4000_MS = 7
        };

        /** How long the chip will wait between readouts in normal mode */
        StandbyTimes standby_time = STANDBY_0_5_MS;

        enum IIRTimeConstant {
            IIR_OFF = 0,
            IIR_2 = 1,
            IIR_4 = 2,
            IIR_8 = 3,
            IIR_16 = 4
        };
        IIRTimeConstant iir_time_constant = IIR_OFF;
    };
}

#endif
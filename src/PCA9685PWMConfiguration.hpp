#ifndef I2CLIB_PWMCONFIGURATION_HPP
#define I2CLIB_PWMCONFIGURATION_HPP

#include <cstdint>

namespace i2clib {
    /** Configuration of a single PWM
     *
     * on_edge and off_edge are the timing (within a 4096-ticks cycle) of resp.
     * the off->on transition and on->off transition.
     *
     * As such, they can't represent the "full ON" and "full OFF" state. The mode
     * argument is meant for that.
     */
    struct PCA9685PWMConfiguration {
        enum Mode {
            MODE_OFF,
            MODE_ON,
            MODE_NORMAL
        };

        Mode mode = MODE_OFF;
        std::uint16_t on_edge = 0;
        std::uint16_t off_edge = 0;

        /** Convert an off-edge value that might be out of the [0, 4096[ range
         * into a proper configuration
         */
        static PCA9685PWMConfiguration fromUnnormalizedOffEdge(int32_t off_edge);
    };
}

#endif
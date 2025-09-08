#include <i2clib/PCA9685PWMConfiguration.hpp>

using namespace i2clib;

/** Convert an off-edge value that might be out of the [0, 4096[ range
 * into a proper configuration
 */
PCA9685PWMConfiguration PCA9685PWMConfiguration::fromUnnormalizedOffEdge(int32_t off_edge)
{
    if (off_edge <= 0) {
        return PCA9685PWMConfiguration{.mode = MODE_OFF};
    }
    else if (off_edge >= 4095) {
        return PCA9685PWMConfiguration{.mode = MODE_ON};
    }

    PCA9685PWMConfiguration c;
    c.mode = MODE_NORMAL;
    c.on_edge = 0;
    c.off_edge = off_edge;
    return c;
}
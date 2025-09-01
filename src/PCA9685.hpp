#ifndef I2CLIB_PCA9685_HPP
#define I2CLIB_PCA9685_HPP

#include <i2clib/I2CBus.hpp>

#include <cstdint>
#include <functional>
#include <vector>

namespace i2clib {
    /** 16 channels, 12 bits PWM generator
     *
     * This drivers is an opinionated implementation of the chip's functions. It relies
     * on the auto-increment function to be enabled, and does not implement the restart
     * function (i.e. one has to stop all PWMs before stopping the chip)
     *
     * The constructor will stop all PWMs, put the chip in sleep mode and set the
     * chip configuration to the manufacturer's default plus the auto-increment enabled.
     */
    class PCA9685 {
    public:
        enum PWMMode {
            PWM_MODE_OFF,
            PWM_MODE_ON,
            PWM_MODE_NORMAL
        };

        /** Configuration of a single PWM
         *
         * on_edge and off_edge are the timing (within a 4096-ticks cycle) of resp.
         * the off->on transition and on->off transition.
         *
         * As such, they can't represent the "full ON" and "full OFF" state. The mode
         * argument is meant for that.
         */
        struct PWMConfiguration {
            PWMMode mode = PWM_MODE_OFF;
            uint16_t on_edge = 0;
            uint16_t off_edge = 0;
        };

    private:
        static constexpr uint8_t MODE1_ALLCALL_ENABLED = 1 << 0;
        static constexpr uint8_t MODE1_SLEEP = 1 << 4;
        static constexpr uint8_t MODE1_AUTO_INCREMENT_ENABLED = 1 << 5;
        static constexpr uint8_t MODE1_EXTERNAL_CLOCK = 1 << 6;
        static constexpr uint8_t MODE1_RESTART = 1 << 7;
        static constexpr uint8_t MODE2_OUTDRV_TOTEM = 1 << 2;
        static constexpr uint8_t PWM_FULL_ON = 1 << 4;
        static constexpr uint8_t PWM_FULL_OFF = 1 << 4;

        /** How many registers there are per PWM */
        static constexpr uint8_t REGISTER_COUNT_PER_PWM = 4;
        /** How many PWMs this chip handles */
        static constexpr uint8_t PWM_COUNT = 16;

        static constexpr uint8_t REGISTER_MODE1 = 0x00;
        static constexpr uint8_t REGISTER_MODE2 = 0x01;
        /** Address of the very first PWM control register */
        static constexpr uint8_t REGISTER_PWM_BEGIN = 0x06;
        static constexpr uint8_t REGISTER_ALL_LED_OFF_H = 0xFD;
        static constexpr uint8_t REGISTER_PRESCALE = 0xFE;

        I2CBus& m_i2c;

        std::uint8_t m_address = 0;
        uint8_t m_mode1 =
            MODE1_SLEEP | MODE1_ALLCALL_ENABLED | MODE1_AUTO_INCREMENT_ENABLED;
        uint8_t m_mode2 = MODE2_OUTDRV_TOTEM;

        void writeMode1();
        void writeMode1(uint8_t value);
        void writeMode2();

        void pwmConfigurationToRegisters(uint8_t* registers,
            PWMConfiguration const& configuration);

        /** @overload internal non-allocating version of writePWMConfigurations */
        void writePWMConfigurations(int pwm,
            PWMConfiguration const* configurations,
            size_t size);

    public:
        static constexpr int OSCILLATOR_PERIOD_NS = 40;

        static std::uint8_t periodToPrescale(std::uint32_t ns);

        /** Create the driver and initialize the chip to defaults
         *
         * See the class documentation for the initialization
         */
        PCA9685(I2CBus& i2c_bus, uint8_t address);

        /** Destroy the driver, stopping PWMs and putting the chip to sleep
         */
        ~PCA9685();

        /** Stop all PWMs (i.e. make them be all off) */
        void stop();

        /** Put the chip to sleep
         *
         * This is required to set the PWM period
         */
        void writeSleepMode();

        /** Write the restart bit of mode 1
         *
         * As a safety measure, if the PWM output is nonzero when the chip is put
         * to sleep, the PWM generation is not resumed directly when it is woken up.
         *
         * In that case, one has a few options, but the main two are:
         * - explicitly configure the PWM outputs again
         * - write the restart bit, after a 500us delay since the wakeup
         */
        void writeRestart();

        /** Enable the external clock
         *
         * Use an external clock connected to the appropriate pin instead of
         * the internal oscillator. Must be called while in sleep mode
         */
        void enableExternalClock();

        /** Put the chip to active
         */
        void writeNormalMode();

        /** Change the PWM cycle */
        void writeCycleDuration(uint32_t ns);

        /** Write the configuration of a single PWM
         *
         * @param mode since on_edge and off_edge are transition times, these values
         *   cannot represent the two extreme "ON" and "OFF" states. `mode` represents
         *   them
         * @param on_edge transition "time" (between 0 and 4095) between off and on
         * @param off_edge transition "time" (between 0 and 4095) between on and off
         */
        void writePWMConfigurations(int pwm, std::vector<PWMConfiguration> const& conf);

        /** Simplified interface to set the duty cycles in [0, 1] */
        void writeDutyCycles(int pwm, std::vector<float> const& cycles);
    };
}

#endif
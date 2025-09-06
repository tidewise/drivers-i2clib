#ifndef I2CLIB_PCA9685_HPP
#define I2CLIB_PCA9685_HPP

#include <i2clib/I2CBus.hpp>
#include <i2clib/PCA9685PWMConfiguration.hpp>

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
     * The prescale parameter allows to change the duration of the PWM cycle -
     * it is global, that is common to all PWM outputs. See \c writePrescale for
     * a complete discussion.
     *
     * We recomment calling \c stop before \c writeSleep. Some devices are known to
     * misbehave if the PWM is non-zero when the PWM generators are active.
     */
    class PCA9685 {
    public:
        using PWMConfiguration = PCA9685PWMConfiguration;

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
        static constexpr float INTERNAL_OSCILLATOR_FREQUENCY = 25e6;

        /** Compute the PWM period from the chip's prescale parameter
         *
         * See \c writePrescale's documentation for a discussion on the prescale
         * parameter
         *
         * @see writePrescale
         */
        static std::uint8_t periodToPrescale(std::uint32_t ns,
            float freq = INTERNAL_OSCILLATOR_FREQUENCY);

        /** Compute the chip's prescale parameter that is closest to the given
         * PWM period
         *
         * @see writePrescale
         */
        static std::uint32_t prescaleToPeriod(std::uint8_t ns,
            float freq = INTERNAL_OSCILLATOR_FREQUENCY);

        /** Create the driver and initialize the chip to defaults
         *
         * See the class documentation for the initialization
         */
        PCA9685(I2CBus& i2c_bus, uint8_t address);

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

        /** Change the duration of the PWM cycle by writing the prescale parameter
         *
         * The chip needs to be in sleep mode to change this
         *
         * Internally, the chip allows for the selection of a PWM period based
         * on a 'prescale' parameter that can be set from 3 to 255. The
         * available periods depend on the oscillator frequency (25MHz by
         * default, can be different if an external oscillator is selected). The
         * chip's datasheet explains how to compute those.
         *
         * \c prescaleToPeriod finds the prescale parameter that is closest to a
         * desired period (given the oscillator frequency). Note that this might
         * not be the prescale parameter you want. Depending on the devices you
         * are controlling with the PWM, you might need a period of "at least"
         * some duration. The i2c_pca9685_ctl tool has `period-to-prescale` and
         * `prescale-to-period` to help you play with the values and select the
         * one that is appropriate.
         */
        void writePrescale(uint8_t prescale);

        /** Write the configuration of a contiguous set of PWMs
         *
         * It configures the PWMs from `pwm` to `pwm + conf.size() - 1`
         *
         * @param pwm the start PWM (0-based)
         * @param conf the PWM configurations
         */
        void writePWMConfigurations(int pwm, std::vector<PWMConfiguration> const& conf);

        /** Simplified interface to set the duty cycles in nanoseconds
         *
         * @param durations the duty durations in nanoseconds
         * @param period the chip's known PWM period. Use \c readPeriod to
         *   read it from the chip configuration, or if you are explicitly
         *   setting the prescale parameter, use \c prescaleToPeriod
         */
        void writeDutyTimes(int pwm,
            std::vector<uint32_t> const& durations,
            uint32_t period);

        /** Simplified interface to set the duty cycles in [0, 1] */
        void writeDutyRatios(int pwm, std::vector<float> const& cycles);

        /** Read the currently configured PWM period in nanoseconds */
        uint32_t readPWMPeriod(float freq = INTERNAL_OSCILLATOR_FREQUENCY);
    };
}

#endif

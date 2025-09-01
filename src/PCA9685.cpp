#include <i2clib/PCA9685.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <stdexcept>
#include <string>

using namespace std;
using namespace i2clib;

uint8_t PCA9685::periodToPrescale(uint32_t ns)
{
    float denom = OSCILLATOR_PERIOD_NS * 4096;
    if (ns < 4 * denom) {
        throw std::invalid_argument(
            "PWM period too low (" + to_string(ns) + "ns, minimum is ~656us)");
    }
    else if (ns > 256 * denom) {
        throw std::invalid_argument(
            "PWM period too high (" + to_string(ns) + "ns, maximum is ~42ms)");
    }

    return std::round(ns / denom) - 1;
}

PCA9685::PCA9685(I2CBus& i2c, uint8_t address)
    : m_i2c(i2c)
    , m_address(address)
{
    stop();
    writeSleepMode();
    writeMode1();
    writeMode2();
}

PCA9685::~PCA9685()
{
    stop();
    writeSleepMode();
}

void PCA9685::writeSleepMode()
{
    m_mode1 |= MODE1_SLEEP;
    writeMode1();
}

void PCA9685::writeNormalMode()
{
    m_mode1 &= ~MODE1_SLEEP;
    writeMode1();
}

void PCA9685::stop()
{
    m_i2c.write(m_address, {REGISTER_ALL_LED_OFF_H, PWM_FULL_OFF});
}

void PCA9685::writeMode1()
{
    m_i2c.write(m_address, {REGISTER_MODE1, m_mode1});
}

void PCA9685::writeMode2()
{
    m_i2c.write(m_address, {REGISTER_MODE2, m_mode2});
}

void PCA9685::writeCycleDuration(uint32_t ns)
{
    uint8_t prescale = periodToPrescale(ns);
    m_i2c.write(m_address, {REGISTER_PRESCALE, prescale});
}

void PCA9685::pwmConfigurationToRegisters(uint8_t* registers,
    PWMConfiguration const& configuration)
{
    switch (configuration.mode) {
        case PWM_MODE_ON:
            registers[1] = PWM_FULL_ON;
            registers[3] = 0;
            return;
        case PWM_MODE_OFF: {
            registers[1] = 0;
            registers[3] = PWM_FULL_OFF;
            return;
        }
        default: {
            if (configuration.on_edge >= 4096) {
                throw invalid_argument(
                    "invalid value for on_edge: " + to_string(configuration.on_edge));
            }
            if (configuration.off_edge >= 4096) {
                throw invalid_argument(
                    "invalid value for off_edge: " + to_string(configuration.off_edge));
            }
            registers[0] = configuration.on_edge & 0xFF;
            registers[1] = configuration.on_edge >> 8;
            registers[2] = configuration.off_edge & 0xFF;
            registers[3] = configuration.off_edge >> 8;
            return;
        }
    }
}

void PCA9685::writePWMConfigurations(int pwm,
    PWMConfiguration const* configurations,
    size_t size)
{
    uint8_t registers[PWM_COUNT * REGISTER_COUNT_PER_PWM + 1];
    registers[0] = REGISTER_PWM_BEGIN + pwm * REGISTER_COUNT_PER_PWM;

    uint8_t* pwm_register = registers + 1;
    for (size_t i = 0; i < size; ++i) {
        pwmConfigurationToRegisters(pwm_register, configurations[i]);
        pwm_register += REGISTER_COUNT_PER_PWM;
    }

    m_i2c.write(m_address, registers, 1 + REGISTER_COUNT_PER_PWM * size);
}

void PCA9685::writePWMConfigurations(int pwm,
    vector<PWMConfiguration> const& configurations)
{
    return writePWMConfigurations(pwm, configurations.data(), configurations.size());
}

void PCA9685::writeDutyCycles(int pwm, vector<float> const& ratios)
{
    PWMConfiguration configurations[PWM_COUNT];
    for (size_t i = 0; i < ratios.size(); ++i) {
        auto& c{configurations[i]};
        int32_t duty_cycle = round(ratios[i] * 4096);
        if (duty_cycle <= 0) {
            c.mode = PWM_MODE_OFF;
        }
        else if (duty_cycle >= 4096) {
            c.mode = PWM_MODE_ON;
        }
        else {
            c.mode = PWM_MODE_NORMAL;
            c.on_edge = 0;
            c.off_edge = duty_cycle - 1;
        }
    }

    writePWMConfigurations(pwm, configurations, ratios.size());
}
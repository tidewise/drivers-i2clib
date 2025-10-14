#include <chrono>
#include <i2clib/MS5837.hpp>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace i2clib;
using namespace std;

MS5837::MS5837(Models model, I2CBus& bus, uint8_t address)
    : m_model(model)
    , m_bus(bus)
    , m_address(address)
{
}

void MS5837::reset()
{
    uint8_t cmd = CMD_RESET;
    m_bus.write(m_address, &cmd, 1);
}

MS5837Measurement MS5837::read(int temperature_osr, int pressure_osr)
{
    int64_t raw_temperature = readRawTemperature(temperature_osr);
    int64_t raw_pressure = readRawPressure(pressure_osr);

    auto [temperature, dT] = compensateRawTemperature(raw_temperature, m_prom);
    auto pressure = compensateRawPressure(raw_pressure, dT, m_prom);

    Measurement result;
    result.time = base::Time::now();
    result.pressure = pressure;
    result.temperature = temperature;
    return result;
}

/** Compute the actual temperature from calibration and raw data */
pair<base::Temperature, int64_t> MS5837::compensateRawTemperature(int32_t raw,
    PROM const& prom)
{
    int64_t dT = raw - (static_cast<int64_t>(prom.C[5]) << 8);
    int64_t temperature = 2000 + ((dT * prom.C[6]) >> 23);
    auto temp = base::Temperature::fromCelsius(static_cast<float>(temperature) / 100);

    return make_pair(temp, dT);
}

/** Compute the actual temperature from calibration and raw data */
base::Pressure MS5837::compensateRawPressure(int32_t raw, int64_t dT, PROM const& prom)
{
    int64_t offset = (static_cast<int64_t>(prom.C[2]) << 16) + ((prom.C[4] * dT) >> 7);
    int64_t sens = (static_cast<int64_t>(prom.C[1]) << 15) + ((prom.C[3] * dT) >> 8);
    int64_t pressure = (((raw * sens) >> 21) - offset) >> 13;
    return base::Pressure::fromBar(static_cast<float>(pressure) / 10000);
}

MS5837::PROM MS5837::readPROM()
{
    PROM result;
    for (int i = 0; i < CMD_PROM_READ_COUNT; ++i) {
        auto data = m_bus.read<2>(m_address, CMD_PROM_READ_BASE + i * 2);
        result.C[i] = static_cast<uint16_t>(data[0]) << 8 | data[1];
    }

    uint8_t crc = crc4(result.C);
    uint8_t actual_crc = (result.C[0] >> 12);
    if (crc != actual_crc) {
        cerr << "Read PROM data\n";
        for (int i = 0; i < CMD_PROM_READ_COUNT; ++i) {
            cerr << hex << result.C[i] << "\n";
        }
        throw runtime_error("Invalid CRC in calibration data, expected: " +
                            to_string(crc) + ", actual: " + to_string(actual_crc));
    }

    return result;
}

int32_t MS5837::readRawPressure(int osr)
{
    if (osr < 0 || osr > CONVERT_OSR_8192) {
        throw std::invalid_argument("OSR value must be between 0 and 5");
    }

    m_bus.write(m_address, {static_cast<uint8_t>(CMD_CONVERT_D1_BASE + 2 * osr)});
    waitConversion(osr);
    return readADC();
}

int32_t MS5837::readRawTemperature(int osr)
{
    if (osr < 0 || osr > CONVERT_OSR_8192) {
        throw std::invalid_argument("OSR value must be between 0 and 5");
    }

    m_bus.write(m_address, {static_cast<uint8_t>(CMD_CONVERT_D2_BASE + 2 * osr)});
    waitConversion(osr);
    return readADC();
}

int32_t MS5837::readADC()
{
    auto data = m_bus.read<3>(m_address, CMD_ADC_READ);
    return static_cast<uint32_t>(data[0]) << 16 | static_cast<uint32_t>(data[1]) << 8 |
           data[2];
}

void MS5837::waitConversion(int osr)
{
    // Duration taken from bluerobotics python driver
    //
    // https://github.com/bluerobotics/ms5837-python/blob/master/ms5837/ms5837.py
    uint32_t nanoseconds = 2500 * (1 << (8 + osr));
    this_thread::sleep_for(std::chrono::nanoseconds(nanoseconds));
}

uint8_t MS5837::crc4(array<uint16_t, CMD_PROM_READ_COUNT> const& prom)
{
    int cnt;
    unsigned int n_rem = 0;
    uint8_t n_bit;

    array<uint16_t, 8> n_prom;
    copy(prom.begin(), prom.end(), n_prom.begin());
    n_prom[0] = n_prom[0] & 0x0FFF;
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) {
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        }
        else {
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        }
        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & (0x8000)) {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    return (n_rem ^ 0x00);
}
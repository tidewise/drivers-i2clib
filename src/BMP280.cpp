#include <i2clib/BMP280.hpp>

#include <base/Float.hpp>

#include <utility>

using namespace base;
using namespace i2clib;
using namespace std;

BMP280::BMP280(I2CBus& bus, std::uint8_t address)
    : m_i2c(bus)
    , m_address(address)
{
    m_calibration = readCalibration();
}

uint8_t BMP280::readID()
{
    return m_i2c.read<1>(m_address, REGISTER_ID)[0];
}

void BMP280::writeMode(DeviceMode mode)
{
    writeConfigurationRegisters(mode, m_conf);
}

void BMP280::sleepAndWriteConfiguration(Configuration const& conf)
{
    writeConfigurationRegisters(MODE_SLEEP, conf);
    m_conf = conf;
}

void BMP280::writeConfigurationRegisters(DeviceMode mode, Configuration const& conf)
{
    uint8_t measurement_control =
        mode | (conf.pressure_oversampling << 2) | (conf.temperature_oversampling << 5);
    uint8_t config = (conf.iir_time_constant << 2) | (conf.standby_time << 5);

    m_i2c.write(m_address, {REGISTER_MEASUREMENT_CONTROL, measurement_control, config});
}

BMP280::RawMeasurements BMP280::readRaw()
{
    RawMeasurements raw;
    auto bytes = m_i2c.read<6>(m_address, REGISTER_PRESSURE_START);
    raw.pressure = (static_cast<uint16_t>(bytes[0]) << 12) | (bytes[1] << 4) | bytes[2];
    raw.temperature = (static_cast<uint16_t>(bytes[3]) << 12) | (bytes[4] << 4) | bytes[5];
    return raw;
}

BMP280::Calibration::Calibration() {
    std::fill(dig_P.begin(), dig_P.end(), base::unknown<double>());
    std::fill(dig_T.begin(), dig_T.end(), base::unknown<double>());
}

uint16_t lsb_msb_to_uint16_t(uint8_t lsb, uint8_t msb) {
    return lsb | static_cast<uint16_t>(msb) << 8;
}

uint16_t lsb_msb_to_int16_t(uint8_t lsb, uint8_t msb) {
    uint16_t u = lsb_msb_to_uint16_t(lsb, msb);
    return reinterpret_cast<int16_t&>(u);
}

BMP280::Calibration BMP280::readCalibration() {
    auto bytes = m_i2c.read<24>(m_address, REGISTER_COMPENSATION_PARAMETERS_START);

    Calibration c;
    c.dig_T[0] = lsb_msb_to_uint16_t(bytes[0], bytes[1]);
    c.dig_T[1] = lsb_msb_to_int16_t(bytes[2], bytes[3]);
    c.dig_T[2] = lsb_msb_to_int16_t(bytes[4], bytes[5]);

    c.dig_P[0] = lsb_msb_to_uint16_t(bytes[6], bytes[7]);
    c.dig_P[1] = lsb_msb_to_int16_t(bytes[8], bytes[9]);
    c.dig_P[2] = lsb_msb_to_int16_t(bytes[10], bytes[11]);
    c.dig_P[3] = lsb_msb_to_int16_t(bytes[12], bytes[13]);
    c.dig_P[4] = lsb_msb_to_int16_t(bytes[14], bytes[15]);
    c.dig_P[5] = lsb_msb_to_int16_t(bytes[16], bytes[17]);
    c.dig_P[6] = lsb_msb_to_int16_t(bytes[18], bytes[19]);
    c.dig_P[7] = lsb_msb_to_int16_t(bytes[20], bytes[21]);
    c.dig_P[8] = lsb_msb_to_int16_t(bytes[22], bytes[23]);

    return c;
}

static Temperature bmp280_compensate_T_double(double adc_T, BMP280::Calibration const& c);
static Pressure bmp280_compensate_P_double(double adc_P,
    base::Temperature t,
    BMP280::Calibration const& c);

BMP280Measurement BMP280::read()
{
    BMP280Measurement result;
    result.time = Time::now();

    auto raw = readRaw();
    if (raw.pressure == 0x80000 || raw.temperature == 0x80000) {
        return result;
    }

    result.temperature = bmp280_compensate_T_double(raw.temperature, m_calibration);
    result.pressure =
        bmp280_compensate_P_double(raw.pressure, result.temperature, m_calibration);
    return result;
}

// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23
// DegC. t_fine carries fine temperature as global value
static Temperature bmp280_compensate_T_double(double adc_T, BMP280::Calibration const& c)
{
    double var1 = (adc_T / 16384.0 - c.dig_T[0] / 1024.0) * c.dig_T[1];
    double var2 = ((adc_T / 131072.0 - c.dig_T[0] / 8192.0) *
                      (adc_T / 131072.0 - c.dig_T[0] / 8192.0)) *
                  c.dig_T[2];
    return Temperature::fromCelsius((var1 + var2) / 5120.0);
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862
// hPa
static Pressure bmp280_compensate_P_double(double adc_P,
    base::Temperature t,
    BMP280::Calibration const& c)
{
    double var1 = t.getCelsius() * 5120.0 / 2.0 - 64000.0;
    double var2 = var1 * var1 * c.dig_P[5] / 32768.0;
    var2 = var2 + var1 * c.dig_P[4] * 2.0;
    var2 = (var2 / 4.0) + (c.dig_P[3] * 65536.0);
    var1 = (c.dig_P[2] * var1 * var1 / 524288.0 + c.dig_P[1] * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * c.dig_P[0];
    if (var1 == 0.0) {
        return base::Pressure(); // avoid exception caused by division by zero
    }
    double p = 1048576.0 - adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = c.dig_P[8] * p * p / 2147483648.0;
    var2 = p * c.dig_P[7] / 32768.0;
    p = p + (var1 + var2 + c.dig_P[6]) / 16.0;
    return Pressure::fromPascal(p);
}

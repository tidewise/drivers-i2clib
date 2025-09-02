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
    raw.temperature =
        (static_cast<uint16_t>(bytes[3]) << 12) | (bytes[4] << 4) | bytes[5];
    return raw;
}

uint16_t lsb_msb_to_uint16_t(uint8_t lsb, uint8_t msb)
{
    return lsb | static_cast<uint16_t>(msb) << 8;
}

uint16_t lsb_msb_to_int16_t(uint8_t lsb, uint8_t msb)
{
    uint16_t u = lsb_msb_to_uint16_t(lsb, msb);
    return reinterpret_cast<int16_t&>(u);
}

BMP280::Calibration BMP280::readCalibration()
{
    auto bytes = m_i2c.read<24>(m_address, REGISTER_COMPENSATION_PARAMETERS_START);

    Calibration c;
    c.dig_T1 = lsb_msb_to_uint16_t(bytes[0], bytes[1]);
    c.dig_T2 = lsb_msb_to_int16_t(bytes[2], bytes[3]);
    c.dig_T3 = lsb_msb_to_int16_t(bytes[4], bytes[5]);

    c.dig_P1 = lsb_msb_to_uint16_t(bytes[6], bytes[7]);
    c.dig_P2 = lsb_msb_to_int16_t(bytes[8], bytes[9]);
    c.dig_P3 = lsb_msb_to_int16_t(bytes[10], bytes[11]);
    c.dig_P4 = lsb_msb_to_int16_t(bytes[12], bytes[13]);
    c.dig_P5 = lsb_msb_to_int16_t(bytes[14], bytes[15]);
    c.dig_P6 = lsb_msb_to_int16_t(bytes[16], bytes[17]);
    c.dig_P7 = lsb_msb_to_int16_t(bytes[18], bytes[19]);
    c.dig_P8 = lsb_msb_to_int16_t(bytes[20], bytes[21]);
    c.dig_P9 = lsb_msb_to_int16_t(bytes[22], bytes[23]);

    return c;
}

static pair<Temperature, int32_t> bmp280_compensate_T_int32(int32_t adc_T,
    BMP280::Calibration const& c);
static Pressure bmp280_compensate_P_int32(int32_t adc_P,
    int32_t t_fine,
    BMP280::Calibration const& c);

BMP280Measurement BMP280::read()
{
    BMP280Measurement result;
    result.time = Time::now();

    auto raw = readRaw();
    if (raw.pressure == 0x80000 || raw.temperature == 0x80000) {
        return result;
    }

    auto compensated_T = bmp280_compensate_T_int32(raw.temperature, m_calibration);
    result.temperature = compensated_T.first;
    result.pressure =
        bmp280_compensate_P_int32(raw.pressure, compensated_T.second, m_calibration);
    return result;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123”
// equals 51.23 DegC. t_fine carries fine temperature as global value
static pair<Temperature, int32_t> bmp280_compensate_T_int32(int32_t adc_T,
    BMP280::Calibration const& c)
{
    int32_t var1 =
        ((((adc_T >> 3) - ((int32_t)c.dig_T1 << 1))) * ((int32_t)c.dig_T2)) >> 11;
    int32_t var2 =
        (((((adc_T >> 4) - ((int32_t)c.dig_T1)) * ((adc_T >> 4) - ((int32_t)c.dig_T1))) >>
             12) *
            ((int32_t)c.dig_T3)) >>
        14;
    int32_t t_fine = var1 + var2;
    auto t = base::Temperature::fromCelsius(
        static_cast<double>((t_fine * 5 + 128) >> 8) / 100.0);
    return make_pair(t, t_fine);
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals
// 96386 Pa = 963.86 hPa
static Pressure bmp280_compensate_P_int32(int32_t adc_P,
    int32_t t_fine,
    BMP280::Calibration const& c)
{
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)c.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)c.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)c.dig_P4) << 16);
    var1 = (((c.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
               ((((int32_t)c.dig_P2) * var1) >> 1)) >>
           18;
    var1 = ((((32768 + var1)) * ((int32_t)c.dig_P1)) >> 15);
    if (var1 == 0) {
        return base::Pressure(); // avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    }
    else {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)c.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)c.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + c.dig_P7) >> 4));
    return Pressure::fromPascal(p);
}

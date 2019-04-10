#include "esphome/defines.h"

#ifdef USE_QMC5883L

#include "esphome/sensor/qmc5883l.h"
#include "esphome/log.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

static const uint8_t SENSORS_GAUSS_TO_MICROTESLA = 100;

static const char *TAG = "sensor.qmc5883l";
static const uint8_t QMC5883L_ADDRESS = 0x0D;
static const uint8_t QMC5883L_REGISTER_DATA_X_LSB = 0x00;
static const uint8_t QMC5883L_REGISTER_DATA_X_MSB = 0x01;
static const uint8_t QMC5883L_REGISTER_DATA_Y_LSB = 0x02;
static const uint8_t QMC5883L_REGISTER_DATA_Y_MSB = 0x03;
static const uint8_t QMC5883L_REGISTER_DATA_Z_LSB = 0x04;
static const uint8_t QMC5883L_REGISTER_DATA_Z_MSB = 0x05;
static const uint8_t QMC5883L_REGISTER_STATUS = 0x06;
static const uint8_t QMC5883L_REGISTER_DATA_T_LSB = 0x07;
static const uint8_t QMC5883L_REGISTER_DATA_T_MSB = 0x08;
static const uint8_t QMC5883L_REGISTER_CONTROL_A = 0x09;
static const uint8_t QMC5883L_REGISTER_CONTROL_B = 0x0A;
static const uint8_t QMC5883L_REGISTER_PERIOD = 0x0B;
static const uint8_t QMC5883L_REGISTER_IDENTIFICATION = 0x0D;

QMC5883LComponent::QMC5883LComponent(I2CComponent *parent, const std::string &x_name, const std::string &y_name,
                                     const std::string &z_name, const std::string &heading_name,
                                     uint32_t update_interval)
    : I2CDevice(parent, QMC5883L_ADDRESS),
      PollingComponent(update_interval),
      x_sensor_(new QMC5883LFieldStrengthSensor(x_name, this)),
      y_sensor_(new QMC5883LFieldStrengthSensor(y_name, this)),
      z_sensor_(new QMC5883LFieldStrengthSensor(z_name, this)),
      heading_sensor_(new QMC5883LHeadingSensor(heading_name, this)) {}

QMC5883LFieldStrengthSensor *QMC5883LComponent::get_x_sensor() const { return this->x_sensor_; }
QMC5883LFieldStrengthSensor *QMC5883LComponent::get_y_sensor() const { return this->y_sensor_; }
QMC5883LFieldStrengthSensor *QMC5883LComponent::get_z_sensor() const { return this->z_sensor_; }
QMC5883LHeadingSensor *QMC5883LComponent::get_heading_sensor() const { return this->heading_sensor_; }

void QMC5883LComponent::set_range(const QMC5883LRange range) { this->range_ = range; }

void QMC5883LComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up QMC5583L...");
  uint8_t id;
  if (!this->read_byte(QMC5883L_REGISTER_IDENTIFICATION, &id)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  if (id != 0xFF) {
    this->error_code_ = ID_REGISTERS;
    this->mark_failed();
    return;
  }

  // Soft Reset
  if (!this->write_byte(QMC5883L_REGISTER_CONTROL_B, 1 << 7)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  delay(10);

  uint8_t control_a = 0;
  // 0bxx000000 << 6 : OSR (Over Sample Ratio) -> 0b00=512, 0b11=64
  control_a |= (0b00) << 6;
  // 0b00xx0000 << 4 : RNG (Full Scale) -> 0b00=2G, 0b01=8g
  control_a |= this->range_ << 4;
  // 0b0000xx00 << 2 : ODR (Output Data Rate) -> 0b00=10Hz, 0b11=200Hz
  control_a |= (0b00) << 2;
  // 0b000000xx << 0 : MOD (Mode Control) -> 0b00=standby, 0b01=continuous
  control_a |= (0b01) << 0;

  if (!this->write_byte(QMC5883L_REGISTER_CONTROL_A, control_a)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  uint8_t control_b = 0;
  // 0bx0000000 << 7 : SOFT_RST (Soft Reset) -> 0b00=disabled, 0b01=enabled
  control_b |= (0b0) << 7;
  // 0b0x000000 << 6 : ROL_PNT (Pointer Roll Over) -> 0b00=disabled, 0b01=enabled
  control_b |= (0b0) << 6;
  // 0b0000000x << 0 : INT_ENB (Interrupt) -> 0b00=disabled, 0b01=enabled
  control_b |= (0b0) << 0;

  if (!this->write_byte(QMC5883L_REGISTER_CONTROL_B, control_b)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  // Default Set/Reset period (Recommended)
  if (!this->write_byte(QMC5883L_REGISTER_PERIOD, 0x01)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
}

void QMC5883LComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "QMC5883L:");
  LOG_I2C_DEVICE(this);
  if (this->error_code_ == COMMUNICATION_FAILED) {
    ESP_LOGE(TAG, "Communication with QMC5883L failed!");
  } else if (this->error_code_ == ID_REGISTERS) {
    ESP_LOGE(TAG, "The ID registers don't match - Is this really an QMC5883L?");
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "X Axis", this->x_sensor_);
  LOG_SENSOR("  ", "Y Axis", this->y_sensor_);
  LOG_SENSOR("  ", "Z Axis", this->z_sensor_);
  LOG_SENSOR("  ", "Heading", this->heading_sensor_);
}

float QMC5883LComponent::get_setup_priority() const { return setup_priority::HARDWARE_LATE; }

void QMC5883LComponent::update() {
  uint16_t raw_x, raw_y, raw_z;
  if (!this->read_byte_16(QMC5883L_REGISTER_DATA_X_LSB, &raw_x) ||
      !this->read_byte_16(QMC5883L_REGISTER_DATA_Y_LSB, &raw_y) ||
      !this->read_byte_16(QMC5883L_REGISTER_DATA_Z_LSB, &raw_z)) {
    this->status_set_warning();
    return;
  }

  ESP_LOGD(TAG, "Got Raw x=%uLSB y=%uLSB z=%uLSB", raw_x, raw_y, raw_z);

  // QMC5883L is LSB first
  raw_x = reverse_bytes_16(raw_x);
  raw_y = reverse_bytes_16(raw_y);
  raw_z = reverse_bytes_16(raw_z);

  ESP_LOGD(TAG, "Got Raw x=%uLSB y=%uLSB z=%uLSB", raw_x, raw_y, raw_z);

  float LSB_Gauss = NAN;
  switch (this->range_) {
    case QMC5883L_RANGE_2_G:
      LSB_Gauss = 12000.0f;
      break;
    case QMC5883L_RANGE_8_G:
      LSB_Gauss = 3000.0f;
      break;
  }

  // in µT
  const float x = int16_t(raw_x) / LSB_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
  const float y = int16_t(raw_y) / LSB_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
  const float z = int16_t(raw_z) / LSB_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
  float heading = atan2f(0.0f - x, y) * 180.0f / M_PI;

  ESP_LOGD(TAG, "Got x=%0.02fµT y=%0.02fµT z=%0.02fµT heading=%0.01f°", x, y, z, heading);

  if (this->x_sensor_ != nullptr)
    this->x_sensor_->publish_state(x);
  if (this->y_sensor_ != nullptr)
    this->y_sensor_->publish_state(y);
  if (this->z_sensor_ != nullptr)
    this->z_sensor_->publish_state(z);
  if (this->heading_sensor_ != nullptr)
    this->heading_sensor_->publish_state(heading);
}

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_QMC5883L

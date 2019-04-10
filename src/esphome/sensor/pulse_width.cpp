#include "esphome/defines.h"

#ifdef USE_PULSE_WIDTH_SENSOR

#include "esphome/sensor/pulse_width.h"

#include "esphome/log.h"
#include "esphome/esphal.h"
#include "esphome/espmath.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

static const char *TAG = "sensor.pulse_width";

PulseWidthBase::PulseWidthBase(GPIOPin *pin) : pin_(pin) {}

void ICACHE_RAM_ATTR PulseWidthBase::gpio_intr(PulseWidthBase *arg) {
  // Do time sensitive things fist;
  const uint32_t now = micros();
  PulseWidthData *side = !arg->isr_pin_->digital_read() ? &arg->high_side_ : &arg->low_side_;

  bool abort = false;

  // Start of the pulses
  if (arg->last_pulse_ == 0)
    abort = true;

  // Handel micros overflow
  else if (arg->last_pulse_ > now)
    abort = true;

  const uint32_t pulse_width = now - arg->last_pulse_;
  arg->last_pulse_ = now;

  if (abort)
    return;

  if (side->avg == 0) {
    side->avg = pulse_width;
    side->min = pulse_width;
    side->max = pulse_width;
  } else {
    side->avg = (side->avg + pulse_width) / 2;
    if (pulse_width > side->max)
      side->max = pulse_width;
    if (pulse_width < side->min)
      side->min = pulse_width;
  }
}

bool PulseWidthBase::pulse_width_setup() {
  this->pin_->setup();
  this->isr_pin_ = this->pin_->to_isr();
  this->pin_->attach_interrupt(PulseWidthBase::gpio_intr, this, CHANGE);

  return true;
}

void PulseWidthBase::reset() {
  disable_interrupts();
  this->high_side_ = {};
  this->low_side_ = {};
  this->last_pulse_ = 0;
  enable_interrupts();
}

GPIOPin *PulseWidthBase::get_pin() { return this->pin_; }

PulseWidthData PulseWidthBase::get_side(bool side) { return side ? this->high_side_ : this->low_side_; }

PulseWidthSensorComponent::PulseWidthSensorComponent(const std::string &pwm_name, const std::string &frequency_name,
                                                     const std::string &high_name, const std::string &high_max_name,
                                                     const std::string &high_min_name, const std::string &low_name,
                                                     const std::string &low_max_name, const std::string &low_min_name,
                                                     GPIOPin *pin, uint32_t update_interval)
    : PollingComponent(update_interval),
      PulseWidthBase(pin),
      pwm_sensor_(new PulseWidthPWMSenor(pwm_name, this)),
      frequency_sensor_(new PulseWidthFrequencySenor(frequency_name, this)),
      high_sensor_(new PulseWidthPulseSenor(high_name, this)),
      high_max_sensor_(new PulseWidthPulseSenor(high_max_name, this)),
      high_min_sensor_(new PulseWidthPulseSenor(high_min_name, this)),
      low_sensor_(new PulseWidthPulseSenor(low_name, this)),
      low_max_sensor_(new PulseWidthPulseSenor(low_max_name, this)),
      low_min_sensor_(new PulseWidthPulseSenor(low_min_name, this)) {}

PulseWidthPWMSenor *PulseWidthSensorComponent::get_pwm_sensor() const { return this->pwm_sensor_; }
PulseWidthFrequencySenor *PulseWidthSensorComponent::get_frequency_sensor() const { return this->frequency_sensor_; }
PulseWidthPulseSenor *PulseWidthSensorComponent::get_high_sensor() const { return this->high_sensor_; }
PulseWidthPulseSenor *PulseWidthSensorComponent::get_high_max_sensor() const { return this->high_max_sensor_; }
PulseWidthPulseSenor *PulseWidthSensorComponent::get_high_min_sensor() const { return this->high_min_sensor_; }
PulseWidthPulseSenor *PulseWidthSensorComponent::get_low_sensor() const { return this->low_sensor_; }
PulseWidthPulseSenor *PulseWidthSensorComponent::get_low_max_sensor() const { return this->low_max_sensor_; }
PulseWidthPulseSenor *PulseWidthSensorComponent::get_low_min_sensor() const { return this->low_min_sensor_; }

void PulseWidthSensorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Pulse Width...");
  if (!this->pulse_width_setup()) {
    this->mark_failed();
    return;
  }
}

void PulseWidthSensorComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Pulse Width:");
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "PWM", this->pwm_sensor_);
  LOG_SENSOR("  ", "Frequency", this->frequency_sensor_);

  LOG_SENSOR("  ", "High", this->high_sensor_);
  LOG_SENSOR("  ", "High Max", this->high_max_sensor_);
  LOG_SENSOR("  ", "High Min", this->high_min_sensor_);

  LOG_SENSOR("  ", "Low", this->low_sensor_);
  LOG_SENSOR("  ", "Low Max", this->low_max_sensor_);
  LOG_SENSOR("  ", "Low Min", this->low_min_sensor_);
}

void PulseWidthSensorComponent::update() {
  const float high = this->get_side(HIGH).avg;
  const float highMax = this->get_side(HIGH).max;
  const float highMin = this->get_side(HIGH).min;

  const float low = this->get_side(LOW).avg;
  const float lowMax = this->get_side(LOW).max;
  const float lowMin = this->get_side(LOW).min;

  const float pwm = (high && low) ? (high / (high + low)) * 100.0 : 0;
  const float frequency = (high && low) ? 1000000.0 / (high + low) : 0;

  ESP_LOGD(TAG, "Got pwm=%.2f%% frequency=%.2fHz high=%.fus low=%.fus", pwm, frequency, high, low);

  this->pwm_sensor_->publish_state(pwm);
  this->frequency_sensor_->publish_state(frequency);

  this->high_sensor_->publish_state(high);
  this->high_max_sensor_->publish_state(highMax);
  this->high_min_sensor_->publish_state(highMin);

  this->low_sensor_->publish_state(low);
  this->low_max_sensor_->publish_state(lowMax);
  this->low_min_sensor_->publish_state(lowMin);

  this->reset();
}

float PulseWidthSensorComponent::get_setup_priority() const { return setup_priority::HARDWARE_LATE; }

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_PULSE_WIDTH_SENSOR

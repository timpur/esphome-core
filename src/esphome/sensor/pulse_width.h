#ifndef ESPHOME_SENSOR_PULSE_WIDTH_H
#define ESPHOME_SENSOR_PULSE_WIDTH_H

#include "esphome/defines.h"

#ifdef USE_PULSE_WIDTH_SENSOR

#include "esphome/sensor/sensor.h"
#include "esphome/esphal.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

struct PulseWidthData {
  uint32_t avg;
  uint32_t min;
  uint32_t max;
};

class PulseWidthBase {
 public:
  PulseWidthBase(GPIOPin *pin);
  bool pulse_width_setup();

  void reset();

  GPIOPin *get_pin();
  PulseWidthData get_side(bool side);

 protected:
  static void gpio_intr(PulseWidthBase *arg);

  GPIOPin *pin_;
  ISRInternalGPIOPin *isr_pin_;
  PulseWidthData high_side_;
  PulseWidthData low_side_;
  uint32_t last_pulse_;
};

extern const char UNIT_HERTZ[] = "Hz";
extern const char UNIT_MICROSECONDS[] = "μs";
extern const char ICON_PULSE[] = "mdi:pulse";

using PulseWidthPWMSenor = EmptyPollingParentSensor<2, ICON_PULSE, UNIT_PERCENT>;
using PulseWidthFrequencySenor = EmptyPollingParentSensor<2, ICON_PULSE, UNIT_HERTZ>;
using PulseWidthPulseSenor = EmptyPollingParentSensor<0, ICON_PULSE, UNIT_MICROSECONDS>;

class PulseWidthSensorComponent : public PollingComponent, public PulseWidthBase {
 public:
  /** Construct the Pulse Width instance with the provided pin and update interval.
   *
   * The pulse width unit will automatically be set and the pulse width is set up
   * to measure the width on rising edges by default.
   *
   * @param pin The pin.
   * @param update_interval The update interval in ms.
   */
  explicit PulseWidthSensorComponent(const std::string &pwm_name, const std::string &frequency_name,

                                     const std::string &high_name, const std::string &high_max_name,
                                     const std::string &high_min_name,

                                     const std::string &low_name, const std::string &low_max_name,
                                     const std::string &low_min_name,

                                     GPIOPin *pin, uint32_t update_interval = 60000);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  PulseWidthPWMSenor *get_pwm_sensor() const;
  PulseWidthFrequencySenor *get_frequency_sensor() const;
  PulseWidthPulseSenor *get_high_sensor() const;
  PulseWidthPulseSenor *get_high_max_sensor() const;
  PulseWidthPulseSenor *get_high_min_sensor() const;
  PulseWidthPulseSenor *get_low_sensor() const;
  PulseWidthPulseSenor *get_low_max_sensor() const;
  PulseWidthPulseSenor *get_low_min_sensor() const;

  void setup() override;
  void update() override;
  float get_setup_priority() const override;
  void dump_config() override;

 protected:
  PulseWidthPWMSenor *pwm_sensor_;
  PulseWidthFrequencySenor *frequency_sensor_;
  PulseWidthPulseSenor *high_sensor_;
  PulseWidthPulseSenor *high_max_sensor_;
  PulseWidthPulseSenor *high_min_sensor_;
  PulseWidthPulseSenor *low_sensor_;
  PulseWidthPulseSenor *low_max_sensor_;
  PulseWidthPulseSenor *low_min_sensor_;
};

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_PULSE_WIDTH_SENSOR
#endif  // ESPHOME_SENSOR_PULSE_WIDTH_H

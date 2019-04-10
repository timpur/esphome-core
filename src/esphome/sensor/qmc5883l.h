#ifndef ESPHOME_QMC_5883_L_H
#define ESPHOME_QMC_5883_L_H

#include "esphome/defines.h"

#ifdef USE_QMC5883L

#include "esphome/sensor/sensor.h"
#include "esphome/i2c_component.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

enum QMC5883LRange { QMC5883L_RANGE_2_G = 0b00, QMC5883L_RANGE_8_G = 0b01 };
enum ErrorCode { NONE = 0, COMMUNICATION_FAILED, ID_REGISTERS };

using QMC5883LFieldStrengthSensor = EmptyPollingParentSensor<1, ICON_MAGNET, UNIT_UT>;
using QMC5883LHeadingSensor = EmptyPollingParentSensor<1, ICON_SCREEN_ROTATION, UNIT_DEGREES>;

class QMC5883LComponent : public PollingComponent, public I2CDevice {
 public:
  QMC5883LComponent(I2CComponent *parent, const std::string &x_name, const std::string &y_name,
                    const std::string &z_name, const std::string &heading_name, uint32_t update_interval = 60000);

  void set_range(QMC5883LRange range);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  QMC5883LFieldStrengthSensor *get_x_sensor() const;
  QMC5883LFieldStrengthSensor *get_y_sensor() const;
  QMC5883LFieldStrengthSensor *get_z_sensor() const;
  QMC5883LHeadingSensor *get_heading_sensor() const;

  void setup() override;
  void update() override;
  float get_setup_priority() const override;
  void dump_config() override;

 protected:
  QMC5883LRange range_{QMC5883L_RANGE_8_G};
  QMC5883LFieldStrengthSensor *x_sensor_;
  QMC5883LFieldStrengthSensor *y_sensor_;
  QMC5883LFieldStrengthSensor *z_sensor_;
  QMC5883LHeadingSensor *heading_sensor_;
  ErrorCode error_code_{NONE};
};

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_QMC5883L

#endif  // ESPHOME_QMC_5883_L_H

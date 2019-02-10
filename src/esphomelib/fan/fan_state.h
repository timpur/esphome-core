#ifndef ESPHOMELIB_FAN_FAN_STATE_H
#define ESPHOMELIB_FAN_FAN_STATE_H

#include "esphomelib/defines.h"

#ifdef USE_FAN

#include "esphomelib/component.h"
#include "esphomelib/automation.h"
#include "esphomelib/esppreferences.h"
#include "esphomelib/fan/fan_traits.h"

ESPHOMELIB_NAMESPACE_BEGIN

namespace fan {

/// Simple enum to represent the speed of a fan.
enum FanSpeed {
  FAN_SPEED_LOW = 0, ///< The fan is running on low speed.
  FAN_SPEED_MEDIUM = 1, ///< The fan is running on medium speed.
  FAN_SPEED_HIGH = 2  ///< The fan is running on high/full speed.
};

template<typename T>
class TurnOnAction;
template<typename T>
class TurnOffAction;
template<typename T>
class ToggleAction;

#ifdef USE_MQTT_FAN
class MQTTFanComponent;
#endif

/** This class is shared between the hardware backend and the MQTT frontend to share state.
 *
 * A fan state has several variables that determine the current state: state (ON/OFF),
 * speed (OFF, LOW, MEDIUM, HIGH), oscillating (ON/OFF) and traits (what features are supported).
 * Both the frontend and the backend can register callbacks whenever a state is changed from the
 * frontend and whenever a state is actually changed and should be pushed to the frontend
 */
class FanState : public Nameable, public Component {
 public:
  /// Construct the fan state with name.
  explicit FanState(const std::string &name);

  /// Register a callback that will be called each time the state changes.
  void add_on_state_callback(std::function<void()> &&callback);

  /// Get the traits of this fan (i.e. what features it supports).
  const FanTraits &get_traits() const;
  /// Set the traits of this fan (i.e. what features it supports).
  void set_traits(const FanTraits &traits);

  template<typename T>
  TurnOnAction<T> *make_turn_on_action();
  template<typename T>
  TurnOffAction<T> *make_turn_off_action();
  template<typename T>
  ToggleAction<T> *make_toggle_action();

  /// The current ON/OFF state of the fan.
  bool state{false};
  /// The current oscillation state of the fan.
  bool oscillating{false};
  /// The current fan speed.
  FanSpeed speed{FAN_SPEED_HIGH};

  class StateCall {
   public:
    explicit StateCall(FanState *state);

    FanState::StateCall &set_state(bool state);
    FanState::StateCall &set_state(optional<bool> state);
    FanState::StateCall &set_oscillating(bool oscillating);
    FanState::StateCall &set_oscillating(optional<bool> oscillating);
    FanState::StateCall &set_speed(FanSpeed speed);
    FanState::StateCall &set_speed(optional<FanSpeed> speed);
    FanState::StateCall &set_speed(const char *speed);

    void perform() const;

   protected:
    FanState *const state_;
    optional<bool> binary_state_;
    optional<bool> oscillating_{};
    optional<FanSpeed> speed_{};
  };

  FanState::StateCall turn_on();
  FanState::StateCall turn_off();
  FanState::StateCall toggle();
  FanState::StateCall make_call();

  void setup() override;
  float get_setup_priority() const override;

#ifdef USE_MQTT_FAN
  MQTTFanComponent *get_mqtt() const;
  void set_mqtt(MQTTFanComponent *mqtt);
#endif

 protected:
  uint32_t hash_base_() override;

  FanTraits traits_{};
  CallbackManager<void()> state_callback_{};
  ESPPreferenceObject rtc_;
#ifdef USE_MQTT_FAN
  MQTTFanComponent *mqtt_{nullptr};
#endif
};

template<typename T>
class TurnOnAction : public Action<T> {
 public:
  explicit TurnOnAction(FanState *state);

  template<typename V>
  void set_oscillating(V value) { this->oscillating_ = value; }
  template<typename V>
  void set_speed(V value) { this->speed_ = value; }

  void play(T x) override;

 protected:
  FanState *state_;
  TemplatableValue<bool, T> oscillating_;
  TemplatableValue<FanSpeed , T> speed_;
};

template<typename T>
class TurnOffAction : public Action<T> {
 public:
  explicit TurnOffAction(FanState *state);

  void play(T x) override;
 protected:
  FanState *state_;
};

template<typename T>
class ToggleAction : public Action<T> {
 public:
  explicit ToggleAction(FanState *state);

  void play(T x) override;
 protected:
  FanState *state_;
};

template<typename T>
ToggleAction<T>::ToggleAction(FanState *state) : state_(state) {

}
template<typename T>
void ToggleAction<T>::play(T x) {
  this->state_->toggle().perform();
  this->play_next(x);
}

template<typename T>
TurnOnAction<T>::TurnOnAction(FanState *state) : state_(state) {

}
template<typename T>
void TurnOnAction<T>::play(T x) {
  auto call = this->state_->turn_on();
  if (this->oscillating_.has_value()) {
    call.set_oscillating(this->oscillating_.value(x));
  }
  if (this->speed_.has_value()) {
    call.set_speed(this->speed_.value(x));
  }
  call.perform();
  this->play_next(x);
}

template<typename T>
TurnOffAction<T>::TurnOffAction(FanState *state) : state_(state) {

}
template<typename T>
void TurnOffAction<T>::play(T x) {
  this->state_->turn_off().perform();
  this->play_next(x);
}

template<typename T>
TurnOnAction<T> *FanState::make_turn_on_action() {
  return new TurnOnAction<T>(this);
}
template<typename T>
TurnOffAction<T> *FanState::make_turn_off_action() {
  return new TurnOffAction<T>(this);
}
template<typename T>
ToggleAction<T> *FanState::make_toggle_action() {
  return new ToggleAction<T>(this);
}

} // namespace fan

ESPHOMELIB_NAMESPACE_END

#include "esphomelib/fan/mqtt_fan_component.h"

#endif //USE_FAN

#endif //ESPHOMELIB_FAN_FAN_STATE_H

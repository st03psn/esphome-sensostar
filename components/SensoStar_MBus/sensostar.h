#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/components/output/binary_output.h" // LED related

#include <vector>

namespace esphome {
namespace sensostar {

class SensoStarComponent : public PollingComponent, public uart::UARTDevice {
 public:
  SensoStarComponent() = default;

#ifdef USE_SENSOR
  SUB_SENSOR(energy)
  SUB_SENSOR(volume)
  SUB_SENSOR(power)
  SUB_SENSOR(flow)
  SUB_SENSOR(temperature_flow)
  SUB_SENSOR(temperature_return)
  SUB_SENSOR(temperature_diff)
  SUB_SENSOR(calculated_power)
  // Estimated flow/return temperature with sub-degree resolution (derived from the 0.01 K delta T)
  SUB_SENSOR(temperature_flow_estimated)
  SUB_SENSOR(temperature_return_estimated)
  // Maximum values reported by the meter (DIF function "maximum")
  SUB_SENSOR(max_power)
  SUB_SENSOR(max_flow)
  SUB_SENSOR(max_temperature_flow)
  SUB_SENSOR(max_temperature_return)
  SUB_SENSOR(max_temperature_diff)
  // Values stored at the last due date (storage number 1)
  SUB_SENSOR(due_date_energy)
  SUB_SENSOR(due_date_volume)
  SUB_SENSOR(operating_days)
#endif

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(status)
  SUB_TEXT_SENSOR(serial_number)
  SUB_TEXT_SENSOR(meter_time)
  SUB_TEXT_SENSOR(due_date)
#endif

  void set_data_led(output::BinaryOutput *data_led) { this->data_led_ = data_led; } // flash LED 
  void set_probe(bool probe) { this->probe_ = probe; } // diagnostic probe mode
  // true: meter truncates flow/return temperature to whole degrees; false: rounds (default)
  void set_temperature_truncated(bool truncated) { this->temperature_truncated_ = truncated; }
  
  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;

  float get_setup_priority() const override;

 protected:
  void publish_nans_();
  void flash_data_led_(); // function for flashing the LED on updated values
    
  std::vector<uint8_t> data_;
  uint32_t last_transmission_{0};
  uint16_t last_send_tx_offset_{0};
  uint8_t receiving_{0};
  uint8_t init_state_{0};
  bool trigger_next_{true};
  bool FCB_;

  // ----- Extended readout: single-item requests after each normal readout -----
  void build_extra_list_();
  void send_extra_(uint32_t now);
  void publish_estimated_temperatures_(double flow_t, int8_t flow_exp, double return_t, int8_t return_exp, double tdiff);
  std::vector<uint8_t> extra_ids_;
  size_t extra_idx_{0};
  uint32_t extra_due_{0};
  bool extra_active_{false};
  bool temperature_truncated_{false};

  // ----- Probe mode (diagnostics) -----
  struct Probe {
    const char *name;
    std::vector<uint8_t> msg;
    bool chain;  // send the next probe on the next cycle without a normal readout in between
  };
  void build_probe_list_();
  void probe_finish_(const char *result);
  bool probe_{false};
  bool probe_active_{false};
  bool probe_turn_{false};
  size_t probe_idx_{0};
  std::vector<Probe> probe_list_;

  output::BinaryOutput *data_led_{nullptr}; // LED related
  uint32_t data_led_off_time_{0}; // LED related
};

}  // namespace sensostar
}  // namespace esphome


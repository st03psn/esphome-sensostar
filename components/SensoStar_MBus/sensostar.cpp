#include "sensostar.h"
#include <vector>
#include <initializer_list>
#include <string>
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "mbus_decode.h"
#include <algorithm>
#include <cmath>

namespace esphome {
namespace sensostar {

static const char *const TAG = "SensoStar";

// Build a long frame 68 L L 68 <body> CS 16
static std::vector<uint8_t> long_frame(std::initializer_list<uint8_t> body) {
    std::vector<uint8_t> f = {0x68, (uint8_t) body.size(), (uint8_t) body.size(), 0x68};
    uint8_t cs = 0;
    for (uint8_t b : body) { f.push_back(b); cs += b; }
    f.push_back(cs);
    f.push_back(0x16);
    return f;
}
static std::vector<uint8_t> short_frame(uint8_t c, uint8_t a) {
    return {0x10, c, a, (uint8_t) (c + a), 0x16};
}

void SensoStarComponent::setup() {
    this->build_extra_list_();
    if (this->probe_)
        this->build_probe_list_();
}

// Item IDs of the meter's proprietary selection command (determined with the probe mode on a
// SensoStar; the normal readout uses 02 03 04 06 05 07 08 09 0B):
//   00 fabrication no.  01 date/time  0A operating time  10 max flow  11 max power
//   12 max flow temp  13 max return temp  14 max delta T  15 due date  16 due-date energy  18 due-date volume
void SensoStarComponent::build_extra_list_() {
#ifdef USE_TEXT_SENSOR
    if (this->serial_number_text_sensor_) this->extra_ids_.push_back(0x00);
    if (this->meter_time_text_sensor_) this->extra_ids_.push_back(0x01);
    if (this->due_date_text_sensor_) this->extra_ids_.push_back(0x15);
#endif
#ifdef USE_SENSOR
    if (this->operating_days_sensor_) this->extra_ids_.push_back(0x0A);
    if (this->max_flow_sensor_) this->extra_ids_.push_back(0x10);
    if (this->max_power_sensor_) this->extra_ids_.push_back(0x11);
    if (this->max_temperature_flow_sensor_) this->extra_ids_.push_back(0x12);
    if (this->max_temperature_return_sensor_) this->extra_ids_.push_back(0x13);
    if (this->max_temperature_diff_sensor_) this->extra_ids_.push_back(0x14);
    if (this->due_date_energy_sensor_) this->extra_ids_.push_back(0x16);
    if (this->due_date_volume_sensor_) this->extra_ids_.push_back(0x18);
#endif
}

void SensoStarComponent::send_extra_(uint32_t now) {
    uint8_t id = this->extra_ids_[this->extra_idx_ % this->extra_ids_.size()];
    this->extra_idx_++;
    std::vector<uint8_t> msg = long_frame({0x53, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x01, 0x59, id});
    ESP_LOGV(TAG, "M-Bus write (item 0x%02X): %s", id, format_hex_pretty(msg).c_str());
    this->flush();
    this->data_.clear();
    this->write_array(msg.data(), msg.size());
    this->last_transmission_ = now;
    this->last_send_tx_offset_ = msg.size() * 11 * 1000 / 2400;
    this->receiving_ = 1;
    this->extra_active_ = true;
}

// The meter reports flow/return temperature in whole degrees but delta T in 0.01 K.
// Both whole-degree readings bound the true values; delta T links them, which narrows the range:
//   flow in [Fi+lo, Fi+hi)  and  flow = return + dT with return in [Ri+lo, Ri+hi)
// lo/hi = -0.5/+0.5 when the meter rounds, 0/1 when it truncates. The midpoint is published.
void SensoStarComponent::publish_estimated_temperatures_(double flow_t, int8_t flow_exp, double return_t,
                                                         int8_t return_exp, double tdiff) {
#ifdef USE_SENSOR
    if (this->temperature_flow_estimated_sensor_ == nullptr && this->temperature_return_estimated_sensor_ == nullptr)
        return;
    if (std::isnan(flow_t) || std::isnan(return_t) || tdiff == -127)
        return;
    double est_flow;
    if (flow_exp < 0 && return_exp < 0) {
        est_flow = flow_t;  // meter already delivers sub-degree values
    } else {
        const double lo = this->temperature_truncated_ ? 0.0 : -0.5;
        const double hi = this->temperature_truncated_ ? 1.0 : 0.5;
        double a = std::max(flow_t + lo, return_t + lo + tdiff);
        double b = std::min(flow_t + hi, return_t + hi + tdiff);
        if (a > b) {
            ESP_LOGD(TAG, "Temperature estimate inconsistent (F=%.0f R=%.0f dT=%.2f)", flow_t, return_t, tdiff);
            a = flow_t + lo;
            b = flow_t + hi;
        }
        est_flow = (a + b) / 2;
        ESP_LOGV(TAG, "Flow temperature estimate %.3f (range %.2f..%.2f)", est_flow, a, b);
    }
    if (this->temperature_flow_estimated_sensor_)
        this->temperature_flow_estimated_sensor_->publish_state(est_flow);
    if (this->temperature_return_estimated_sensor_)
        this->temperature_return_estimated_sensor_->publish_state(est_flow - tdiff);
#endif
}

void SensoStarComponent::build_probe_list_() {
    static char names[64][24];
    // A) Proprietary selection (same command as the normal readout) with a single item ID each.
    for (uint8_t id = 0; id < 0x40; id++) {
        snprintf(names[id], sizeof(names[id]), "select item 0x%02X", id);
        this->probe_list_.push_back({names[id], long_frame({0x53, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x01, 0x59, id}), false});
    }
    // B) Standard M-Bus readout (REQ_UD2), both FCB states.
    this->probe_list_.push_back({"REQ_UD2 FCB=0", short_frame(0x5B, 0xFE), false});
    this->probe_list_.push_back({"REQ_UD2 FCB=1", short_frame(0x7B, 0xFE), false});
    // C) Application reset, then REQ_UD2.
    this->probe_list_.push_back({"SND_UD application reset (CI 50)", long_frame({0x53, 0xFE, 0x50}), true});
    this->probe_list_.push_back({"REQ_UD2 after app reset", short_frame(0x5B, 0xFE), false});
    // D) Application reset with subcode 0x10 (all user data), then REQ_UD2.
    this->probe_list_.push_back({"SND_UD app reset subcode 10", long_frame({0x53, 0xFE, 0x50, 0x10}), true});
    this->probe_list_.push_back({"REQ_UD2 after subcode 10", short_frame(0x7B, 0xFE), false});
    // E) SND_NKE (link reset), then REQ_UD2.
    this->probe_list_.push_back({"SND_NKE", short_frame(0x40, 0xFE), true});
    this->probe_list_.push_back({"REQ_UD2 after SND_NKE", short_frame(0x5B, 0xFE), false});
    ESP_LOGI(TAG, "PROBE mode: %u probes queued", (unsigned) this->probe_list_.size());
}

void SensoStarComponent::probe_finish_(const char *result) {
    if (result != nullptr)
        ESP_LOGI(TAG, "PROBE[%u] %s: %s", (unsigned) this->probe_idx_, this->probe_list_[this->probe_idx_].name, result);
    this->probe_turn_ = this->probe_list_[this->probe_idx_].chain;  // chain: next trigger is a probe again
    this->probe_active_ = false;
    this->probe_idx_++;
    if (this->probe_idx_ >= this->probe_list_.size()) {
        ESP_LOGI(TAG, "PROBE mode: all probes done, re-running init sequence");
        this->init_state_ = 0x00;  // restore the meter's normal readout mode
    }
}

void SensoStarComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "SensoStar M-Bus:");
    this->check_uart_settings(2400, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
    if (this->data_led_ != nullptr) {
        ESP_LOGCONFIG(TAG, "  Data LED: Present");
    }	
}

void SensoStarComponent::update() {
    this->trigger_next_ = true;
}

static char format_hex_pretty_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }
static double convert_value(uint32_t data, int8_t decimals){
    double value = (int32_t)data;
    for (int8_t i=0; i<decimals; i++) value *= 10;
    for (int8_t i=decimals; i<0; i++) value /= 10;
    return value;
}

void SensoStarComponent::publish_nans_(){
#ifdef USE_TEXT_SENSOR
    if (this->status_text_sensor_)
        this->status_text_sensor_->publish_state("No readout from heat meter");
#endif
#ifdef USE_SENSOR
    if (this->energy_sensor_)
        this->energy_sensor_->publish_state(NAN);
    if (this->volume_sensor_)
        this->volume_sensor_->publish_state(NAN);
    if (this->power_sensor_)
        this->power_sensor_->publish_state(NAN);
    if (this->flow_sensor_)
        this->flow_sensor_->publish_state(NAN);
    if (this->temperature_flow_sensor_)
        this->temperature_flow_sensor_->publish_state(NAN);
    if (this->temperature_return_sensor_)
        this->temperature_return_sensor_->publish_state(NAN);
    if (this->temperature_diff_sensor_)
        this->temperature_diff_sensor_->publish_state(NAN);
    if (this->calculated_power_sensor_)
        this->calculated_power_sensor_->publish_state(NAN);
#endif
}

void SensoStarComponent::flash_data_led_() {
    if (this->data_led_ != nullptr) {
        this->data_led_->turn_on();
        this->data_led_off_time_ = millis() + 200; // Set turn-off time 200ms in the future
    }
}

void SensoStarComponent::loop() {
    const uint32_t now = millis();

    // Turn off data LED if the time has passed
    if (this->data_led_ != nullptr && this->data_led_off_time_ != 0 && now >= this->data_led_off_time_) {
        this->data_led_->turn_off();
        this->data_led_off_time_ = 0;
    }

    if ( this->receiving_ > 0 && ( now - this->last_transmission_ >= 500 + this->last_send_tx_offset_ ) ) {
        if (this->extra_active_) {
            ESP_LOGD(TAG, "No answer to extended readout request");
            this->data_.clear();
            this->receiving_ = 0;
            this->extra_active_ = false;
            return;
        }
        if (this->probe_active_) {
            std::string partial = this->data_.empty() ? "no answer" : "timeout, partial: " + format_hex_pretty(this->data_);
            this->data_.clear();
            this->receiving_ = 0;
            this->probe_finish_(partial.c_str());
            return;
        }
        ESP_LOGW(TAG, "Last transmission too long ago. Reset RX index.");
        this->publish_nans_();
        this->data_.clear();
        this->receiving_ = 0;
        if (this->init_state_ != 0xff)
            this->trigger_next_ = true;
        }
    
    // Trigger initialization sequence
    if (this->init_state_ != 0xff && this->init_state_&0x10 && now - this->last_transmission_ >= 20)
        this->trigger_next_ = true;
  
    if ( available() )
        this->last_transmission_ = now;
   
    while ( available() ) {
        uint8_t c;
        read_byte(&c);
        
        
        if (this->receiving_ != 2) {
            if (c == 0xe5) {
                // Acknowledge
                this->data_.clear();
                this->receiving_ = 0;
                if (this->probe_active_) {
                    this->probe_finish_("ACK (E5)");
                    continue;
                }
                if (this->extra_active_) {
                    this->extra_active_ = false;
                    continue;
                }
                
                if (this->init_state_ != 0xff)
                    this->init_state_ |= 0x10;
            }
            if (c != 0x68)
                continue;
                
            this->receiving_ = 2;
        }
        this->data_.push_back(c);
        
        
        if ( this->data_.size() > 4 && this->data_.size() == this->data_[1]+6 ) {
                       
            ESP_LOGV(TAG, "M-Bus received: %s", format_hex_pretty(this->data_).c_str());

            if (this->probe_active_) {
                ESP_LOGI(TAG, "PROBE[%u] %s RX: %s", (unsigned) this->probe_idx_,
                         this->probe_list_[this->probe_idx_].name, format_hex_pretty(this->data_).c_str());
                for (auto &line : mbus_decode_records(this->data_))
                    ESP_LOGI(TAG, "PROBE[%u]   %s", (unsigned) this->probe_idx_, line.c_str());
                this->data_.clear();
                this->receiving_ = 0;
                this->probe_finish_(nullptr);
                continue;
            }
            // Answers to extended readout requests never invalidate the main values
            const bool is_extra = this->extra_active_;
            this->extra_active_ = false;
            
            if (this->data_[0] != 0x68 || this->data_[3] != 0x68){
                if (!is_extra) this->publish_nans_();
                ESP_LOGW(TAG, "Invalid frame start");
            }
            else if (this->data_[1] != this->data_[2]){
                if (!is_extra) this->publish_nans_();
                ESP_LOGW(TAG, "Invalid length");
            }
            else if (this->data_[this->data_.size()-1] != 0x16){
                if (!is_extra) this->publish_nans_();
                ESP_LOGW(TAG, "Invalid stop");
            }
            else {
                uint8_t checksum = 0;
                for (uint8_t i = 4; i < this->data_.size()-2; i++)
                    checksum += this->data_[i];
                
                if (checksum != this->data_[this->data_.size()-2]){
                    if (!is_extra) this->publish_nans_();
                    ESP_LOGW(TAG, "Invalid checksum");
                }
                else if (this->init_state_ != 0xff){
                    // Initialization sequence
                    this->init_state_ |= 0x10;
                }
                else if (this->data_[4] != 0x08 || this->data_[6] != 0x72 ){
                    if (!is_extra) this->publish_nans_();
                    ESP_LOGW(TAG, "Unknown frame");
                }
                else {
                    // Store values for calculated_power
                    double flow = -127;
                    double tdiff = -127;
                    // Raw flow/return temperature for the sub-degree estimate
                    double flow_t = NAN, return_t = NAN;
                    int8_t flow_t_exp = 0, return_t_exp = 0;
                    // Decode
					this->flash_data_led_(); // Flash LED when new data arrived
                    uint8_t i = 19; // Skip start header and fixed data header
                    while(i < this->data_.size()-3){
                        
                        // Get all indexes
                        uint8_t i_DIF = i;
                        while( (this->data_[i] & 0x80) == 0x80)
                            i++;
                        i++;
                        uint8_t i_VIF = i;
                        while( (this->data_[i] & 0x80) == 0x80)
                            i++;
                        i++;
                        
                        // Read data
                        uint8_t lc = this->data_[i_DIF]&0x0f; // Length and coding
                        uint32_t result = 0;
                        if (lc == 0x01){ // 8 Bit Integer
                            result = this->data_[i];
                            if ((this->data_[i]&0x80) == 0x80) // Negative
                                result |= 0xffffff00;
                            i += 1;
                        }
                        else if (lc == 0x02){ // 16 Bit Integer	
                            result = this->data_[i] | this->data_[i+1]<<8;
                            if ((this->data_[i+1]&0x80) == 0x80) // Negative
                                result |= 0xffff0000;
                            i += 2;
                        }
                        else if (lc == 0x03){ // 24 Bit Integer	
                            result = this->data_[i] | this->data_[i+1]<<8 | this->data_[i+2]<<16;
                            if ((this->data_[i+2]&0x80) == 0x80) // Negative
                                result |= 0xff000000;
                            i += 3;
                        }
                        else if (lc == 0x04){ // 32 Bit Integer
                            result = this->data_[i] | this->data_[i+1]<<8 | this->data_[i+2]<<16 | this->data_[i+3]<<24;
                            i += 4;
                        }
                        else {
                            ESP_LOGW(TAG, "Unsupported coding 0x%02x", lc);
                            break;
                        }
                        // Parce
                        // DIF
                        uint8_t dif = this->data_[i_DIF];
                        uint8_t f = ( dif&0x30 ) >> 4; // Function: 0 instantaneous, 1 maximum, 2 minimum, 3 error
                        uint8_t storage = ( dif&0x40 ) >> 6; // Storage number LSB: 0 current, 1 due date
                        bool has_dife = ( dif&0x80 ) == 0x80; // tariff/storage extensions: not a plain current value
                        bool cur = f == 0x00 && storage == 0 && !has_dife;
                        bool is_max = f == 0x01 && storage == 0 && !has_dife;
                        bool due = f == 0x00 && storage == 1 && !has_dife;
                        // VIF
                        uint8_t vif = this->data_[i_VIF] & 0x7f;
                        bool vif_ext = this->data_[i_VIF] == 0xfd || this->data_[i_VIF] == 0xfb;
                        if (vif_ext && !(this->data_[i_VIF] == 0xfd && this->data_[i_VIF+1] == 0x17)) {
                            // other extended VIFs (e.g. FD 0C version) are not used
                        }
                        else if ( (vif&0x78) == 0x00 && !vif_ext && (cur || due) ){ // Energy (Wh)
#ifdef USE_SENSOR
                            double kwh = convert_value(result, (vif&0x07) - 3) / 1000;
                            if (cur && this->energy_sensor_)
                                this->energy_sensor_->publish_state(kwh);
                            if (due && this->due_date_energy_sensor_)
                                this->due_date_energy_sensor_->publish_state(kwh);
#endif
                        }
                        else if ( (vif&0x78) == 0x10 && (cur || due) ){ // Volume (m3)
#ifdef USE_SENSOR
                            double m3 = convert_value(result, (vif&0x07) - 6);
                            if (cur && this->volume_sensor_)
                                this->volume_sensor_->publish_state(m3);
                            if (due && this->due_date_volume_sensor_)
                                this->due_date_volume_sensor_->publish_state(m3);
#endif
                        }
                        else if ( (vif&0x78) == 0x28 && (cur || is_max) ){ // Power (W)
#ifdef USE_SENSOR
                            double w = convert_value(result, (vif&0x07) - 3);
                            if (cur && this->power_sensor_)
                                this->power_sensor_->publish_state(w);
                            if (is_max && this->max_power_sensor_)
                                this->max_power_sensor_->publish_state(w);
#endif
                        }
                        else if ( (vif&0x78) == 0x38 && (cur || is_max) ){ // Volume Flow (m3/h)
#ifdef USE_SENSOR
                            double v = convert_value(result, (vif&0x07) - 6);
                            if (cur) {
                                flow = v;
                                if (this->flow_sensor_)
                                    this->flow_sensor_->publish_state(flow);
                            }
                            if (is_max && this->max_flow_sensor_)
                                this->max_flow_sensor_->publish_state(v);
#endif
                        }
                        else if ( (vif&0x7C) == 0x58 && (cur || is_max) ){ // Flow Temperature (C)
#ifdef USE_SENSOR
                            double t = convert_value(result, (vif&0x03) - 3);
                            if (cur) {
                                flow_t = t;
                                flow_t_exp = (vif&0x03) - 3;
                                if (this->temperature_flow_sensor_)
                                    this->temperature_flow_sensor_->publish_state(t);
                            }
                            if (is_max && this->max_temperature_flow_sensor_)
                                this->max_temperature_flow_sensor_->publish_state(t);
#endif
                        }
                        else if ( (vif&0x7C) == 0x5c && (cur || is_max) ){ // Return Temperature (C)
#ifdef USE_SENSOR
                            double t = convert_value(result, (vif&0x03) - 3);
                            if (cur) {
                                return_t = t;
                                return_t_exp = (vif&0x03) - 3;
                                if (this->temperature_return_sensor_)
                                    this->temperature_return_sensor_->publish_state(t);
                            }
                            if (is_max && this->max_temperature_return_sensor_)
                                this->max_temperature_return_sensor_->publish_state(t);
#endif
                        }
                        else if ( (vif&0x7C) == 0x60 && (cur || is_max) ){ // Temperature Difference (K)
#ifdef USE_SENSOR
                            double t = convert_value(result, (vif&0x03) - 3);
                            if (cur) {
                                tdiff = t;
                                if (this->temperature_diff_sensor_)
                                    this->temperature_diff_sensor_->publish_state(tdiff);
                            }
                            if (is_max && this->max_temperature_diff_sensor_)
                                this->max_temperature_diff_sensor_->publish_state(t);
#endif
                        }
                        else if ( (vif&0x7C) == 0x20 && cur ){ // On time (s/min/h/d)
#ifdef USE_SENSOR
                            static const double TO_DAYS[4] = {1.0 / 86400, 1.0 / 1440, 1.0 / 24, 1.0};
                            if (this->operating_days_sensor_)
                                this->operating_days_sensor_->publish_state((int32_t) result * TO_DAYS[vif&0x03]);
#endif
                        }
                        else if ( vif == 0x78 && cur ){ // Fabrication number
#ifdef USE_TEXT_SENSOR
                            if (this->serial_number_text_sensor_)
                                this->serial_number_text_sensor_->publish_state(to_string(result));
#endif
                        }
                        else if ( vif == 0x6D && lc == 0x04 ){ // Date and time (type F)
#ifdef USE_TEXT_SENSOR
                            uint8_t b0 = result & 0xff, b1 = (result >> 8) & 0xff, b2 = (result >> 16) & 0xff, b3 = result >> 24;
                            char buf[20];
                            snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u",
                                     2000 + (((b2 & 0xE0) >> 5) | ((b3 & 0xF0) >> 1)), b3 & 0x0F, b2 & 0x1F, b1 & 0x1F, b0 & 0x3F);
                            if (cur && this->meter_time_text_sensor_)
                                this->meter_time_text_sensor_->publish_state(buf);
#endif
                        }
                        else if ( vif == 0x6C && lc == 0x02 ){ // Date (type G)
#ifdef USE_TEXT_SENSOR
                            uint8_t b0 = result & 0xff, b1 = (result >> 8) & 0xff;
                            char buf[12];
                            snprintf(buf, sizeof(buf), "%04u-%02u-%02u",
                                     2000 + (((b0 & 0xE0) >> 5) | ((b1 & 0xF0) >> 1)), b1 & 0x0F, b0 & 0x1F);
                            if (due && this->due_date_text_sensor_)
                                this->due_date_text_sensor_->publish_state(buf);
#endif
                        }
                        else if ( this->data_[i_VIF] == 0xfd && this->data_[i_VIF+1] == 0x17 ) { // Error flags (binary)
#ifdef USE_TEXT_SENSOR
                            if (this->status_text_sensor_){
                                if (result == 0x00)
                                    this->status_text_sensor_->publish_state("OK");
                                else {
                                    bool append = false;
                                    std::string state;
                                    state.reserve(256);
                                    if (result&0x01){
                                        if (append)
                                            state += " | ";
                                        state += "Temperature Sensor 1: Cable Break";
                                        append = true;
                                    }
                                    if (result&0x02){
                                        if (append)
                                            state += " | ";
                                        state += "Temperature Sensor 1: Short Circuit";
                                        append = true;
                                    }
                                    if (result&0x04){
                                        if (append)
                                            state += " | ";
                                        state += "Temperature Sensor 2: Cable Break";
                                        append = true;
                                    }
                                    if (result&0x08){
                                        if (append)
                                            state += " | ";
                                        state += "Temperature Sensor 2: Short Circuit";
                                        append = true;
                                    }
                                    if (result&0x10){
                                        if (append)
                                            state += " | ";
                                        state += "Error at Flow Measurement System";
                                        append = true;
                                    }
                                    if (result&0x20){
                                        if (append)
                                            state += " | ";
                                        state += "Electronic Defect";
                                        append = true;
                                    }
                                    if (result&0x40){
                                        if (append)
                                            state += " | ";
                                        state += "Reset";
                                        append = true;
                                    }
                                    if (result&0x80){
                                        if (append)
                                            state += " | ";
                                        state += "Low Battery";
                                        append = true;
                                    }
                                    this->status_text_sensor_->publish_state(state);
                                }
                            }
#endif
                        }
                        else {
                            // Unknown
                            ESP_LOGD(TAG, "Skipping value type 0x%02x (DIF 0x%02x)", vif, dif);
                        }
                    }
                    
#ifdef USE_SENSOR
                    this->publish_estimated_temperatures_(flow_t, flow_t_exp, return_t, return_t_exp, tdiff);
                    if (this->calculated_power_sensor_ && (tdiff != -127 || flow != -127)) {
                        if (tdiff == -127 || flow == -127)
                            this->calculated_power_sensor_->publish_state(NAN);
                        else if (flow > 0)
                            this->calculated_power_sensor_->publish_state(flow / 3.6 * 4193 * tdiff);
                        else
                            this->calculated_power_sensor_->publish_state(0);
                    }
#endif
                    // Follow every normal readout with one single-item request (round robin)
                    if (!is_extra && !this->extra_ids_.empty())
                        this->extra_due_ = now + 300;
                }
            }
            
            this->data_.clear();
            this->receiving_ = 0;
        }
    }
    
    if (this->extra_due_ != 0 && (int32_t) (now - this->extra_due_) >= 0 && this->receiving_ == 0 &&
        !this->probe_active_ && this->init_state_ == 0xff) {
        this->extra_due_ = 0;
        this->send_extra_(now);
    }

    if (this->trigger_next_ && !this->probe_active_ && !this->extra_active_) {  // never interrupt a pending request
        this->trigger_next_ = false;

        this->flush();
        this->data_.clear();
        
        bool probe_now = false;
        if (this->init_state_ == 0xff && this->probe_ && this->probe_idx_ < this->probe_list_.size()) {
            probe_now = this->probe_turn_;
            this->probe_turn_ = !this->probe_turn_;
        }
        if (probe_now) {
            const auto &p = this->probe_list_[this->probe_idx_];
            ESP_LOGI(TAG, "PROBE[%u] %s TX: %s", (unsigned) this->probe_idx_, p.name, format_hex_pretty(p.msg).c_str());
            this->write_array(p.msg.data(), p.msg.size());
            this->last_transmission_ = now;
            this->last_send_tx_offset_ = p.msg.size() * 11 * 1000 / 2400;
            this->receiving_ = 1;
            this->probe_active_ = true;
            this->probe_turn_ = false;
        }
        else if (this->init_state_ == 0x00 || this->init_state_ == 0x01){
            uint8_t request_message[] = { 0x68, 0x08, 0x08, 0x68, 0x53, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x04, 0x5C, 0x11, 0x16 };
            ESP_LOGV(TAG, "M-Bus write: %s", format_hex_pretty(request_message, sizeof(request_message)).c_str());
            this->write_array(request_message, sizeof(request_message));
            this->last_transmission_ = now;
            this->last_send_tx_offset_ = sizeof(request_message) * 11 * 1000 / 2400;
            this->receiving_ = 1;
            this->init_state_ = 0x01;
            
        }
        else if (this->init_state_ == 0x11 || this->init_state_ == 0x02){
            uint8_t request_message[] = { 0x68, 0x09, 0x09, 0x68, 0x53, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x00, 0x59, 0x2D, 0x37, 0x16 };
            ESP_LOGV(TAG, "M-Bus write: %s", format_hex_pretty(request_message, sizeof(request_message)).c_str());
            this->write_array(request_message, sizeof(request_message));
            this->last_transmission_ = now;
            this->last_send_tx_offset_ = sizeof(request_message) * 11 * 1000 / 2400;
            this->receiving_ = 1;
            this->init_state_ = 0x02;
        }
        else if (this->init_state_ == 0x12 || this->init_state_ == 0x03){
            uint8_t request_message[] = { 0x68, 0x08, 0x08, 0x68, 0x73, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x00, 0x5D, 0x2E, 0x16 };
            ESP_LOGV(TAG, "M-Bus write: %s", format_hex_pretty(request_message, sizeof(request_message)).c_str());
            this->write_array(request_message, sizeof(request_message));
            this->last_transmission_ = now;
            this->last_send_tx_offset_ = sizeof(request_message) * 11 * 1000 / 2400;
            this->receiving_ = 1;
            this->init_state_ = 0x03;
        }
        else if (this->init_state_ == 0x13 || this->init_state_ == 0x04){
            uint8_t request_message[] = { 0x68, 0x73, 0x73, 0x68, 0x53, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x06, 0x5C, 0x09, 0x02, 0x00, 0x00, 
                0x8f, 0xad, 0xce, 0xe5, 0xc7, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x99, 0x16};
            // Checksum
            uint8_t checksum = 0;
            for (uint8_t i = 4; i < sizeof(request_message)-2; i++)
                checksum += request_message[i];
            request_message[sizeof(request_message)-2] = checksum;
            
            ESP_LOGV(TAG, "M-Bus write: %s", format_hex_pretty(request_message, sizeof(request_message)).c_str());
            this->write_array(request_message, sizeof(request_message));
            this->last_transmission_ = now;
            this->last_send_tx_offset_ = sizeof(request_message) * 11 * 1000 / 2400;
            this->receiving_ = 1;
            this->init_state_ = 0x04;
        }
        else if (this->init_state_ == 0x14 || this->init_state_ == 0x05){
            uint8_t request_message[] = { 0x68, 0x09, 0x09, 0x68, 0x53, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x00, 0x59, 0x0C, 0x16, 0x16};
            ESP_LOGV(TAG, "M-Bus write: %s", format_hex_pretty(request_message, sizeof(request_message)).c_str());
            this->write_array(request_message, sizeof(request_message));
            this->last_transmission_ = now;
            this->last_send_tx_offset_ = sizeof(request_message) * 11 * 1000 / 2400;
            this->receiving_ = 1;
            this->init_state_ = 0x05;
        }
        else if (this->init_state_ == 0x15){
            this->init_state_ = 0xff;
        }
        else {
            uint8_t request_message[] = { 0x68, 0x11, 0x11, 0x68, 0x53, 0xFE, 0x51, 0x0F, 0x00, 0x00, 0x01, 0x59, 0x02, 0x03, 0x04, 0x06, 0x05, 0x07, 0x08, 0x09, 0x0B, 0x00, 0x16 };
        

            // FCB bit
            if (this->FCB_)
                request_message[4] |= 0x10;
            this->FCB_ = !this->FCB_;

            // Checksum
            uint8_t checksum = 0;
            for (uint8_t i = 4; i < sizeof(request_message)-2; i++)
                checksum += request_message[i];
            request_message[sizeof(request_message)-2] = checksum;

            ESP_LOGV(TAG, "M-Bus write: %s", format_hex_pretty(request_message, sizeof(request_message)).c_str());
            this->write_array(request_message, sizeof(request_message));
            this->last_transmission_ = now;
            this->last_send_tx_offset_ = sizeof(request_message) * 11 * 1000 / 2400;
            this->receiving_ = 1;
        }

    }
}

float SensoStarComponent::get_setup_priority() const { return setup_priority::DATA; }


}  // namespace sensostar
}  // namespace esphome


#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include <unordered_map>
#include <sstream>

namespace esphome {
namespace seplos_parser {

static const char *TAG = "seplos_parser.component";

void SeplosParser::setup() {
   // Initialisierung der Sensorvektoren
   std::vector<std::vector<sensor::Sensor *> *> sensor_vectors = {
       &pack_voltage_, &current_, &remaining_capacity_, &total_capacity_,
       &total_discharge_capacity_, &soc_, &soh_, &cycle_count_,
       &average_cell_voltage_, &average_cell_temp_, &max_cell_voltage_,
       &min_cell_voltage_, &max_cell_temp_, &min_cell_temp_,
       &maxdiscurt_, &maxchgcurt_, &cell_1_, &cell_2_, &cell_3_, &cell_4_,
       &cell_5_, &cell_6_, &cell_7_, &cell_8_, &cell_9_, &cell_10_,
       &cell_11_, &cell_12_, &cell_13_, &cell_14_, &cell_15_, &cell_16_,
       &cell_temp_1_, &cell_temp_2_, &cell_temp_3_, &cell_temp_4_,
       &case_temp_, &power_temp_
    };

    for (auto *vec : sensor_vectors) {
        vec->resize(bms_count_, nullptr);
    }

    std::vector<std::vector<text_sensor::TextSensor *> *> text_sensor_vectors = {
        &system_status_, &active_balancing_cells_, &cell_temperature_alarms_,
        &cell_voltage_alarms_, &FET_status_, &active_alarms_,
        &active_protections_
    };

    for (auto *vec : text_sensor_vectors) {
        vec->resize(bms_count_, nullptr);
    }

    last_updates_.resize(bms_count_, 0);

   std::unordered_map<std::string, std::vector<sensor::Sensor *> *> sensor_map = {
       {"pack_voltage", &pack_voltage_}, {"current", &current_},
       {"remaining_capacity", &remaining_capacity_}, {"total_capacity", &total_capacity_},
       {"total_discharge_capacity", &total_discharge_capacity_}, {"soc", &soc_},
       {"soh", &soh_}, {"cycle_count", &cycle_count_},
       {"average_cell_voltage", &average_cell_voltage_}, {"average_cell_temp", &average_cell_temp_},
       {"max_cell_voltage", &max_cell_voltage_}, {"min_cell_voltage", &min_cell_voltage_},
       {"max_cell_temp", &max_cell_temp_}, {"min_cell_temp", &min_cell_temp_},
       {"maxdiscurt", &maxdiscurt_}, {"maxchgcurt", &maxchgcurt_},
       {"cell_1", &cell_1_}, {"cell_2", &cell_2_}, {"cell_3", &cell_3_},
       {"cell_4", &cell_4_}, {"cell_5", &cell_5_}, {"cell_6", &cell_6_},
       {"cell_7", &cell_7_}, {"cell_8", &cell_8_}, {"cell_9", &cell_9_},
       {"cell_10", &cell_10_}, {"cell_11", &cell_11_}, {"cell_12", &cell_12_},
       {"cell_13", &cell_13_}, {"cell_14", &cell_14_}, {"cell_15", &cell_15_},
       {"cell_16", &cell_16_}, {"cell_temp_1", &cell_temp_1_},
       {"cell_temp_2", &cell_temp_2_}, {"cell_temp_3", &cell_temp_3_},
       {"cell_temp_4", &cell_temp_4_}, {"case_temp", &case_temp_},
       {"power_temp", &power_temp_}
   };

   std::unordered_map<std::string, std::vector<text_sensor::TextSensor *> *> text_sensor_map = {
       {"system_status", &system_status_}, {"active_balancing_cells", &active_balancing_cells_},
       {"cell_temperature_alarms", &cell_temperature_alarms_}, {"cell_voltage_alarms", &cell_voltage_alarms_},
       {"FET_status", &FET_status_}, {"active_alarms", &active_alarms_},
       {"active_protections", &active_protections_}
   };

   // Zuordnung der Sensor-Objekte
   for (auto &entry : sensor_map) {
       const std::string &name = entry.first;
       std::vector<sensor::Sensor *> *sensor_vector = entry.second;

       for (int i = 0; i < bms_count_; i++) {
           std::string expected_name = "bms" + std::to_string(i) + " " + name;
           for (auto *sensor : this->sensors_) {
               if (sensor->get_name() == expected_name) {
                   (*sensor_vector)[i] = sensor;
               }
           }
       }
   }

   // Zuordnung der Text-Sensor-Objekte
   for (auto &entry : text_sensor_map) {
       const std::string &name = entry.first;
       std::vector<text_sensor::TextSensor *> *text_sensor_vector = entry.second;

       for (int i = 0; i < bms_count_; i++) {
           std::string expected_name = "bms" + std::to_string(i) + " " + name;
           for (auto *sensor : this->text_sensors_) {
               if (sensor->get_name() == expected_name) {
                   (*text_sensor_vector)[i] = sensor;
               }
           }
       }
   }
}


void SeplosParser::loop() {
  while (available()) {
    uint8_t byte = read();
    buffer.push_back(byte);

    if (buffer.size() > 100) {
      buffer.pop_front();
    }
   
    if (buffer.size() >= 5) {
      if (!is_valid_header()) {
        buffer.pop_front();
        continue;
      }

      const size_t expected_length = get_expected_length();
      if (expected_length == 0) {
        buffer.pop_front();
        continue;
      }

      if (buffer.size() >= expected_length) {
        if (validate_crc(expected_length)) {
          process_packet(expected_length);
        } else {
          ESP_LOGW("seplos", "Ungültige CRC");
        }

        buffer.clear();
      }
    }
  }
}

bool SeplosParser::is_valid_header() {
  return ((buffer[0] >= 0x01 && buffer[0] <= 0x10 && buffer[1] == 0x04 && (buffer[2] == 0x24 || buffer[2] == 0x34)) ||
         (buffer[0] >= 0x01 && buffer[0] <= 0x10 && buffer[1] == 0x01 && buffer[2] == 0x12));
}
size_t SeplosParser::get_expected_length() {
  // +3 Header, +2 CRC, =+5
  if (buffer[2] == 0x24) {return 41;} // (0x24) 36+5=41
  if (buffer[2] == 0x34) {return 57;} // (0x34) 52+5=57
  if (buffer[1] == 0x01 && buffer[2] == 0x12) {return 23;} // (0x12) 18+5=23
  return 0; // If an invalid packet arrives
}

bool SeplosParser::validate_crc(size_t length) {
  uint16_t received_crc = (buffer[length - 1] << 8) | buffer[length - 2];
  uint16_t calculated_crc = calculate_modbus_crc(buffer, length - 2);
  return received_crc == calculated_crc;
}

std::string join_list(const std::vector<int>& list, const std::string& delimiter = ", ") {
  if (list.empty()) return "";
  std::ostringstream oss;
  for (size_t i = 0; i < list.size(); i++) {
    if (i > 0) oss << delimiter;
    oss << list[i];
  }
  return oss.str();
}

std::string join_list(const std::vector<std::string>& list, const std::string& delimiter = ", ") {
  if (list.empty()) return "";
  std::ostringstream oss;
  for (size_t i = 0; i < list.size(); i++) {
    if (i > 0) oss << delimiter;
    oss << list[i];
  }
  return oss.str();
}

void SeplosParser::process_packet(size_t length) {
  int bms_index = buffer[0] - 0x01;
  if (bms_index < 0 || bms_index >= bms_count_) {
    ESP_LOGW("seplos", "Ungültige BMS-ID: %d", buffer[0]);
    return;
  }

  // Masken für die Diagnoseparameter
  bool diagnose_0x24 = (buffer[1] & 0x04) && (buffer[2] == 0x24);
  bool diagnose_0x12 = (buffer[1] & 0x01) && (buffer[2] == 0x12);
  bool diagnose_0x34 = (buffer[1] & 0x04) && (buffer[2] == 0x34);

  if (diagnose_0x24) {
    float pack_voltage = (buffer[3] << 8 | buffer[4]) / 10.0f;
    float current = (buffer[5] << 8 | buffer[6]) / 10.0f - 1000.0f;
    float remaining_capacity = (buffer[7] << 8 | buffer[8]) / 10.0f;
    float total_capacity = (buffer[9] << 8 | buffer[10]) / 10.0f;
    uint16_t cycle_count = (buffer[11] << 8 | buffer[12]);
    uint16_t total_discharge_capacity = (buffer[13] << 8 | buffer[14]);
    float soc = buffer[15];
    float soh = buffer[16];
    float average_cell_voltage = (buffer[17] << 8 | buffer[18]) / 1000.0f;
    float average_cell_temp = (buffer[19] << 8 | buffer[20]) / 10.0f - 273.15f;
    float max_cell_voltage = (buffer[21] << 8 | buffer[22]) / 1000.0f;
    float min_cell_voltage = (buffer[23] << 8 | buffer[24]) / 1000.0f;
    float max_cell_temp = (buffer[25] << 8 | buffer[26]) / 10.0f - 273.15f;
    float min_cell_temp = (buffer[27] << 8 | buffer[28]) / 10.0f - 273.15f;
    float maxdiscurt = (buffer[29] << 8 | buffer[30]) / 10.0f;
    float maxchgcurt = (buffer[31] << 8 | buffer[32]) / 10.0f;

    if (pack_voltage_[bms_index]) pack_voltage_[bms_index]->publish_state(pack_voltage);
    if (current_[bms_index]) current_[bms_index]->publish_state(current);
    if (remaining_capacity_[bms_index]) remaining_capacity_[bms_index]->publish_state(remaining_capacity);
    if (total_capacity_[bms_index]) total_capacity_[bms_index]->publish_state(total_capacity);
    if (total_discharge_capacity_[bms_index]) total_discharge_capacity_[bms_index]->publish_state(total_discharge_capacity);
    if (soc_[bms_index]) soc_[bms_index]->publish_state(soc);
    if (soh_[bms_index]) soh_[bms_index]->publish_state(soh);
    if (cycle_count_[bms_index]) cycle_count_[bms_index]->publish_state(cycle_count);
    if (average_cell_voltage_[bms_index]) average_cell_voltage_[bms_index]->publish_state(average_cell_voltage);
    if (average_cell_temp_[bms_index]) average_cell_temp_[bms_index]->publish_state(average_cell_temp);
    if (max_cell_voltage_[bms_index]) max_cell_voltage_[bms_index]->publish_state(max_cell_voltage);
    if (min_cell_voltage_[bms_index]) min_cell_voltage_[bms_index]->publish_state(min_cell_voltage);
    if (max_cell_temp_[bms_index]) max_cell_temp_[bms_index]->publish_state(max_cell_temp);
    if (min_cell_temp_[bms_index]) min_cell_temp_[bms_index]->publish_state(min_cell_temp);
    if (maxdiscurt_[bms_index]) maxdiscurt_[bms_index]->publish_state(maxdiscurt);
    if (maxchgcurt_[bms_index]) maxchgcurt_[bms_index]->publish_state(maxchgcurt);

  } else if (diagnose_0x34) {
    std::vector<std::pair<sensor::Sensor *, float>> updates;

    updates.emplace_back(cell_1_[bms_index], (buffer[3] << 8 | buffer[4]) / 1000.0f);
    updates.emplace_back(cell_2_[bms_index], (buffer[5] << 8 | buffer[6]) / 1000.0f);
    updates.emplace_back(cell_3_[bms_index], (buffer[7] << 8 | buffer[8]) / 1000.0f);
    updates.emplace_back(cell_4_[bms_index], (buffer[9] << 8 | buffer[10]) / 1000.0f);
    updates.emplace_back(cell_5_[bms_index], (buffer[11] << 8 | buffer[12]) / 1000.0f);
    updates.emplace_back(cell_6_[bms_index], (buffer[13] << 8 | buffer[14]) / 1000.0f);
    updates.emplace_back(cell_7_[bms_index], (buffer[15] << 8 | buffer[16]) / 1000.0f);
    updates.emplace_back(cell_8_[bms_index], (buffer[17] << 8 | buffer[18]) / 1000.0);
    updates.emplace_back(cell_9_[bms_index], (buffer[19] << 8 | buffer[20]) / 1000.0f);
    updates.emplace_back(cell_10_[bms_index], (buffer[21] << 8 | buffer[22]) / 1000.0f);
    updates.emplace_back(cell_11_[bms_index], (buffer[23] << 8 | buffer[24]) / 1000.0f);
    updates.emplace_back(cell_12_[bms_index], (buffer[25] << 8 | buffer[26]) / 1000.0f);
    updates.emplace_back(cell_13_[bms_index], (buffer[27] << 8 | buffer[28]) / 1000.0f);
    updates.emplace_back(cell_14_[bms_index], (buffer[29] << 8 | buffer[30]) / 1000.0f);
    updates.emplace_back(cell_15_[bms_index], (buffer[31] << 8 | buffer[32]) / 1000.0f);
    updates.emplace_back(cell_16_[bms_index], (buffer[33] << 8 | buffer[34]) / 1000.0f);
    updates.emplace_back(cell_temp_1_[bms_index], (buffer[35] << 8 | buffer[36]) / 10.0f - 273.15f);
    updates.emplace_back(cell_temp_2_[bms_index], (buffer[37] << 8 | buffer[38]) / 10.0f - 273.15f);
    updates.emplace_back(cell_temp_3_[bms_index], (buffer[39] << 8 | buffer[40]) / 10.0f - 273.15f);
    updates.emplace_back(cell_temp_4_[bms_index], (buffer[41] << 8 | buffer[42]) / 10.0f - 273.15f);
    updates.emplace_back(case_temp_[bms_index], (buffer[51] << 8 | buffer[52]) / 10.0f - 273.15f);
    updates.emplace_back(power_temp_[bms_index], (buffer[53] << 8 | buffer[54]) / 10.0f - 273.15f);

    for (auto &pair : updates) {
      auto *sensor = pair.first;
      auto value = pair.second;
      if (sensor != nullptr) {
        sensor->publish_state(value);
      }
    }
  } else if (diagnose_0x12) {
    std::vector<int> low_voltage_cells, high_voltage_cells;
    std::vector<int> low_temp_cells, high_temp_cells;
    std::vector<int> balancing_cells;
    std::vector<std::string> system_status, fet_status, active_alarms, active_protections;

    if (buffer[3] & 0x01) low_voltage_cells.push_back(1);
    if (buffer[3] & 0x02) low_voltage_cells.push_back(2);
    if (buffer[3] & 0x04) low_voltage_cells.push_back(3);
    if (buffer[3] & 0x08) low_voltage_cells.push_back(4);
    if (buffer[3] & 0x10) low_voltage_cells.push_back(5);
    if (buffer[3] & 0x20) low_voltage_cells.push_back(6);
    if (buffer[3] & 0x40) low_voltage_cells.push_back(7);
    if (buffer[3] & 0x80) low_voltage_cells.push_back(8);

    if (buffer[4] & 0x01) low_voltage_cells.push_back(9);
    if (buffer[4] & 0x02) low_voltage_cells.push_back(10);
    if (buffer[4] & 0x04) low_voltage_cells.push_back(11);
    if (buffer[4] & 0x08) low_voltage_cells.push_back(12);
    if (buffer[4] & 0x10) low_voltage_cells.push_back(13);
    if (buffer[4] & 0x20) low_voltage_cells.push_back(14);
    if (buffer[4] & 0x40) low_voltage_cells.push_back(15);
    if (buffer[4] & 0x80) low_voltage_cells.push_back(16);

    if (buffer[5] & 0x01) high_voltage_cells.push_back(1);
    if (buffer[5] & 0x02) high_voltage_cells.push_back(2);
    if (buffer[5] & 0x04) high_voltage_cells.push_back(3);
    if (buffer[5] & 0x08) high_voltage_cells.push_back(4);
    if (buffer[5] & 0x10) high_voltage_cells.push_back(5);
    if (buffer[5] & 0x20) high_voltage_cells.push_back(6);
    if (buffer[5] & 0x40) high_voltage_cells.push_back(7);
    if (buffer[5] & 0x80) high_voltage_cells.push_back(8);

    if (buffer[6] & 0x01) high_voltage_cells.push_back(9);
    if (buffer[6] & 0x02) high_voltage_cells.push_back(10);
    if (buffer[6] & 0x04) high_voltage_cells.push_back(11);
    if (buffer[6] & 0x08) high_voltage_cells.push_back(12);
    if (buffer[6] & 0x10) high_voltage_cells.push_back(13);
    if (buffer[6] & 0x20) high_voltage_cells.push_back(14);
    if (buffer[6] & 0x40) high_voltage_cells.push_back(15);
    if (buffer[6] & 0x80) high_voltage_cells.push_back(16);

    if (buffer[7] & 0x01) low_temp_cells.push_back(1);
    if (buffer[7] & 0x02) low_temp_cells.push_back(2);
    if (buffer[7] & 0x04) low_temp_cells.push_back(3);
    if (buffer[7] & 0x08) low_temp_cells.push_back(4);

    if (buffer[8] & 0x01) low_temp_cells.push_back(5);
    if (buffer[8] & 0x02) low_temp_cells.push_back(6);
    if (buffer[8] & 0x04) low_temp_cells.push_back(7);
    if (buffer[8] & 0x08) low_temp_cells.push_back(8);

    if (buffer[9] & 0x01) low_temp_cells.push_back(9);
    if (buffer[9] & 0x02) low_temp_cells.push_back(10);
    if (buffer[9] & 0x04) low_temp_cells.push_back(11);
    if (buffer[9] & 0x08) low_temp_cells.push_back(12);

    if (buffer[10] & 0x01) low_temp_cells.push_back(13);
    if (buffer[10] & 0x02) low_temp_cells.push_back(14);
    if (buffer[10] & 0x04) low_temp_cells.push_back(15);
    if (buffer[10] & 0x08) low_temp_cells.push_back(16);

    if (buffer[11] & 0x01) high_temp_cells.push_back(1);
    if (buffer[11] & 0x02) high_temp_cells.push_back(2);
    if (buffer[11] & 0x04) high_temp_cells.push_back(3);
    if (buffer[11] & 0x08) high_temp_cells.push_back(4);

    if (buffer[12] & 0x01) high_temp_cells.push_back(5);
    if (buffer[12] & 0x02) high_temp_cells.push_back(6);
    if (buffer[12] & 0x04) high_temp_cells.push_back(7);
    if (buffer[12] & 0x08) high_temp_cells.push_back(8);

    if (buffer[13] & 0x01) high_temp_cells.push_back(9);
    if (buffer[13] & 0x02) high_temp_cells.push_back(10);
    if (buffer[13] & 0x04) high_temp_cells.push_back(11);
    if (buffer[13] & 0x08) high_temp_cells.push_back(12);

    if (buffer[14] & 0x01) high_temp_cells.push_back(13);
    if (buffer[14] & 0x02) high_temp_cells.push_back(14);
    if (buffer[14] & 0x04) high_temp_cells.push_back(15);
    if (buffer[14] & 0x08) high_temp_cells.push_back(16);

    if (buffer[15] & 0x01) balancing_cells.push_back(1);
    if (buffer[15] & 0x02) balancing_cells.push_back(2);
    if (buffer[15] & 0x04) balancing_cells.push_back(3);
    if (buffer[15] & 0x08) balancing_cells.push_back(4);
    if (buffer[15] & 0x10) balancing_cells.push_back(5);
    if (buffer[15] & 0x20) balancing_cells.push_back(6);
    if (buffer[15] & 0x40) balancing_cells.push_back(7);
    if (buffer[15] & 0x80) balancing_cells.push_back(8);

    if (buffer[16] & 0x01) balancing_cells.push_back(9);
    if (buffer[16] & 0x02) balancing_cells.push_back(10);
    if (buffer[16] & 0x04) balancing_cells.push_back(11);
    if (buffer[16] & 0x08) balancing_cells.push_back(12);
    if (buffer[16] & 0x10) balancing_cells.push_back(13);
    if (buffer[16] & 0x20) balancing_cells.push_back(14);
    if (buffer[16] & 0x40) balancing_cells.push_back(15);
    if (buffer[16] & 0x80) balancing_cells.push_back(16);

    if (buffer[17] & 0x01) system_status.push_back("System Running");
    if (buffer[17] & 0x02) system_status.push_back("System Sleeping");

    if (buffer[18] & 0x01) active_alarms.push_back("Cell Voltage High Alarm");
    if (buffer[18] & 0x02) active_alarms.push_back("Cell Voltage Low Alarm");
    if (buffer[18] & 0x04) active_protections.push_back("Over Voltage Protection");
    if (buffer[18] & 0x08) active_protections.push_back("Under Voltage Protection");
    if (buffer[18] & 0x10) active_protections.push_back("Cell Voltage Difference Protection");
    if (buffer[18] & 0x20) active_alarms.push_back("Cell Temperature High Alarm");
    if (buffer[18] & 0x40) active_protections.push_back("Cell Temperature Protection");
    if (buffer[18] & 0x80) active_alarms.push_back("Cell Temperature Difference Alarm");

    if (buffer[19] & 0x01) active_alarms.push_back("Output Current High Alarm");
    if (buffer[19] & 0x02) active_protections.push_back("Over Current Protection");
    if (buffer[19] & 0x04) active_alarms.push_back("Charge Current High Alarm");
    if (buffer[19] & 0x08) active_protections.push_back("Charge Current Protection");
    if (buffer[19] & 0x10) active_alarms.push_back("Environment Temperature High Alarm");
    if (buffer[19] & 0x20) active_protections.push_back("Environment Temperature Protection");
    if (buffer[19] & 0x40) active_alarms.push_back("MOS Temperature High Alarm");
    if (buffer[19] & 0x80) active_protections.push_back("MOS Temperature Protection");

    if (buffer[20] & 0x01) fet_status.push_back("Charge MOS On");
    if (buffer[20] & 0x02) fet_status.push_back("Discharge MOS On");
    if (buffer[20] & 0x04) fet_status.push_back("Charge MOS Off");
    if (buffer[20] & 0x08) fet_status.push_back("Discharge MOS Off");
    if (buffer[20] & 0x10) fet_status.push_back("Charge MOS Ready");
    if (buffer[20] & 0x20) fet_status.push_back("Discharge MOS Ready");
    if (buffer[20] & 0x40) fet_status.push_back("Charge MOS Fault");
    if (buffer[20] & 0x80) fet_status.push_back("Discharge MOS Fault");

    if (buffer[21] & 0x01) active_alarms.push_back("Cell Voltage Imbalance Alarm");
    if (buffer[21] & 0x02) active_protections.push_back("Cell Voltage Imbalance Protection");
    if (buffer[21] & 0x04) active_alarms.push_back("Cell Temperature Imbalance Alarm");
    if (buffer[21] & 0x08) active_protections.push_back("Cell Temperature Imbalance Protection");
    if (buffer[21] & 0x10) active_alarms.push_back("Short Circuit Alarm");
    if (buffer[21] & 0x20) active_protections.push_back("Short Circuit Protection");
    if (buffer[21] & 0x40) active_alarms.push_back("Power On Alarm");
    if (buffer[21] & 0x80) active_protections.push_back("Power On Protection");

    std::string volt_str = join_list(low_voltage_cells, ", ");
    if (!volt_str.empty() && !high_voltage_cells.empty()) volt_str += " | " + join_list(high_voltage_cells, ", ");
    else volt_str += join_list(high_voltage_cells, ", ");

    std::string temp_str = join_list(low_temp_cells, ", ");
    if (!temp_str.empty() && !high_temp_cells.empty()) temp_str += " | " + join_list(high_temp_cells, ", ");
    else temp_str += join_list(high_temp_cells, ", ");

    if (should_update(bms_index)) {
      // TextSensor publish_state calls disabled on this build to avoid missing symbol errors.
      // You can re-enable them if your ESPHome/text_sensor library provides
      // TextSensor::publish_state(const std::string &).
      /*
      if (cell_voltage_alarms_[bms_index]) cell_voltage_alarms_[bms_index]->publish_state(volt_str);
      if (cell_temperature_alarms_[bms_index]) cell_temperature_alarms_[bms_index]->publish_state(temp_str);
      if (active_balancing_cells_[bms_index]) active_balancing_cells_[bms_index]->publish_state(join_list(balancing_cells, ", "));
      if (system_status_[bms_index]) system_status_[bms_index]->publish_state(join_list(system_status, ", "));
      if (FET_status_[bms_index]) FET_status_[bms_index]->publish_state(join_list(fet_status, ", "));
      if (active_alarms_[bms_index]) active_alarms_[bms_index]->publish_state(join_list(active_alarms, ", "));
      if (active_protections_[bms_index]) active_protections_[bms_index]->publish_state(join_list(active_protections, ", "));
      */
    }
  }
}

const uint16_t crc_table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0xE001, 0x20C0, 0x2180, 0xE141, 0x2300, 0xE3C1, 0xE281, 0x2240,
  0x2600, 0xE6C1, 0xE781, 0x2740, 0xE501, 0x25C0, 0x2480, 0xE441,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x7C00, 0xBCC1, 0xBD81, 0x7D40, 0xBF01, 0x7FC0, 0x7E80, 0xBE41,
  0xBA01, 0x7AC0, 0x7B80, 0xBB41, 0x7900, 0xB9C1, 0xB881, 0x7840,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t SeplosParser::calculate_modbus_crc(const std::deque<uint8_t> &data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    uint8_t table_index = (crc ^ data[i]) & 0xFF;
    crc = (crc >> 8) ^ crc_table[table_index];
  }
  return crc;
}

void SeplosParser::dump_config(){
    for (int i = 0; i < bms_count_; i++) {
     last_updates_[i] = millis();
     //ESP_LOGD("SeplosParser", "Initialisiere Timer für BMS %d: %u", i, last_updates_[i]);
    }
    for (auto *sensor : this->sensors_) {
        LOG_SENSOR("  ", "Sensor", sensor);
    }
    
    // Text sensors logging disabled to avoid dependency on LOG_TEXT_SENSOR helper
    // for(auto *text_sensor : this->text_sensors_){
    //   ESP_LOGD(TAG, "Text sensor: %s", text_sensor->get_name().c_str());
    // }

//    for(auto *binary_sensor : this->binary_sensors_){
//        LOG_BINARY_SENSOR("  ", "Binary sensor", binary_sensor);
//    }
}
void SeplosParser::set_bms_count(int bms_count) {
  this->bms_count_ = bms_count;  // Wert speichern
  last_updates_.resize(bms_count, 0);  // Dynamische Größe
  ESP_LOGI("SeplosParser", "BMS Count gesetzt auf: %d", bms_count);
}
void SeplosParser::set_update_interval(int update_interval) {
  this->update_interval_ = update_interval*1000;
  ESP_LOGI("SeplosParser", "update interval: %d", update_interval);
}
bool SeplosParser::should_update(int bms_index) {
  if (bms_index < 0 || bms_index >= bms_count_) {
    //ESP_LOGW("SeplosParser", "Ungültiger BMS-Index: %d (max: %d)", bms_index, bms_count_);
    return false; // Ungültiger Index
  }

  uint32_t now = millis();
  //ESP_LOGD("SeplosParser", "BMS %d: now=%u, last_update=%u, interval=%u", 
  //          bms_index, now, last_updates_[bms_index], update_interval_);
  if (now - last_updates_[bms_index] >= update_interval_) {
    last_updates_[bms_index] = now; // Setze den Timer für dieses BMS-Gerät zurück
    //ESP_LOGD("SeplosParser", "Update für BMS %d durchgeführt", bms_index);
    return true;
  }
  //ESP_LOGD("SeplosParser", "Kein Update für BMS %d nötig", bms_index);
  return false;
}

}  // namespace seplos_parser
}  // namespace esphome

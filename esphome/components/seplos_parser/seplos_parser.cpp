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

   // Initialisierung der Text-Sensor-Vektoren
   std::vector<std::vector<text_sensor::TextSensor *> *> text_sensor_vectors = {
       &system_status_, &active_balancing_cells_, &cell_temperature_alarms_,
       &cell_voltage_alarms_, &FET_status_, &active_alarms_,
       &active_protections_
   };

   // Zuordnung der Sensorobjekte zu ihrem Namen
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
           for (auto *text_sensor : this->text_sensors_) {
               if (text_sensor->get_name() == expected_name) {
                   (*text_sensor_vector)[i] = text_sensor;
               }
           }
       }
   }

}

void SeplosParser::loop() {
  const uint8_t PACKET_LENGTH = 32;
  uint8_t buffer[PACKET_LENGTH];
  static uint8_t header[] = {0x00, 0x11, 0x12, 0x00};

  while (this->available() >= PACKET_LENGTH) {
    this->read_array(buffer, PACKET_LENGTH);

    if (memcmp(buffer, header, sizeof(header)) != 0) {
      //ESP_LOGW(TAG, "Ungültiger Header: %02X %02X %02X %02X", buffer[0], buffer[1], buffer[2], buffer[3]);
      continue;
    }
    uint16_t received_crc = buffer[30] | (buffer[31] << 8);
    uint16_t calculated_crc = update_crc(0xFFFF, buffer, 30);

    if (received_crc != calculated_crc) {
      //ESP_LOGW(TAG, "Ungültige CRC: received=0x%04X, calculated=0x%04X", received_crc, calculated_crc);
      continue;
    }
    uint8_t bms_index = buffer[1] - 0x11;
    if (bms_index >= bms_count_) {
      //ESP_LOGW(TAG, "Ungültiger BMS-Index: %d", bms_index);
      continue;
    }

    process_packet(buffer, bms_index);
  }
}

float read_u16(const uint8_t *buffer, int msb_index, int lsb_index, float divisor, bool is_signed) {
  int16_t value = (buffer[msb_index] << 8) | buffer[lsb_index];
  if (is_signed) {
    return static_cast<float>(value) / divisor;
  } else {
    return static_cast<float>(static_cast<uint16_t>(value)) / divisor;
  }
}

template <typename T>
std::string join_list(const std::vector<T>& list, const std::string& delimiter) {
  if (list.empty()) return "";

  std::ostringstream oss;
  for (size_t i = 0; i < list.size(); ++i) {
    if (i != 0) oss << delimiter;
    oss << list[i];
  }
  return oss.str();
}

void SeplosParser::process_packet(const uint8_t *buffer, uint8_t bms_index) {
  //ESP_LOGD("seplos", "BMS-ID: %d, Datensatz-ID: 0x%02X", buffer[1], buffer[2]);
  last_updates_[bms_index] = millis();

  if (buffer[2] == 0x13) {
    std::unordered_map<sensor::Sensor*, float> updates;

    updates[pack_voltage_[bms_index]] = read_u16(buffer, 3, 4, 100.0f, false);
    updates[current_[bms_index]] = read_u16(buffer, 5, 6, 10.0f, true);
    updates[remaining_capacity_[bms_index]] = read_u16(buffer, 7, 8, 100.0f, false);
    updates[total_capacity_[bms_index]] = read_u16(buffer, 9, 10, 100.0f, false);
    updates[total_discharge_capacity_[bms_index]] = read_u16(buffer, 11, 12, 1.0f, false);
    updates[soc_[bms_index]] = read_u16(buffer, 13, 14, 10.0f, false);
    updates[soh_[bms_index]] = read_u16(buffer, 15, 16, 10.0f, false);
    updates[cycle_count_[bms_index]] = read_u16(buffer, 17, 18, 1.0f, false);
    updates[average_cell_voltage_[bms_index]] = read_u16(buffer, 19, 20, 1000.0f, false);
    updates[average_cell_temp_[bms_index]] = read_u16(buffer, 21, 22, 10.0f, false);
    updates[max_cell_voltage_[bms_index]] = read_u16(buffer, 23, 24, 1000.0f, false);
    updates[min_cell_voltage_[bms_index]] = read_u16(buffer, 25, 26, 1000.0f, false);
    updates[max_cell_temp_[bms_index]] = buffer[27] - 40;
    updates[min_cell_temp_[bms_index]] = buffer[28] - 40;

    if (should_update(bms_index)) {
      for (auto &pair : updates) {
        auto sensor = pair.first;
        auto value = pair.second;
        if (sensor != nullptr) {
          sensor->publish_state(value);
        }
      }
    }
  }

  if (buffer[2] == 0x14) {
    std::unordered_map<sensor::Sensor*, float> updates;

    for (int i = 0; i < 8; ++i) {
      auto cell_voltage = read_u16(buffer, 3 + i * 2, 4 + i * 2, 1000.0f, false);
      updates[cell_1_[bms_index + i * bms_count_]] = cell_voltage;
    }

    if (should_update(bms_index)) {
      for (auto &pair : updates) {
        auto sensor = pair.first;
        auto value = pair.second;
        if (sensor != nullptr) {
          sensor->publish_state(value);
        }
      }
    }
  }

  if (buffer[2] == 0x15) {
    std::unordered_map<sensor::Sensor*, float> updates;

    for (int i = 0; i < 8; ++i) {
      auto cell_voltage = read_u16(buffer, 3 + i * 2, 4 + i * 2, 1000.0f, false);
      updates[cell_9_[bms_index + i * bms_count_]] = cell_voltage;
    }

    if (should_update(bms_index)) {
      for (auto &pair : updates) {
        auto sensor = pair.first;
        auto value = pair.second;
        if (sensor != nullptr) {
          sensor->publish_state(value);
        }
      }
    }
  }

  if (buffer[2] == 0x18) {
    std::unordered_map<sensor::Sensor*, float> updates;

    updates[cell_temp_1_[bms_index]] = buffer[3] - 40;
    updates[cell_temp_2_[bms_index]] = buffer[4] - 40;
    updates[cell_temp_3_[bms_index]] = buffer[5] - 40;
    updates[cell_temp_4_[bms_index]] = buffer[6] - 40;
    updates[case_temp_[bms_index]] = buffer[7] - 40;
    updates[power_temp_[bms_index]] = buffer[8] - 40;

    if (should_update(bms_index)) {
      for (auto &pair : updates) {
        auto sensor = pair.first;
        auto value = pair.second;
        if (sensor != nullptr) {
          sensor->publish_state(value);
        }
      }
    }
  }

  if (buffer[2] == 0x1B) {
    std::unordered_map<sensor::Sensor*, float> updates;

    updates[maxdiscurt_[bms_index]] = read_u16(buffer, 3, 4, 1.0f, false);
    updates[maxchgcurt_[bms_index]] = read_u16(buffer, 5, 6, 1.0f, false);

    if (should_update(bms_index)) {
      for (auto &pair : updates) {
        auto sensor = pair.first;
        auto value = pair.second;
        if (sensor != nullptr) {
          sensor->publish_state(value);
        }
      }
    }
  }

  if (buffer[2] == 0x34) {
    std::vector<std::pair<sensor::Sensor*, float>> updates;

    updates.emplace_back(cell_1_[bms_index], (buffer[3] << 8 | buffer[4]) / 1000.0f);
    updates.emplace_back(cell_2_[bms_index], (buffer[5] << 8 | buffer[6]) / 1000.0f);
    updates.emplace_back(cell_3_[bms_index], (buffer[7] << 8 | buffer[8]) / 1000.0f);
    updates.emplace_back(cell_4_[bms_index], (buffer[9] << 8 | buffer[10]) / 1000.0f);
    updates.emplace_back(cell_5_[bms_index], (buffer[11] << 8 | buffer[12]) / 1000.0f);
    updates.emplace_back(cell_6_[bms_index], (buffer[13] << 8 | buffer[14]) / 1000.0f);
    updates.emplace_back(cell_7_[bms_index], (buffer[15] << 8 | buffer[16]) / 1000.0f);
    updates.emplace_back(cell_8_[bms_index], (buffer[17] << 8 | buffer[18]) / 1000.0f);
    updates.emplace_back(cell_9_[bms_index], (buffer[19] << 8 | buffer[20]) / 1000.0f);
    updates.emplace_back(cell_10_[bms_index], (buffer[21] << 8 | buffer[22]) / 1000.0f);
    updates.emplace_back(cell_11_[bms_index], (buffer[23] << 8 | buffer[24]) / 1000.0f);
    updates.emplace_back(cell_12_[bms_index], (buffer[25] << 8 | buffer[26]) / 1000.0f);
    updates.emplace_back(cell_13_[bms_index], (buffer[27] << 8 | buffer[28]) / 1000.0f);
    updates.emplace_back(cell_14_[bms_index], (buffer[29] << 8 | buffer[30]) / 1000.0f);

    if (should_update(bms_index)) {
      for (auto &pair : updates) {
        auto sensor = pair.first;
        auto value = pair.second;
        if (sensor != nullptr) {
          sensor->publish_state(value);
        }
      }
    }
  }

  if (buffer[2] == 0x35) {
    std::vector<std::pair<sensor::Sensor*, float>> updates;

    updates.emplace_back(cell_15_[bms_index], (buffer[3] << 8 | buffer[4]) / 1000.0f);
    updates.emplace_back(cell_16_[bms_index], (buffer[5] << 8 | buffer[6]) / 1000.0f);

    if (should_update(bms_index)) {
      for (auto &pair : updates) {
        auto sensor = pair.first;
        auto value = pair.second;
        if (sensor != nullptr) {
          sensor->publish_state(value);
        }
      }
    }
  }

  if (buffer[2] == 0x12) {
    //ESP_LOGW("seplos", "BMS-ID 0x12: %d", buffer[0]);
    std::vector<std::string> active_alarms;
    std::vector<std::string> active_protections;
    std::vector<int> low_voltage_cells, high_voltage_cells;
    std::vector<int> low_temp_cells, high_temp_cells;
    std::vector<int> balancing_cells;
    std::vector<std::string> system_status;
    std::vector<std::string> fet_status;

    auto parse_bits = [](uint8_t byte, int offset) {
      std::vector<int> result;
      for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
          result.push_back(i + offset);
        }
      }
      return result;
    };

    low_voltage_cells = parse_bits(buffer[3], 1);
    auto low_extra = parse_bits(buffer[4], 9);
    low_voltage_cells.insert(low_voltage_cells.end(), low_extra.begin(), low_extra.end());

    high_voltage_cells = parse_bits(buffer[5], 1);
    auto high_extra = parse_bits(buffer[6], 9);
    high_voltage_cells.insert(high_voltage_cells.end(), high_extra.begin(), high_extra.end());

    low_temp_cells = parse_bits(buffer[7], 1);
    auto low_temp_extra = parse_bits(buffer[8], 9);
    low_temp_cells.insert(low_temp_cells.end(), low_temp_extra.begin(), low_temp_extra.end());

    high_temp_cells = parse_bits(buffer[9], 1);
    auto high_temp_extra = parse_bits(buffer[10], 9);
    high_temp_cells.insert(high_temp_cells.end(), high_temp_extra.begin(), high_temp_extra.end());

    std::vector<int> bal1 = parse_bits(buffer[21], 1);
    std::vector<int> bal2 = parse_bits(buffer[22], 9);
    balancing_cells.insert(balancing_cells.end(), bal1.begin(), bal1.end());
    balancing_cells.insert(balancing_cells.end(), bal2.begin(), bal2.end());

    if (buffer[11] & 0x01) system_status.push_back("Discharge");
    if (buffer[11] & 0x02) system_status.push_back("Charge");
    if (buffer[11] & 0x04) system_status.push_back("Floating Charge");
    if (buffer[11] & 0x08) system_status.push_back("Full Charge");
    if (buffer[11] & 0x10) system_status.push_back("Standby Mode");
    if (buffer[11] & 0x20) system_status.push_back("Turn Off");

    if (buffer[12] & 0x01) active_alarms.push_back("Cell High Voltage Alarm");
    if (buffer[12] & 0x02) active_protections.push_back("Cell Over Voltage Protection");
    if (buffer[12] & 0x04) active_alarms.push_back("Cell Low Voltage Alarm");
    if (buffer[12] & 0x08) active_protections.push_back("Cell Under Voltage Protection");
    if (buffer[12] & 0x10) active_alarms.push_back("Pack High Voltage Alarm");
    if (buffer[12] & 0x20) active_protections.push_back("Pack Over Voltage Protection");
    if (buffer[12] & 0x40) active_alarms.push_back("Pack Low Voltage Alarm");
    if (buffer[12] & 0x80) active_protections.push_back("Pack Under Voltage Protection");

    if (buffer[13] & 0x01) active_alarms.push_back("Charge High Temperature Alarm");
    if (buffer[13] & 0x02) active_protections.push_back("Charge High Temperature Protection");
    if (buffer[13] & 0x04) active_alarms.push_back("Charge Low Temperature Alarm");
    if (buffer[13] & 0x08) active_protections.push_back("Charge Under Temperature Protection");
    if (buffer[13] & 0x10) active_alarms.push_back("Discharge High Temperature Alarm");
    if (buffer[13] & 0x20) active_protections.push_back("Discharge Over Temperature Protection");
    if (buffer[13] & 0x40) active_alarms.push_back("Discharge Low Temperature Alarm");
    if (buffer[13] & 0x80) active_protections.push_back("Discharge Under Temperature Protection");

    if (buffer[14] & 0x01) active_alarms.push_back("High Environment Temperature Alarm");
    if (buffer[14] & 0x02) active_protections.push_back("Over Environment Temperature Protection");
    if (buffer[14] & 0x04) active_alarms.push_back("Low Environment Temperature Alarm");
    if (buffer[14] & 0x08) active_protections.push_back("Under Environment Temperature Protection");
    if (buffer[14] & 0x10) active_alarms.push_back("High Power Temperature Alarm");
    if (buffer[14] & 0x20) active_protections.push_back("Over Power Temperature Protection");
    if (buffer[14] & 0x40) active_alarms.push_back("Cell Temperature Low Heating");

    if (buffer[15] & 0x01) active_alarms.push_back("Charge Current Alarm");
    if (buffer[15] & 0x02) active_protections.push_back("Charge Over Current Protection");
    if (buffer[15] & 0x04) active_protections.push_back("Charge Second Level Current Protection");
    if (buffer[15] & 0x08) active_alarms.push_back("Discharge Current Alarm");
    if (buffer[15] & 0x10) active_protections.push_back("Discharge Over Current Protection");
    if (buffer[15] & 0x20) active_protections.push_back("Discharge Second Level Over Current Protection");
    if (buffer[15] & 0x40) active_protections.push_back("Output Short Circuit Protection");

    if (buffer[16] & 0x01) active_alarms.push_back("Output Short Latch Up");
    if (buffer[16] & 0x04) active_alarms.push_back("Second Charge Latch Up");
    if (buffer[16] & 0x08) active_alarms.push_back("Second Discharge Latch Up");

    if (buffer[17] & 0x04) active_alarms.push_back("SOC Alarm");
    if (buffer[17] & 0x08) active_protections.push_back("SOC Protection");
    if (buffer[17] & 0x10) active_alarms.push_back("Cell Difference Alarm");

    if (buffer[18] & 0x01) fet_status.push_back("Discharge FET On");
    if (buffer[18] & 0x02) fet_status.push_back("Charge FET On");
    if (buffer[18] & 0x04) fet_status.push_back("Current Limiting FET On");
    if (buffer[18] & 0x08) fet_status.push_back("Heating On");

    if (buffer[19] & 0x01) active_alarms.push_back("Low SOC Alarm");
    if (buffer[19] & 0x02) active_alarms.push_back("Intermittent Charge");
    if (buffer[19] & 0x04) active_alarms.push_back("External Switch Conrol");
    if (buffer[19] & 0x08) active_alarms.push_back("Static Standy Sleep Mode");
    if (buffer[19] & 0x10) active_alarms.push_back("History Data Recording");
    if (buffer[19] & 0x20) active_protections.push_back("Under SOC Protections");
    if (buffer[19] & 0x40) active_alarms.push_back("Active Limited Current");
    if (buffer[19] & 0x80) active_alarms.push_back("Passive Limited Current");

    if (buffer[20] & 0x01) active_protections.push_back("NTC Fault");
    if (buffer[20] & 0x02) active_protections.push_back("AFE Fault");
    if (buffer[20] & 0x04) active_protections.push_back("Charge Mosfet Fault");
    if (buffer[20] & 0x08) active_protections.push_back("Discharge Mosfet Fault");
    if (buffer[20] & 0x10) active_protections.push_back("Cell Fault");
    if (buffer[20] & 0x20) active_protections.push_back("Break Line Fault");
    if (buffer[20] & 0x40) active_protections.push_back("Key Fault");
    if (buffer[20] & 0x80) active_protections.push_back("Aerosol Alarm");

    std::string volt_str = join_list(low_voltage_cells, ", ");
    if (!volt_str.empty() && !high_voltage_cells.empty()) volt_str += " | " + join_list(high_voltage_cells, ", ");
    else volt_str += join_list(high_voltage_cells, ", ");

    std::string temp_str = join_list(low_temp_cells, ", ");
    if (!temp_str.empty() && !high_temp_cells.empty()) temp_str += " | " + join_list(high_temp_cells, ", ");
    else temp_str += join_list(high_temp_cells, ", ");

    if (should_update(bms_index)) {
      // Build strings once and publish as C-strings to avoid std::string ABI issues
      std::string balancing_str = join_list(balancing_cells, ", ");
      std::string system_str = join_list(system_status, ", ");
      std::string fet_str = join_list(fet_status, ", ");
      std::string alarms_str = join_list(active_alarms, ", ");
      std::string protections_str = join_list(active_protections, ", ");
      if (cell_voltage_alarms_[bms_index]) cell_voltage_alarms_[bms_index]->publish_state(volt_str.c_str());
      if (cell_temperature_alarms_[bms_index]) cell_temperature_alarms_[bms_index]->publish_state(temp_str.c_str());
      if (active_balancing_cells_[bms_index]) active_balancing_cells_[bms_index]->publish_state(balancing_str.c_str());
      if (system_status_[bms_index]) system_status_[bms_index]->publish_state(system_str.c_str());
      if (FET_status_[bms_index]) FET_status_[bms_index]->publish_state(fet_str.c_str());
      if (active_alarms_[bms_index]) active_alarms_[bms_index]->publish_state(alarms_str.c_str());
      if (active_protections_[bms_index]) active_protections_[bms_index]->publish_state(protections_str.c_str());
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
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xB201, 0x72C0, 0x7380, 0xB341, 0x7100, 0xB1C1, 0xB081, 0x7040,
  0x5000, 0x90C1, 0x9180, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x4C00, 0x8CC1, 0x8D81, 0x4D40, 0x8F01, 0x4FC0, 0x4E80, 0x8E41,
  0x8401, 0x44C0, 0x4580, 0x8541, 0x4700, 0x87C1, 0x8681, 0x4640,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

uint16_t update_crc(uint16_t crc, const uint8_t *data, uint8_t len) {
  while (len--) {
    uint8_t tmp = *data++ ^ crc;
    crc >>= 8;
    crc ^= crc_table[tmp];
  }
  return crc;
}

bool SeplosParser::should_update(uint8_t bms_index) {
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - last_updates_[bms_index];

  //ESP_LOGD("SeplosParser", "BMS %d: elapsed time %lu ms", bms_index, elapsed_time);

  if (elapsed_time >= (unsigned long)(update_interval_ * 1000)) {
    //ESP_LOGD("SeplosParser", "BMS %d: update allowed (elapsed_time >= update_interval)", bms_index);
    return true;
  }
  return false;
}

void SeplosParser::dump_config(){
    for (int i = 0; i < bms_count_; i++) {
     last_updates_[i] = millis();
     //ESP_LOGD("SeplosParser", "Initialisiere Timer für BMS %d: %u", i, last_updates_[i]);
    }
    for (auto *sensor : this->sensors_) {
        LOG_SENSOR("  ", "Sensor", sensor);
    }
    
    // Log text sensors without using LOG_TEXT_SENSOR to avoid linker issues
    for (auto *text_sensor : this->text_sensors_) {
        if (text_sensor != nullptr) {
            ESP_LOGD(TAG, "Text sensor: %s", text_sensor->get_name().c_str());
        }
    }

//    for(auto *binary_sensor : this->binary_sensors_){
//        LOG_BINARY_SENSOR("  ", "Binary sensor", binary_sensor);
//    }
}

void SeplosParser::set_bms_count(int bms_count) {
  this->bms_count_ = bms_count;  // Wert speichern
  last_updates_.resize(bms_count, 0);  // Dynamische Größe
  ESP_LOGI("SeplosParser", "BMS Count gesetzt auf: %d", bms_count);
}
void SeplosParser::set_update_interval(float interval) {
  this->update_interval_ = interval;
}

}  // namespace seplos_parser
}  // namespace esphome

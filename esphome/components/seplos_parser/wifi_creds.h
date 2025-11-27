#pragma once

#include "esphome.h"

#ifdef USE_ESP32
extern "C" {
  #include "esp_wifi.h"
}
#endif

namespace esphome {
namespace wifi_creds {

inline std::string get_wifi_ssid() {
#ifdef USE_ESP32
  wifi_config_t conf;
  esp_wifi_get_config(WIFI_IF_STA, &conf);
  return std::string(reinterpret_cast<char *>(conf.sta.ssid));
#else
  return std::string("");
#endif
}

inline std::string get_wifi_password() {
#ifdef USE_ESP32
  wifi_config_t conf;
  esp_wifi_get_config(WIFI_IF_STA, &conf);
  return std::string(reinterpret_cast<char *>(conf.sta.password));
#else
  return std::string("");
#endif
}

}  // namespace wifi_creds
}  // namespace esphome

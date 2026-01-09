/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "ConfigNVS.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

Preferences prefs;
Config cfg;

// (same helper functions from earlier reply)
static bool parseMac48(const char* s, uint8_t out[6]) {
  int v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
    for (int i=0;i<6;i++) out[i] = (uint8_t)v[i];
    return true;
  }
  return false;
}

void loadConfigFromNVS() {
  prefs.begin("cfg", true);
  cfg.invertX = prefs.getBool("invX",    cfg.invertX);
  cfg.invertY = prefs.getBool("invY",    cfg.invertY);
  cfg.joyX      = prefs.getInt("pin_joyX",      cfg.joyX);
  cfg.joyY      = prefs.getInt("pin_joyY",      cfg.joyY);
  cfg.buttonAX      = prefs.getInt("pin_btnAX",      cfg.buttonAX);
  cfg.buttonBY      = prefs.getInt("pin_btnBY",      cfg.buttonBY);
  cfg.buttonBP     = prefs.getInt("pin_btnBP",     cfg.buttonBP);
  cfg.buttonST     = prefs.getInt("pin_btnST",     cfg.buttonST);
  cfg.buttonStartSelect  = prefs.getInt("pin_btnStSel",  cfg.buttonStartSelect);
  cfg.play_sound = prefs.getBool("play_sound", cfg.play_sound);
  cfg.boot_debug = prefs.getBool("boot_debug", cfg.boot_debug);
  cfg.linux_mode = prefs.getBool("linux_mode", cfg.linux_mode);  
  cfg.right_not_left = prefs.getBool("right_not_left", cfg.right_not_left);
  cfg.wifi_channel = prefs.getUInt("wifi_channel", cfg.wifi_channel);
  cfg.stick_deadzone = prefs.getUInt("stick_deadzn", cfg.stick_deadzone);
  cfg.trigger_deadzone = prefs.getUInt("trigger_deadzn", cfg.trigger_deadzone);
  cfg.esp_interval_us = prefs.getUInt("esp_interval", cfg.esp_interval_us);
  if (prefs.getBytesLength("peer_mac") == 6) {
    prefs.getBytes("peer_mac", cfg.peer_mac, 6);
  }
  prefs.end();
}

void saveConfigToNVS() {
  prefs.begin("cfg", false);
  prefs.putInt("pin_joyX", cfg.joyX);
  prefs.putInt("pin_joyY", cfg.joyY);
  prefs.putBool("invX",    cfg.invertX);
  prefs.putBool("invY",    cfg.invertY);
  prefs.putBool("right_not_left",    cfg.right_not_left);
  prefs.putBool("linux_mode",    cfg.linux_mode);
  prefs.putBool("play_sound",    cfg.play_sound);
  prefs.putBool("boot_debug",    cfg.boot_debug);
  prefs.putInt("pin_btnAX",      cfg.buttonAX);
  prefs.putInt("pin_btnBY",      cfg.buttonBY);
  prefs.putInt("pin_btnBP",     cfg.buttonBP);
  prefs.putInt("pin_btnST",     cfg.buttonST);
  prefs.putInt("pin_btnStSel",  cfg.buttonStartSelect);
  prefs.putUInt("wifi_channel",  cfg.wifi_channel);
  prefs.putUInt("stick_deadzn",  cfg.stick_deadzone);
  prefs.putUInt("trigger_deadzn",  cfg.trigger_deadzone);
  prefs.putUInt("esp_interval",  cfg.esp_interval_us);
  prefs.putBytes("peer_mac", cfg.peer_mac, 6);
  prefs.end();
}

bool applyJsonToConfig(const String& jsonLine, String& errMsg) {
  StaticJsonDocument<2048> doc;
  auto err = deserializeJson(doc, jsonLine);
  if (err) { errMsg = err.c_str(); return false; }

  if (doc.containsKey("pins")) {
    auto pins = doc["pins"];
    if (pins.containsKey("buttonAX"))      cfg.buttonAX      = pins["buttonAX"];
    if (pins.containsKey("buttonBY"))      cfg.buttonBY      = pins["buttonBY"];
    if (pins.containsKey("buttonBP"))     cfg.buttonBP     = pins["buttonBP"];
    if (pins.containsKey("buttonST"))     cfg.buttonST     = pins["buttonST"];
    if (pins.containsKey("buttonStartSelect"))  cfg.buttonStartSelect  = pins["buttonStartSelect"];
    if (pins.containsKey("joyX"))         cfg.joyX         = pins["joyX"];
    if (pins.containsKey("joyY"))         cfg.joyY         = pins["joyY"];

  }

  if (doc.containsKey("axes")) {
    auto axes = doc["axes"];
    if (axes.containsKey("invertX")) cfg.invertX = axes["invertX"];
    if (axes.containsKey("invertY")) cfg.invertY = axes["invertY"];
  }

  if (doc.containsKey("config")) {
    auto config = doc["config"];
    if (config.containsKey("play_sound")) cfg.play_sound = config["play_sound"];
    if (config.containsKey("boot_debug")) cfg.boot_debug = config["boot_debug"];
    if (config.containsKey("right_not_left")) cfg.right_not_left = config["right_not_left"];
    if (config.containsKey("linux_mode")) cfg.linux_mode = config["linux_mode"];
    if (config.containsKey("stick_deadzone")) cfg.stick_deadzone = config["stick_deadzone"];
    if (config.containsKey("trigger_deadzone")) cfg.trigger_deadzone = config["trigger_deadzone"];
  }

  if (doc.containsKey("espnow_peer")) {
    const char* mac = doc["espnow_peer"];
    uint8_t out[6];
    if (!parseMac48(mac, out)) { errMsg = "Bad MAC format"; return false; }
    memcpy(cfg.peer_mac, out, 6);
  }
  if (doc.containsKey("wifi_channel")) {
    cfg.wifi_channel = doc["wifi_channel"];
  }
    if (doc.containsKey("esp_interval_us")) {
    cfg.esp_interval_us = doc["esp_interval_us"];
  }
  return true;
}
bool provisionOrLoad(Stream &io, unsigned long windowMs) {
  bool wrote = false;
  unsigned long t0 = millis();

  auto readLineWithTimeout = [&](unsigned long ms, String &out)->bool {
    unsigned long start = millis();
    out = "";
    while (millis() - start < ms) {
      while (io.available()) {
        char c = (char)io.read();
        start = millis(); // got a char, restart the timeout
        if (c == '\n' || c == '\r') {
          if (out.length() > 0) return true; // got a line
          // ignore leading CR/LF
        } else {
          out += c;
          if (out.length() > 1024) return true; // safety cap
        }
      }
      delay(1);
    }
    io.println("readLineWithTimeout - Timeout!");
    io.println("Payload: " + out);
    return false;
  };

  // Attempt to load in all cases. This allows partial JSON configs from user. 
  // Whatever is currently in NVS will override cfg defaults.
  loadConfigFromNVS();

  // Wait for first command line within window
  String cmd;
  if (!readLineWithTimeout(windowMs, cmd)) {
    // No command -> normal boot: load persisted config
    io.println("provisionOrLoad - No command -> just loadConfigFromNVS()");
    //loadConfigFromNVS();
    return false;
  }

  cmd.trim();
  // Normalize
  for (auto &ch : cmd) ch = toupper(ch);

  io.println("provisionOrLoad - got cmd: " + cmd);

  if (cmd == "READ") {
    //loadConfigFromNVS();
    io.println(configToJsonString()); // one-line JSON
    // ESP Now mac
    uint8_t mac[6] = {0};
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    io.printf("My ESP-NOW MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    io.println(F("OK"));
    return false; // read-only
  }

  if (cmd == "WRITE") {
    // Small extra window to get the JSON line
    String payload;
    if (!readLineWithTimeout(5000, payload)) {
      io.println(F("ERR missing JSON after WRITE"));
      // Still load persisted config to continue
      //loadConfigFromNVS();
      return false;
    }
    io.println("Payload: " + payload);
    String err;
    if (applyJsonToConfig(payload, err)) {
      saveConfigToNVS();
      io.println(F("OK"));
      wrote = true;
    } else {
      io.print(F("ERR "));
      io.println(err);
    }
    return wrote;
  }

  io.println("Warning [provisionOrLoad]. Unknown command: "+cmd);
  loadConfigFromNVS();

  return false;

}


String configToJsonString() {
  StaticJsonDocument<2024> doc;

  JsonObject pins = doc.createNestedObject("pins");
  pins["buttonAX"]      = cfg.buttonAX;
  pins["buttonBY"]      = cfg.buttonBY;
  pins["buttonBP"]     = cfg.buttonBP;
  pins["buttonST"]     = cfg.buttonST;
  pins["buttonStartSelect"]  = cfg.buttonStartSelect;
  pins["joyX"]         = cfg.joyX;
  pins["joyY"]         = cfg.joyY;

  JsonObject axes = doc.createNestedObject("axes");
  axes["invertX"] = cfg.invertX;
  axes["invertY"] = cfg.invertY;

  JsonObject config = doc.createNestedObject("config");
  config["play_sound"] = cfg.play_sound;
  config["boot_debug"] = cfg.boot_debug;
  config["right_not_left"] = cfg.right_not_left;
  config["stick_deadzone"] = cfg.stick_deadzone;
  config["trigger_deadzone"] = cfg.trigger_deadzone;
  config["linux_mode"] = cfg.linux_mode;

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           cfg.peer_mac[0], cfg.peer_mac[1], cfg.peer_mac[2],
           cfg.peer_mac[3], cfg.peer_mac[4], cfg.peer_mac[5]);
  doc["espnow_peer"] = macStr;
  doc["wifi_channel"] = cfg.wifi_channel;
  doc["esp_interval_us"] = cfg.esp_interval_us;

  String out;
  serializeJson(doc, out);  // compact one-line JSON
  return out;
}

void printConfigJson(Stream &out) {
  out.println(configToJsonString());
}


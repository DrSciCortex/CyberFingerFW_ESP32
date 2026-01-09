/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once
#include <Arduino.h>
#include <Preferences.h>

// Simple struct for your configuration
struct Config {
  int buttonAX = 43;
  int buttonBY = 44;
  int buttonBP = 38;
  int buttonST = 39;
  int buttonStartSelect  = 0;
  int joyX    = 17;
  int joyY    = 18;

  bool invertX = false;
  bool invertY = false;

  bool play_sound = true;
  bool boot_debug = false;
  bool right_not_left = true;
  bool linux_mode = false;

  uint8_t peer_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  uint16_t wifi_channel = 6;
  uint16_t esp_interval_us = 5000;
  uint16_t stick_deadzone = 250;
  uint16_t trigger_deadzone = 2;

};

// global instance
extern Config cfg;

// functions
void loadConfigFromNVS();
void saveConfigToNVS();
void printConfigJson(Stream &out);
bool applyJsonToConfig(const String& jsonLine, String& errMsg);

// Try to provision from a stream within a short window.
// Returns true if new config was received and saved, false otherwise.
// Returns true iff a WRITE happened (config changed). READ / timeout / unknown cmd return false.
bool provisionOrLoad(Stream &io, unsigned long windowMs = 500);

String configToJsonString();


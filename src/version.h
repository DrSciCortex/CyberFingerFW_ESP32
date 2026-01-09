/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

// version.h
#pragma once

/*
void printFirmwareVersion() {
    Serial.println(FW_VERSION_FULL_STR);
}
*/

// Semantic versioning
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 0

// Optional pre-release / metadata
// Set to empty string "" for official releases
#define FW_VERSION_SUFFIX "-beta"   // e.g. "-alpha", "-rc1", ""
//#define FW_VERSION_BUILD  "2025-03-15"  // or git hash
#define FW_VERSION_BUILD GIT_COMMIT_HASH

// Convenience macros
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// "1.4.1"
#define FW_VERSION_STR \
    STR(FW_VERSION_MAJOR) "." STR(FW_VERSION_MINOR) "." STR(FW_VERSION_PATCH)

// "1.4.1-beta+2025-03-15"
#define FW_VERSION_FULL_STR \
    FW_VERSION_STR FW_VERSION_SUFFIX "+" FW_VERSION_BUILD

#pragma once
#include <stdint.h>
#include <stdbool.h>
/* Sector 11 is excluded from the application linker region. Never erase in flight. */
#define MISSION_STORE_ADDRESS 0x080E0000UL
#define MISSION_STORE_SIZE (128U * 1024U)
typedef enum { MISSION_STORE_EMPTY, MISSION_STORE_VALID, MISSION_STORE_ERROR } MissionStoreResult;
/* Payload: state bits 0..7, legacy ignored bit 8, HDRM complete bit 9,
 * legacy ground pressure bits 10..26 accepted for old records but ignored by
 * the application. New calibration lives in CalibrationStore records. */
#define MISSION_PRESSURE_SHIFT 10U
#define MISSION_PRESSURE_MASK 0x1ffffU
MissionStoreResult MissionStore_Load(uint32_t *payload);
bool MissionStore_Save(uint32_t payload);
/* Shared append allocator for versioned records, 4 or 8 words. Last word is
 * the caller's commit marker. Both journals share sector 11 without erasing. */
bool MissionStore_AppendWords(const uint32_t *words, uint32_t count);
/* Backend split permits power-interruption tests without hardware. */
uint32_t MissionStore_ReadWord(uint32_t offset);
bool MissionStore_WriteWord(uint32_t offset, uint32_t value);

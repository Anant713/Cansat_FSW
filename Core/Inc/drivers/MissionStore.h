#pragma once
#include <stdint.h>
#include <stdbool.h>
/* Sector 11 is excluded from the application linker region. Never erase in flight. */
#define MISSION_STORE_ADDRESS 0x080E0000UL
#define MISSION_STORE_SIZE (128U * 1024U)
typedef enum { MISSION_STORE_EMPTY, MISSION_STORE_VALID, MISSION_STORE_ERROR } MissionStoreResult;
/* Payload: state bits 0..7, legacy ignored bit 8, HDRM complete bit 9,
 * rounded ground pressure in Pa bits 10..26 (zero = unset). */
#define MISSION_PRESSURE_SHIFT 10U
#define MISSION_PRESSURE_MASK 0x1ffffU
MissionStoreResult MissionStore_Load(uint32_t *payload);
bool MissionStore_Save(uint32_t payload);
/* Backend split permits power-interruption tests without hardware. */
uint32_t MissionStore_ReadWord(uint32_t offset);
bool MissionStore_WriteWord(uint32_t offset, uint32_t value);

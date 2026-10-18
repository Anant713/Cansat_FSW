#include "drivers/MissionStore.h"
#include "stm32f4xx_hal.h"

uint32_t MissionStore_ReadWord(uint32_t offset)
{
    return *(volatile const uint32_t *)(uintptr_t)(MISSION_STORE_ADDRESS + offset);
}
bool MissionStore_WriteWord(uint32_t offset, uint32_t value)
{
    if ((offset & 3U) || offset >= MISSION_STORE_SIZE ||
        MissionStore_ReadWord(offset) != UINT32_MAX) return false;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    HAL_StatusTypeDef result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                                MISSION_STORE_ADDRESS + offset, value);
    HAL_StatusTypeDef locked = HAL_FLASH_Lock();
    /* The startup scan may have cached erased words. Polling Program does not
     * flush the F4 flash caches; invalidate before readback/recovery scans. */
    FLASH_FlushCaches();
    return result == HAL_OK && locked == HAL_OK && MissionStore_ReadWord(offset) == value;
}

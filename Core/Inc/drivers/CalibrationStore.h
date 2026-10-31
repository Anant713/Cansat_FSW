#pragma once
#include <stdbool.h>

typedef struct {
    float groundPressurePa;
    float gyroBiasDps[3];
} SensorCalibrationRecord;
typedef enum { CAL_STORE_EMPTY, CAL_STORE_VALID, CAL_STORE_ERROR } CalibrationStoreResult;

/* Versioned, CRC-protected records in the shared sector-11 append journal.
 * No erase; latest complete valid record wins. Values are one atomic tuple.
 * A valid prior record is retained if a newer append is torn/corrupt. */
CalibrationStoreResult CalibrationStore_Load(SensorCalibrationRecord *record);
bool CalibrationStore_Save(const SensorCalibrationRecord *record);

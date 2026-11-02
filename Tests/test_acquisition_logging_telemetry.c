/* Same acquisition/logging harness and periods, with scheduled XBee telemetry.
 * Build this entry point OR test_acquisition_logging.c, never both. */
#define TEST_WITH_TELEMETRY 1
/* Shared harness calibrates/saves ground pressure and gyro on launch-pad startup. */
#include "test_acquisition_logging.c"

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#ifndef HAL_BARO_WIND_COMP_ENABLED
#define HAL_BARO_WIND_COMP_ENABLED 1
#endif

#ifndef AP_BARO_ENABLED
#define AP_BARO_ENABLED 1
#endif

// backend support:
#ifndef AP_BARO_BACKEND_DEFAULT_ENABLED
#define AP_BARO_BACKEND_DEFAULT_ENABLED 1
#endif

#ifndef AP_BARO_BMP085_ENABLED
#define AP_BARO_BMP085_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_BMP280_ENABLED
#define AP_BARO_BMP280_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_BMP388_ENABLED
#define AP_BARO_BMP388_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_DPS280_ENABLED
#define AP_BARO_DPS280_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_DUMMY_ENABLED
#define AP_BARO_DUMMY_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_EXTERNALAHRS_ENABLED
#define AP_BARO_EXTERNALAHRS_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#ifndef AP_BARO_FBM320_ENABLED
#define AP_BARO_FBM320_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_ICM20789_ENABLED
#define AP_BARO_ICM20789_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_ICP101XX_ENABLED
#define AP_BARO_ICP101XX_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_ICP201XX_ENABLED
#define AP_BARO_ICP201XX_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_KELLERLD_ENABLED
#define AP_BARO_KELLERLD_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_LPS2XH_ENABLED
#define AP_BARO_LPS2XH_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_MS56XX_ENABLED
#define AP_BARO_MS56XX_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_MSP_ENABLED
#define AP_BARO_MSP_ENABLED HAL_MSP_SENSORS_ENABLED
#endif

#ifndef AP_SIM_BARO_ENABLED
#define AP_SIM_BARO_ENABLED AP_SIM_ENABLED
#endif

#ifndef AP_BARO_SPL06_ENABLED
#define AP_BARO_SPL06_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BARO_DRONECAN_ENABLED
#define AP_BARO_DRONECAN_ENABLED (AP_BARO_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS)
#endif


#ifndef AP_BARO_PROBE_EXTERNAL_I2C_BUSES
#define AP_BARO_PROBE_EXTERNAL_I2C_BUSES 1
#endif

#ifndef AP_BARO_PROBE_EXT_PARAMETER_ENABLED
#define AP_BARO_PROBE_EXT_PARAMETER_ENABLED AP_BARO_PROBE_EXTERNAL_I2C_BUSES || AP_BARO_MSP_ENABLED
#endif

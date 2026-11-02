#ifndef Q3_CALIBRATION_PROFILE_H
#define Q3_CALIBRATION_PROFILE_H

#include <stdint.h>

#define Q3_CALIBRATION_SOURCE_CUSTOM            0U
#define Q3_CALIBRATION_SOURCE_GENERATOR_SCOPE   1U
#define Q3_CALIBRATION_SOURCE_GENERATOR_DIRECT  2U
#define Q3_CALIBRATION_SOURCE_OSCILLOSCOPE      3U
#define Q3_CALIBRATION_SOURCE_AN9238_NOMINAL    4U
#define Q3_CALIBRATION_SOURCE_MAX               \
    Q3_CALIBRATION_SOURCE_AN9238_NOMINAL

/*
 * AN9238 SMA nominal range is -5 V..+5 V, represented by 4096 ADC codes.
 * This is an unmeasured starting point. Both channels need independent
 * measured gain/offset calibration before precision claims are made.
 * Legacy source identities are retained for the future UI protocol, but
 * their initial values must not carry the old board's measured gain.
 */
#define Q3_CALIBRATION_BASE_MV_PER_CODE (10000.0f / 4096.0f)
#define Q3_CALIBRATION_BASE_ZERO_OFFSET_MV 0.0f

#define Q3_CALIBRATION_GENERATOR_SCOPE_MV_PER_CODE \
    Q3_CALIBRATION_BASE_MV_PER_CODE
#define Q3_CALIBRATION_GENERATOR_SCOPE_ZERO_OFFSET_MV \
    Q3_CALIBRATION_BASE_ZERO_OFFSET_MV
#define Q3_CALIBRATION_GENERATOR_DIRECT_MV_PER_CODE \
    Q3_CALIBRATION_BASE_MV_PER_CODE
#define Q3_CALIBRATION_GENERATOR_DIRECT_ZERO_OFFSET_MV \
    Q3_CALIBRATION_BASE_ZERO_OFFSET_MV
#define Q3_CALIBRATION_OSCILLOSCOPE_MV_PER_CODE \
    Q3_CALIBRATION_BASE_MV_PER_CODE
#define Q3_CALIBRATION_OSCILLOSCOPE_ZERO_OFFSET_MV \
    Q3_CALIBRATION_BASE_ZERO_OFFSET_MV

#define Q3_CALIBRATION_STATUS_IDLE          0U
#define Q3_CALIBRATION_STATUS_APPLIED       1U
#define Q3_CALIBRATION_STATUS_INVALID       2U

typedef struct {
    uint32_t source;
    float mv_per_code;
    float zero_offset_mv;
} q3_calibration_profile_t;

#endif /* Q3_CALIBRATION_PROFILE_H */

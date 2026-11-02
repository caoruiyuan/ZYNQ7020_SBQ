#include "q3_calibration.h"

#include <stddef.h>

/* Provisional AN9238 connector scale; replace after per-channel bench calibration. */
static q3_calibration_profile_t ActiveProfile = {
    Q3_CALIBRATION_SOURCE_AN9238_NOMINAL,
    Q3_CALIBRATION_BASE_MV_PER_CODE,
    Q3_CALIBRATION_BASE_ZERO_OFFSET_MV
};

int q3_calibration_profile_is_valid(
    const q3_calibration_profile_t *profile)
{
    if (profile == NULL) {
        return 0;
    }
    if (profile->source > Q3_CALIBRATION_SOURCE_MAX) {
        return 0;
    }
    /*
     * Comparisons also reject NaN. The broad limits catch corrupt mailbox or
     * Flash values while leaving ample room for future analogue front ends.
     */
    if (!(profile->mv_per_code >= 0.000001f) ||
        !(profile->mv_per_code <= 5.0f) ||
        !(profile->zero_offset_mv >= -10000.0f) ||
        !(profile->zero_offset_mv <= 10000.0f)) {
        return 0;
    }
    return 1;
}

int q3_calibration_set_profile(
    const q3_calibration_profile_t *profile)
{
    if (!q3_calibration_profile_is_valid(profile)) {
        return -1;
    }

    ActiveProfile = *profile;
    return 0;
}

void q3_calibration_get_profile(q3_calibration_profile_t *profile)
{
    if (profile != NULL) {
        *profile = ActiveProfile;
    }
}

float q3_calibration_adc_codes_to_mv(float adc_codes)
{
    return adc_codes * ActiveProfile.mv_per_code -
           ActiveProfile.zero_offset_mv;
}

float q3_calibration_mv_to_adc_codes(float millivolts)
{
    return (millivolts + ActiveProfile.zero_offset_mv) /
           ActiveProfile.mv_per_code;
}

float q3_calibration_ac_codes_to_mv(float ac_codes)
{
    return ac_codes * ActiveProfile.mv_per_code;
}

float q3_calibration_amplitude_codes_to_peak_mv(float amplitude_codes)
{
    const float magnitude =
        (amplitude_codes < 0.0f) ? -amplitude_codes : amplitude_codes;

    return q3_calibration_ac_codes_to_mv(magnitude);
}

float q3_calibration_peak_mv_to_amplitude_codes(float peak_mv)
{
    const float magnitude = (peak_mv < 0.0f) ? -peak_mv : peak_mv;

    return magnitude / ActiveProfile.mv_per_code;
}

float q3_calibration_amplitude_codes_to_vpp_mv(float amplitude_codes)
{
    return 2.0f *
           q3_calibration_amplitude_codes_to_peak_mv(amplitude_codes);
}

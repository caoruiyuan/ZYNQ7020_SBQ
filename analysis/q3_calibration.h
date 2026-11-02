#ifndef Q3_CALIBRATION_H
#define Q3_CALIBRATION_H

#include "q3_calibration_profile.h"

/*
 * Runtime profile control. The unmeasured AN9238 nominal profile is active
 * after reset; the application can replace it after bench calibration.
 */
int q3_calibration_profile_is_valid(
    const q3_calibration_profile_t *profile);
int q3_calibration_set_profile(
    const q3_calibration_profile_t *profile);
void q3_calibration_get_profile(q3_calibration_profile_t *profile);

/*
 * Signed sample conversion includes the profile's DC zero offset.
 */
float q3_calibration_adc_codes_to_mv(float adc_codes);
float q3_calibration_mv_to_adc_codes(float millivolts);

/*
 * AC-amplitude conversion. A least-squares sinusoid amplitude is a positive
 * peak magnitude, so a DC zero offset must never be added or subtracted.
 * Spectrum values shown to the user are explicitly peak-to-peak values.
 */
float q3_calibration_ac_codes_to_mv(float ac_codes);
float q3_calibration_amplitude_codes_to_peak_mv(float amplitude_codes);
float q3_calibration_peak_mv_to_amplitude_codes(float peak_mv);
float q3_calibration_amplitude_codes_to_vpp_mv(float amplitude_codes);

#endif /* Q3_CALIBRATION_H */

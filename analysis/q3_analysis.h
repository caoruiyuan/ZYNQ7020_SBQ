#ifndef Q3_ANALYSIS_H
#define Q3_ANALYSIS_H

#include "q3_analysis_control.h"

#include <stdint.h>

#define Q3_ANALYSIS_MAX_COMPONENTS 9U

/*
 * Runtime component threshold in tenths of one percent. Zero disables the
 * relative filter and fixed 0.5 mV peak-amplitude floor; adaptive-noise
 * detection remains active. For example, 15 means 1.5%.
 */
int q3_analysis_set_component_threshold_tenths_percent(
    uint32_t tenths_percent);
uint32_t q3_analysis_get_component_threshold_tenths_percent(void);

typedef struct {
    uint32_t valid;
    uint32_t component_count;
    uint32_t harmonics[Q3_ANALYSIS_MAX_COMPONENTS];
    float component_frequency_hz[Q3_ANALYSIS_MAX_COMPONENTS];
    float component_amplitude_codes[Q3_ANALYSIS_MAX_COMPONENTS];
    float component_phase_rad[Q3_ANALYSIS_MAX_COMPONENTS];
    float component_relative_phase_rad[Q3_ANALYSIS_MAX_COMPONENTS];
    float fundamental_bin;
    float fundamental_hz;
    float fundamental_phase_rad;
    float fundamental_amplitude_codes;
    float peak_to_peak_codes;
    float rms_codes;
    float waveform_max_abs_codes;
} q3_analysis_result_t;

/*
 * MATLAB filtered_ls equivalent operating on the PL packet:
 *   - local spectral peaks with a median-noise threshold
 *   - up to twenty-four candidates separated by eight FFT bins
 *   - configurable off/0.1%..10.0% component pool in 0.1% steps
 *   - fixed five-percent H1 qualification
 *   - integer-harmonic model selection (H1..H64, two-bin tolerance)
 *   - up to nine retained components ordered by harmonic number
 *   - three-bin logarithmic parabolic interpolation
 *   - power-weighted multi-harmonic fundamental estimate
 *   - two phase-drift refinement iterations
 *   - unwindowed time-domain harmonic least-squares fit
 *
 * For nonzero settings, the MATLAB 0.5 mV peak-amplitude threshold is
 * converted through the centralized AC-amplitude calibration function and
 * the FFT block exponent.
 */
int q3_analysis_run(const int32_t *time_samples,
                    uint32_t time_sample_count,
                    const uint32_t *fft_words,
                    uint32_t fft_slot_count,
                    uint32_t fft_first_bin,
                    uint32_t fft_length,
                    uint32_t fft_block_exponent,
                    float sample_rate_hz,
                    uint16_t *display_spectrum,
                    uint32_t display_point_count,
                    q3_analysis_result_t *result);

float q3_analysis_waveform_at_phase(
    const q3_analysis_result_t *result, float phase_rad);

#endif /* Q3_ANALYSIS_H */

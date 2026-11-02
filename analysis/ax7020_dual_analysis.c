#include "ax7020_dual_analysis.h"

#include <string.h>

/* Core 1 calls this synchronously; keep the 144 KiB work area off its stack. */
static int32_t TimeCh1[AX7020_FRAME_SAMPLES];
static int32_t TimeCh2[AX7020_FRAME_SAMPLES];
static int32_t FftCh1[AX7020_FFT_VALID_SLOTS * 2U];
static int32_t FftCh2[AX7020_FFT_VALID_SLOTS * 2U];

#define AX7020_MAX_DISPLAY_POINTS 128U

int ax7020_dual_analysis_run(const uint32_t *packet, size_t word_count,
                             uint16_t *ch1_spectrum,
                             uint16_t *ch2_spectrum,
                             uint32_t display_point_count,
                             ax7020_dual_analysis_result_t *result)
{
    if (result == NULL || ch1_spectrum == NULL || ch2_spectrum == NULL ||
        display_point_count == 0U ||
        display_point_count > AX7020_MAX_DISPLAY_POINTS) {
        return -1;
    }
    (void)memset(result, 0, sizeof(*result));
    (void)memset(ch1_spectrum, 0,
                 display_point_count * sizeof(*ch1_spectrum));
    (void)memset(ch2_spectrum, 0,
                 display_point_count * sizeof(*ch2_spectrum));
    if (ax7020_packet_validate(packet, word_count, &result->packet) != 0 ||
        ax7020_packet_unpack_time(packet, word_count, TimeCh1, TimeCh2,
                                  AX7020_FRAME_SAMPLES) != 0 ||
        ax7020_packet_split_fft(packet, word_count, FftCh1, FftCh2,
                                AX7020_FFT_VALID_SLOTS * 2U) != 0) {
        return -1;
    }

    if (q3_analysis_run(TimeCh1, AX7020_FRAME_SAMPLES,
                        (const uint32_t *)FftCh1, AX7020_FFT_VALID_SLOTS,
                        AX7020_FFT_FIRST_BIN, AX7020_FRAME_SAMPLES,
                        result->packet.fft_exponent,
                        (float)result->packet.sample_rate_hz,
                        ch1_spectrum, display_point_count,
                        &result->ch1) != 0 ||
        q3_analysis_run(TimeCh2, AX7020_FRAME_SAMPLES,
                        (const uint32_t *)FftCh2, AX7020_FFT_VALID_SLOTS,
                        AX7020_FFT_FIRST_BIN, AX7020_FRAME_SAMPLES,
                        result->packet.fft_exponent,
                        (float)result->packet.sample_rate_hz,
                        ch2_spectrum, display_point_count,
                        &result->ch2) != 0) {
        return -2;
    }
    return 0;
}

#ifndef AX7020_DUAL_ANALYSIS_H
#define AX7020_DUAL_ANALYSIS_H

#include "q3_analysis.h"
#include "../common/ax7020_dual_packet.h"

typedef struct {
    ax7020_packet_info_t packet;
    q3_analysis_result_t ch1;
    q3_analysis_result_t ch2;
} ax7020_dual_analysis_result_t;

/* Both spectra have display_point_count entries. Result codes are in
 * ADC-equivalent units and use the provisional AN9238 nominal calibration. */
int ax7020_dual_analysis_run(const uint32_t *packet, size_t word_count,
                             uint16_t *ch1_spectrum,
                             uint16_t *ch2_spectrum,
                             uint32_t display_point_count,
                             ax7020_dual_analysis_result_t *result);

#endif

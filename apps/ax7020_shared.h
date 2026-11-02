#ifndef AX7020_SHARED_H
#define AX7020_SHARED_H

#include <stdint.h>

#define AX7020_DMA_ADDR        0x11000000U
#define AX7020_MAILBOX_ADDR    0x11100000U
#define AX7020_CORE1_ENTRY    0x18000000U
#define AX7020_FRAMEBUFFER_0  0x10000000U
#define AX7020_FRAMEBUFFER_1  0x10200000U
#define AX7020_MAILBOX_MAGIC  0x41374D42U
#define AX7020_WAVE_POINTS    320U
#define AX7020_SPECTRUM_POINTS 128U
#define AX7020_COMPONENTS_MAX 9U
#define AX7020_MAILBOX_VERSION 7U

#define AX7020_FLAG_VALID          0x01U
#define AX7020_FLAG_DMA_ERROR      0x02U
#define AX7020_FLAG_DMA_TIMEOUT    0x04U
#define AX7020_FLAG_PACKET_ERROR   0x08U
#define AX7020_FLAG_PL_ERROR       0x10U
#define AX7020_FLAG_ANALYSIS_ERROR 0x20U

typedef struct {
    uint32_t order;
    float frequency_hz;
    float peak_mv;
} ax7020_component_t;

typedef struct {
    float frequency_hz;
    float peak_to_peak_mv;
    float rms_mv;
    float dc_mv;
    float maximum_mv;
    float minimum_mv;
    float adc_mean_codes;
    uint32_t overrange_count;
    uint32_t component_count;
    ax7020_component_t components[AX7020_COMPONENTS_MAX];
    int16_t waveform_codes[AX7020_WAVE_POINTS];
    uint16_t spectrum[AX7020_SPECTRUM_POINTS];
} ax7020_channel_snapshot_t;

typedef struct {
    uint32_t frame_sequence;
    uint32_t flags;
    uint32_t pl_status;
    uint32_t dma_status;
    uint32_t packet_debug[8];
    uint32_t sample_rate_hz;
    uint32_t waveform_sample_rate_hz;
    uint32_t acquisition_us;
    uint32_t processing_us;
    float waveform_span_us;
    uint32_t spectrum_span_hz;
    uint32_t waveform_triggered;
    uint32_t waveform_independent;
    uint32_t capture_mode;
    uint32_t calibration_ack;
    uint32_t calibration_status;
    float calibration_zero_codes[2];
    float calibration_mv_per_code[2];
    ax7020_channel_snapshot_t channel[2];
} ax7020_snapshot_t;

typedef struct __attribute__((aligned(64))) {
    uint32_t magic;
    uint32_t version;
    uint32_t published_slot;
    uint32_t published_sequence;
    uint32_t heartbeat;
    uint32_t capture_enabled;
    uint32_t waveform_cycles;
    uint32_t trigger_channel;
    uint32_t auto_enabled;
    int32_t trigger_level_codes;
    uint32_t trigger_rising;
    uint32_t trigger_normal;
    uint32_t independent_phase_display;
    float requested_span_us;
    uint32_t component_threshold_tenths_percent;
    uint32_t capture_mode_request;
    uint32_t calibration_request;
    uint32_t calibration_action;
    uint32_t calibration_channel;
    float calibration_reference_vpp_mv;
    float calibration_zero_codes[2];
    float calibration_mv_per_code[2];
    ax7020_snapshot_t slots[2];
} ax7020_mailbox_t;

#endif

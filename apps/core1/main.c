#include "ax7020_shared.h"
#include "ax7020_dual_analysis.h"
#include "ax7020_waveform_trigger.h"

#include "sleep.h"
#include "xaxidma.h"
#include "xgpio.h"
#include "xil_mmu.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xiltimer.h"

#include <math.h>
#include <string.h>

#define DMA_TIMEOUT_US 500000U
#define NOMINAL_MV_PER_CODE (10000.0f / 4096.0f)

static XAxiDma dma;
static XGpio control;
static volatile ax7020_mailbox_t *const mailbox =
    (volatile ax7020_mailbox_t *)AX7020_MAILBOX_ADDR;
static const uint32_t *const packet = (const uint32_t *)AX7020_DMA_ADDR;
static ax7020_dual_analysis_result_t analysis;
static ax7020_snapshot_t snapshot;
static uint32_t capture_slow_mode = 1U;
static uint32_t trigger_state;
static unsigned mode_stable_frames;
static int16_t previous_waveform[2][AX7020_WAVE_POINTS];
static float zero_codes[2];
static float mv_per_code[2] = {NOMINAL_MV_PER_CODE, NOMINAL_MV_PER_CODE};
static uint32_t calibration_ack;
static uint32_t calibration_status;
static float previous_span_us;
static uint32_t previous_waveform_rate_hz;
static int previous_waveform_valid;
static unsigned previous_trigger_channel;
static unsigned previous_capture_mode;
static unsigned previous_independent_requested;
static unsigned previous_waveform_independent;
static float automatic_span_us;
static unsigned automatic_span_channel = 2U;

static uint32_t elapsed_us(XTime start)
{
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(((now - start) * 1000000ULL) / COUNTS_PER_SECOND);
}

static int reset_dma(void)
{
    unsigned retry;
    XAxiDma_Reset(&dma);
    for (retry = 0; retry < 100000U; ++retry) {
        if (XAxiDma_ResetIsDone(&dma)) {
            return XST_SUCCESS;
        }
    }
    return XST_FAILURE;
}

static void reset_pipeline(void)
{
    trigger_state = 0U;
    XGpio_DiscreteWrite(&control, 1, (capture_slow_mode << 2) | 2U);
    usleep(10U);
    XGpio_DiscreteWrite(&control, 1, capture_slow_mode << 2);
}

static int initialize_hardware(void)
{
    XAxiDma_Config *config;
    int status;

    Xil_SetTlbAttributes(AX7020_DMA_ADDR, NORM_NONCACHE);
    Xil_SetTlbAttributes(AX7020_MAILBOX_ADDR, NORM_NONCACHE);
#ifdef SDT
    status = XGpio_Initialize(&control, XPAR_XGPIO_0_BASEADDR);
#else
    status = XGpio_Initialize(&control, XPAR_XGPIO_0_DEVICE_ID);
#endif
    if (status != XST_SUCCESS) return status;
    XGpio_SetDataDirection(&control, 1, 0U);
    XGpio_SetDataDirection(&control, 2, 0xffffffffU);
    XGpio_DiscreteWrite(&control, 1, capture_slow_mode << 2);

#ifdef SDT
    config = XAxiDma_LookupConfig(XPAR_XAXIDMA_0_BASEADDR);
#else
    config = XAxiDma_LookupConfig(XPAR_XAXIDMA_0_DEVICE_ID);
#endif
    if (config == NULL) return XST_DEVICE_NOT_FOUND;
    status = XAxiDma_CfgInitialize(&dma, config);
    if (status != XST_SUCCESS || XAxiDma_HasSg(&dma) || !config->HasS2Mm)
        return XST_FAILURE;
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    status = reset_dma();
    if (status == XST_SUCCESS) reset_pipeline();
    return status;
}

static void publish(void)
{
    uint32_t next = (mailbox->published_slot ^ 1U) & 1U;
    mailbox->slots[next] = snapshot;
    dmb();
    mailbox->published_slot = next;
    mailbox->published_sequence++;
    mailbox->heartbeat++;
}

static int packet_sample(unsigned channel, unsigned index)
{
    uint32_t pair = packet[AX7020_TIME_FIRST_WORD + index];
    return (int16_t)(channel ? (pair >> 16) : pair);
}

static int raw_packet_sample(unsigned channel, unsigned index)
{
    uint32_t pair = packet[AX7020_RAW_FIRST_WORD + index];
    return (int16_t)(channel ? (pair >> 16) : pair);
}

static int has_fundamental(const q3_analysis_result_t *result)
{
    return result->valid && result->fundamental_amplitude_codes >= 4.0f &&
           isfinite(result->fundamental_hz) &&
           isfinite(result->fundamental_phase_rad) &&
           result->fundamental_hz >= 80.0f &&
           result->fundamental_hz <= 200000.0f;
}

static float choose_automatic_span(float frequency_hz, unsigned cycles,
                                   unsigned channel)
{
    static const float divisions_us[] = {0.125f, 0.25f, 0.5f, 1.0f,
        2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f, 200.0f,
        500.0f, 1000.0f, 2000.0f, 4000.0f};
    float desired = (float)cycles * 1000000.0f / frequency_hz;
    unsigned i;
    if (automatic_span_channel == channel && automatic_span_us > 0.0f &&
        desired >= automatic_span_us * 0.60f &&
        desired <= automatic_span_us * 0.95f)
        return automatic_span_us;
    for (i = 0U; i < sizeof(divisions_us) / sizeof(divisions_us[0]); ++i) {
        if (8.0f * divisions_us[i] >= desired) break;
    }
    if (i == sizeof(divisions_us) / sizeof(divisions_us[0])) --i;
    automatic_span_us = 8.0f * divisions_us[i];
    automatic_span_channel = channel;
    return automatic_span_us;
}

static void fill_waveforms(void)
{
    float start = 0.0f;
    float channel_start[2];
    float requested_span = mailbox->requested_span_us;
    float span_us = (float)(AX7020_FRAME_SAMPLES - 1U) * 1000000.0f /
                    (float)snapshot.sample_rate_hz;
    float raw_frame_us = (float)(AX7020_FRAME_SAMPLES - 1U) * 1000000.0f /
                         (float)AX7020_ADC_CLOCK_HZ;
    float span;
    float display_rate;
    unsigned reference = mailbox->trigger_channel & 1U;
    unsigned cycles = mailbox->waveform_cycles == 3U ? 3U : 1U;
    const q3_analysis_result_t *results[2] = {&analysis.ch1, &analysis.ch2};
    int trigger_found = 0;
    int use_raw;
    unsigned channel, i;

    if (isfinite(requested_span) && requested_span > 0.0f) {
        span_us = requested_span;
    } else if (has_fundamental(results[reference])) {
        span_us = choose_automatic_span(results[reference]->fundamental_hz,
                                        cycles, reference);
    }
    use_raw = span_us <= raw_frame_us || mailbox->capture_mode_request == 3U;
    display_rate = use_raw ? (float)AX7020_ADC_CLOCK_HZ :
                             (float)snapshot.sample_rate_hz;
    span = span_us * display_rate / 1000000.0f;
    if (span > (float)(AX7020_FRAME_SAMPLES - 1U))
        span = (float)(AX7020_FRAME_SAMPLES - 1U);
    if (span < 1.0f) span = 1.0f;

    if (span < (float)(AX7020_FRAME_SAMPLES - 1U)) {
        int level = (int)lroundf(zero_codes[reference] +
            (float)mailbox->trigger_level_codes * NOMINAL_MV_PER_CODE /
            mv_per_code[reference]);
        unsigned last = (unsigned)((float)(AX7020_FRAME_SAMPLES - 1U) - span);
        trigger_found = ax7020_find_trigger(
            packet + (use_raw ? AX7020_RAW_FIRST_WORD : AX7020_TIME_FIRST_WORD),
            AX7020_FRAME_SAMPLES, reference, last, level,
            mailbox->trigger_rising != 0U, &start);
    }
    if (!trigger_found && mailbox->trigger_normal && previous_waveform_valid &&
        previous_trigger_channel == reference &&
        previous_capture_mode == (unsigned)use_raw &&
        previous_independent_requested ==
            mailbox->independent_phase_display &&
        fabsf(previous_span_us - span_us) < span_us * 0.02f) {
        for (channel = 0U; channel < 2U; ++channel)
            memcpy(snapshot.channel[channel].waveform_codes,
                   previous_waveform[channel], sizeof(previous_waveform[channel]));
        snapshot.waveform_span_us = previous_span_us;
        snapshot.waveform_sample_rate_hz = previous_waveform_rate_hz;
        snapshot.waveform_independent = previous_waveform_independent;
        return;
    }
    snapshot.waveform_span_us = span * 1000000.0f / display_rate;
    snapshot.waveform_sample_rate_hz = (uint32_t)display_rate;
    snapshot.waveform_triggered = trigger_found ? 1U : 0U;
    channel_start[0] = start;
    channel_start[1] = start;
    if (trigger_found && mailbox->independent_phase_display != 0U &&
        span < (float)(AX7020_FRAME_SAMPLES - 1U)) {
        unsigned other = reference ^ 1U;
        unsigned last = (unsigned)((float)(AX7020_FRAME_SAMPLES - 1U) - span);
        int level = (int)lroundf(snapshot.channel[other].adc_mean_codes);
        float other_start;
        if (ax7020_find_trigger(
                packet + (use_raw ? AX7020_RAW_FIRST_WORD : AX7020_TIME_FIRST_WORD),
                AX7020_FRAME_SAMPLES, other, last, level,
                mailbox->trigger_rising != 0U, &other_start)) {
            channel_start[other] = other_start;
            snapshot.waveform_independent = 1U;
        }
    }
    for (channel = 0U; channel < 2U; ++channel) {
        ax7020_channel_snapshot_t *out = &snapshot.channel[channel];
        for (i = 0U; i < AX7020_WAVE_POINTS; ++i) {
            float source = channel_start[channel] + (float)i * span /
                                   (float)(AX7020_WAVE_POINTS - 1U);
            unsigned index = (unsigned)source;
            float value;
            if (use_raw) {
                float fraction = source - (float)index;
                float first = (float)raw_packet_sample(channel, index);
                float second = (float)raw_packet_sample(channel,
                    index < AX7020_FRAME_SAMPLES - 1U ? index + 1U : index);
                value = first + fraction * (second - first);
            } else {
                float fraction = source - (float)index;
                float first = (float)packet_sample(channel, index);
                float second = (float)packet_sample(channel,
                    index < AX7020_FRAME_SAMPLES - 1U ? index + 1U : index);
                value = first + fraction * (second - first);
            }
            value = (value - zero_codes[channel]) *
                    mv_per_code[channel] / NOMINAL_MV_PER_CODE;
            out->waveform_codes[i] = (int16_t)lroundf(value);
        }
        memcpy(previous_waveform[channel], out->waveform_codes,
               sizeof(previous_waveform[channel]));
    }
    previous_span_us = snapshot.waveform_span_us;
    previous_waveform_rate_hz = snapshot.waveform_sample_rate_hz;
    previous_trigger_channel = reference;
    previous_capture_mode = (unsigned)use_raw;
    previous_independent_requested = mailbox->independent_phase_display;
    previous_waveform_independent = snapshot.waveform_independent;
    previous_waveform_valid = 1;
}

static void update_capture_mode(void)
{
    const q3_analysis_result_t *results[2] = {&analysis.ch1, &analysis.ch2};
    const q3_analysis_result_t *reference = results[mailbox->trigger_channel & 1U];
    float frequency = has_fundamental(reference) ?
                      reference->fundamental_hz : 0.0f;

    if (mailbox->capture_mode_request == 1U ||
        mailbox->capture_mode_request == 3U) {
        capture_slow_mode = 0U;
        return;
    }
    if (mailbox->capture_mode_request == 2U) {
        capture_slow_mode = 1U;
        return;
    }
    if (!mailbox->auto_enabled) return;
    if (mailbox->requested_span_us > 2000.0f) {
        capture_slow_mode = 1U;
        mode_stable_frames = 0U;
        return;
    }
    if (capture_slow_mode) {
        mode_stable_frames = frequency >= 10000.0f ?
                             mode_stable_frames + 1U : 0U;
        if (mode_stable_frames >= 4U) {
            capture_slow_mode = 0U;
            mode_stable_frames = 0U;
        }
    } else {
        mode_stable_frames = frequency < 6000.0f ?
                             mode_stable_frames + 1U : 0U;
        if (mode_stable_frames >= 4U) {
            capture_slow_mode = 1U;
            mode_stable_frames = 0U;
        }
    }
}

static void fill_channel(unsigned channel, const q3_analysis_result_t *result)
{
    ax7020_channel_snapshot_t *out = &snapshot.channel[channel];
    double sum = 0.0;
    double squares = 0.0;
    int minimum = 32767;
    int maximum = -32768;
    unsigned i;

    for (i = 0; i < AX7020_FRAME_SAMPLES; ++i) {
        int sample = packet_sample(channel, i);
        if (sample < minimum) minimum = sample;
        if (sample > maximum) maximum = sample;
        sum += sample;
        squares += (double)sample * sample;
    }
    out->dc_mv = ((float)(sum / AX7020_FRAME_SAMPLES) -
                  zero_codes[channel]) * mv_per_code[channel];
    out->adc_mean_codes = (float)(sum / AX7020_FRAME_SAMPLES);
    out->minimum_mv = ((float)minimum - zero_codes[channel]) * mv_per_code[channel];
    out->maximum_mv = ((float)maximum - zero_codes[channel]) * mv_per_code[channel];
    out->frequency_hz = has_fundamental(result) ?
        result->fundamental_hz : 0.0f;
    out->peak_to_peak_mv = has_fundamental(result) ?
        result->peak_to_peak_codes * mv_per_code[channel] :
        (float)(maximum - minimum) * mv_per_code[channel];
    out->rms_mv = has_fundamental(result) ?
        result->rms_codes * mv_per_code[channel] :
        (float)sqrt(squares / AX7020_FRAME_SAMPLES -
                    2.0 * zero_codes[channel] * (sum / AX7020_FRAME_SAMPLES) +
                    zero_codes[channel] * zero_codes[channel]) * mv_per_code[channel];
    out->overrange_count = channel ? analysis.packet.ch2_overrange_count :
                                     analysis.packet.ch1_overrange_count;
    out->component_count = has_fundamental(result) ?
        result->component_count : 0U;
    if (out->component_count > AX7020_COMPONENTS_MAX)
        out->component_count = AX7020_COMPONENTS_MAX;
    for (i = 0; i < out->component_count; ++i) {
        out->components[i].order = result->harmonics[i];
        out->components[i].frequency_hz = result->component_frequency_hz[i];
        out->components[i].peak_mv =
            result->component_amplitude_codes[i] * mv_per_code[channel];
    }
}

static void fill_spectra(void)
{
    static const uint32_t spans[] = {2000U, 5000U, 10000U, 20000U,
        50000U, 100000U, 200000U, 500000U, 1000000U, 2000000U,
        3300000U};
    float maximum_frequency = 0.0f;
    unsigned channel, component, index;
    for (channel = 0U; channel < 2U; ++channel) {
        ax7020_channel_snapshot_t *data = &snapshot.channel[channel];
        for (component = 0U; component < data->component_count; ++component)
            if (data->components[component].frequency_hz > maximum_frequency)
                maximum_frequency = data->components[component].frequency_hz;
    }
    snapshot.spectrum_span_hz = 3300000U;
    if (maximum_frequency == 0.0f) snapshot.spectrum_span_hz = 200000U;
    else for (index = 0U; index < sizeof(spans) / sizeof(spans[0]); ++index) {
        if (maximum_frequency * 1.1f <= (float)spans[index]) {
            snapshot.spectrum_span_hz = spans[index];
            break;
        }
    }
    for (channel = 0U; channel < 2U; ++channel) {
        ax7020_channel_snapshot_t *data = &snapshot.channel[channel];
        float maximum_peak = 0.0f;
        memset(data->spectrum, 0, sizeof(data->spectrum));
        for (component = 0U; component < data->component_count; ++component)
            if (data->components[component].peak_mv > maximum_peak)
                maximum_peak = data->components[component].peak_mv;
        if (maximum_peak < 0.001f) continue;
        for (component = 0U; component < data->component_count; ++component) {
            unsigned bucket = (unsigned)(data->components[component].frequency_hz *
                (AX7020_SPECTRUM_POINTS - 1U) / snapshot.spectrum_span_hz);
            unsigned height = (unsigned)(data->components[component].peak_mv *
                100.0f / maximum_peak + 0.5f);
            if (bucket >= AX7020_SPECTRUM_POINTS) continue;
            if (height > 100U) height = 100U;
            if (height > data->spectrum[bucket]) data->spectrum[bucket] = height;
        }
    }
}

static void handle_calibration(void)
{
    uint32_t request = mailbox->calibration_request;
    unsigned channel = mailbox->calibration_channel;
    unsigned action = mailbox->calibration_action;
    float reference = mailbox->calibration_reference_vpp_mv;
    float candidate;

    if (request == calibration_ack) return;
    calibration_status = 3U;
    if (channel < 2U) {
        if (action == 1U) {
            const ax7020_channel_snapshot_t *input = &snapshot.channel[channel];
            if (input->maximum_mv - input->minimum_mv < 50.0f) {
                zero_codes[channel] = input->adc_mean_codes;
                calibration_status = 1U;
            } else {
                calibration_status = 2U;
            }
        } else if (action == 2U && isfinite(reference) &&
                   reference >= 10.0f && reference <= 10000.0f &&
                   snapshot.channel[channel].peak_to_peak_mv > 50.0f &&
                   snapshot.channel[channel].frequency_hz >= 80.0f) {
            candidate = reference / (snapshot.channel[channel].peak_to_peak_mv /
                                     mv_per_code[channel]);
            if (isfinite(candidate) && candidate >= 0.1f && candidate <= 5.0f) {
                mv_per_code[channel] = candidate;
                calibration_status = 1U;
            }
        } else if (action == 3U) {
            candidate = mailbox->calibration_mv_per_code[channel];
            if (isfinite(candidate) && candidate >= 0.1f && candidate <= 5.0f &&
                isfinite(mailbox->calibration_zero_codes[channel]) &&
                fabsf(mailbox->calibration_zero_codes[channel]) <= 4096.0f) {
                mv_per_code[channel] = candidate;
                zero_codes[channel] = mailbox->calibration_zero_codes[channel];
                calibration_status = 1U;
            }
        }
    }
    calibration_ack = request;
}

int main(void)
{
    XTime start;
    int status;

    memset((void *)mailbox, 0, sizeof(*mailbox));
    mailbox->version = AX7020_MAILBOX_VERSION;
    mailbox->capture_enabled = 1U;
    mailbox->waveform_cycles = 1U;
    mailbox->auto_enabled = 1U;
    mailbox->trigger_rising = 1U;
    mailbox->independent_phase_display = 1U;
    mailbox->component_threshold_tenths_percent = 15U;
    dmb();
    mailbox->magic = AX7020_MAILBOX_MAGIC;

    status = initialize_hardware();
    if (status != XST_SUCCESS) {
        memset(&snapshot, 0, sizeof(snapshot));
        snapshot.flags = AX7020_FLAG_DMA_ERROR;
        snapshot.pl_status = (uint32_t)status;
        publish();
        for (;;) { mailbox->heartbeat++; usleep(100000U); }
    }

    for (;;) {
        uint32_t dma_status;
        uint32_t threshold = mailbox->component_threshold_tenths_percent;
        if (threshold <= 100U)
            (void)q3_analysis_set_component_threshold_tenths_percent(threshold);
        if (!mailbox->capture_enabled) {
            mailbox->heartbeat++;
            usleep(10000U);
            continue;
        }
        memset(&snapshot, 0, sizeof(snapshot));
        status = XAxiDma_SimpleTransfer(&dma, AX7020_DMA_ADDR,
                                        AX7020_PACKET_BYTES,
                                        XAXIDMA_DEVICE_TO_DMA);
        if (status != XST_SUCCESS) {
            snapshot.flags = AX7020_FLAG_DMA_ERROR;
            goto recover;
        }
        XGpio_DiscreteWrite(&control, 1,
                            (capture_slow_mode << 2) | trigger_state);
        usleep(1U);
        trigger_state ^= 1U;
        XGpio_DiscreteWrite(&control, 1,
                            (capture_slow_mode << 2) | trigger_state);
        XTime_GetTime(&start);
        do {
            dma_status = XAxiDma_ReadReg(dma.RegBase,
                XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET);
            if (dma_status & (XAXIDMA_ERR_ALL_MASK | XAXIDMA_IRQ_ERROR_MASK)) {
                snapshot.flags = AX7020_FLAG_DMA_ERROR;
                goto recover;
            }
            if (elapsed_us(start) > DMA_TIMEOUT_US) {
                snapshot.flags = AX7020_FLAG_DMA_TIMEOUT;
                goto recover;
            }
        } while (!(dma_status & XAXIDMA_IDLE_MASK));
        dmb();
        snapshot.acquisition_us = elapsed_us(start);
        snapshot.pl_status = XGpio_DiscreteRead(&control, 2);
        snapshot.dma_status = dma_status;
        snapshot.packet_debug[0] = packet[0];
        snapshot.packet_debug[1] = packet[1];
        snapshot.packet_debug[2] = packet[3];
        snapshot.packet_debug[3] = packet[AX7020_TRAILER_FIRST_WORD];
        snapshot.packet_debug[4] = packet[AX7020_TRAILER_FIRST_WORD + 9U];
        snapshot.packet_debug[5] = packet[AX7020_TRAILER_FIRST_WORD + 15U];
        snapshot.packet_debug[6] = packet[15];
        snapshot.packet_debug[7] = packet[AX7020_TRAILER_FIRST_WORD + 11U];
        status = ax7020_dual_analysis_run(packet, AX7020_PACKET_WORDS,
                snapshot.channel[0].spectrum,
                snapshot.channel[1].spectrum,
                AX7020_SPECTRUM_POINTS, &analysis);
        if (status != 0) {
            snapshot.flags = status == -1 ? AX7020_FLAG_PACKET_ERROR :
                                            AX7020_FLAG_ANALYSIS_ERROR;
            goto recover;
        }
        snapshot.frame_sequence = analysis.packet.frame_sequence;
        snapshot.sample_rate_hz = analysis.packet.sample_rate_hz;
        if (analysis.packet.status & 0x1cU) snapshot.flags |= AX7020_FLAG_PL_ERROR;
        fill_channel(0U, &analysis.ch1);
        fill_channel(1U, &analysis.ch2);
        fill_spectra();
        handle_calibration();
        snapshot.calibration_ack = calibration_ack;
        snapshot.calibration_status = calibration_status;
        snapshot.calibration_zero_codes[0] = zero_codes[0];
        snapshot.calibration_zero_codes[1] = zero_codes[1];
        snapshot.calibration_mv_per_code[0] = mv_per_code[0];
        snapshot.calibration_mv_per_code[1] = mv_per_code[1];
        snapshot.capture_mode = capture_slow_mode ? 2U : 1U;
        fill_waveforms();
        snapshot.flags |= AX7020_FLAG_VALID;
        snapshot.processing_us = elapsed_us(start);
        publish();
        update_capture_mode();
        continue;
recover:
        snapshot.processing_us = elapsed_us(start);
        publish();
        (void)reset_dma();
        reset_pipeline();
        usleep(1000U);
    }
}

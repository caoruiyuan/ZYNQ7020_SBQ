#include "ax7020_ui.h"
#include "ax7020_calibration_store.h"

#include "lvgl.h"
#include "xil_io.h"
#include "xil_printf.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LV_FONT_DECLARE(ui_font_scope_extra_16);
LV_FONT_DECLARE(ui_font_scope_extra_20);

#define UI_AVERAGE_MAX 50U
#define UI_CHART_POINTS AX7020_WAVE_POINTS
#define CHART_X 68
#define CHART_Y 84
#define CHART_W 500
#define CHART_H 240
#define NOMINAL_MV_PER_CODE (10000.0f / 4096.0f)

typedef struct {
    float frequency_hz;
    float peak_to_peak_mv;
    float rms_mv;
    float dc_mv;
} average_sample_t;

static volatile ax7020_mailbox_t *control;
static lv_obj_t *pages[3];
static lv_obj_t *navigation[3];
static lv_obj_t *view_buttons[2];
static lv_obj_t *channel_buttons[2];
static lv_obj_t *stabilize_button;
static lv_obj_t *cycle_buttons[2];
static lv_obj_t *auto_button;
static lv_obj_t *cursor_button;
static lv_obj_t *persistence_button;
static lv_obj_t *status_label;
static lv_obj_t *frame_label;
static lv_obj_t *hold_label;
static lv_obj_t *chart;
static lv_chart_series_t *trace[2];
static lv_chart_series_t *history_trace[2][2];
static lv_chart_cursor_t *cursors[2];
static lv_chart_cursor_t *trigger_cursor;
static lv_obj_t *x_axis_label[3];
static lv_obj_t *y_axis_label[3];
static lv_obj_t *trigger_label;
static lv_obj_t *timebase_label;
static lv_obj_t *voltage_label;
static lv_obj_t *trigger_source_button;
static lv_obj_t *trigger_slope_button;
static lv_obj_t *trigger_mode_button;
static lv_obj_t *trigger_level_slider;
static lv_obj_t *timebase_slider;
static lv_obj_t *voltage_slider;
static lv_obj_t *sample_rate_label;
static lv_obj_t *sample_rate_slider;
static lv_obj_t *grid_label;
static lv_obj_t *grid_slider;
static lv_obj_t *calibration_label[2];
static lv_obj_t *calibration_status_label;
static lv_obj_t *scope_controls[9];
static lv_obj_t *numeric_modal;
static lv_obj_t *numeric_input;
static lv_obj_t *numeric_keyboard;
static lv_obj_t *single_button;
static unsigned numeric_kind;
static unsigned calibration_channel;
static uint32_t last_calibration_ack;
static unsigned pending_calibration_action;
static float calibration_reference_mv = 1000.0f;
static int grid_brightness = 30;
static unsigned capture_rate_mode;
static float precise_timebase_us = 50.0f;
static float precise_voltage_mv = 200.0f;
static int32_t chart_values[2][UI_CHART_POINTS];
static int32_t history_values[2][2][UI_CHART_POINTS];
static lv_obj_t *chart_title;
static lv_obj_t *metric_label[2];
static lv_obj_t *metric_title[2];
static lv_obj_t *cursor_panel;
static lv_obj_t *cursor_mode_button[2];
static lv_obj_t *cursor_readout[4];
static lv_obj_t *component_label;
static lv_obj_t *average_label;
static lv_obj_t *threshold_label;
static average_sample_t history[UI_AVERAGE_MAX][2];
static average_sample_t displayed_average[2];
static ax7020_snapshot_t displayed;
static unsigned history_write;
static unsigned history_count;
static unsigned average_window = 1U;
static unsigned threshold_tenths_percent = 15U;
static unsigned active_page;
static unsigned active_view;
static unsigned active_channel;
static unsigned cycles = 1U;
static unsigned cursor_mode;
static unsigned cursor_position[2] = {250U, 750U};
static unsigned dragging_cursor;
static unsigned voltage_scale_index = 4U;
static unsigned timebase_index = 5U;
static unsigned trigger_channel;
static int trigger_rising = 1;
static int trigger_normal;
static int independent_phase_display = 1;
static int auto_timebase = 1;
static int auto_voltage = 1;
static int persistence_enabled;
static int history_valid;
static uint32_t history_frame_sequence;
static int chart_limit = 2048;
static int hold;
static int single_armed;
static int have_displayed;
static int auto_setup_pending;
static const unsigned voltage_divisions_mv[] =
    {10U, 20U, 50U, 100U, 200U, 500U, 1000U, 2000U, 5000U};
static const unsigned time_divisions_us[] =
    {1U, 2U, 5U, 10U, 20U, 50U, 100U, 200U, 500U,
     1000U, 2000U, 4000U};
static void format_tenth(char *buffer, size_t size, float value);
#ifdef AX7020_DISPLAY_DIAGNOSTICS
static int first_update_logged;

static void log_objects(const char *stage)
{
    xil_printf("界面 %s：帧标签=%08x 指标=%08x,%08x 标题=%08x "
               "图表=%08x 波形=%08x 谐波=%08x 状态=%08x\r\n",
        stage, (unsigned)(uintptr_t)frame_label,
        (unsigned)(uintptr_t)metric_label[0],
        (unsigned)(uintptr_t)metric_label[1],
        (unsigned)(uintptr_t)chart_title, (unsigned)(uintptr_t)chart,
        (unsigned)(uintptr_t)trace[0], (unsigned)(uintptr_t)component_label,
        (unsigned)(uintptr_t)status_label);
}
#endif

static lv_color_t accent(unsigned channel)
{
    return lv_color_hex(channel ? 0xF5A442 : 0x39C7BD);
}

static lv_obj_t *make_label(lv_obj_t *parent, const char *text,
                            int x, int y, int width, const lv_font_t *font,
                            lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_width(label, width);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_label_set_text(label, text);
    return label;
}

static lv_obj_t *make_button(lv_obj_t *parent, const char *text,
                             int x, int y, int width, int height,
                             lv_event_cb_t callback, unsigned id)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_t *label;
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, height);
    lv_obj_set_style_radius(button, 4, 0);
    lv_obj_set_style_pad_all(button, 0, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x26333D), 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)id);
    label = make_label(button, text, 0, 0, width, &ui_font_scope_extra_16,
                       lv_color_hex(0xF0F5F5));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    return button;
}

static void mark_selected(lv_obj_t *button, int selected, lv_color_t color)
{
    if (button == NULL) return;
    lv_obj_set_style_bg_color(button,
        selected ? color : lv_color_hex(0x26333D), 0);
    lv_obj_set_style_text_color(button,
        selected ? lv_color_hex(0x07151A) : lv_color_hex(0xF0F5F5), 0);
    lv_obj_set_style_text_color(lv_obj_get_child(button, 0),
        selected ? lv_color_hex(0x07151A) : lv_color_hex(0xF0F5F5), 0);
}

static void update_controls(void)
{
    unsigned i;
    for (i = 0; i < 3U; ++i)
        mark_selected(navigation[i], i == active_page, accent(0));
    for (i = 0; i < 2U; ++i) {
        mark_selected(view_buttons[i], i == active_view, accent(0));
        mark_selected(channel_buttons[i], i == active_channel, accent(i));
        mark_selected(cycle_buttons[i], (i ? 3U : 1U) == cycles, accent(0));
    }
    mark_selected(auto_button, auto_timebase && auto_voltage, accent(0));
    mark_selected(stabilize_button, independent_phase_display, accent(1));
    if (stabilize_button != NULL)
        lv_label_set_text(lv_obj_get_child(stabilize_button, 0),
            independent_phase_display ? "双稳" : "同步");
    mark_selected(cursor_button, cursor_mode != 0U, accent(0));
    mark_selected(persistence_button, persistence_enabled, accent(1));
    if (cursor_button != NULL)
        lv_label_set_text(lv_obj_get_child(cursor_button, 0),
            cursor_mode != 0U ? "关闭游标" : "游标");
    for (i = 0; i < 2U; ++i)
        mark_selected(cursor_mode_button[i], cursor_mode == i + 1U, accent(0));
    if (status_label != NULL)
        lv_label_set_text(status_label,
            single_armed ? "单次" : (hold ? "已暂停" : "采集中"));
    if (hold_label != NULL)
        lv_label_set_text(hold_label, hold ? "运行" : "保持");
}

static void format_time(char *buffer, size_t size, float microseconds)
{
    if (microseconds < 10.0f) {
        unsigned thousandths = (unsigned)(microseconds * 1000.0f + 0.5f);
        (void)snprintf(buffer, size, "%u.%03u us",
                       thousandths / 1000U, thousandths % 1000U);
        return;
    }
    unsigned tenths = (unsigned)(microseconds * 10.0f + 0.5f);
    if (tenths < 10000U)
        (void)snprintf(buffer, size, "%u.%u us",
                       tenths / 10U, tenths % 10U);
    else
        (void)snprintf(buffer, size, "%u.%u ms",
                       tenths / 10000U, (tenths / 1000U) % 10U);
}

static void format_frequency_axis(char *buffer, size_t size, uint32_t hz)
{
    if (hz >= 1000000U)
        (void)snprintf(buffer, size, "%u.%u MHz",
                       (unsigned)(hz / 1000000U),
                       (unsigned)((hz % 1000000U) / 100000U));
    else if (hz >= 1000U)
        (void)snprintf(buffer, size, "%u.%u kHz",
                       (unsigned)(hz / 1000U),
                       (unsigned)((hz % 1000U) / 100U));
    else
        (void)snprintf(buffer, size, "%u Hz", (unsigned)hz);
}

static void update_axis_labels(void)
{
    char midpoint[32], endpoint[32];
    if (active_view == 0U) {
        unsigned maximum_mv = (unsigned)(chart_limit * NOMINAL_MV_PER_CODE + 0.5f);
        if (maximum_mv >= 1000U) {
            lv_label_set_text_fmt(y_axis_label[0], "+%u.%u V",
                maximum_mv / 1000U, (maximum_mv / 100U) % 10U);
            lv_label_set_text_fmt(y_axis_label[2], "-%u.%u V",
                maximum_mv / 1000U, (maximum_mv / 100U) % 10U);
        } else {
            lv_label_set_text_fmt(y_axis_label[0], "+%u mV", maximum_mv);
            lv_label_set_text_fmt(y_axis_label[2], "-%u mV", maximum_mv);
        }
        lv_label_set_text(y_axis_label[1], "0 mV");
        format_time(midpoint, sizeof(midpoint), displayed.waveform_span_us / 2.0f);
        format_time(endpoint, sizeof(endpoint), displayed.waveform_span_us);
    } else {
        lv_label_set_text(y_axis_label[0], "100%");
        lv_label_set_text(y_axis_label[1], "50%");
        lv_label_set_text(y_axis_label[2], "0%");
        format_frequency_axis(midpoint, sizeof(midpoint),
                              displayed.spectrum_span_hz / 2U);
        format_frequency_axis(endpoint, sizeof(endpoint),
                              displayed.spectrum_span_hz);
    }
    lv_label_set_text(x_axis_label[0], "0");
    lv_label_set_text(x_axis_label[1], midpoint);
    lv_label_set_text(x_axis_label[2], endpoint);
}

static void update_cursors(void)
{
    int panel_visible = cursor_mode != 0U && active_view == 0U;
    for (unsigned i = 0U; i < 2U; ++i) {
        if (metric_title[i] != NULL) {
            if (panel_visible) lv_obj_add_flag(metric_title[i], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_remove_flag(metric_title[i], LV_OBJ_FLAG_HIDDEN);
        }
        if (metric_label[i] != NULL) {
            if (panel_visible) lv_obj_add_flag(metric_label[i], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_remove_flag(metric_label[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (cursor_panel != NULL) {
        if (panel_visible) lv_obj_remove_flag(cursor_panel, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(cursor_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (cursor_mode == 0U || active_view != 0U || !have_displayed ||
        cursors[0] == NULL || cursors[1] == NULL) return;
    for (unsigned i = 0U; i < 2U; ++i) {
        if (cursor_mode == 1U)
            lv_chart_set_cursor_pos_x(chart, cursors[i],
                (int32_t)(cursor_position[i] * (CHART_W - 2) / 1000U));
        else
            lv_chart_set_cursor_pos_y(chart, cursors[i],
                (int32_t)(cursor_position[i] * (CHART_H - 2) / 1000U));
    }
    unsigned difference = cursor_position[0] > cursor_position[1] ?
        cursor_position[0] - cursor_position[1] :
        cursor_position[1] - cursor_position[0];
    if (cursor_mode == 1U) {
        float delta_us = displayed.waveform_span_us * difference / 1000.0f;
        char value[32];
        format_time(value, sizeof(value),
                    displayed.waveform_span_us * cursor_position[0] / 1000.0f);
        lv_label_set_text_fmt(cursor_readout[0], "A  %s", value);
        format_time(value, sizeof(value),
                    displayed.waveform_span_us * cursor_position[1] / 1000.0f);
        lv_label_set_text_fmt(cursor_readout[1], "B  %s", value);
        format_time(value, sizeof(value), delta_us);
        lv_label_set_text_fmt(cursor_readout[2], "dT  %s", value);
        if (delta_us > 0.0f) {
            format_frequency_axis(value, sizeof(value),
                (uint32_t)(1000000.0f / delta_us + 0.5f));
            lv_label_set_text_fmt(cursor_readout[3], "1/dT  %s", value);
        } else {
            lv_label_set_text(cursor_readout[3], "1/dT  --");
        }
    } else {
        char voltage[24];
        for (unsigned i = 0U; i < 2U; ++i) {
            float value = chart_limit * NOMINAL_MV_PER_CODE *
                          (1.0f - 2.0f * cursor_position[i] / 1000.0f);
            format_tenth(voltage, sizeof(voltage), value);
            lv_label_set_text_fmt(cursor_readout[i], "%c  %s mV",
                                  i == 0U ? 'A' : 'B', voltage);
        }
        format_tenth(voltage, sizeof(voltage),
            2.0f * chart_limit * NOMINAL_MV_PER_CODE * difference / 1000.0f);
        lv_label_set_text_fmt(cursor_readout[2], "dV  %s mV", voltage);
        lv_label_set_text(cursor_readout[3], "");
    }
}

static void update_trigger_cursor(void)
{
    if (trigger_cursor == NULL || active_view != 0U) return;
    int level = control->trigger_level_codes;
    int y = (chart_limit - level) * (CHART_H - 2) /
            (2 * chart_limit);
    if (y < 0) y = 0;
    if (y > CHART_H - 2) y = CHART_H - 2;
    lv_chart_set_cursor_pos_y(chart, trigger_cursor, y);
}

static void reset_cursors(void)
{
    if (trigger_cursor != NULL) {
        lv_chart_remove_cursor(chart, trigger_cursor);
        trigger_cursor = NULL;
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        if (cursors[i] != NULL) {
            lv_chart_remove_cursor(chart, cursors[i]);
            cursors[i] = NULL;
        }
    }
    if (cursor_mode != 0U && active_view == 0U) {
        lv_dir_t direction = cursor_mode == 1U ? LV_DIR_VER : LV_DIR_HOR;
        cursors[0] = lv_chart_add_cursor(chart, lv_color_hex(0xF5D062), direction);
        cursors[1] = lv_chart_add_cursor(chart, lv_color_hex(0xF19579), direction);
    }
    if (active_view == 0U)
        trigger_cursor = lv_chart_add_cursor(chart,
            lv_color_hex(0x91AD55), LV_DIR_HOR);
    update_cursors();
    update_trigger_cursor();
}

static void chart_touch_event(lv_event_t *event)
{
    lv_point_t point;
    lv_area_t area;
    int coordinate, limit, position;
    if (cursor_mode == 0U || active_view != 0U) return;
    if (lv_event_get_code(event) == LV_EVENT_PRESSED) {
        dragging_cursor = 2U;
    }
    lv_indev_get_point(lv_indev_active(), &point);
    lv_obj_get_coords(chart, &area);
    coordinate = cursor_mode == 1U ? point.x - area.x1 : point.y - area.y1;
    limit = cursor_mode == 1U ? CHART_W - 2 : CHART_H - 2;
    if (coordinate < 0) coordinate = 0;
    if (coordinate > limit) coordinate = limit;
    position = coordinate * 1000 / limit;
    if (dragging_cursor > 1U) {
        int first_distance = position > (int)cursor_position[0] ?
            position - (int)cursor_position[0] :
            (int)cursor_position[0] - position;
        int second_distance = position > (int)cursor_position[1] ?
            position - (int)cursor_position[1] :
            (int)cursor_position[1] - position;
        dragging_cursor = first_distance <= second_distance ? 0U : 1U;
    }
    cursor_position[dragging_cursor] = (unsigned)position;
    update_cursors();
}

static void format_tenth(char *buffer, size_t size, float value)
{
    int scaled = (int)(value * 10.0f + (value < 0.0f ? -0.5f : 0.5f));
    unsigned magnitude = (unsigned)(scaled < 0 ? -scaled : scaled);
    (void)snprintf(buffer, size, "%s%u.%u", scaled < 0 ? "-" : "",
                   magnitude / 10U, magnitude % 10U);
}

static average_sample_t averaged(unsigned channel)
{
    average_sample_t value = {0};
    unsigned count = history_count < average_window ?
                     history_count : average_window;
    unsigned i;
    for (i = 0; i < count; ++i) {
        unsigned slot = (history_write + UI_AVERAGE_MAX - 1U - i) %
                        UI_AVERAGE_MAX;
        value.frequency_hz += history[slot][channel].frequency_hz;
        value.peak_to_peak_mv += history[slot][channel].peak_to_peak_mv;
        value.rms_mv += history[slot][channel].rms_mv;
        value.dc_mv += history[slot][channel].dc_mv;
    }
    if (count != 0U) {
        value.frequency_hz /= count;
        value.peak_to_peak_mv /= count;
        value.rms_mv /= count;
        value.dc_mv /= count;
    }
    return value;
}

static void render(void)
{
    char text[600];
    char frequency[24], vpp[24], rms[24], dc[24];
    unsigned channel;
    unsigned i;
    const ax7020_channel_snapshot_t *selected;
    if (!have_displayed) return;
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) log_objects("render");
#endif

#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) xil_printf("界面绘制：帧标签\r\n");
#endif
    if (frame_label != NULL)
        lv_label_set_text_fmt(frame_label, "#%u  %u us / %u us",
            (unsigned)displayed.frame_sequence,
            (unsigned)displayed.acquisition_us,
            (unsigned)displayed.processing_us);
    if (status_label != NULL && !hold && !single_armed)
        lv_label_set_text(status_label,
            trigger_normal && !displayed.waveform_triggered ?
            "等待触发" : "采集中");
    for (channel = 0; channel < 2U; ++channel) {
        average_sample_t value = displayed_average[channel];
        format_tenth(frequency, sizeof(frequency), value.frequency_hz);
        format_tenth(vpp, sizeof(vpp), value.peak_to_peak_mv);
        format_tenth(rms, sizeof(rms), value.rms_mv);
        format_tenth(dc, sizeof(dc), value.dc_mv);
        (void)snprintf(text, sizeof(text),
            "基频 %s Hz\n峰峰值 %s mV\n有效值 %s mV\nDC %s mV",
            frequency, vpp, rms, dc);
        if (metric_label[channel] != NULL)
            lv_label_set_text(metric_label[channel], text);
    }
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) xil_printf("界面绘制：指标完成\r\n");
#endif
    selected = &displayed.channel[active_channel];
    if (active_view == 0U) {
        int maximum_absolute = 0;
        int limit = 64;
        const char *mode = independent_phase_display ?
            (displayed.waveform_independent ? "独立稳定·相位不可比" :
                                                "独立稳定待触发") :
            (trigger_channel ? "同步 CH2触发" : "同步 CH1触发");
        for (channel = 0U; channel < 2U; ++channel)
            for (i = 0U; i < UI_CHART_POINTS; ++i) {
                int sample = displayed.channel[channel].waveform_codes[i];
                int magnitude = sample < 0 ? -sample : sample;
                if (magnitude > maximum_absolute) maximum_absolute = magnitude;
            }
        if (auto_voltage) {
            while (limit < maximum_absolute + maximum_absolute / 10 &&
                   limit < 32768) limit *= 2;
            if (limit < chart_limit &&
                maximum_absolute > chart_limit * 3 / 5)
                limit = chart_limit;
            if (limit > chart_limit &&
                maximum_absolute < chart_limit * 9 / 10)
                limit = chart_limit;
        } else {
            limit = (int)(4.0f * precise_voltage_mv /
                          NOMINAL_MV_PER_CODE + 0.5f);
        }
        chart_limit = limit;
        if (chart_title != NULL) {
            if (displayed.waveform_sample_rate_hz < 1000000U)
                lv_label_set_text_fmt(chart_title,
                    "%s  %u.%u kSPS", mode,
                    (unsigned)(displayed.waveform_sample_rate_hz / 1000U),
                    (unsigned)((displayed.waveform_sample_rate_hz % 1000U) / 100U));
            else
                lv_label_set_text_fmt(chart_title,
                    "%s  %u.%u MSPS", mode,
                    (unsigned)(displayed.waveform_sample_rate_hz / 1000000U),
                    (unsigned)((displayed.waveform_sample_rate_hz % 1000000U) / 100000U));
        }
        if (chart != NULL)
            lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -limit, limit);
        if (history_frame_sequence != displayed.frame_sequence) {
            for (channel = 0U; channel < 2U; ++channel) {
                memcpy(history_values[channel][1], history_values[channel][0],
                       sizeof(history_values[channel][1]));
                memcpy(history_values[channel][0], chart_values[channel],
                       sizeof(history_values[channel][0]));
            }
            if (history_frame_sequence != 0U && history_valid < 2)
                ++history_valid;
            history_frame_sequence = displayed.frame_sequence;
        }
        for (channel = 0U; channel < 2U; ++channel) {
            for (i = 0U; i < UI_CHART_POINTS; ++i)
                chart_values[channel][i] =
                    displayed.channel[channel].waveform_codes[i];
            for (i = 0U; i < 2U; ++i)
                lv_chart_hide_series(chart, history_trace[channel][i],
                    !persistence_enabled || history_valid <= (int)i);
        }
    } else {
        if (chart_title != NULL)
            lv_label_set_text(chart_title, "双通道幅频特性");
        if (chart != NULL)
            lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
        for (channel = 0U; channel < 2U; ++channel) {
            for (i = 0U; i < UI_CHART_POINTS; ++i) {
                unsigned index = i * (AX7020_SPECTRUM_POINTS - 1U) /
                                 (UI_CHART_POINTS - 1U);
                chart_values[channel][i] =
                    displayed.channel[channel].spectrum[index];
            }
            for (i = 0U; i < 2U; ++i)
                lv_chart_hide_series(chart, history_trace[channel][i], true);
        }
    }
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) xil_printf("界面绘制：图表数值完成\r\n");
#endif
    if (chart != NULL) lv_chart_refresh(chart);
    update_axis_labels();
    update_trigger_cursor();
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) xil_printf("界面绘制：图表刷新完成\r\n");
#endif

    text[0] = '\0';
    if (active_view == 1U) {
        for (i = 0; i < selected->component_count &&
                    i < AX7020_COMPONENTS_MAX; ++i) {
            char freq[24], peak[24], row[80];
            format_tenth(freq, sizeof(freq),
                         selected->components[i].frequency_hz);
            format_tenth(peak, sizeof(peak), selected->components[i].peak_mv);
            (void)snprintf(row, sizeof(row), "H%u %sHz %smV%s",
                (unsigned)selected->components[i].order, freq, peak,
                i % 2U == 1U ? "\n" : "      ");
            if (strlen(text) + strlen(row) < sizeof(text))
                (void)strcat(text, row);
        }
    }
    if (component_label != NULL && active_view == 1U)
        lv_label_set_text(component_label, text[0] ? text : "谐波 --");
    update_cursors();
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) xil_printf("界面绘制：谐波完成\r\n");
#endif
}

static void apply_auto_setup(void)
{
    const ax7020_channel_snapshot_t *input;
    float frequency, desired, level_mv, peak_mv;
    unsigned source = trigger_channel;
    unsigned index;
    char formatted[32];

    if (have_displayed) {
        const ax7020_channel_snapshot_t *other =
            &displayed.channel[source ^ 1U];
        input = &displayed.channel[source];
        if ((input->frequency_hz < 80.0f ||
             input->peak_to_peak_mv < 20.0f) &&
            other->frequency_hz >= 80.0f &&
            other->peak_to_peak_mv >= 20.0f)
            source ^= 1U;
        else if (input->frequency_hz < 80.0f &&
                 other->frequency_hz < 80.0f &&
                 other->peak_to_peak_mv > input->peak_to_peak_mv * 2.0f)
            source ^= 1U;
    }
    input = &displayed.channel[source];
    trigger_channel = source;
    control->trigger_channel = source;
    trigger_rising = 1;
    control->trigger_rising = 1U;
    trigger_normal = 0;
    control->trigger_normal = 0U;
    capture_rate_mode = 0U;
    control->capture_mode_request = 0U;

    frequency = have_displayed ? input->frequency_hz : 0.0f;
    auto_setup_pending = frequency < 80.0f;
    desired = frequency >= 80.0f && frequency <= 200000.0f ?
              250000.0f / frequency : 50.0f;
    for (index = 0U; index + 1U < sizeof(time_divisions_us) /
         sizeof(time_divisions_us[0]); ++index)
        if ((float)time_divisions_us[index] >= desired) break;
    timebase_index = index;
    precise_timebase_us = (float)time_divisions_us[index];
    control->requested_span_us = 8.0f * precise_timebase_us;

    level_mv = have_displayed ?
        0.5f * (input->maximum_mv + input->minimum_mv) : 0.0f;
    if (!isfinite(level_mv)) level_mv = 0.0f;
    if (level_mv > 5000.0f) level_mv = 5000.0f;
    if (level_mv < -5000.0f) level_mv = -5000.0f;
    control->trigger_level_codes = (int32_t)lroundf(
        level_mv / NOMINAL_MV_PER_CODE);

    peak_mv = have_displayed ?
        fmaxf(fabsf(input->maximum_mv), fabsf(input->minimum_mv)) : 500.0f;
    desired = peak_mv * 1.25f / 4.0f;
    for (index = 0U; index + 1U < sizeof(voltage_divisions_mv) /
         sizeof(voltage_divisions_mv[0]); ++index)
        if ((float)voltage_divisions_mv[index] >= desired) break;
    voltage_scale_index = index;
    precise_voltage_mv = (float)voltage_divisions_mv[index];
    auto_timebase = 0;
    auto_voltage = 0;

    format_time(formatted, sizeof(formatted), precise_timebase_us);
    lv_label_set_text_fmt(timebase_label, "时基 %s/格", formatted);
    lv_label_set_text_fmt(voltage_label, "幅度 %u mV/格",
                          voltage_divisions_mv[voltage_scale_index]);
    format_tenth(formatted, sizeof(formatted), level_mv);
    lv_label_set_text_fmt(trigger_label, "触发 %s mV", formatted);
    lv_label_set_text(lv_obj_get_child(trigger_source_button, 0),
                      source ? "CH2触发" : "CH1触发");
    lv_label_set_text(lv_obj_get_child(trigger_slope_button, 0), "上升沿");
    lv_label_set_text(lv_obj_get_child(trigger_mode_button, 0), "自动触发");
    lv_slider_set_value(timebase_slider, timebase_index, LV_ANIM_OFF);
    lv_slider_set_value(voltage_slider, voltage_scale_index, LV_ANIM_OFF);
    lv_slider_set_value(trigger_level_slider, control->trigger_level_codes,
                        LV_ANIM_OFF);
    lv_slider_set_value(sample_rate_slider, 0, LV_ANIM_OFF);
    lv_label_set_text(sample_rate_label, "采样档位 AUTO");
    history_valid = 0;
    update_trigger_cursor();
    render();
}

static void button_event(lv_event_t *event)
{
    unsigned id = (unsigned)(uintptr_t)lv_event_get_user_data(event);
    if (id < 3U) {
        active_page = id;
        for (unsigned i = 0; i < 3U; ++i) {
            if (pages[i] == NULL) continue;
            if (i == id) lv_obj_remove_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
        }
    } else if (id == 3U || id == 4U) {
        active_view = id - 3U;
        if (active_view != 0U) cursor_mode = 0U;
        for (unsigned item = 0U; item < 9U; ++item) {
            if (scope_controls[item] == NULL) continue;
            if (active_view == 0U)
                lv_obj_remove_flag(scope_controls[item], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(scope_controls[item], LV_OBJ_FLAG_HIDDEN);
        }
        if (active_view == 0U)
            lv_obj_add_flag(component_label, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_remove_flag(component_label, LV_OBJ_FLAG_HIDDEN);
        history_valid = 0;
        history_frame_sequence = displayed.frame_sequence;
        for (unsigned channel = 0U; channel < 2U; ++channel)
            for (unsigned age = 0U; age < 2U; ++age)
                for (unsigned point = 0U; point < UI_CHART_POINTS; ++point)
                    history_values[channel][age][point] = LV_CHART_POINT_NONE;
        reset_cursors();
        render();
    } else if (id == 5U || id == 6U) {
        active_channel = id - 5U;
        render();
    } else if (id == 7U || id == 8U) {
        cycles = id == 7U ? 1U : 3U;
        control->waveform_cycles = cycles;
    } else if (id == 9U) {
        hold = !hold;
        single_armed = 0;
    } else if (id == 10U) {
        hold = 0;
        single_armed = 1;
    } else if (id == 11U) {
        apply_auto_setup();
    } else if (id == 12U) {
        if (active_view != 0U) {
            active_view = 0U;
            lv_obj_send_event(view_buttons[0], LV_EVENT_CLICKED, NULL);
        }
        cursor_mode = cursor_mode == 0U ? 1U : 0U;
        reset_cursors();
        render();
    } else if (id == 22U || id == 23U) {
        cursor_mode = id == 22U ? 1U : 2U;
        reset_cursors();
        render();
    } else if (id == 13U) {
        persistence_enabled = !persistence_enabled;
        render();
    } else if (id == 14U) {
        trigger_channel ^= 1U;
        control->trigger_channel = trigger_channel;
        render();
    } else if (id == 15U) {
        trigger_rising = !trigger_rising;
        control->trigger_rising = trigger_rising ? 1U : 0U;
    } else if (id == 16U) {
        trigger_normal = !trigger_normal;
        control->trigger_normal = trigger_normal ? 1U : 0U;
    } else if (id == 17U || id == 18U) {
        calibration_channel = id - 17U;
        lv_label_set_text_fmt(calibration_status_label,
            "当前校准 CH%u", calibration_channel + 1U);
    } else if (id == 19U || id == 20U) {
        control->calibration_channel = calibration_channel;
        control->calibration_action = id == 19U ? 1U : 2U;
        control->calibration_reference_vpp_mv = calibration_reference_mv;
        pending_calibration_action = control->calibration_action;
        dmb();
        control->calibration_request++;
        lv_label_set_text(calibration_status_label, "正在采集校准数据...");
    } else if (id == 21U) {
        independent_phase_display = !independent_phase_display;
        control->independent_phase_display = independent_phase_display ? 1U : 0U;
        history_valid = 0;
        render();
    }
    if (trigger_source_button != NULL)
        lv_label_set_text(lv_obj_get_child(trigger_source_button, 0),
            trigger_channel ? "CH2触发" : "CH1触发");
    if (trigger_slope_button != NULL)
        lv_label_set_text(lv_obj_get_child(trigger_slope_button, 0),
            trigger_rising ? "上升沿" : "下降沿");
    if (trigger_mode_button != NULL)
        lv_label_set_text(lv_obj_get_child(trigger_mode_button, 0),
            trigger_normal ? "正常触发" : "自动触发");
    update_controls();
}

static void trigger_level_event(lv_event_t *event)
{
    int codes = lv_slider_get_value(lv_event_get_target(event));
    char value[24];
    control->trigger_level_codes = codes;
    format_tenth(value, sizeof(value), codes * NOMINAL_MV_PER_CODE);
    lv_label_set_text_fmt(trigger_label, "触发 %s mV", value);
    update_trigger_cursor();
}

static void timebase_event(lv_event_t *event)
{
    char value[24];
    timebase_index = (unsigned)lv_slider_get_value(lv_event_get_target(event));
    auto_timebase = 0;
    precise_timebase_us = (float)time_divisions_us[timebase_index];
    control->requested_span_us = 8.0f * precise_timebase_us;
    format_time(value, sizeof(value), (float)time_divisions_us[timebase_index]);
    lv_label_set_text_fmt(timebase_label, "时基 %s/格", value);
    update_controls();
}

static void voltage_event(lv_event_t *event)
{
    voltage_scale_index =
        (unsigned)lv_slider_get_value(lv_event_get_target(event));
    auto_voltage = 0;
    precise_voltage_mv = (float)voltage_divisions_mv[voltage_scale_index];
    lv_label_set_text_fmt(voltage_label, "幅度 %u mV/格",
        voltage_divisions_mv[voltage_scale_index]);
    update_controls();
    render();
}

static void average_event(lv_event_t *event)
{
    average_window = (unsigned)lv_slider_get_value(lv_event_get_target(event));
    history_count = 0U;
    history_write = 0U;
    lv_label_set_text_fmt(average_label, "均值帧数  %u", average_window);
}

static void threshold_event(lv_event_t *event)
{
    threshold_tenths_percent =
        (unsigned)lv_slider_get_value(lv_event_get_target(event));
    control->component_threshold_tenths_percent = threshold_tenths_percent;
    lv_label_set_text_fmt(threshold_label, "谐波阈值  %u.%u%%",
        threshold_tenths_percent / 10U,
        threshold_tenths_percent % 10U);
}

static void sample_rate_event(lv_event_t *event)
{
    capture_rate_mode = (unsigned)lv_slider_get_value(lv_event_get_target(event));
    control->capture_mode_request = capture_rate_mode;
    lv_label_set_text(sample_rate_label, capture_rate_mode == 0U ?
        "采样档位  AUTO" : capture_rate_mode == 1U ?
        "采样档位  8.125 MSPS" : capture_rate_mode == 2U ?
        "采样档位  507.8 kSPS" : "采样档位  65 MSPS 原始");
}

static void grid_event(lv_event_t *event)
{
    grid_brightness = lv_slider_get_value(lv_event_get_target(event));
    lv_obj_set_style_line_opa(chart, (lv_opa_t)grid_brightness,
                              LV_PART_MAIN);
    lv_label_set_text_fmt(grid_label, "网格亮度  %u%%",
                          (unsigned)(grid_brightness * 100 / 255));
}

static void numeric_keyboard_event(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    char *end;
    float value;
    if (code != LV_EVENT_READY && code != LV_EVENT_CANCEL) return;
    if (code == LV_EVENT_READY) {
        value = strtof(lv_textarea_get_text(numeric_input), &end);
        if (end != lv_textarea_get_text(numeric_input) && *end == '\0' &&
            isfinite(value)) {
            if (numeric_kind == 0U && value >= 0.125f && value <= 4000.0f) {
                precise_timebase_us = value;
                auto_timebase = 0;
                control->requested_span_us = 8.0f * value;
                {
                    char formatted[32];
                    format_time(formatted, sizeof(formatted), value);
                    lv_label_set_text_fmt(timebase_label, "时基 %s/格", formatted);
                }
            } else if (numeric_kind == 1U && value >= 1.0f && value <= 5000.0f) {
                precise_voltage_mv = value;
                auto_voltage = 0;
                {
                    char formatted[24];
                    format_tenth(formatted, sizeof(formatted), value);
                    lv_label_set_text_fmt(voltage_label, "幅度 %s mV/格", formatted);
                }
                render();
            } else if (numeric_kind == 2U && value >= -5000.0f && value <= 5000.0f) {
                int codes = (int)lroundf(value / NOMINAL_MV_PER_CODE);
                control->trigger_level_codes = codes;
                lv_slider_set_value(trigger_level_slider, codes, LV_ANIM_OFF);
                {
                    char formatted[24];
                    format_tenth(formatted, sizeof(formatted), value);
                    lv_label_set_text_fmt(trigger_label, "触发 %s mV", formatted);
                }
                update_trigger_cursor();
            } else if (numeric_kind == 3U && value >= 10.0f && value <= 10000.0f) {
                calibration_reference_mv = value;
                {
                    char formatted[24];
                    format_tenth(formatted, sizeof(formatted), value);
                    lv_label_set_text_fmt(calibration_status_label,
                        "参考正弦 %s mVpp", formatted);
                }
            } else if (numeric_kind == 4U) {
                capture_rate_mode = value >= 30000000.0f ? 3U :
                    value >= 3000000.0f ? 1U : value >= 100000.0f ? 2U : 0U;
                lv_slider_set_value(sample_rate_slider, capture_rate_mode, LV_ANIM_OFF);
                control->capture_mode_request = capture_rate_mode;
                lv_label_set_text(sample_rate_label, capture_rate_mode == 0U ?
                    "采样档位  AUTO" : capture_rate_mode == 1U ?
                    "采样档位  8.125 MSPS" : capture_rate_mode == 2U ?
                    "采样档位  507.8 kSPS" : "采样档位  65 MSPS 原始");
            }
        }
    }
    lv_obj_delete_async(numeric_modal);
    numeric_modal = NULL;
    numeric_keyboard = NULL;
    update_controls();
}

static void numeric_clear_event(lv_event_t *event)
{
    (void)event;
    lv_textarea_set_text(numeric_input, "");
}

static void numeric_open_event(lv_event_t *event)
{
    lv_obj_t *keyboard;
    char initial[32];
    numeric_kind = (unsigned)(uintptr_t)lv_event_get_user_data(event);
    if (numeric_modal != NULL) return;
    numeric_modal = lv_obj_create(lv_screen_active());
    lv_obj_set_pos(numeric_modal, 180, 40);
    lv_obj_set_size(numeric_modal, 440, 400);
    lv_obj_set_style_bg_color(numeric_modal, lv_color_hex(0x182A32), 0);
    lv_obj_set_style_border_color(numeric_modal, accent(0), 0);
    lv_obj_set_style_radius(numeric_modal, 4, 0);
    lv_obj_set_style_pad_all(numeric_modal, 0, 0);
    lv_obj_remove_flag(numeric_modal, LV_OBJ_FLAG_SCROLLABLE);
    make_label(numeric_modal,
        numeric_kind == 0U ? "时基 us/格" :
        numeric_kind == 1U ? "幅度 mV/格" :
        numeric_kind == 2U ? "触发电平 mV" :
        numeric_kind == 3U ? "参考正弦 mVpp" : "采样率 Hz",
        15, 4, 310, &ui_font_scope_extra_16,
        lv_color_hex(0xD7E1E2));
    numeric_input = lv_textarea_create(numeric_modal);
    lv_obj_set_pos(numeric_input, 15, 30);
    lv_obj_set_size(numeric_input, 340, 48);
    lv_textarea_set_one_line(numeric_input, true);
    make_button(numeric_modal, "C", 362, 30, 54, 48,
                numeric_clear_event, 0U);
    if (numeric_kind == 0U) {
        unsigned scaled = (unsigned)(precise_timebase_us * 1000.0f + 0.5f);
        (void)snprintf(initial, sizeof(initial), "%u.%03u",
                       scaled / 1000U, scaled % 1000U);
    } else if (numeric_kind == 1U)
        format_tenth(initial, sizeof(initial), precise_voltage_mv);
    else if (numeric_kind == 2U)
        format_tenth(initial, sizeof(initial),
            (float)control->trigger_level_codes * NOMINAL_MV_PER_CODE);
    else if (numeric_kind == 3U)
        format_tenth(initial, sizeof(initial), calibration_reference_mv);
    else
        (void)snprintf(initial, sizeof(initial), "%u",
            capture_rate_mode == 3U ? 65000000U :
            capture_rate_mode == 2U ? 507812U : 8125000U);
    lv_textarea_set_text(numeric_input, initial);
    keyboard = lv_keyboard_create(numeric_modal);
    numeric_keyboard = keyboard;
    lv_obj_set_align(keyboard, LV_ALIGN_TOP_LEFT);
    lv_obj_set_pos(keyboard, 15, 88);
    lv_obj_set_size(keyboard, 400, 275);
    lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(keyboard, numeric_input);
    lv_obj_add_event_cb(keyboard, numeric_keyboard_event, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(keyboard, numeric_keyboard_event, LV_EVENT_CANCEL, NULL);
}

void ax7020_ui_create(volatile ax7020_mailbox_t *mailbox)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *header;
    lv_obj_t *slider;
    lv_obj_t *hold_button;
    unsigned i;
    control = mailbox;
    control->independent_phase_display = 1U;
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0C151B), 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(0xECF2F2), 0);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    header = lv_obj_create(screen);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_size(header, 800, 44);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x16252C), 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_all(header, 0, 0);
    lv_obj_remove_flag(header, LV_OBJ_FLAG_SCROLLABLE);
    make_label(header, "ZYNQ7020多功能示波器", 12, 10, 295,
               &ui_font_scope_extra_20, lv_color_hex(0xEFF5F5));
    navigation[0] = make_button(header, "测量", 310, 6, 82, 32,
                                button_event, 0U);
    navigation[1] = make_button(header, "校准", 397, 6, 82, 32,
                                button_event, 1U);
    navigation[2] = make_button(header, "更多", 484, 6, 82, 32,
                                button_event, 2U);
    status_label = make_label(header, "采集中", 604, 12, 100,
                              &ui_font_scope_extra_16, accent(0));
    make_label(header, "ADC", 750, 15, 40,
               &lv_font_montserrat_12, lv_color_hex(0x91A6AD));

    for (i = 0; i < 3U; ++i) {
        pages[i] = lv_obj_create(screen);
        lv_obj_set_pos(pages[i], 0, 44);
        lv_obj_set_size(pages[i], 800, 436);
        lv_obj_set_style_radius(pages[i], 0, 0);
        lv_obj_set_style_border_width(pages[i], 0, 0);
        lv_obj_set_style_pad_all(pages[i], 0, 0);
        lv_obj_set_style_bg_color(pages[i], lv_color_hex(0x0C151B), 0);
        lv_obj_remove_flag(pages[i], LV_OBJ_FLAG_SCROLLABLE);
        if (i != 0U) lv_obj_add_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
    }

    view_buttons[0] = make_button(pages[0], "时域", 10, 8, 62, 32,
                                  button_event, 3U);
    view_buttons[1] = make_button(pages[0], "频域", 76, 8, 62, 32,
                                  button_event, 4U);
    channel_buttons[0] = make_button(pages[0], "CH1", 184, 8, 62, 32,
                                     button_event, 5U);
    channel_buttons[1] = make_button(pages[0], "CH2", 250, 8, 62, 32,
                                     button_event, 6U);
    stabilize_button = make_button(pages[0], "双稳", 316, 8, 60, 32,
                                   button_event, 21U);
    make_label(pages[0], "周期", 380, 15, 42, &ui_font_scope_extra_16,
               lv_color_hex(0x9EADB2));
    cycle_buttons[0] = make_button(pages[0], "1", 425, 8, 48, 32,
                                   button_event, 7U);
    cycle_buttons[1] = make_button(pages[0], "3", 477, 8, 48, 32,
                                   button_event, 8U);
    auto_button = make_button(pages[0], "AUTO", 535, 8, 72, 32,
                              button_event, 11U);
    cursor_button = make_button(pages[0], "游标", 611, 8, 88, 32,
                                button_event, 12U);
    persistence_button = make_button(pages[0], "余晖", 703, 8, 88, 32,
                                     button_event, 13U);

    chart_title = make_label(pages[0], "双通道时域  CH1触发  1周期", 68, 53, 500,
                             &ui_font_scope_extra_16, lv_color_hex(0xEAF2F2));
    chart = lv_chart_create(pages[0]);
    lv_obj_set_pos(chart, CHART_X, CHART_Y);
    lv_obj_set_size(chart, CHART_W, CHART_H);
    lv_obj_set_style_bg_color(chart, lv_color_hex(0x122029), 0);
    lv_obj_set_style_border_color(chart, lv_color_hex(0x34454C), 0);
    lv_obj_set_style_line_color(chart, lv_color_hex(0x62727B), LV_PART_MAIN);
    lv_obj_set_style_line_opa(chart, grid_brightness, LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, UI_CHART_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -2048, 2048);
    lv_chart_set_div_line_count(chart, 9, 9);
    lv_obj_add_flag(chart, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(chart, chart_touch_event, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(chart, chart_touch_event, LV_EVENT_PRESSING, NULL);
    for (i = 0U; i < 2U; ++i) {
        for (unsigned age = 0U; age < 2U; ++age) {
            lv_color_t faded = lv_color_hex(i ?
                (age ? 0x59432D : 0x86613A) :
                (age ? 0x245452 : 0x287D78));
            history_trace[i][age] = lv_chart_add_series(chart, faded,
                LV_CHART_AXIS_PRIMARY_Y);
            for (unsigned point = 0U; point < UI_CHART_POINTS; ++point)
                history_values[i][age][point] = LV_CHART_POINT_NONE;
            lv_chart_set_series_ext_y_array(chart, history_trace[i][age],
                history_values[i][age]);
            lv_chart_hide_series(chart, history_trace[i][age], true);
        }
    }
    for (i = 0U; i < 2U; ++i) {
        trace[i] = lv_chart_add_series(chart, accent(i),
                                       LV_CHART_AXIS_PRIMARY_Y);
        lv_chart_set_series_ext_y_array(chart, trace[i], chart_values[i]);
    }
    trigger_cursor = lv_chart_add_cursor(chart,
        lv_color_hex(0x91AD55), LV_DIR_HOR);
    y_axis_label[0] = make_label(pages[0], "+5000 mV", 0, 82, 65,
        &lv_font_montserrat_12, lv_color_hex(0xB1C1C6));
    y_axis_label[1] = make_label(pages[0], "0 mV", 0, 198, 65,
        &lv_font_montserrat_12, lv_color_hex(0xB1C1C6));
    y_axis_label[2] = make_label(pages[0], "-5000 mV", 0, 309, 65,
        &lv_font_montserrat_12, lv_color_hex(0xB1C1C6));
    for (i = 0U; i < 3U; ++i)
        lv_obj_set_style_text_align(y_axis_label[i], LV_TEXT_ALIGN_RIGHT, 0);
    x_axis_label[0] = make_label(pages[0], "0", 68, 328, 90,
        &lv_font_montserrat_12, lv_color_hex(0xB1C1C6));
    x_axis_label[1] = make_label(pages[0], "", 274, 328, 90,
        &lv_font_montserrat_12, lv_color_hex(0xB1C1C6));
    x_axis_label[2] = make_label(pages[0], "", 478, 328, 90,
        &lv_font_montserrat_12, lv_color_hex(0xB1C1C6));
    lv_obj_set_style_text_align(x_axis_label[1], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_align(x_axis_label[2], LV_TEXT_ALIGN_RIGHT, 0);
    component_label = make_label(pages[0], "谐波 --", 68, 350, 500,
                                  &lv_font_montserrat_12, lv_color_hex(0xC7D4D6));
    lv_obj_add_flag(component_label, LV_OBJ_FLAG_HIDDEN);

    for (i = 0; i < 2U; ++i) {
        int y = i ? 205 : 54;
        metric_title[i] = make_label(pages[0], i ? "CH2" : "CH1", 590, y, 196,
                                     &lv_font_montserrat_20, accent(i));
        metric_label[i] = make_label(pages[0],
            "基频 -- Hz\n峰峰值 -- mV\n有效值 -- mV\nDC -- mV",
            590, y + 31, 196, &ui_font_scope_extra_16,
            lv_color_hex(0xE5EEEE));
        lv_obj_set_style_text_line_space(metric_label[i], 7, 0);
    }
    cursor_panel = lv_obj_create(pages[0]);
    lv_obj_set_pos(cursor_panel, 584, 50);
    lv_obj_set_size(cursor_panel, 208, 278);
    lv_obj_set_style_radius(cursor_panel, 0, 0);
    lv_obj_set_style_border_width(cursor_panel, 0, 0);
    lv_obj_set_style_pad_all(cursor_panel, 0, 0);
    lv_obj_set_style_bg_color(cursor_panel, lv_color_hex(0x0C151B), 0);
    lv_obj_remove_flag(cursor_panel, LV_OBJ_FLAG_SCROLLABLE);
    make_label(cursor_panel, "游标测量", 6, 4, 196,
               &ui_font_scope_extra_20, lv_color_hex(0xEAF2F2));
    cursor_mode_button[0] = make_button(cursor_panel, "时间", 6, 42, 94, 34,
                                         button_event, 22U);
    cursor_mode_button[1] = make_button(cursor_panel, "电压", 108, 42, 94, 34,
                                         button_event, 23U);
    for (i = 0U; i < 4U; ++i)
        cursor_readout[i] = make_label(cursor_panel, "--", 8, 91 + 43 * i, 194,
            &ui_font_scope_extra_16, lv_color_hex(0xE5EEEE));
    lv_obj_add_flag(cursor_panel, LV_OBJ_FLAG_HIDDEN);
    frame_label = make_label(pages[0], "等待 ADC", 590, 386, 198,
                             &lv_font_montserrat_12,
                             lv_color_hex(0x94A6AD));
    hold_button = make_button(pages[0], "保持", 590, 342, 92, 36,
                              button_event, 9U);
    hold_label = lv_obj_get_child(hold_button, 0);
    single_button = make_button(pages[0], "单次", 690, 342, 92, 36,
                                button_event, 10U);

    make_label(pages[1], "校准", 24, 20, 200, &ui_font_scope_extra_20,
               lv_color_hex(0xEAF2F2));
    calibration_label[0] = make_label(pages[1], "CH1  2.441 mV/code", 24, 70, 500,
               &ui_font_scope_extra_16, accent(0));
    calibration_label[1] = make_label(pages[1], "CH2  2.441 mV/code", 24, 110, 500,
               &ui_font_scope_extra_16, accent(1));
    make_button(pages[1], "CH1", 24, 165, 100, 40, button_event, 17U);
    make_button(pages[1], "CH2", 132, 165, 100, 40, button_event, 18U);
    make_button(pages[1], "0 V 零点", 24, 225, 150, 44, button_event, 19U);
    make_button(pages[1], "正弦幅度", 184, 225, 150, 44, button_event, 20U);
    make_button(pages[1], "参考 Vpp", 345, 225, 150, 44,
                numeric_open_event, 3U);
    calibration_status_label = make_label(pages[1],
        "当前校准 CH1，参考正弦 1000.0 mVpp", 24, 300, 740,
        &ui_font_scope_extra_16, lv_color_hex(0xD7E1E2));

    make_label(pages[2], "测量设置", 24, 20, 240, &ui_font_scope_extra_20,
               lv_color_hex(0xEAF2F2));
    average_label = make_label(pages[2], "均值帧数  1", 24, 83, 330,
                               &ui_font_scope_extra_16, lv_color_hex(0xD7E1E2));
    slider = lv_slider_create(pages[2]);
    lv_obj_set_pos(slider, 26, 128);
    lv_obj_set_size(slider, 330, 20);
    lv_slider_set_range(slider, 1, UI_AVERAGE_MAX);
    lv_slider_set_value(slider, 1, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider, average_event, LV_EVENT_VALUE_CHANGED, NULL);
    threshold_label = make_label(pages[2], "谐波阈值  1.5%", 24, 190, 330,
                                 &ui_font_scope_extra_16, lv_color_hex(0xD7E1E2));
    slider = lv_slider_create(pages[2]);
    lv_obj_set_pos(slider, 26, 235);
    lv_obj_set_size(slider, 330, 20);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 15, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider, threshold_event, LV_EVENT_VALUE_CHANGED, NULL);
    sample_rate_label = make_label(pages[2], "采样档位  AUTO", 420, 82, 300,
        &ui_font_scope_extra_16, lv_color_hex(0xD7E1E2));
    make_button(pages[2], "精确输入", 650, 80, 120, 36,
                numeric_open_event, 4U);
    sample_rate_slider = lv_slider_create(pages[2]);
    lv_obj_set_pos(sample_rate_slider, 422, 140);
    lv_obj_set_size(sample_rate_slider, 340, 20);
    lv_slider_set_range(sample_rate_slider, 0, 3);
    lv_slider_set_value(sample_rate_slider, 0, LV_ANIM_OFF);
    lv_obj_add_event_cb(sample_rate_slider, sample_rate_event,
                        LV_EVENT_VALUE_CHANGED, NULL);
    grid_label = make_label(pages[2], "网格亮度  12%", 420, 200, 300,
        &ui_font_scope_extra_16, lv_color_hex(0xD7E1E2));
    grid_slider = lv_slider_create(pages[2]);
    lv_obj_set_pos(grid_slider, 422, 260);
    lv_obj_set_size(grid_slider, 340, 20);
    lv_slider_set_range(grid_slider, 0, 180);
    lv_slider_set_value(grid_slider, grid_brightness, LV_ANIM_OFF);
    lv_obj_add_event_cb(grid_slider, grid_event, LV_EVENT_VALUE_CHANGED, NULL);

    timebase_label = lv_obj_get_child(scope_controls[0] =
        make_button(pages[0], "时基 AUTO", 10, 345, 168, 32,
                    numeric_open_event, 0U), 0);
    timebase_slider = scope_controls[1] = lv_slider_create(pages[0]);
    lv_obj_set_pos(timebase_slider, 16, 386);
    lv_obj_set_size(timebase_slider, 156, 12);
    lv_slider_set_range(timebase_slider, 0, 11);
    lv_slider_set_value(timebase_slider, timebase_index, LV_ANIM_OFF);
    lv_obj_add_event_cb(timebase_slider, timebase_event, LV_EVENT_VALUE_CHANGED, NULL);

    voltage_label = lv_obj_get_child(scope_controls[2] =
        make_button(pages[0], "幅度 AUTO", 202, 345, 168, 32,
                    numeric_open_event, 1U), 0);
    voltage_slider = scope_controls[3] = lv_slider_create(pages[0]);
    lv_obj_set_pos(voltage_slider, 208, 386);
    lv_obj_set_size(voltage_slider, 156, 12);
    lv_slider_set_range(voltage_slider, 0, 8);
    lv_slider_set_value(voltage_slider, voltage_scale_index, LV_ANIM_OFF);
    lv_obj_add_event_cb(voltage_slider, voltage_event, LV_EVENT_VALUE_CHANGED, NULL);

    trigger_label = lv_obj_get_child(scope_controls[4] =
        make_button(pages[0], "触发 0.0 mV", 394, 345, 168, 32,
                    numeric_open_event, 2U), 0);
    trigger_level_slider = scope_controls[5] = lv_slider_create(pages[0]);
    lv_obj_set_pos(trigger_level_slider, 400, 386);
    lv_obj_set_size(trigger_level_slider, 156, 12);
    lv_slider_set_range(trigger_level_slider, -2048, 2048);
    lv_slider_set_value(trigger_level_slider, 0, LV_ANIM_OFF);
    lv_obj_add_event_cb(trigger_level_slider, trigger_level_event,
        LV_EVENT_VALUE_CHANGED, NULL);
    trigger_source_button = scope_controls[6] = make_button(pages[0],
        "CH1触发", 10, 405, 168, 28,
        button_event, 14U);
    trigger_slope_button = scope_controls[7] = make_button(pages[0],
        "上升沿", 202, 405, 168, 28,
        button_event, 15U);
    trigger_mode_button = scope_controls[8] = make_button(pages[0],
        "自动触发", 394, 405, 168, 28,
        button_event, 16U);
    for (i = 0U; i < 3U; ++i) {
        lv_obj_t *version = make_label(pages[i], "v14", 738, 414, 48,
            &lv_font_montserrat_12, lv_color_hex(0x81939A));
        lv_obj_set_style_text_align(version, LV_TEXT_ALIGN_RIGHT, 0);
    }
    update_controls();
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    log_objects("已创建");
#endif
}

void ax7020_ui_update(const ax7020_snapshot_t *snapshot)
{
    unsigned channel;
    if (!(snapshot->flags & AX7020_FLAG_VALID)) {
        if (status_label != NULL)
            lv_label_set_text(status_label, "采集错误");
        return;
    }
    if (snapshot->calibration_ack != last_calibration_ack) {
        last_calibration_ack = snapshot->calibration_ack;
        if (pending_calibration_action != 0U && calibration_status_label != NULL) {
            if (snapshot->calibration_status == 1U) {
                int saved = ax7020_calibration_save(snapshot);
                lv_label_set_text(calibration_status_label,
                    saved == 0 ? "校准成功，已保存到 SD 卡" :
                                 "校准成功，SD 卡保存失败");
            } else {
                lv_label_set_text(calibration_status_label,
                    snapshot->calibration_status == 2U ?
                    "零点信号不稳定，请接 0 V 后重试" :
                    "参考信号无效或幅度过小");
            }
            pending_calibration_action = 0U;
        }
    }
    for (channel = 0U; channel < 2U; ++channel) {
        if (calibration_label[channel] != NULL) {
            char gain[24], zero[24];
            format_tenth(gain, sizeof(gain),
                         snapshot->calibration_mv_per_code[channel] * 1000.0f);
            format_tenth(zero, sizeof(zero),
                         snapshot->calibration_zero_codes[channel]);
            lv_label_set_text_fmt(calibration_label[channel],
                "CH%u  %s uV/code  zero %s code", channel + 1U, gain, zero);
        }
    }
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) {
        log_objects("首次更新前");
        xil_printf("界面首次更新：写指针=%u 数量=%u 大小=%u\r\n",
            history_write, history_count, (unsigned)sizeof(*snapshot));
    }
#endif
    for (channel = 0; channel < 2U; ++channel) {
        const ax7020_channel_snapshot_t *data = &snapshot->channel[channel];
        history[history_write][channel].frequency_hz = data->frequency_hz;
        history[history_write][channel].peak_to_peak_mv =
            data->peak_to_peak_mv;
        history[history_write][channel].rms_mv = data->rms_mv;
        history[history_write][channel].dc_mv = data->dc_mv;
    }
    history_write = (history_write + 1U) % UI_AVERAGE_MAX;
    if (history_count < UI_AVERAGE_MAX) ++history_count;
    if (hold) return;
    displayed = *snapshot;
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) log_objects("首次更新复制后");
#endif
    displayed_average[0] = averaged(0U);
    displayed_average[1] = averaged(1U);
    have_displayed = 1;
    if (auto_setup_pending &&
        ((displayed.channel[0].frequency_hz >= 80.0f &&
          displayed.channel[0].peak_to_peak_mv >= 20.0f) ||
         (displayed.channel[1].frequency_hz >= 80.0f &&
          displayed.channel[1].peak_to_peak_mv >= 20.0f)))
        apply_auto_setup();
    else
        render();
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    if (!first_update_logged) {
        log_objects("first-update-done");
        first_update_logged = 1;
    }
#endif
    if (single_armed) {
        single_armed = 0;
        hold = 1;
        update_controls();
    }
}

void ax7020_ui_core1_failed(void)
{
    if (status_label != NULL)
        lv_label_set_text(status_label, "采集错误");
}

static void keyboard_click(lv_obj_t *object)
{
    if (object != NULL) (void)lv_obj_send_event(object, LV_EVENT_CLICKED, NULL);
}

static void keyboard_step(lv_obj_t *slider, int amount)
{
    int value;
    if (slider == NULL) return;
    value = lv_slider_get_value(slider) + amount;
    lv_slider_set_value(slider, value, LV_ANIM_OFF);
    (void)lv_obj_send_event(slider, LV_EVENT_VALUE_CHANGED, NULL);
}

void ax7020_ui_key(uint32_t key)
{
    if (numeric_modal != NULL) {
        if (key == 27U) {
            (void)lv_obj_send_event(numeric_keyboard, LV_EVENT_CANCEL, NULL);
        } else if (key == '\r' || key == '\n') {
            (void)lv_obj_send_event(numeric_keyboard, LV_EVENT_READY, NULL);
        } else if (key == 8U || key == 127U) {
            lv_textarea_delete_char(numeric_input);
        } else if ((key >= '0' && key <= '9') ||
                   key == '.' || key == '-' || key == '+') {
            lv_textarea_add_char(numeric_input, key);
        }
        return;
    }
    switch (key) {
    case 'a': case 'A': keyboard_click(auto_button); break;
    case ' ': keyboard_click(hold_label != NULL ?
                             lv_obj_get_parent(hold_label) : NULL); break;
    case 's': case 'S': keyboard_click(single_button); break;
    case 'm': case 'M': keyboard_click(navigation[0]); break;
    case 'k': case 'K': keyboard_click(navigation[1]); break;
    case 'o': case 'O': keyboard_click(navigation[2]); break;
    case 'f': case 'F': keyboard_click(view_buttons[1]); break;
    case 'w': case 'W': keyboard_click(view_buttons[0]); break;
    case '1':
    case '2':
        if (trigger_channel != key - '1')
            keyboard_click(trigger_source_button);
        keyboard_click(channel_buttons[key - '1']);
        break;
    case 'r': case 'R': keyboard_click(trigger_slope_button); break;
    case 'n': case 'N': keyboard_click(trigger_mode_button); break;
    case 'c': case 'C': keyboard_click(cursor_button); break;
    case 'p': case 'P': keyboard_click(persistence_button); break;
    case 'b': case 'B': keyboard_click(stabilize_button); break;
    case 't': case 'T': keyboard_click(scope_controls[0]); break;
    case 'v': case 'V': keyboard_click(scope_controls[2]); break;
    case 'l': case 'L': keyboard_click(scope_controls[4]); break;
    case '[': keyboard_step(timebase_slider, -1); break;
    case ']': keyboard_step(timebase_slider, 1); break;
    case '-': keyboard_step(voltage_slider, -1); break;
    case '=': keyboard_step(voltage_slider, 1); break;
    case ',': keyboard_step(trigger_level_slider, -10); break;
    case '.': keyboard_step(trigger_level_slider, 10); break;
    case 'q': case 'Q':
        keyboard_step(sample_rate_slider,
            capture_rate_mode == 3U ? -3 : 1);
        break;
    case 'g': case 'G':
        keyboard_step(grid_slider, grid_brightness >= 150 ?
            -grid_brightness : 30);
        break;
    default: break;
    }
}

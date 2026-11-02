#include "ax7020_usb_keyboard.h"

#include <string.h>

static uint8_t previous[8];

void ax7020_usb_keyboard_reset_report(void)
{
    memset(previous, 0, sizeof(previous));
}

static uint32_t key_to_ascii(uint8_t code, uint8_t modifiers)
{
    const int shifted = (modifiers & 0x22U) != 0U;
    if (code >= 0x04U && code <= 0x1dU)
        return (shifted ? 'A' : 'a') + code - 0x04U;
    if (code >= 0x1eU && code <= 0x26U)
        return (shifted ? "!@#$%^&*(" : "123456789")[code - 0x1eU];
    if (code == 0x27U) return shifted ? ')' : '0';
    switch (code) {
    case 0x28U: case 0x58U: return '\r';
    case 0x29U: return 27U;
    case 0x2aU: return 8U;
    case 0x2cU: return ' ';
    case 0x2dU: return shifted ? '_' : '-';
    case 0x2eU: return shifted ? '+' : '=';
    case 0x2fU: return shifted ? '{' : '[';
    case 0x30U: return shifted ? '}' : ']';
    case 0x36U: return shifted ? '<' : ',';
    case 0x37U: return shifted ? '>' : '.';
    case 0x54U: return '/';
    case 0x55U: return '*';
    case 0x56U: return '-';
    case 0x57U: return '+';
    case 0x59U: case 0x5aU: case 0x5bU: case 0x5cU:
    case 0x5dU: case 0x5eU: case 0x5fU: case 0x60U:
    case 0x61U: return '1' + code - 0x59U;
    case 0x62U: return '0';
    case 0x63U: return '.';
    default: return 0U;
    }
}

void ax7020_usb_keyboard_report(const uint8_t report[8],
                                void (*emit)(uint32_t key))
{
    unsigned i, j;
    if (report == NULL || emit == NULL) return;
    /* HID error rollover means that the set of held keys is unknown. */
    for (i = 2U; i < 8U; ++i)
        if (report[i] >= 1U && report[i] <= 3U) return;
    for (i = 2U; i < 8U; ++i) {
        uint8_t code = report[i];
        uint32_t key;
        if (code == 0U) continue;
        for (j = 2U; j < 8U; ++j)
            if (previous[j] == code) break;
        if (j != 8U) continue;
        key = key_to_ascii(code, report[0]);
        if (key != 0U) emit(key);
    }
    memcpy(previous, report, sizeof(previous));
}

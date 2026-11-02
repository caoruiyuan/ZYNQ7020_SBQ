#ifndef AX7020_USB_KEYBOARD_H
#define AX7020_USB_KEYBOARD_H

#include <stdint.h>

int ax7020_usb_keyboard_init(void);
void ax7020_usb_keyboard_task(void);

/* One ASCII command for each new key press in a boot keyboard report. */
void ax7020_usb_keyboard_report(const uint8_t report[8],
                                void (*emit)(uint32_t key));
void ax7020_usb_keyboard_reset_report(void);

#endif

#ifndef AX7020_UI_H
#define AX7020_UI_H

#include "ax7020_shared.h"

void ax7020_ui_create(volatile ax7020_mailbox_t *mailbox);
void ax7020_ui_update(const ax7020_snapshot_t *snapshot);
void ax7020_ui_core1_failed(void);
void ax7020_ui_key(uint32_t key);

#endif

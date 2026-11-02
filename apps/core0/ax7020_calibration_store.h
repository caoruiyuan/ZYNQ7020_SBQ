#ifndef AX7020_CALIBRATION_STORE_H
#define AX7020_CALIBRATION_STORE_H

#include "ax7020_shared.h"

int ax7020_calibration_load(volatile ax7020_mailbox_t *mailbox);
int ax7020_calibration_save(const ax7020_snapshot_t *snapshot);

#endif

#include "ax7020_calibration_store.h"

#include "ff.h"
#include "sleep.h"
#include "xil_io.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define CALIBRATION_MAGIC 0x41435837U
#define CALIBRATION_FILE "0:/AXCAL.BIN"

typedef struct {
    uint32_t magic;
    uint32_t version;
    float zero_codes[2];
    float mv_per_code[2];
    uint32_t checksum;
} calibration_record_t;

static FATFS filesystem;
static int mounted;

static uint32_t checksum(const calibration_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t value = 2166136261U;
    unsigned i;
    for (i = 0U; i < sizeof(*record) - sizeof(record->checksum); ++i)
        value = (value ^ bytes[i]) * 16777619U;
    return value;
}

static int mount_card(void)
{
    if (mounted) return 0;
    if (f_mount(&filesystem, "0:/", 1) != FR_OK) return -1;
    mounted = 1;
    return 0;
}

int ax7020_calibration_load(volatile ax7020_mailbox_t *mailbox)
{
    calibration_record_t record;
    FIL file;
    UINT read_count;
    unsigned i;
    unsigned retry;
    if (mount_card() != 0 ||
        f_open(&file, CALIBRATION_FILE, FA_READ) != FR_OK) return -1;
    read_count = 0U;
    FRESULT result = f_read(&file, &record, sizeof(record), &read_count);
    (void)f_close(&file);
    if (result != FR_OK || read_count != sizeof(record) ||
        record.magic != CALIBRATION_MAGIC || record.version != 1U ||
        checksum(&record) != record.checksum) return -1;
    for (i = 0U; i < 2U; ++i) {
        if (!isfinite(record.zero_codes[i]) ||
            fabsf(record.zero_codes[i]) > 4096.0f ||
            !isfinite(record.mv_per_code[i]) ||
            record.mv_per_code[i] < 0.1f || record.mv_per_code[i] > 5.0f)
            return -1;
    }
    for (i = 0U; i < 2U; ++i) {
        mailbox->calibration_zero_codes[i] = record.zero_codes[i];
        mailbox->calibration_mv_per_code[i] = record.mv_per_code[i];
        mailbox->calibration_channel = i;
        mailbox->calibration_action = 3U;
        dmb();
        mailbox->calibration_request++;
        for (retry = 0U; retry < 2000U; ++retry) {
            if (mailbox->slots[mailbox->published_slot & 1U].calibration_ack ==
                mailbox->calibration_request) break;
            usleep(1000U);
        }
        if (retry == 2000U) return -1;
    }
    return 0;
}

int ax7020_calibration_save(const ax7020_snapshot_t *snapshot)
{
    calibration_record_t record;
    FIL file;
    UINT written;
    FRESULT result;
    unsigned i;
    if (mount_card() != 0) return -1;
    memset(&record, 0, sizeof(record));
    record.magic = CALIBRATION_MAGIC;
    record.version = 1U;
    for (i = 0U; i < 2U; ++i) {
        record.zero_codes[i] = snapshot->calibration_zero_codes[i];
        record.mv_per_code[i] = snapshot->calibration_mv_per_code[i];
    }
    record.checksum = checksum(&record);
    result = f_open(&file, CALIBRATION_FILE, FA_CREATE_ALWAYS | FA_WRITE);
    if (result != FR_OK) return -1;
    written = 0U;
    result = f_write(&file, &record, sizeof(record), &written);
    if (result == FR_OK) result = f_sync(&file);
    (void)f_close(&file);
    return result == FR_OK && written == sizeof(record) ? 0 : -1;
}

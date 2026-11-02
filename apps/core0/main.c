#include "ax7020_shared.h"
#include "ax7020_calibration_store.h"
#include "ax7020_ui.h"
#include "ax7020_usb_keyboard.h"
#include "lv_port_touch.h"
#include "lv_port_zynq.h"
#include "lvgl.h"

#include "sleep.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xil_mmu.h"
#include "xil_printf.h"
#include "xgpiops.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xiltimer.h"
#include "xuartps.h"
#include "xuartps_hw.h"

#include <string.h>

#define CORE1_START_VECTOR 0xFFFFFFF0U
#define HDMI_ENABLE_GPIO_PIN 54U
#define HDMI_TOUCH_WAIT_MS 3000U

static volatile ax7020_mailbox_t *const mailbox =
    (volatile ax7020_mailbox_t *)AX7020_MAILBOX_ADDR;
static uint32_t last_sequence;
static uint32_t last_capture_log_ms;
static uint32_t last_capture_flags = UINT32_MAX;
static uint32_t last_performance_log_ms;
static uint32_t last_display_frames;
static uint32_t acquired_frames;
static uint64_t acquisition_total_us;
static uint64_t processing_total_us;
static uint64_t ui_total_us;
static uint64_t lvgl_total_us;
static XGpioPs hdmi_gpio;
static int hdmi_gpio_ready;
static int hdmi_enabled;

void outbyte(char character)
{
    while (XUartPs_IsTransmitFull(XPAR_UART1_BASEADDR)) { }
    XUartPs_WriteReg(XPAR_UART1_BASEADDR, XUARTPS_FIFO_OFFSET,
                     (uint32_t)(uint8_t)character);
}

char inbyte(void)
{
    while (!XUartPs_IsReceiveData(XPAR_UART1_BASEADDR)) { }
    return (char)XUartPs_ReadReg(XPAR_UART1_BASEADDR, XUARTPS_FIFO_OFFSET);
}

static void initialize_uart(void)
{
    XUartPs uart;
#ifdef SDT
    XUartPs_Config *config = XUartPs_LookupConfig(XPAR_UART1_BASEADDR);
#else
    XUartPs_Config *config = XUartPs_LookupConfig(XPAR_XUARTPS_1_DEVICE_ID);
#endif
    if (config != NULL &&
        XUartPs_CfgInitialize(&uart, config, config->BaseAddress) == XST_SUCCESS) {
        (void)XUartPs_SetBaudRate(&uart, 115200U);
        XUartPs_EnableUart(&uart);
    }
}

static void initialize_hdmi_enable(void)
{
    XGpioPs_Config *config =
        XGpioPs_LookupConfig((u32)XPAR_XGPIOPS_0_BASEADDR);
    if (config == NULL ||
        XGpioPs_CfgInitialize(&hdmi_gpio, config, config->BaseAddr) !=
            XST_SUCCESS) {
        xil_printf("HDMI 使能 GPIO 初始化失败\r\n");
        return;
    }
    XGpioPs_WritePin(&hdmi_gpio, HDMI_ENABLE_GPIO_PIN, 0U);
    XGpioPs_SetDirectionPin(&hdmi_gpio, HDMI_ENABLE_GPIO_PIN, 1U);
    XGpioPs_SetOutputEnablePin(&hdmi_gpio, HDMI_ENABLE_GPIO_PIN, 1U);
    hdmi_gpio_ready = 1;
}

static void enable_hdmi_after_touch(uint32_t elapsed_ms)
{
    if (hdmi_enabled || !hdmi_gpio_ready ||
        (!lv_port_touch_is_online() && elapsed_ms < HDMI_TOUCH_WAIT_MS))
        return;

    XGpioPs_WritePin(&hdmi_gpio, HDMI_ENABLE_GPIO_PIN, 1U);
    hdmi_enabled = 1;
    xil_printf("HDMI 5V 已使能：启动后 %u ms，GT911 在线=%u\r\n",
               (unsigned)elapsed_ms, (unsigned)lv_port_touch_is_online());
}

static int start_core1(void)
{
    unsigned retry;
    Xil_Out32(CORE1_START_VECTOR, AX7020_CORE1_ENTRY);
    Xil_DCacheFlushRange(CORE1_START_VECTOR, sizeof(uint32_t));
    dsb();
    for (retry = 0; retry < 1000U; ++retry) {
        __asm__ volatile("sev" : : : "memory");
        usleep(1000U);
        if (mailbox->magic == AX7020_MAILBOX_MAGIC &&
            mailbox->version == AX7020_MAILBOX_VERSION)
            return XST_SUCCESS;
    }
    return XST_FAILURE;
}

static int read_snapshot(ax7020_snapshot_t *copy)
{
    uint32_t before = mailbox->published_sequence;
    uint32_t slot;
    if (before == last_sequence || mailbox->magic != AX7020_MAILBOX_MAGIC ||
        mailbox->version != AX7020_MAILBOX_VERSION)
        return 0;
    dmb();
    slot = mailbox->published_slot & 1U;
    memcpy(copy, (const void *)&mailbox->slots[slot], sizeof(*copy));
    dmb();
    if (before != mailbox->published_sequence) return 0;
    last_sequence = before;
    return 1;
}

static void log_capture_status(const ax7020_snapshot_t *frame, uint32_t now_ms)
{
    const uint32_t error_flags = frame->flags & ~AX7020_FLAG_VALID;
    const int healthy = (frame->flags & AX7020_FLAG_VALID) != 0U &&
                        error_flags == 0U;

    if (frame->flags == last_capture_flags &&
        (healthy || (uint32_t)(now_ms - last_capture_log_ms) < 5000U))
        return;
    if (last_capture_flags != UINT32_MAX &&
        (uint32_t)(now_ms - last_capture_log_ms) < 1000U)
        return;

    if (healthy) {
        if (last_capture_flags != UINT32_MAX)
            xil_printf("采集恢复：帧号=%u\r\n",
                       (unsigned)frame->frame_sequence);
    } else {
        xil_printf("采集故障：标志=0x%02x 帧号=%u PL=%08x DMA=%08x\r\n",
                   (unsigned)frame->flags, (unsigned)frame->frame_sequence,
                   (unsigned)frame->pl_status, (unsigned)frame->dma_status);
    }
    if (!healthy && (error_flags & AX7020_FLAG_PACKET_ERROR) != 0U)
        xil_printf("数据包故障：PL=%08x DMA=%08x 包头=%08x 版本=%08x 字数=%u 包尾=%08x FFT数=%u 包尾字数=%u 原始头=%u 原始尾=%u\r\n",
            (unsigned)frame->pl_status, (unsigned)frame->dma_status,
            (unsigned)frame->packet_debug[0], (unsigned)frame->packet_debug[1],
            (unsigned)frame->packet_debug[2], (unsigned)frame->packet_debug[3],
            (unsigned)frame->packet_debug[4], (unsigned)frame->packet_debug[5],
            (unsigned)frame->packet_debug[6], (unsigned)frame->packet_debug[7]);
    last_capture_flags = frame->flags;
    last_capture_log_ms = now_ms;
}

int main(void)
{
    static ax7020_snapshot_t frame;
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    uint32_t last_display_log_ms = 0U;
    int first_handler_returned = 0;
#endif
    int status;

    initialize_uart();
    xil_printf("AX7020 双通道示波器 v14 启动\r\n");
    initialize_hdmi_enable();
    Xil_SetTlbAttributes(AX7020_MAILBOX_ADDR, NORM_NONCACHE);
    memset((void *)mailbox, 0, sizeof(*mailbox));

    lv_init();
    status = lv_port_zynq_init();
    if (status != XST_SUCCESS) {
        xil_printf("显示 VDMA 初始化失败：状态=%d\r\n", status);
        for (;;) { usleep(100000U); }
    }
    status = lv_port_touch_init();
    if (status != XST_SUCCESS)
        xil_printf("GT911 输入设备初始化失败：状态=%d\r\n", status);
    ax7020_ui_create(mailbox);

#ifdef AX7020_DISPLAY_DIAGNOSTICS
    xil_printf("显示：开始首次界面刷新\r\n");
#endif
    lv_refr_now(NULL);
#ifdef AX7020_DISPLAY_DIAGNOSTICS
    xil_printf("显示：首次界面刷新完成\r\n");
    first_handler_returned = 1;
#endif

    status = start_core1();
    if (status != XST_SUCCESS) {
        xil_printf("核1启动失败或双核邮箱版本不匹配\r\n");
        ax7020_ui_core1_failed();
    } else if (ax7020_calibration_load(mailbox) == 0) {
        xil_printf("已从 SD 卡加载校准值\r\n");
    } else {
        xil_printf("校准：使用标称值，或 SD 卡校准文件不可用\r\n");
    }
    (void)ax7020_usb_keyboard_init();
    for (;;) {
        enable_hdmi_after_touch(lv_tick_get());
        ax7020_usb_keyboard_task();
        while (XUartPs_IsReceiveData(XPAR_UART1_BASEADDR)) {
            uint32_t key = XUartPs_ReadReg(XPAR_UART1_BASEADDR,
                                            XUARTPS_FIFO_OFFSET) & 0xffU;
            ax7020_ui_key(key);
        }
        if (read_snapshot(&frame)) {
            XTime update_start, update_end;
            ++acquired_frames;
            uint32_t now_ms = lv_tick_get();
            acquisition_total_us += frame.acquisition_us;
            processing_total_us += frame.processing_us;
            log_capture_status(&frame, now_ms);
            XTime_GetTime(&update_start);
            ax7020_ui_update(&frame);
            XTime_GetTime(&update_end);
            ui_total_us += (uint64_t)(update_end - update_start) *
                           1000000ULL / COUNTS_PER_SECOND;
        }
        {
            XTime refresh_start, refresh_end;
            XTime_GetTime(&refresh_start);
            (void)lv_timer_handler();
            XTime_GetTime(&refresh_end);
            lvgl_total_us += (uint64_t)(refresh_end - refresh_start) *
                             1000000ULL / COUNTS_PER_SECOND;
        }
        {
            uint32_t now_ms = lv_tick_get();
            uint32_t interval = now_ms - last_performance_log_ms;
            if (interval >= 5000U) {
                uint32_t display_frames = lv_port_zynq_frame_count();
                xil_printf("系统帧率：采集=%u 帧/秒，显示=%u 帧/秒；"
                           "采集均值=%u us，处理均值=%u us，界面均值=%u us，"
                           "LVGL累计=%u ms\r\n",
                    (unsigned)(acquired_frames * 1000U / interval),
                    (unsigned)((display_frames - last_display_frames) *
                               1000U / interval),
                    (unsigned)(acquired_frames ?
                               acquisition_total_us / acquired_frames : 0U),
                    (unsigned)(acquired_frames ?
                               processing_total_us / acquired_frames : 0U),
                    (unsigned)(acquired_frames ?
                               ui_total_us / acquired_frames : 0U),
                    (unsigned)(lvgl_total_us / 1000U));
                acquired_frames = 0U;
                acquisition_total_us = 0U;
                processing_total_us = 0U;
                ui_total_us = 0U;
                lvgl_total_us = 0U;
                last_display_frames = display_frames;
                last_performance_log_ms = now_ms;
            }
        }
#ifdef AX7020_DISPLAY_DIAGNOSTICS
        if (first_handler_returned == 1) {
            xil_printf("显示：测量处理已返回\r\n");
            first_handler_returned = 2;
        }
        if ((uint32_t)(lv_tick_get() - last_display_log_ms) >= 2000U) {
            last_display_log_ms = lv_tick_get();
            lv_port_zynq_log_status();
        }
#endif
        usleep(1000U);
    }
}

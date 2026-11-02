#include "ax7020_usb_keyboard.h"
#include "ax7020_ui.h"
#include "tusb.h"
#include "host/hcd.h"
#include "portable/ehci/ehci_api.h"

#include "sleep.h"
#include "xgpiops.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"

#define USB_BASE ((uintptr_t)XPAR_USB0_BASEADDR)
#define USB_CAPLENGTH 0x100U
#define USB_COMMAND 0x140U
#define USB_STATUS 0x144U
#define USB_PORTSC 0x184U
#define USB_MODE 0x1a8U
#define USB_PHY_RESET_MIO 46U

static int host_ready;
static uint8_t keyboard_addr;
static uint8_t keyboard_instance;
static int need_report;

void osal_task_delay(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) usleep(1000U);
}

bool hcd_dcache_clean(const void *addr, uint32_t length)
{
    Xil_DCacheFlushRange((INTPTR)addr, length);
    return true;
}

bool hcd_dcache_invalidate(const void *addr, uint32_t length)
{
    Xil_DCacheInvalidateRange((INTPTR)addr, length);
    return true;
}

bool hcd_dcache_clean_invalidate(const void *addr, uint32_t length)
{
    Xil_DCacheFlushRange((INTPTR)addr, length);
    Xil_DCacheInvalidateRange((INTPTR)addr, length);
    return true;
}

bool hcd_init(uint8_t rhport)
{
    unsigned retry;
    uint32_t port;
    if (rhport != 0U || (Xil_In32(USB_BASE + USB_CAPLENGTH) & 0xffU) != 0x40U)
        return false;

    Xil_Out32(USB_BASE + USB_COMMAND, 2U);
    for (retry = 0U; retry < 10000U; ++retry) {
        if ((Xil_In32(USB_BASE + USB_COMMAND) & 2U) == 0U) break;
        usleep(10U);
    }
    if (retry == 10000U) return false;

    Xil_Out32(USB_BASE + USB_MODE, 3U);
    port = Xil_In32(USB_BASE + USB_PORTSC);
    /* Zynq-7000 USB0 uses an external 8-bit ULPI PHY. */
    port = (port & ~(3U << 30U | 0x2aU)) | (2U << 30U);
    Xil_Out32(USB_BASE + USB_PORTSC, port);
    if (!ehci_init(rhport, (uint32_t)(USB_BASE + USB_CAPLENGTH),
                   (uint32_t)(USB_BASE + USB_COMMAND)))
        return false;
    port = Xil_In32(USB_BASE + USB_PORTSC);
    xil_printf("USB 主机就绪：能力=%02x 端口=%08x\r\n",
               (unsigned)(Xil_In32(USB_BASE + USB_CAPLENGTH) & 0xffU),
               (unsigned)port);
    if ((port & 1U) != 0U && (port & 2U) == 0U)
        hcd_event_device_attach(rhport, false);
    return true;
}

void hcd_int_enable(uint8_t rhport) { (void)rhport; }
void hcd_int_disable(uint8_t rhport) { (void)rhport; }

static int release_phy_reset(void)
{
    XGpioPs gpio;
    XGpioPs_Config *config = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_BASEADDR);
    if (config == NULL ||
        XGpioPs_CfgInitialize(&gpio, config, config->BaseAddr) != XST_SUCCESS)
        return XST_FAILURE;
    XGpioPs_SetDirectionPin(&gpio, USB_PHY_RESET_MIO, 1U);
    XGpioPs_SetOutputEnablePin(&gpio, USB_PHY_RESET_MIO, 1U);
    XGpioPs_WritePin(&gpio, USB_PHY_RESET_MIO, 0U);
    usleep(10000U);
    XGpioPs_WritePin(&gpio, USB_PHY_RESET_MIO, 1U);
    usleep(50000U);
    return XST_SUCCESS;
}

int ax7020_usb_keyboard_init(void)
{
    if (release_phy_reset() != XST_SUCCESS) {
        xil_printf("USB PHY 复位 GPIO 不可用\r\n");
        return XST_FAILURE;
    }
    if (!tuh_init(0U)) {
        xil_printf("USB 主机初始化失败\r\n");
        return XST_FAILURE;
    }
    host_ready = 1;
    return XST_SUCCESS;
}

void ax7020_usb_keyboard_task(void)
{
    uint32_t status;
    if (!host_ready) return;
    status = Xil_In32(USB_BASE + USB_STATUS);
    if ((status & 0x1000U) != 0U) return;
    if ((status & (1U | 2U | 4U | 0x20U | 0x80U)) != 0U)
        hcd_int_handler(0U, false);
    tuh_task_ext(0U, false);
    if (need_report && keyboard_addr != 0U &&
        tuh_hid_receive_report(keyboard_addr, keyboard_instance))
        need_report = 0;
}

void tuh_mount_cb(uint8_t address)
{
    uint16_t vendor = 0U, product = 0U;
    (void)tuh_vid_pid_get(address, &vendor, &product);
    xil_printf("USB 设备已连接：地址=%u VID=%04x PID=%04x\r\n",
               (unsigned)address, (unsigned)vendor, (unsigned)product);
}

void tuh_umount_cb(uint8_t address)
{
    xil_printf("USB 设备已移除：地址=%u\r\n", (unsigned)address);
}

void tuh_hid_mount_cb(uint8_t address, uint8_t instance,
                      const uint8_t *descriptor, uint16_t length)
{
    (void)descriptor;
    (void)length;
    if (tuh_hid_interface_protocol(address, instance) != HID_ITF_PROTOCOL_KEYBOARD) {
        xil_printf("USB HID 接口已忽略：地址=%u 协议=%u\r\n",
                   (unsigned)address,
                   (unsigned)tuh_hid_interface_protocol(address, instance));
        return;
    }
    if (keyboard_addr != 0U) return;
    keyboard_addr = address;
    keyboard_instance = instance;
    need_report = 1;
    ax7020_usb_keyboard_reset_report();
    xil_printf("USB 键盘已连接：地址=%u 实例=%u\r\n",
               (unsigned)address, (unsigned)instance);
}

void tuh_hid_umount_cb(uint8_t address, uint8_t instance)
{
    if (address != keyboard_addr || instance != keyboard_instance) return;
    keyboard_addr = 0U;
    need_report = 0;
    ax7020_usb_keyboard_reset_report();
    xil_printf("USB 键盘已移除\r\n");
}

void tuh_hid_report_received_cb(uint8_t address, uint8_t instance,
                                const uint8_t *report, uint16_t length)
{
    if (address != keyboard_addr || instance != keyboard_instance) return;
    if (length >= 8U)
        ax7020_usb_keyboard_report(report, ax7020_ui_key);
    need_report = 1;
}

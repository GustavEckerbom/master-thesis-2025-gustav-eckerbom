#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "mhu.h"
#include "mhu_driver.h"
#include "services_lib_api.h"
#include "services_lib_interface.h"
#include "M55_HP.h"
#include "Driver_GPIO.h"

// MHU base addresses
#define SE_MHU0_RECV_BASE      0x40040000
#define SE_MHU0_SEND_BASE      0x40050000
#define NUM_MHU                1
#define MHU_M55_SE_MHU0        0
#define MAXIMUM_TIMEOUT        0x01000000

#define FREQ_10MHZ             10000000U

#define 	BOARD_GPIO_PINHEAD_GPIO_PORT 12
#define 	BOARD_GPIO_PINHEAD_GPIO_PIN 0

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_GPIO_PINHEAD_GPIO_PORT);
ARM_DRIVER_GPIO *BOARD_IOHEADER_GPIOdrv = &ARM_Driver_GPIO_(BOARD_GPIO_PINHEAD_GPIO_PORT);

// MHU IRQ numbers (platform specific — check CMSIS headers if needed)
#define MHU_SESS_S_RX_IRQ_IRQn   (63)
#define MHU_SESS_S_TX_IRQ_IRQn   (64)

static uint32_t sender_base_address_list[NUM_MHU]   = { SE_MHU0_SEND_BASE };
static uint32_t receiver_base_address_list[NUM_MHU] = { SE_MHU0_RECV_BASE };

static mhu_driver_in_t s_mhu_driver_in;
static mhu_driver_out_t s_mhu_driver_out;

/** Dummy weak implementations if not using CMSIS startup */
void __attribute__((weak)) MHU_SESS_S_RX_IRQHandler(void) { s_mhu_driver_out.receiver_irq_handler(MHU_M55_SE_MHU0); }
void __attribute__((weak)) MHU_SESS_S_TX_IRQHandler(void) { s_mhu_driver_out.sender_irq_handler(MHU_M55_SE_MHU0); }

/**
 * Setup IRQ for MHU communication
 */
static void setup_irq(int irq_num)
{
    NVIC_DisableIRQ((IRQn_Type)irq_num);
    NVIC_ClearPendingIRQ((IRQn_Type)irq_num);
    NVIC_SetPriority((IRQn_Type)irq_num, 2);
    NVIC_EnableIRQ((IRQn_Type)irq_num);
}

/**
 * Initialize the MHU driver and register callbacks
 */
static void mhu_initialize(void)
{
    s_mhu_driver_in.sender_base_address_list    = sender_base_address_list;
    s_mhu_driver_in.receiver_base_address_list  = receiver_base_address_list;
    s_mhu_driver_in.mhu_count                   = NUM_MHU;
    s_mhu_driver_in.send_msg_acked_callback     = SERVICES_send_msg_acked_callback;
    s_mhu_driver_in.rx_msg_callback             = SERVICES_rx_msg_callback;
    s_mhu_driver_in.debug_print                 = SERVICES_print;

    MHU_driver_initialize(&s_mhu_driver_in, &s_mhu_driver_out);

    setup_irq(MHU_SESS_S_RX_IRQ_IRQn);
    setup_irq(MHU_SESS_S_TX_IRQ_IRQn);
}

/**
 * Set the HP core frequency to 10 MHz
 */
static void configure_hp_clock_to_10mhz(uint32_t services_handle)
{
    uint32_t error_code;
    uint32_t ret = SERVICES_clocks_set_ES0_frequency(services_handle, CLOCK_FREQUENCY_10MHZ, &error_code);

    if (ret != SERVICE_SUCCESS || error_code != SERVICE_SUCCESS) {
        SERVICES_print("[ERROR] Failed to set HP core frequency to 10MHz. ret=0x%08X, err=0x%08X\n", ret, error_code);
    } else {
        SERVICES_print("[INFO] HP core frequency successfully set to 10MHz\n");
    }
}

/**
 * Main entry point
 */
int main(void)
{
    BOARD_IOHEADER_GPIOdrv->Initialize(BOARD_GPIO_PINHEAD_GPIO_PIN, NULL);
	BOARD_IOHEADER_GPIOdrv->PowerControl(BOARD_GPIO_PINHEAD_GPIO_PIN, ARM_POWER_FULL);
	BOARD_IOHEADER_GPIOdrv->SetDirection(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
	BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    // Step 1: Initialize MHU + IRQ
    mhu_initialize();

    // Step 2: Setup SE Services
    SERVICES_Setup(s_mhu_driver_out.send_message, MAXIMUM_TIMEOUT);

    // Step 3: Register channel and get handle
    uint32_t services_handle = SERVICES_register_channel(MHU_M55_SE_MHU0, 0);

    // Optional: Wait a bit for SE sync
    SERVICES_wait_ms(10);

    // Step 4: Set HP core clock
    configure_hp_clock_to_10mhz(services_handle);

    while (1); // Stay alive
}

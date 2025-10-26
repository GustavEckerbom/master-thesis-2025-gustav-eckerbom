// board_gpio_drivers.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"        // BOARD_*_GPIO_PORT macros
#include "Driver_GPIO.h"

#ifdef __cplusplus
}
#endif

// --- Vendor-provided driver instances (DECLARATIONS only) ---

// LEDs / indicators
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LED_TOP_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LED_BOT_GPIO_PORT);

// IO header
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_GPIO_PINHEAD_GPIO_PORT);

// Camera module pins (outputs / misc)
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_TRIG_IN_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_DGPIO_GPIO_PORT);

// Camera ULP pins
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_ULP_ENB_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_ULP_TH_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_ULP_RSTN_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_ULP_WAKEUP_GPIO_PORT);

// User push button
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_USER_PUSH_GPIO_PORT);

// Camera reset / transmission disable
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_RSTN_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PXRSTN_GPIO_PORT);

// Camera power enables
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V1_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V8_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_2V5_GPIO_PORT);

// Optional: sync/interrupt GPIOs used elsewhere
#ifdef BOARD_VSYNC_INTERRUPT_GPIO_PORT
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_VSYNC_INTERRUPT_GPIO_PORT);
#endif

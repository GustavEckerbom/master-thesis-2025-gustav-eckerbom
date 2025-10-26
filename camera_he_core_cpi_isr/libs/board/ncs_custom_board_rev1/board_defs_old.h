/******************************************************************************
 * @file     board.h
 * @brief    BOARD API
 ******************************************************************************/
/* 
    Pin number was set before the final revison of the board
    Uncomment carefully and check schmatic that the pin number is correct and wasn't changed
    in the final revision of the board
*/


#define BOARD_LED_COUNT                         2
#define BOARD_BUTTON_COUNT                      1

// User I2C accessible on pinheader
#define BOARD_I2C_INSTANCE                      2
// #define BOARD_I2C_SCL_PORT                      5
// #define BOARD_I2C_SCL_PIN_NO                    6
// #define BOARD_I2C_SDA_PORT                      5
// #define BOARD_I2C_SDA_PIN_NO                    7

// User UART accessible on pinheader 
#define BOARD_UART_INSTANCE                     2 // CHANGE THIS BACK TO 1 for custom board

// #define BOARD_UART_RX_PORT                      
// #define BOARD_UART_RX_PIN_NO                    
// #define BOARD_UART_TX_PORT                      
// #define BOARD_UART_TX_PIN_NO     

// User button
#define BOARD_USER_BUTTON_GPIO_PORT             15
#define BOARD_USER_BUTTON_PIN                   1

// User LED TOP 
#define BOARD_LED_TOP_GPIO_PORT                 12
#define BOARD_LED_TOP_PIN                       0

// User LED BOT
#define BOARD_LED_BOT_GPIO_PORT                 2
#define BOARD_LED_BOT_PIN                       4

// CAM module defs
// The I2C address of the GenX320 sensor is fixed on the CM2 module as 0x3Ch (011 1100b).
#define BOARD_CAM_I2C_INSTANCE                  0
#define BOARD_CAM_I2C_SDA_PORT                  3
#define BOARD_CAM_I2C_SDA_PIN                   5
#define BOARD_CAM_I2C_SCL_PORT                  3
#define BOARD_CAM_I2C_SCL_PIN                   4

#define BOARD_CAM_TRIG_IN_GPIO_PORT             9
#define BOARD_CAM_TRIG_IN_PIN                   1

#define BOARD_CAM_DGPIO_GPIO_PORT               9
#define BOARD_CAM_DGPIO_PIN                     5

// ULP PINS
#define BOARD_CAM_ULP_ENB_GPIO_PORT             11
#define BOARD_CAM_ULP_ENB_GPIO_PIN              2

#define BOARD_CAM_ULP_TH_GPIO_PORT              15
#define BOARD_CAM_ULP_TH_GPIO_PIN               3

#define BOARD_CAM_ULP_RST_GPIO_PORT             7
#define BOARD_CAM_ULP_RST_GPIO_PIN              4

#define BOARD_CAM_ULP_WAKEUP_GPIO_PORT          7
#define BOARD_CAM_ULP_WAKEUP_GPIO_PIN           7

// Camera power resest pin
#define BOARD_CAM_RESET_GPIO_PORT               7
#define BOARD_CAM_RESET_GPIO_PIN                3

// Camera transmission disbale pin
#define BOARD_CAM_PXRSTN_GPIO_PORT              15
#define BOARD_CAM_PXRSTN_GPIO_PIN               2

// Camera power supply enable pins
#define BOARD_CAM_PSU_EN_1V1_GPIO_PORT          15
#define BOARD_CAM_PSU_EN_1V1_GPIO_PIN           7

#define BOARD_CAM_PSU_EN_1V8_GPIO_PORT          15
#define BOARD_CAM_PSU_EN_1V8_GPIO_PIN           5

#define BOARD_CAM_PSU_EN_2V5_GPIO_PORT          2
#define BOARD_CAM_PSU_EN_2V5_GPIO_PIN           5
 
// CAM CPI defs
// #define BOARD_CAM_CPI_D0_PORT                   8
// #define BOARD_CAM_CPI_D0_PIN                    0

// #define BOARD_CAM_CPI_D1_PORT                   8
// #define BOARD_CAM_CPI_D1_PIN                    1

// #define BOARD_CAM_CPI_D2_PORT                   8
// #define BOARD_CAM_CPI_D2_PIN                    2

// #define BOARD_CAM_CPI_D3_PORT                   8
// #define BOARD_CAM_CPI_D3_PIN                    3

// #define BOARD_CAM_CPI_D4_PORT                   8
// #define BOARD_CAM_CPI_D4_PIN                    4

// #define BOARD_CAM_CPI_D5_PORT                   8
// #define BOARD_CAM_CPI_D5_PIN                    5

// #define BOARD_CAM_CPI_D6_PORT                   8
// #define BOARD_CAM_CPI_D6_PIN                    6

// #define BOARD_CAM_CPI_D7_PORT                   8
// #define BOARD_CAM_CPI_D7_PIN                    7

// #define BOARD_CAM_CPI_HSYNC_PORT                10
// #define BOARD_CAM_CPI_HSYNC_PIN                 0

// #define BOARD_CAM_CPI_VSYNC_PORT                10
// #define BOARD_CAM_CPI_VSYNC_PIN                 1   

// #define BOARD_CAM_CPI_PCLK_PORT                 10
// #define BOARD_CAM_CPI_PCLK_PIN                  2

// #define BOARD_CAM_CPI_XVCLK_PORT                10
// #define BOARD_CAM_CPI_XVCLK_PIN                 3



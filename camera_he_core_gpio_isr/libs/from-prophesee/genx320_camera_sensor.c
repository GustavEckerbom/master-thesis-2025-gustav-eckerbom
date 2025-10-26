/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     genx320_camera_sensor.c
 * @author   Gustav Eckerbom
 * @email    gustavec@kth.se
 * @version  V1.0.0
 * @date     24-March-2025
 * @brief    Prophesee GenX320 Camera Sensor driver.
 * @bug      None.
 * @Note     Supports event-based image streaming over LPCPI interface.
 ******************************************************************************/

/* System Includes */
#include "RTE_Device.h"
#include "Camera_Sensor.h"
#include "Camera_Sensor_i2c.h"
#include "Driver_GPIO.h"
#include "Driver_CPI.h"
#include "sys_ctrl_cpi.h"
#include "board.h"
#include "genx320.h"
#include "system_utils.h"
#include "sys_ctrl_cpi.h"
#include "psee_issd.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "clock_runtime.h"
#include "M55_HE.h"

/* Private Defines */
#define TIO_start		100
#define Tana_start		100
#define Tclk_start		200
#define trstn_start		100

#define tclk_stop		100
#define Tana_stop		100
#define TIO_stop		100
#define Tdig_stop		100

#if RTE_GENX320_CAMERA_SENSOR_LPCPI_ENABLE

#define CAMERA_SENSOR_I2C_LPCPI_INSTANCE  BOARD_CAM_I2C_INSTANCE 

/* GenX320 I2C Slave Address */
#define GENX320_CAMERA_SENSOR_SLAVE_ADDR   0x3C


#define REG_CPI_PKT_SELF_TEST 0x801C


/* Current ISSD */
const struct issd *current_issd = NULL;

/* Wrapper function for i2c read
 *  read register value from ARX3A0 Camera Sensor registers
 *   using i2c read API \ref camera_sensor_i2c_read
 *
 *  for ARX3A0 Camera Sensor specific i2c configurations
 *   see \ref ARX3A0_camera_sensor_i2c_cnfg
 */
#define GENX320_READ_REG(reg_addr, reg_value, reg_size) \
    camera_sensor_i2c_read(&lpcpi_genx320_camera_sensor_i2c_cnfg, \
            reg_addr,  \
            reg_value, \
            (CAMERA_SENSOR_I2C_REG_SIZE)reg_size)

/* Wrapper function for i2c write
 *  write register value to ARX3A0 Camera Sensor registers
 *   using i2c write API \ref camera_sensor_i2c_write.
 *
 *  for GENX320 Camera Sensor specific i2c configurations
 *   see \ref GENX320_camera_sensor_i2c_cnfg
 */
#define GENX320_WRITE_REG(reg_addr, reg_value, reg_size) \
    camera_sensor_i2c_write(&lpcpi_genx320_camera_sensor_i2c_cnfg, \
            reg_addr,  \
            reg_value, \
            (CAMERA_SENSOR_I2C_REG_SIZE)reg_size)


/* Wrapper function for i2c write
 *  write register value to GENX320 Camera Sensor registers
 *  using i2c write API \ref camera_sensor_i2c_write.
 *
 *  for ARX3A0 Camera Sensor specific i2c configurations
 *   see \ref GENX320_camera_sensor_i2c_cnfg
*/
#define GENX320_WRITE_SEQ_REG(reg_addr, data, size, count) \
    camera_sensor_i2c_write_burst(&lpcpi_genx320_camera_sensor_i2c_cnfg, \
            reg_addr, \
            data, \
            (CAMERA_SENSOR_I2C_REG_SIZE)size, \
            (uint32_t)count)
        
/* Forward declarations for camera operations */
static int32_t genx320_LPCPI_Init(void);
static int32_t genx320_LPCPI_Uninit(void);
static int32_t genx320_LPCPI_Start(void);
static int32_t genx320_LPCPI_Stop(void);
static int32_t genx320_LPCPI_Control(uint32_t control, uint32_t arg);

/* I2C Driver Instance */
extern ARM_DRIVER_I2C ARM_Driver_I2C_(CAMERA_SENSOR_I2C_LPCPI_INSTANCE);

/**
  \brief GENX320 Camera Sensor slave i2c Configuration
  \ref CAMERA_SENSOR_SLAVE_I2C_CONFIG
  */
CAMERA_SENSOR_SLAVE_I2C_CONFIG lpcpi_genx320_camera_sensor_i2c_cnfg =
{
    .drv_i2c                        = &ARM_Driver_I2C_(CAMERA_SENSOR_I2C_LPCPI_INSTANCE),
    .bus_speed                      = ARM_I2C_BUS_SPEED_FAST,
    .cam_sensor_slave_addr          = GENX320_CAMERA_SENSOR_SLAVE_ADDR,
    .cam_sensor_slave_reg_addr_type = CAMERA_SENSOR_I2C_REG_ADDR_TYPE_16BIT,
};

/**
  \brief LPCPI GENX320 Camera Sensor Configurations
  \ref CPI_INFO
*/
 static CPI_INFO lpcpi_genx320_config =
 {
     .pixelclk_pol    = RTE_GENX320_CAMERA_SENSOR_LPCPI_PIXEL_CLK_POL,
     .hsync_pol       = RTE_GENX320_CAMERA_SENSOR_LPCPI_HSYNC_POL,
     .vsync_pol       = RTE_GENX320_CAMERA_SENSOR_LPCPI_VSYNC_POL,
     .vsync_wait      = RTE_GENX320_CAMERA_SENSOR_LPCPI_VSYNC_WAIT,
     .vsync_mode      = RTE_GENX320_CAMERA_SENSOR_LPCPI_VSYNC_MODE,
     .data_mode       = RTE_GENX320_CAMERA_SENSOR_LPCPI_DATA_MODE,
     .data_endianness = RTE_GENX320_CAMERA_SENSOR_LPCPI_DATA_ENDIANNESS,
     .code10on8       = RTE_GENX320_CAMERA_SENSOR_LPCPI_CODE10ON8,
 };

/* Camera Sensor Operations struct */
static CAMERA_SENSOR_OPERATIONS lpcpi_genx320_ops = {
    .Init    = genx320_LPCPI_Init,
    .Uninit  = genx320_LPCPI_Uninit,
    .Start   = genx320_LPCPI_Start,
    .Stop    = genx320_LPCPI_Stop,
    .Control = genx320_LPCPI_Control,
};

/* CAMERA_SENSOR_DEVICE struct */
/**
  \brief LPCPI GENX320 Camera Sensor Device Structure
Contains:
- LPCPI GENX320 Camera Sensor Configurations
- GENX320 Camera Sensor Operations
\ref CAMERA_SENSOR_DEVICE
*/
CAMERA_SENSOR_DEVICE lpcpi_genx320_camera_sensor = {
    .interface = CAMERA_SENSOR_INTERFACE_PARALLEL,
    .width     = RTE_GENX320_CAMERA_SENSOR_LPCPI_FRAME_WIDTH,     
    .height    = RTE_GENX320_CAMERA_SENSOR_LPCPI_FRAME_HEIGHT,
    .cpi_info  = &lpcpi_genx320_config,
    .ops       = &lpcpi_genx320_ops
};

/**
  * @brief Platform dependent sleep function for delay in us. 
  * Make sure to initialize the system timer before see system_utils.h.
  * @param duration Time delay in microseconds 
  * Max delay is 100_000 us
  */
 static void psee_sleep_us_imp(uint32_t duration) {
    /* Covert duration to ns*/
    sys_busy_loop_us(duration);
    return;
}

/**
 * @brief Platform dependent sleep function for delay in ms. 
 * Make sure to initialize the system timer before see system_utils.h.
 * @param duration Time delay in milliseconds. No practical max duration.
 */
void psee_sleep_ms_imp(uint32_t duration) {
    const uint32_t max_chunk_ms = 100; // 100 ms

    while (duration > 0) {
        uint32_t current_delay_ms = (duration > max_chunk_ms) ? max_chunk_ms : duration;
        sys_busy_loop_us((uint64_t)current_delay_ms * 1000); // Convert to ns
        duration -= current_delay_ms;
    }
}

/* Helper Functions from sensor wrapper as specified from psee */
/**
 * @brief Platform dependent function to perform a single-write operation to the sensor's register.
 * @param reg Sensor's register to which the data needs to be written
 * @param data Data to be written
 */
void psee_sensor_write(uint16_t reg, uint32_t data) {
    int32_t ret = GENX320_WRITE_REG(reg, data, CAMERA_SENSOR_I2C_REG_SIZE_32BIT);
    if (ret != ARM_DRIVER_OK) {
        printf("I2C write failed: reg=0x%04X, data=0x%08X\n\r", reg, data);
    }
}


/**
 * @brief Platform dependent function to perform a single-read operation from the sensor's register.
 * @param reg Sensor's register from which the data needs to be read
 * @param data Pointer to the variable where the data needs to be stored
 *
 */
void psee_sensor_read(uint16_t reg, uint32_t* data) {
    int32_t ret = GENX320_READ_REG(reg, data, CAMERA_SENSOR_I2C_REG_SIZE_32BIT);
    
    if (ret != ARM_DRIVER_OK) {
        printf("[I2C ERROR] psee_sensor_read failed: REG=0x%04X\n\r", reg);
    }
}


/**
 * @brief Platform dependent function to perform a sequential-read operation from the sensor's register.
 *        Uses Alif CPI I2C abstraction layer to read each 32-bit word individually.
 *
 * @param reg     Sensor's starting register address (16-bit) from which the data needs to be read
 * @param data    Pointer to the array where the data will be stored (each element is 32-bit)
 * @param n_word  Number of 32-bit words to read
 */
void psee_sensor_sequential_read(uint16_t reg, uint32_t* data, uint16_t n_word) {
    int32_t ret = ARM_DRIVER_OK;

    for (uint16_t i = 0; i < n_word; i++) {
        uint32_t word = 0;

        // Read one 32-bit word from the sensor using Alif's I2C abstraction
        ret = camera_sensor_i2c_read(&lpcpi_genx320_camera_sensor_i2c_cnfg,
                                     reg + (i * 4), // Increment address by 4 assuming 32-bit aligned registers
                                     &word,
                                     CAMERA_SENSOR_I2C_REG_SIZE_32BIT);

        if (ret != ARM_DRIVER_OK) {
            printf("[I2C ERROR] Sequential read failed at reg=0x%04X (word %u)\n\r", reg + (i * 4), i);
            return;
        }

        data[i] = word;  // Already big-endian to little-endian converted by the driver
    }
}


/**
 * @brief Platform dependent function to perform a sequential-write operation to the sensor's register.
 *        Parses the data and uses Alif CPI I2C driver abstraction.
 * @param register_data Address of the data array that needs to be written.
 *                      Format: [reg_high, reg_low, data0, data1, ..., dataN]
 * @param n_bytes Total number of bytes in register_data (should be >= 3).
 */
void psee_sensor_sequential_write(uint8_t *register_data, uint16_t n_bytes) {
    if (n_bytes < 3) {
        printf("[I2C ERROR] Sequential write failed: too few bytes (got %u, need >= 3)\n\r", n_bytes);
        return;
    }

    // Extract register address (assuming big-endian register address)
    uint16_t reg_addr = ((uint16_t)register_data[0] << 8) | register_data[1];

    // Calculate how many full 32-bit words are in the remaining data
    uint16_t data_bytes = n_bytes - 2;
    if (data_bytes % 4 != 0) {
        printf("[I2C ERROR] Sequential write failed: data size (%u) not divisible by 4\n\r", data_bytes);
        return;
    }

    uint32_t burst_len = data_bytes / 4;

    // Prepare pointer to data values (skip the 2-byte register address)
    void *reg_values = &register_data[2];

    int32_t ret = camera_sensor_i2c_write_burst(&lpcpi_genx320_camera_sensor_i2c_cnfg,
                                                reg_addr,
                                                reg_values,
                                                CAMERA_SENSOR_I2C_REG_SIZE_32BIT,
                                                burst_len);

    if (ret != ARM_DRIVER_OK) {
        printf("[I2C ERROR] Sequential write failed at reg=0x%04X, burst_len=%lu\n\r", reg_addr, (unsigned long)burst_len);
    }
}

/**
 * @brief Platform dependent initialization routine.
 * @note This function can only run after PINMUX initialization.
*/

void psee_sensor_platform_init() {
    
    // Camera module Power supply enable pins
	extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V1_GPIO_PORT);
	extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V8_GPIO_PORT); 
	extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_2V5_GPIO_PORT);
    extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_ULP_ENB_GPIO_PORT);
    extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_RSTN_GPIO_PORT);
    extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PXRSTN_GPIO_PORT);

    ARM_DRIVER_GPIO *BOARD_CAM_PSU_EN_1V1_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V1_GPIO_PORT);
	ARM_DRIVER_GPIO *BOARD_CAM_PSU_EN_1V8_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V8_GPIO_PORT);
	ARM_DRIVER_GPIO *BOARD_CAM_PSU_EN_2V5_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_2V5_GPIO_PORT);
    ARM_DRIVER_GPIO *BOARD_CAM_ULP_ENB_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_ULP_ENB_GPIO_PORT);
    ARM_DRIVER_GPIO *BOARD_CAM_RESET_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_RSTN_GPIO_PORT);
    ARM_DRIVER_GPIO *BOARD_CAM_PXRSTN_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PXRSTN_GPIO_PORT);

    printf("Powering up the camera module...\n\r");
	/* Switch ON PSU_EN_1V1 */
    BOARD_CAM_PSU_EN_1V1_GPIO_GPIOdrv->SetValue(BOARD_CAM_PSU_EN_1V1_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
	psee_sleep_us_imp(TIO_start);

    /* Turn on pixel reset to allow data streaming */
    BOARD_CAM_PXRSTN_GPIOdrv->SetValue(BOARD_CAM_PXRSTN_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);

    /* Switch ON PSU_EN_1V8 */
    BOARD_CAM_PSU_EN_1V8_GPIO_GPIOdrv->SetValue(BOARD_CAM_PSU_EN_1V8_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
	psee_sleep_us_imp(Tana_start);

	/* Switch ON PSU_EN_2V5 */
    BOARD_CAM_PSU_EN_2V5_GPIO_GPIOdrv->SetValue(BOARD_CAM_PSU_EN_2V5_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);

	/* Toggle ON ULP_ENB, open drain */
    BOARD_CAM_ULP_ENB_GPIO_GPIOdrv->SetValue(BOARD_CAM_ULP_ENB_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
	psee_sleep_us_imp(Tclk_start);
}

/**
  \fn           int32_t genx320_Init(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
                                     CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)

  \brief        Initialize GENX320 Camera Sensor I2C driver configured to match the camera sensor.
  \param[in]    cpi_genx320_camera_sensor  : Pointer to camera sensor.
  \param[in]    i2c_cfg  : Pointer to Camera Sensor i2c configuration.
  \return       \ref execution_status
  */
 static int32_t genx320_Init(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
    CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)
{
    printf("Initializing GENX320 Camera Sensor...\n\r");
    // Initialize and configure the camera sensor I2C driver
    int32_t ret;
    ret = camera_sensor_i2c_init(&lpcpi_genx320_camera_sensor_i2c_cnfg);

    if (ret != ARM_DRIVER_OK) {
        printf("I2C initialization failed: %d\n\r", ret);
        return ret;
    }

    // Initialize the platform, Power up sequence of the the sensor
	psee_sensor_platform_init();
    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t genx320_Uninit(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
                                       CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)
  \brief        Un-initialize GENX320 Camera Sensor. not implemened yet 
  \param[in]    none
  \return       \ref execution_status
  */
 static int32_t genx320_Uninit(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
    CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)
{
    return ARM_DRIVER_OK;
}
  /**
  \fn           int32_t genx320_Start(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
                                      CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)
  \brief        Start GENX320 Camera Sensor Streaming.
  \param[in]    cpi_genx320_camera_sensor  : Poter to camera sensor.
  \param[in]    i2c_cfg  : Pointer to Camera Sensor i2c configuration.
  \return       \ref execution_status
  */
static int32_t genx320_Start(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
    CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)
{  
    // Put sensor into streaming mode
    psee_sensor_start(current_issd);

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t genx320_Stop(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
                                     CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)
  \brief        Stop GENX320 Camera Sensor Streaming.
  \param[in]    cpi_genx320_camera_sensor  : Poter to camera sensor.
  \param[in]    i2c_cfg  : Pointer to Camera Sensor i2c configuration.
  \return       \ref execution_status
  */
 static int32_t genx320_Stop(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
    CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg)
{
    psee_sensor_stop(current_issd);    
    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t genx320_Control(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
                                        CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg,
                                        uint32_t control,
                                        uint32_t arg)
  \brief        Control GENX320 Camera Sensor.
  \param[in]    cpi_genx320_camera_sensor  : Poter to camera sensor.
  \param[in]    i2c_cfg  : Pointer to Camera Sensor i2c configuration.
  \param[in]    control  : Operation
  \param[in]    arg      : Argument of operation, 
    *                      arg = 0 - bootup configuration
  \return       \ref execution_status
  */
 static int32_t genx320_Control(CAMERA_SENSOR_DEVICE *cpi_genx320_camera_sensor,
    CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c_cfg,
    uint32_t control,
    uint32_t arg)
{
    if (control == CPI_CAMERA_SENSOR_CONFIGURE) 
    {     
        int32_t ret = 0;
        // Configure at boot up
        if (arg == 0) { 
            
            // Enable XVCLK
            set_lpcpi_pixel_clk(RTE_GENX320_CAMERA_SENSOR_CLK_SCR_DIV);

            extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_RSTN_GPIO_PORT);
            ARM_DRIVER_GPIO *BOARD_CAM_RESET_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_RSTN_GPIO_PORT);
            
            // Toggle off RSTN 
            psee_sleep_us_imp(trstn_start);
            ret = BOARD_CAM_RESET_GPIO_GPIOdrv->SetValue(BOARD_CAM_RSTN_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
        }
        
        // Initialize the sensor 
	    current_issd = psee_open_evt();

	    // Set Standard biases
        printf("Setting Standard Biases\r\n");
	    psee_sensor_set_biases(&genx320_default_biases);
        
    return ARM_DRIVER_OK;  
    }
    return ARM_DRIVER_OK; 
}

static int32_t genx320_LPCPI_Init(void)
{
    return genx320_Init(&lpcpi_genx320_camera_sensor, &lpcpi_genx320_camera_sensor_i2c_cnfg);
}

static int32_t genx320_LPCPI_Uninit(void)
{
    return genx320_Uninit(&lpcpi_genx320_camera_sensor, &lpcpi_genx320_camera_sensor_i2c_cnfg);
}

static int32_t genx320_LPCPI_Start(void)
{
    return genx320_Start(&lpcpi_genx320_camera_sensor, &lpcpi_genx320_camera_sensor_i2c_cnfg);
}

static int32_t genx320_LPCPI_Stop(void)
{
    return genx320_Stop(&lpcpi_genx320_camera_sensor, &lpcpi_genx320_camera_sensor_i2c_cnfg);
}

static int32_t genx320_LPCPI_Control(uint32_t control, uint32_t arg)
{
    return genx320_Control(&lpcpi_genx320_camera_sensor, &lpcpi_genx320_camera_sensor_i2c_cnfg, control, arg);
}

/* Registering CPI sensor */
LPCAMERA_SENSOR(lpcpi_genx320_camera_sensor)

#endif /* RTE_GENX320_CAMERA_SENSOR_LPCPI_ENABLE */
/**
 ******************************************************************************
 * @file    psee_sensor_wrapper_alif.c
 * @author  PSEE Applications Team
 * @brief	Platform specific wrapper functions
 *
 ******************************************************************************
 * @attention
 * Copyright (c) Prophesee S.A.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 ******************************************************************************
 */

 #include "system_utils.h"
 #include "Driver_I2C.h"
 #include "Driver_GPIO.h"
 #include "board.h"
 /* User Includes */
 #include "genx320.h"
 #include "sys_ctrl_cpi.h"
 
 /* Private Defines */
 #define TIO_start		100
 #define Tana_start		100
 #define Tclk_start		200
 #define trstn_start		100
 
 #define tclk_stop		100
 #define Tana_stop		100
 #define TIO_stop		100
 #define Tdig_stop		100
 
 
 #define DCMI_PWR_EN_Pin GPIO_PIN_13
 #define DCMI_PWR_EN_GPIO_Port GPIOH
 
 extern ARM_DRIVER_I2C ARM_Driver_I2C_(BOARD_CAM_I2C_INSTANCE);
 static ARM_DRIVER_I2C *psee_i2c_drv = &ARM_Driver_I2C_(BOARD_CAM_I2C_INSTANCE);
 
 static volatile uint32_t psee_cb_status = 0;
 
 /* CB event flag defs */
 typedef enum _I2C_CB_EVENT{
     I2C_CB_EVENT_TRANSFER_DONE        = (1 << 0),
     I2C_CB_EVENT_ADDRESS_NACK         = (1 << 1),
     I2C_CB_EVENT_TRANSFER_INCOMPLETE  = (1 << 2)
 }I2C_CB_EVENT;
 
 static void psee_i2c_transfer_cb(uint32_t event)
 {
     if (event & ARM_I2C_EVENT_TRANSFER_DONE)
     {
         /* Transfer or receive is finished */
         psee_cb_status |= I2C_CB_EVENT_TRANSFER_DONE;
     }
 
     if (event & ARM_I2C_EVENT_ADDRESS_NACK)
     {
         /* Address NACKED */
         psee_cb_status |= I2C_CB_EVENT_ADDRESS_NACK;
     }
 
     if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE)
     {
         /* Transfer or receive is incomplete */
         psee_cb_status |= I2C_CB_EVENT_TRANSFER_INCOMPLETE;
     }
 }
 
 /**
  * @brief Platform dependent sleep function for delay in us. 
  * Make sure to initialize the system timer before see system_utils.h.
  * @param duration Time delay in microseconds 
  * Max delay is 100_000 us
  */
 static void psee_sleep_us_imp(uint32_t duration) {
     /* Covert duration to ns*/
     sys_busy_loop_ns(duration*1000);
 
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
         sys_busy_loop_ns((uint64_t)current_delay_ms * 1000000); // Convert to ns
         duration -= current_delay_ms;
     }
 }
 
 
 /**
  * @brief Platform dependent function to perform a single-write operation to the sensor's register.
  * @param reg Sensor's register to which the data needs to be written
  * @param data Data to be written
  */
 int32_t psee_sensor_write(uint16_t reg, uint32_t data) {
     uint16_t buf[3];
 
     /* Write reg addr */
     buf[0] = __builtin_bswap16 (reg);
 
     /* Write value */
     buf[1] = __builtin_bswap16 ((data >> 16));
     buf[2] = __builtin_bswap16 ((data & 0xFFFF));
 
     return psee_i2c_drv->MasterTransmit(I2C_ADDRESS, (uint8_t *) buf, sizeof(buf), false);
 }
 
 /**
  * @brief Platform dependent function to perform a single-read operation from the sensor's register.
  * @param reg Sensor's register from which the data needs to be read
  * @param data Pointer to the variable where the data needs to be stored
  * @return Execution status from I2C-drivrutinen.
  */
 
 int32_t psee_sensor_read(uint16_t reg, uint32_t* data) {
     uint16_t recv[2] = {0};
     int32_t ret;
 
     // Prepare the register address for the sensor and swap byte order
     uint16_t swap_reg = __builtin_bswap16(reg);
 
     // Send register address and xfer pending true to psee sensor 
     ret = psee_i2c_drv->MasterTransmit(I2C_ADDRESS, (uint8_t *)&swap_reg, sizeof(swap_reg), true);
     if (ret != ARM_DRIVER_OK) {
         return ret;
     }
 
     // Receive the data from the sensor set xfer pending flase
     ret = psee_i2c_drv->MasterReceive(I2C_ADDRESS, (uint8_t *)recv, sizeof(recv), false);
     if (ret != ARM_DRIVER_OK) {
         return ret;
     }
 
     // Combine the received data into a 32-bit word and convert to native endianness
     *data = (__builtin_bswap16(recv[0]) << 16) | __builtin_bswap16(recv[1]);
 
     return ret;
 }
 
 /**
  * @brief Performs a sequential read operation from the sensor's registers.
  * @param reg   The starting register address from which data will be read.
  * @param data  Pointer to the array where the read 32-bit words will be stored.
  * @param n_word The number of 32-bit words to be read.
  * @return ARM_DRIVER_OK on success, or an error code if an I2C transfer fails.
  */
 int32_t psee_sensor_sequential_read(uint16_t reg, uint32_t* data, uint16_t n_word) {
     int32_t ret;
 
     // Convert the register address to big-endian format as expected by the sensor.
     uint16_t swap_reg = __builtin_bswap16(reg);
 
     // Send the register address to the sensor while keeping the bus active for a combined transaction.
     ret = psee_i2c_drv->MasterTransmit(I2C_ADDRESS, (uint8_t *)&swap_reg, sizeof(swap_reg), true);
     if (ret != ARM_DRIVER_OK) {
         return ret;
     }
 
     // Receive the sequential data from the sensor; each word is 4 bytes.
     ret = psee_i2c_drv->MasterReceive(I2C_ADDRESS, (uint8_t *)data, n_word * sizeof(uint32_t), false);
     if (ret != ARM_DRIVER_OK) {
         return ret;
     }
 
     // Convert each 32-bit word from the sensor's big-endian format to the native endianness.
     for (uint32_t d_index = 0; d_index < n_word; d_index++) {
         uint16_t *p_data = (uint16_t *)&data[d_index];
         data[d_index] = (__builtin_bswap16(p_data[0]) << 16) | __builtin_bswap16(p_data[1]);
     }
 
     return ret;
 }
 
 /**
  * @brief Platform dependent function to perform a sequential-write operation to the sensor's register.
  * @param register_data Address of the data array that needs to be written
  * @param n_bytes Total number of bytes that needs to be written
  */
 int32_t psee_sensor_sequential_write(uint8_t *register_data, uint16_t n_bytes) {
 
     return psee_i2c_drv->MasterTransmit(I2C_ADDRESS, register_data, n_bytes, false);
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
     extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_RESET_GPIO_PORT);
 
 
     ARM_DRIVER_GPIO *BOARD_CAM_PSU_EN_1V1_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V1_GPIO_PORT);
     ARM_DRIVER_GPIO *BOARD_CAM_PSU_EN_1V8_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V8_GPIO_PORT);
     ARM_DRIVER_GPIO *BOARD_CAM_PSU_EN_2V5_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_2V5_GPIO_PORT);
     ARM_DRIVER_GPIO *BOARD_CAM_ULP_ENB_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_ULP_ENB_GPIO_PORT);
     ARM_DRIVER_GPIO *BOARD_CAM_RESET_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_RESET_GPIO_PORT);
     
     // Initialize I2C driver for the prophesee camera
     psee_i2c_drv->Initialize(psee_i2c_transfer_cb);  // No event callback needed
     psee_i2c_drv->PowerControl(ARM_POWER_FULL);
     psee_i2c_drv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);  // Set I2C speed
 
     // Not sure what this does in the original code ?? 
     /*-------------------- Without Modified Hardware-----------------------------*/
     // HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);
     // HAL_Delay(1);
 
     /* Turn on power supplies of Saphir (PSU_EN) */
     // HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_SET);
     // HAL_Delay(1);
 
     /*-------------------- With Modified Hardware-----------------------------*/
     
     /*
     * @note ULP_ENB would be already logic high.
     */
 
     /* Switch ON PSU_EN_1V1 */
     BOARD_CAM_PSU_EN_1V1_GPIO_GPIOdrv->SetValue(BOARD_CAM_PSU_EN_1V1_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
     psee_sleep_us_imp(TIO_start);
 
     /* Switch ON PSU_EN_1V8 */
     BOARD_CAM_PSU_EN_1V8_GPIO_GPIOdrv->SetValue(BOARD_CAM_PSU_EN_1V8_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
     psee_sleep_us_imp(Tana_start);
 
     /* Switch ON PSU_EN_2V5 */
     BOARD_CAM_PSU_EN_2V5_GPIO_GPIOdrv->SetValue(BOARD_CAM_PSU_EN_2V5_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
 
     /* Toggle ON ULP_ENB, open drain */
     BOARD_CAM_ULP_ENB_GPIO_GPIOdrv->SetValue(BOARD_CAM_ULP_ENB_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
     psee_sleep_us_imp(Tclk_start);
 
     /* Enable external clock to the sensor*/
     set_cpi_pixel_clk(CPI_PIX_CLKSEL_400MZ, RTE_GENX320_CAMERA_SENSOR_CSI_CLK_SCR_DIV);
 
     /* Start the 10MHz PWM signal */
     psee_sleep_us_imp(trstn_start);
 
     /* Toggle ON RSTN */
     BOARD_CAM_RESET_GPIO_GPIOdrv->SetValue(BOARD_CAM_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
 }
 
 
 
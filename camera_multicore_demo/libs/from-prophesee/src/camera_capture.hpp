/*
 * SPDX-FileCopyrightText: Copyright 2023 Arm Limited and/or its
 * affiliates <open-source-office@arm.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CAMERA_CAPTURE_HPP
#define CAMERA_CAPTURE_HPP

#include <cstdint>
#include "Driver_GPIO.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */
extern uint8_t number_of_frames;
extern uint32_t frame_counter;
extern uint32_t total_time; 

extern ARM_DRIVER_GPIO *BOARD_PINHEAD_GPIOdrv;

}
namespace arm {
namespace app {

enum class ColourFilter {
    BGGR,
    GBRG,
    GRBG,
    RGGB,
    Invalid
};

typedef struct {
    bool camera_on;
    bool configure_next_frame : 1;
    bool frame_complete       : 1;
    bool camera_error         : 1;
    bool dropped_frame        : 1;
} arm_camera_status_t;

extern arm_camera_status_t camera_status;


/**
 * @brief Initialise the camera capture interface.
 *
 * @return int: 0 if successful, error code otherwise
 */
int CameraCaptureInit();
/**
 * @brief Starts the camera capture (does not wait for it to finish)
 *
 * @param raw_image     Raw image pointer - should be the same as what is passed to the init
 * function
 * @return int: 0 if successful, error code otherwise.
 */
int CameraCaptureStart(uint8_t* raw_image);

/**
 * @brief Starts the camera capture in streaming mode (does not wait for it to finish)
 *
 * @param raw_image     Raw image pointer - should be the same as what is passed to the init
 * function
 * @return int: 0 if successful, error code otherwise.
*/

int CameraCaptureStartStream(uint8_t* raw_image);
/**
 * @brief Stops the camera capture streaming mode (does not wait for it to finish)
 *
 * @param raw_image     Raw image pointer - should be the same as what is passed to the init
 * function
 * @return int: 0 if successful, error code otherwise.
*/

int CameraCaptureStopStreaming();


/**
 * @brief   Waits for the camera capture to complete (or times out).
 */
 void CameraCaptureWaitForFrame();

/**
 * @brief   Called when a new frame arrives to configure the frame index for the next frame and set the buffer flags properly.
*/
void LockNextFrame();

/**
 * @brief Get a cropped, colour corrected RGB frame from a RAW frame.
 *
 * @param[in] rawImgData        Pointer to the source (RAW) image.
 * @param[in] rawImgWidth       Width of the source image.
 * @param[in] rawImgHeight      Height of the source image.
 * @param[in] rawImgCropOffsetX Offset for X-axis from the source image (crop starts here).
 * @param[in] rawImgCropOffsetY Offset for Y-axis from the source image (crop starts here).
 * @param[out] rgbImgData       Pointer to the destination image (RGB) buffer.
 * @param[in] rgbImgWidth       Width of destination image.
 * @param[in] rgbImgHeight      Height of destination image.
 * @param[in] bayerFormat       Bayer format description code.
 * @return bool                 True if successful, false otherwise.
 */
bool CropAndDebayer(
    const uint8_t* rawImgData,
    uint32_t rawImgWidth,
    uint32_t rawImgHeight,
    uint32_t rawImgCropOffsetX,
    uint32_t rawImgCropOffsetY,
    uint8_t* rgbImgData,
    uint32_t rgbImgWidth,
    uint32_t rgbImgHeight,
    ColourFilter bayerFormat);

} /* namespace app */
} /* namespace arm */

#endif /* CAMERA_CAPTURE_HPP */

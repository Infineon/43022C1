/*
 * Copyright 2016-2025, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
/** @file
 *
 * AIROC BTSDK App HAL Common Utilities
 */
#ifndef _WICED_BT_APP_HAL_COMMON_H_
#define _WICED_BT_APP_HAL_COMMON_H_

#include "bt_types.h"
#include "wiced.h"

//#define WICED_BT_TRACE_ENABLE

/******************************************************************************
 *                                                          Constants
 ******************************************************************************/

/* Number of GPIOs that can be configured using the AIROC BTSDK app hal interface */
#define GPIO_NUM_MAX        16

/* GPIO flag bit (this is different than definition that used in gpio driver source code (GPIO_INPUT_ENABLE, etc) */
#define GPIO_INPUT          0x0000
#define GPIO_OUTPUT         0x0001

/******************************************************************************
 *                                                        Type Definitions
 ******************************************************************************/

/******************************************************************************
 *                                                          Function Prototypes
 ******************************************************************************/

/**  AIRCO BTSDK App Common HAL Initialization
 *
 * @return   None
 */
void wiced_bt_app_hal_init ( void );

/**  Blinks the LED
 *
 *@param[in]    on_ms             : LED on duration in milli seconds
 *
  *@param[in]    off_ms             : LED off duration in milliseconds
 *
  *@param[in]    num_of_blinks             : LED number of blinks
 *
 * @return   None
 */
void wiced_bt_app_hal_led_blink(uint16_t on_ms, uint16_t off_ms, uint8_t num_of_blinks );

/**  Turns the LED On
 *
 * @return   None
 */
void wiced_bt_app_hal_led_on( void );

/**  Turns the LED Off
 *
 * @return   None
 */
void wiced_bt_app_hal_led_off( void );

#ifdef WICED_HAL_POWER_DOWN_FLASH_SUPPORT
/**  Turns OFF Flash
 *
 * @return   WICED_TRUE if Success, else WICED_FALSE
 */
wiced_bool_t wiced_bt_app_hal_power_down_flash( void );
#endif // WICED_HAL_POWER_DOWN_FLASH_SUPPORT

#endif //_WICED_BT_APP_HAL_COMMON_H_

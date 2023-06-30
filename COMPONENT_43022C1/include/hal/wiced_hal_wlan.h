/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef __WICED_HAL_WLAN_H__
#define __WICED_HAL_WLAN_H__

#include "wiced_data_types.h"

/********************* GLOBAL DECLARATIONS **********************************/

/**
 *  \brief Assert WLAN power up signal. This API does not have any impact if WL_REG_ON pin is set high.
 *
 *  \param [in] power_on_flag TRUE - Power On or FALSE - Power off WLAN
 *  \return none
 */
void wiced_hal_wlan_override_wlregOn( wiced_bool_t power_on_flag );

/**
 *  \brief Remove SDIO line isolation after WLAN power-up using wiced_hal_wlan_override_wlregOn.
 *         This API does not have any impact if WL_REG_ON pin is set high.
 *
 *  \param none
 *  \return none
 */
void wiced_hal_wlan_remove_sdio_isolation( void );

#endif

/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
* List of parameters and defined functions needed to utilize the
* watchdog.
*
*/

#ifndef __WICED_WDOG_RESET_H__
#define __WICED_WDOG_RESET_H__

#include "wiced.h"
#include "wiced_hal_gpio.h"

/**  \addtogroup WatchdogInterface Watchdog Interface
* \ingroup HardwareDrivers
* Defines a driver for the watchdog interface. This driver manages the
* hardware watchdog countdown timer. When enabled, the watchdog timer will generate
* an interrupt when the timer counts down to zero, then will reload the counter.
* If the counter counts down to zero again, the hardware performs a device reset.
* The lowest priority "idle" thread restarts the watchdog counter preiodically,
* and other operations that may keep the system busy call wiced_hal_wdog_restart to
* extend the watchdog countdown.
*/
/*! @{ */

/******************************************************************************
*** Function prototypes.
******************************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// Execute a soft reset of the system.
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void wiced_hal_wdog_reset_system(void);

#ifdef ARLO_SUPPORT
#ifndef TZ_ACTIVE
///////////////////////////////////////////////////////////////////////////////
/// \brief Function to check whether reset happened due to Watchdog timer expiry
///
/// \return TRUE - if reset is due to Watchdog, FALSE otherwise
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_watchdog_get_reset_reason(void);

///////////////////////////////////////////////////////////////////////////////
/// \brief Function to configure gpio and its active level to be set on WDT expiry
///
/// \param [in] gpio BT GPIO to be asserted before watchdog reset(only WICED_GPIO_00 - WICED_GPIO_15 are valid).
/// \param [in] active_level WICED_GPIO_ACTIVE_LOW/WICED_GPIO_ACTIVE_HIGH
/// \return WICED_SUCCESS - if GPIO is configured sucessfully, else WICED_BADARG.
///////////////////////////////////////////////////////////////////////////////
wiced_result_t wiced_watchdog_configure_reset_gpio(wiced_bt_gpio_numbers_t gpio, wiced_bool_t active_level);

///////////////////////////////////////////////////////////////////////////////
/// \brief Function to add application required delay on WDT expiry after asserting HOST_WAKE/configured GPIO
/// and before triggering core-dump transfer
///
/// \param [in] delay_ms delay in ms (default delay is 0ms).
/// \return none
///////////////////////////////////////////////////////////////////////////////
void wiced_watchdog_configure_reset_delay(uint32_t delay_ms);
#endif  //#ifndef TZ_ACTIVE
#endif  //#ifdef ARLO_SUPPORT
#endif // __WICED_WDOG_RESET_H__

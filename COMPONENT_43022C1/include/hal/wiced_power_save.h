/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
*
* Wiced Low Power Mode Utilities
*
*/
#ifndef _WICED_POWER_SAVE_H_
#define _WICED_POWER_SAVE_H_

/*****************************************************************************
 *              Type Definitions
 *****************************************************************************/
/** Wake Up Sources .Used when calling wiced_start_power_save.*/
typedef enum
{
    WICED_WAKE_SOURCE_GPIO = 0x01, /**<  Wake from GPIO */
    WICED_WAKE_SOURCE_TIMEOUT = 0x02, /**<  Wake on timeout, Value of 0 means infinite timeout */
    WICED_WAKE_SOURCE_ALL = WICED_WAKE_SOURCE_GPIO|WICED_WAKE_SOURCE_TIMEOUT
}wiced_wake_source_t;

/**
 * Power Save callback
 *
 * Callback to the application on entering to power save mode or if entering to the power save
 * mode is aborted
 * Registered using wiced_power_save_register_enter_cback() and
 * wiced_power_save_register_abort_cback()
 *
 * @return void
 */
typedef void wiced_power_save_cback_t( void );

/**
 * Power Save approve callback
 *
 * Callback to the application to check if the application approves the device to go to power save
 *mode
 * Registered using wiced_power_save_register_approve_cback().
 *
 * @return  time to sleep - if the application decides to go to power save mode then it returns amount of time to sleep,
 *                          it may return zero to not to go to power save mode.
 */
typedef uint32_t wiced_power_save_approve_cback_t ( void );

/*****************************************************************************
 *          Function Prototypes
 *****************************************************************************/

/**
 * Function         wiced_power_save_start
 *
 * To start the power save mode of operation. This is the most efficient power save option available.
 * Be cautious on using this power save mode. Application will be restarted on exiting out of the
 * power save mode. If the application desires to save any state before going to power save it
 * can do so by using wiced_power_save_store_state() and retrieve the state on coming back
 * from power save by using wiced_power_save_retrieve_state()
 *
 * @param[in]      wake_source        : Bit mask of sources to wake the device from power save
 *                                                     mode
 *
 * @param[in]      timeout_ms        : Timeout in milliseconds  to wake the device from power
 *                                                   save mode. Used  when WICED_WAKE_SOURCE_TIMEOUT
 *                                                   is specified as wake source
 *
 *
 * @return          void
 *
 */
void wiced_power_save_start( wiced_wake_source_t wake_source,
                                   uint32_t timeout_ms );

/**
 * Function         wiced_power_save_register_approve_cback
 *
 * Registers the callback function which will be invoked to check if the application approves
 * the device to go to power save mode
 *
 * @param[in]      p_cback        : Callback function which will be invoked to check if the
 *                                              application approves  the device to go to power save mode
 *
 * @return          void
 *
 */
void wiced_power_save_register_approve_cback(
                                   wiced_power_save_approve_cback_t* p_cback );

/**
 * Function         wiced_power_save_register_enter_cback
 *
 * Registers the callback function which will be invoked just before device entering the
 * power save mode
 *
 * @param[in]      p_cback        : callback function which will be invoked just before device
 *                                              entering the power save mode
 *
 * @return          void
 *
 */

void wiced_power_save_register_enter_cback(
                                  wiced_power_save_cback_t* p_cback );

/**
 * Function         wiced_power_save_register_abort_cback
 *
 * Registers the callback function which will be invoked if entering to the power save mode
 *is aborted
 *
 * @param[in]      p_cback        : callback function which will be invoked if entering to the
 *                                               power save  mode is aborted
 *
 * @return          void
 *
 */
void wiced_power_save_register_abort_cback(
                                  wiced_power_save_cback_t* p_cback );

/**
 * Function         wiced_power_save_store_state
 *
 * Application can invoke this function to store any state values before going to power save
 * mode
 *
 * @param[in]      value        : Value to be saved.
 *
 * @return          void
 *
 */
void wiced_power_save_store_state( uint16_t value );

/**
 * Function         wiced_power_save_retrieve_state
 *
 * Application can invoke this function to retrieve any saved state values on waking from power
 * save mode
 *
 * @return         the saved value
 *
 */
uint16_t wiced_power_save_retrieve_state( void );

/**
 * Function         wiced_sleep_config
 *
 * Application can invoke this function to enable/disbale low power mode of operation. Application
 * resumes  on waking from sleep. Application can also configure the active modes of the BT WAKE GPIO
 * and the HOST WAKE GPIO using this function.
 * BT WAKE GPIO -Host can wake the device from sleep by setting the BT wake pin, active if the device is in sleep mode
 * HOST WAKE GPIO - Device can wake the host from sleep by setting the HOST wake pin, active
 *
 * @param[in]      enable        : Enable or disable the sleep mode
 *
 * @param[in]      bt_wake_gpio_active_mode        : Active Mode of the BT Wake pin
 *
 * @param[in]      host_wake_gpio_active_mode        : Active Mode of the Host Wake pin
 *
 * @return           void
 *
 */
void wiced_sleep_config ( wiced_bool_t enable, wiced_bool_t bt_wake_gpio_active_mode, wiced_bool_t host_wake_gpio_active_mode );

#endif // _WICED_POWER_SAVE_H_

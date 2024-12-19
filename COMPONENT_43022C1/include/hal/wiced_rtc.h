/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 * @defgroup Transport     WICED real timer clock interface
 * @ingroup HardwareDrivers
 *
 */
#pragma once

#include "wiced_bt_dev.h"
#include "wiced_hal_pspi.h"
#include "wiced_transport_heap.h"


/*****************************************************************************
**                                                 Function Declarations
*****************************************************************************/

/**
 * Function         wiced_rtc_init
 *
 * Initialize the RTC block.
 *
 * * @param[in]  void
 *
 * @return     : None
 */
void wiced_rtc_init(void);


/**
 * Function         wiced_rtc_get_raw_clock
 *
 * Reads Real time clock value from hardware clock.
 *
 * * @param[in]  rtcClock : Real time clock
 *
 * @return     : None
 */
void wiced_rtc_get_raw_clock(tRTC_REAL_TIME_CLOCK *rtcClock);


/**
 * Function         wiced_rtc_get_time
 *
 *  get current time in RTC_time format
 *
 * * @param[in]  timebuf: current time in rtc format
 *
 * @return     : None
 */
void wiced_rtc_get_time(RtcTime *timebuf);


/**
 * Function         wiced_rtc_set_reference_time
 *
 *  set original reference time.
 *
 * * @param[in]  ref_time: rtc time
 *
 * @return     : None
 */
void wiced_rtc_set_reference_time(RtcTime* ref_time);


/**
 * Function         wiced_rtc_set_reference_time
 *
 *  set the current time
 *
 * * @param[in]  newTime : rtc time for setting.
 *
 * @return     : SUCCESS/FALSE
 */
BOOL32 wiced_set_rtc_time(RtcTime  *newTime);


/**
 * Function         wiced_rtc_ctime
 *

 * convert the tm object pointed by timer to
 * c string containing a human-readable verion of the correspoing local time and data
 *
 * the returned string has the folloing format
 *
 * Mmm dd hh:mm:ss yyyy
 * where
 *      Mmm - the month in letters
 *      dd  - the day of the month
 *      hh:mm:ss -  time
 *      yyyy - year
 * * @param[in]  timer:  rtc time
 * * @param[in]  outbuf: string output
 *
 * @return     : SUCCESS/FALSE
 */
BOOL32 wiced_rtc_ctime(RtcTime *timer, char *outbuf);

/**
 * Function         wiced_seconds_to_rtc_time
 *
 *  conver the 32 bit seconds to RTC_time broken down format
 *
 * * @param[in]  second:  seconds
 * * @param[in]  rtctime: rtc format output
 *
 * @return     : NONE
 */
void wiced_seconds_to_rtc_time(UINT32 second, RtcTime *rtctime);

/**
 * Function         wiced_rtc_time_to_seconds
 *
 *  convert RTC_time broken-down format to 32-bits seocnds
 *
 * * @param[in]  rtctime :  rtc time
 * * @param[in]  second  :  seconds
 *
 * @return     : NONE
 */
void wiced_rtc_time_to_seconds(RtcTime *rtctime, UINT32 *second);

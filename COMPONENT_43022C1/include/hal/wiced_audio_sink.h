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
 * Defines the utilities used by the A2DP sink for configuring the audio data path for streaming
 */
#pragma once

/* wiced_audio_route_t */
#include "wiced_bt_audio.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup  wicedbt_a2dp        Advanced Audio Distribution Profile (A2DP) Sink
 * @ingroup     wicedbt_av
 * @{
 */


/******************************************************
 *                 Type Definitions
 ******************************************************/

/** overrun flow control flag */
typedef enum
{
    WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,
    WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLOW_CONTROL_SOURCE
} wiced_bt_a2dp_sink_overrun_control_t;

/** Audio data route configuration
 *
 * Note :
 *      --------------------------------------------------------------------
 *     |  Codec  |        Route                        |   Output data type |
 *     |--------------------------------------------------------------------|
 *     |         |  AUDIO_ROUTE_UART                   |      Decoded       |
 *     |   SBC   |  AUDIO_ROUTE_I2S                    |      Decoded       |
 *     |         |  AUDIO_ROUTE_COMPRESSED_TRANSPORT   |      Encoded       |
 *     |--------------------------------------------------------------------|
 *     |         |  AUDIO_ROUTE_UART                   |      Not Supported |
 *     |   AAC   |  AUDIO_ROUTE_I2S                    |      Not Supported |
 *     |         |  AUDIO_ROUTE_COMPRESSED_TRANSPORT   |      Encoded       |
 *      --------------------------------------------------------------------
 *
 */
typedef struct
{
    wiced_audio_route_t             route;      /**< Audio routing path */
    wiced_bt_a2dp_sink_data_cb_t    data_cb;    /**< App callback if audio route is APP*/
    wiced_bool_t                    is_master;  /**< I2S slave/master if audio route is I2S */
} wiced_bt_a2dp_sink_route_config;

/** Audio data route configuration */
typedef struct
{
    wiced_audio_route_t             route;      /**< Audio routing path */
    wiced_bool_t                    is_master;  /**< I2S slave/master if audio route is I2S */
}wiced_bt_a2dp_source_route_config;

/** Audio quality tuning parameters
 *Note
 *
 * The average buffer level, over a period of time, is compared to the target buffer level
 * (calculated based on the Target BUF Depth %) and used to compute the necessary clock
 * adjustment in ppm (Level Correction). If the Level Correction stays within the ppm range
 * defined by Level Correction Threshold High and Level Correction Threshold Low, then the
 * algorithm only applies small clock adjustments that are proportional to the amplitude of the
 * Level Correction (typically, 1.5%~2.0% of the Level Correction value). Otherwise, if the
 * Level Correction falls outside of the Level Correction Threshold High and Level Correction
 * Threshold Low interval for 3 successive adjustment periods, the algorithm applies large
 * clock adjustments defined Adjustment ppm Max or Adjustment ppm Min in an effort to get
 * back within range*/
typedef struct
{
    uint16_t                              buf_depth_ms;                  /**< in msec, Max value allowed is 300 */
    uint8_t                               start_buf_depth;               /**< start playback percentage of the buffer depth */
    uint8_t                               target_buf_depth;              /**< target level percentage of the buffer depth */
    wiced_bt_a2dp_sink_overrun_control_t  overrun_control;               /**< overrun flow control flag */
    int32_t                               adj_ppm_max;                   /**< Max PPM adjustment value */
    int32_t                               adj_ppm_min;                   /**< Min PPM adjustment value */
    int32_t                               adj_ppb_per_msec;              /**< PPM adjustment per milli second */
    int32_t                               lvl_correction_threshold_high; /**< Level correction threshold high value */
    int32_t                               lvl_correction_threshold_low;  /**< Level correction threshold low value */
    int32_t                               adj_proportional_gain;         /**< Proportional component of total PPM adjustment */
    int32_t                               adj_integral_gain;             /**< Integral component of total PPM adjustment */
} wiced_bt_a2dp_sink_audio_tuning_params_t;




/** Play Status Indication Callback. This will be invoked when there is a change in the play status
 *     Play status values
 *           JITTER_IDLE_STATE  0x1
 *           JITTER_NORMAL_STATE 0x2
 *           JITTER_UNDERRUN_STATE  0x4
 *           JITTER_OVERRUN_STATE 0x8
 */
typedef void (*wiced_audio_play_status_ind_t)( uint8_t play_status);


/*************************************************************
 *                 Function Declarations
 *************************************************************/

/** Wiced audio sink initialization
 *
 *  @param p_param    pointer to the tuning parameters
 *
 *  @return wiced_result_t (WICED_BT_XXX)
 */
wiced_result_t wiced_audio_sink_config_init( wiced_bt_a2dp_sink_audio_tuning_params_t* p_param );

/** API to configure the sink to perform SBC decoding and to route the decoded
 * audio data over I2S or UART on starting the streaming
 *
 *Called by the application when the streaming is to be started
 *
 *  @param handle  AVDT connection handle
 *
 *  @param audio_route - Audio route to be used to send the decoded audio data,
 *                                 I2S or UART
 *
 *  @param cp_type  Content protection type. Refer AVDT_CP_TYPE defined in wiced_bt_avdt.h
 *
 *  @param p_codec_info    Codec information
 *
 *  @return wiced_result_t (WICED_BT_XXX)
 */
wiced_result_t wiced_audio_sink_configure( uint16_t handle, uint32_t is_master, uint32_t audio_route, uint16_t cp_type,wiced_bt_a2dp_codec_info_t* p_codec_info );

/** API to reset the sink on stopping/suspending the streaming
 *
 *Called by the application when the streaming is stopped/suspended
 *
 *  @param handle  AVDT connection handle
 *
 *  @return wiced_result_t (WICED_BT_XXX)
 */
wiced_result_t wiced_audio_sink_reset( uint16_t handle );

/** To mute/unmute the audio
 *
 *Called by the application to mute the audio when playing the music. Application shall
 * this function to unmute also to restart playing the music
 *
 *  @param enable    1 to mute, 0 to unmute
 *
 *  @param ramp_ms    ramp up/down time in milli seconds
 *
 *  @return wiced_result_t (WICED_BT_XXX)
 */
wiced_result_t wiced_audio_sink_mute( wiced_bool_t enable, uint16_t ramp_ms );

/**
 *  Function         wiced_audio_set_i2s_out_sf
 *
 *  API to configure output I2S sampling frequency
 *
 *  Called by the application before starting the streaming
 *
 *  @param out_sf  Desired output i2s sampling frequency (ex. 16000/32000/44100/48000)
 *
 *  @return wiced_bool_t  TRUE in success case otherwise FALSE
 *
 *  NOTE : This API must be called before calling the API wiced_audio_sink_configure
 */
wiced_bool_t wiced_audio_set_i2s_out_sf(uint32_t out_sf);

/**
 *  Function         wiced_audio_register_play_status_ind_cback
 *
 *  Debug API to  register for play status indication
 *
 *  Called by the application before starting the streaming
 *
 *  @param[in] cback : Callback function to be invoked on play status change
 *
 *  @return None
 *
 */
void wiced_audio_register_play_status_ind_cback( wiced_audio_play_status_ind_t cback );

/**
 *  Function         wiced_audio_get_jitter_buffer_state
 *
 *  Debug API to get the jitter buffer state
 *
 *  @param[out] p_current_buf_ms : current jitter buffer size in msec
 *
 *  @param[out] p_clk_tuning : current clock tuning value
 *
 *  @return None
 *
 */
void wiced_audio_get_jitter_buffer_state( uint16_t* p_current_buf_ms, uint16_t* p_clk_tuning );

/**
 *  Function         wiced_audio_sink_set_arip_I2SPinOut
 *
 *  API to set dedicated I2S PinOut, input value depends on chip design
 *
 *  @param[in] ARIP_I2S_PIN_OUT_AUD1/ARIP_I2S_PIN_OUT_AUD2/ARIP_I2S_PIN_OUT_AUD3
 *
 *  @return None
 *
 */
void wiced_audio_sink_set_arip_I2SPinOut( uint8_t arip_I2SPinOut );
/** @} */ // end of wiced_audio_sink

#ifdef __cplusplus
} /*extern "C" */
#endif

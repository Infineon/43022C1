#ifndef __WICED_AUDIO_CONFIG_CMD_H__
#define __WICED_AUDIO_CONFIG_CMD_H__

#include "wiced_bt_a2dp_defs.h"
#include "adva/lite_host/uipc_msg.h"

/*******************************************************************************
*                       Type Definitions
*******************************************************************************/
typedef enum {
    WICED_AUDIO_GET_CONFIG_RESULT = 0,
    WICED_AUDIO_ROLE_INIT,
    WICED_AUDIO_SINK_CONFIG_INIT,
    WICED_AUDIO_SINK_CONFIGURE,
    WICED_AUDIO_SINK_RESET,
    WICED_AUDIO_SCO_DATA,
    WICED_AUDIO_CONFIG_PCM,
    WICED_AUDIO_SOURCE_CONFIGURE,
    WICED_AUDIO_SOURCE_RESET,
    WICED_AUDIO_I2S_INJECT_INIT,
    WICED_AUDIO_I2S_INJECT_ENABLE,
    WICED_AUDIO_SCO_HOOK_INIT,
} wiced_audio_config_cmd_list_t;

typedef struct {
    uint32_t role;
} wiced_audio_role_init_cmd_t;

typedef struct {
    uint16_t handle;
    uint32_t is_master;
    uint32_t audio_route;
    uint16_t cp_type;
    wiced_bt_a2dp_codec_info_t* p_codec_info;
} wiced_audio_sink_configure_cmd_t;

typedef struct {
    uint16_t handle;
} wiced_audio_sink_reset_cmd_t;

typedef struct {
    uint32_t data_len;
    uint8_t* p_buf;
} wiced_audio_sink_transmit_data_t;

typedef struct {
    uint16_t conn_handle;
    uint32_t data_len;
    uint8_t* p_data;
} wiced_audio_sco_data_cmd_t;

typedef struct {
    uint32_t is_master;
    uint32_t audio_route;
    tCODEC_INFO_SBC uCodec;
    uint32_t src_sf;
    uint16_t lcid;
} wiced_audio_source_configure_cmd_t;

typedef struct {
    uint16_t lcid;
} wiced_audio_source_reset_cmd_t;

typedef struct {
    uint8_t enable;
    uint32_t sampleRate;
} wiced_audio_i2s_aud_inject_cmd_t;

#endif //  __WICED_AUDIO_CONFIG_CMD_H__

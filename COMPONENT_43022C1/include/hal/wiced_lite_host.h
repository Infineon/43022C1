#ifndef __WICED_LITE_HOST_H__
#define __WICED_LITE_HOST_H__

/*******************************************************************************
*                       Type Definitions
*******************************************************************************/
typedef enum {
    AUDIO_INSERT_AUDIO_INFO = 0,
    AUDIO_INSERT_FILL_FIFO,
    AUDIO_INSERT_SCO_INFO,
    AUDIO_INSERT_SCO_MIC,
    AUDIO_INSERT_SCO_SPK,
    AUDIO_INSERT_SCO_SPK_INSERTED_ZEROS,
} wiced_lite_host_event_t;

typedef struct {
    uint32_t event;
    void *data;
} wiced_lite_host_event_data_t;

/*****************************************************************
* Function: wiced_lite_host_setEvent()
*
* Abstract: Call this function to send event to wiced_lite_host thread
*
* Input/Output:
*    event and data pointer
*
* Return:
*    None
*
* Side Effects:
*
*****************************************************************/
void wiced_lite_host_setEvent( UINT32 event, void *data );

#endif // __WICED_LITE_HOST_H__

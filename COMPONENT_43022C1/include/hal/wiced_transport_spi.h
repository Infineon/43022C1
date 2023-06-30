/*
 * Broadcom Proprietary and Confidential. © 2016 Broadcom.
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */
/** @file
*
*/
#ifndef _WICED_TRANSPORT_SPI_H_
#define _WICED_TRANSPORT_SPI_H_

#include "transport/bttransport.h"
#include "bt/btcore/misc/bcs_dma.h"

#define WICED_HCI_HEADER_LENGTH     5

typedef enum
{
    SPI_TX_DATA_EVENT        = TRANSPORT_NEW_MSG_IN_MSG_QUEUE,
    SPI_RX_DMA_DONE_EVENT    = ( 1 << 2 ),
    SPI_WAKEUP_EVENT         = ( 1 << 3 ),
}wiced_trans_spi_events_t;

typedef enum
{
    SPI_INIT_STATE,
    SPI_WAIT_FOR_RX_DMA_DONE_STATE,
    SPI_TX_START_STATE,
    SPI_RX_START_STATE,
    SPI_TX_DONE_STATE,
    SPI_RX_DONE_STATE,
    SPI_WAIT_FOR_CS_INTR_STATE,
}wiced_trans_spi_states_t;

typedef enum
{
    SPI_WAIT_FOR_HEADER_STATE = 1,
    SPI_WAIT_FOR_TX_COMPLETE_STATE,
    SPI_WAIT_FOR_RX_COMPLETE_STATE,
}wiced_trans_spi_rx_dma_sub_states_t;

typedef struct
{
    DMA_TRANSFER_REQUEST_t        rx_dma;
    DMA_TRANSFER_REQUEST_t        tx_dma;
    //! The event flag where events regarding async SPI operations will be sent.
    OSAPI_EVENT_GROUP*            eventFlag;
    uint8_t                       spi_state;
    uint8_t                       spi_sub_state;
    uint8_t                       wakeup_event;
    uint8_t                       rx_dma_event;
    MSG_t*                        rx_msg;
    uint16_t                      rx_packet_length;
    MSG_t*                        tx_msg;
    uint16_t                      tx_packet_length;
    uint8_t                       header[WICED_HCI_HEADER_LENGTH];
}wiced_trans_spi_driver_state_t;

typedef struct
{
    //! Bluetooth transport base structure.
    BTTRANSPORT_t                       bttransport;
    wiced_trans_spi_driver_state_t      driver_state;
}BTSPI_TRANSPORT_t;


uint8_t wiced_transport_spi_get_device_role ( void );
wiced_result_t wiced_transport_spi_slave_tx_data( uint8_t* p_data, uint16_t length );
wiced_result_t wiced_transport_spi_init( void );


#endif //_WICED_TRANSPORT_SPI_H_

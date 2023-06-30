#ifndef WICED_HAL_SCB_CONTEXT_H
#define WICED_HAL_SCB_CONTEXT_H

#include "legacy_interface/bsp/inc/intctl.h"
#include "foundation/threadx/cy_tx_thread.h"
#include "foundation/hal/scb/cy_scb.h"
#include "foundation/hal/dev_drivers/dev_driver.h"

#include "wiced_hal_scb_common.h"

/** Enable below flags according to feature flags.
    If below flags were not enabled means code was not compiling for that peripheral */

#define  ENABLED_WICED_HAL_SCB_UART
#define  ENABLED_WICED_HAL_SCB_SPI
#define  ENABLED_WICED_HAL_SCB_I2C

/** static code */
#define MAYBE_STATIC

/** API's should not expose. Called only from wiced calls */
#define INTERNAL

INTERNAL uint8_t is_peripheral_mapped(wiced_hal_scb_peripheral_t peripheral, uint8_t *scb_hw);
INTERNAL uint8_t get_pin_mapped_scb(wiced_hal_scb_peripheral_t peripheral);
INTERNAL uint8_t update_context(wiced_hal_scb_peripheral_t peripheral, uint8_t scb);


typedef enum wiced_hal_scb_hw_e{
#ifdef scb0_adr_base
    WICED_HAL_SCB0,
#define WICED_HAL_SCB0_HW scb0_adr_base
#endif
#ifdef scb1_adr_base
    WICED_HAL_SCB1,
#define WICED_HAL_SCB1_HW scb1_adr_base
#endif
    WICED_HAL_MAX_SCB
}wiced_hal_scb_hw_t;

typedef struct scb_hw_e{
    CySCB_Type * const hw_base;
    const uint64_t intr_mask;
}scb_hw_t;

/** Wiced hal SCB driver context */
#ifdef ENABLED_WICED_HAL_SCB_UART

INTERNAL void puart_deinit(void);
INTERNAL void puart_update_context(void *context);
INTERNAL void puart_irq_handler(void);

typedef struct wiced_hal_uart_context_e{
    scb_hw_t *hw;                       /**< SCB HW */
    cy_stc_scb_uart_context_t context;  /**< SCB UART Context */
    cy_stc_scb_uart_config_t config;    /**< SCB UART Config */
    void (*cb_app_rx)(void*);           /**< Application rx call back */
    union{
        struct{
            uint8_t init:1;             /**< 1 - SUCCESS and 0 - FAIL */
            uint8_t map:1;              /**< 1 - SUCCESS and 0 - FAIL */
            uint8_t tx_en:1;            /**< TX Enable */
            uint8_t rx_en:1;            /**< RX Enable */
            uint8_t scb:2;              /**< 0 - SCB0, 1 - SCB1 ... 3- NONE */
            uint8_t reserved:2;
        }bit;
        uint8_t value;
    }sts;                               /**< Status */
}wiced_hal_uart_context_t;
#endif

/** Wiced hal SPI SCB driver context */
#ifdef ENABLED_WICED_HAL_SCB_SPI

INTERNAL void pspi_deinit(uint8_t spi);
INTERNAL void pspi_update_context(uint8 spi, void *context);
INTERNAL void pspi_irq_handler(uint8_t spi);

typedef struct wiced_hal_spi_context_e{
    scb_hw_t *hw;                        /**< SCB HW */
    cy_stc_scb_spi_context_t context;    /**< SCB SPI Context */
    cy_stc_scb_spi_config_t config;      /**< SCB SPI Config */
    uint32_t error;                      /**< Interrupt handler error for PDL driver events */
    union{
        struct{
            uint8_t init:1;              /**< 1 - SUCCESS and 0 - FAIL */
            uint8_t map:1;               /**< 1 - SUCCESS and 0 - FAIL */
            uint8_t s_tx_enb:1;          /**< Slave TX Enable */
            uint8_t s_rx_enb:1;          /**< Slave RX Enable */
            uint8_t pending:2;           /**< 0 - NONE, 1- RX, 2 - TX, 3 - TX and RX */
            uint8_t scb:2;               /**< 0 - SCB0, 1 - SCB1 ... 3- NONE */
        }bit;
        uint8_t value;
    }sts;                                /**< Status */
}wiced_hal_spi_context_t;
#endif

/** Wiced hal I2C SCB driver context */
#ifdef ENABLED_WICED_HAL_SCB_I2C

INTERNAL void i2c_deinit(void);
INTERNAL void i2c_update_context(void *context);
INTERNAL void i2c_irq_handler(void);

typedef struct wiced_hal_i2c_context_e{
    scb_hw_t *hw;                        /**< SCB HW */
    cy_stc_scb_i2c_context_t context;    /**< SCB Context */
    cy_stc_scb_i2c_config_t config;      /**< SCB Config */
    uint32_t scb_block_freq;             /**< SCB Block freq */
    uint32_t error;                      /**< Interrupt handler error for PDL driver events */
    uint32_t time_out_us;                /**< Time out in us for read/write/combo read */
    union{
        struct{
            uint8_t init:1;              /**< 1 - SUCCESS and 0 - FAIL */
            uint8_t map:1;               /**< 1 - SUCCESS and 0 - FAIL */
            uint8_t pending:2;           /**< 0 - NONE, 1- RX, 2 - TX, 3 - TX and RX */
            uint8_t scb:2;              /**< 0 - SCB0, 1 - SCB1 ... 3- NONE */
            uint8_t reserved:2;
        }bit;
        uint8_t value;
    }sts;                                /**< Status */
}wiced_hal_i2c_context_t;
#endif

#endif // WICED_HAL_SCB_CONTEXT_H

#ifndef WICED_HAL_SCB_COMMON_H
#define WICED_HAL_SCB_COMMON_H

#include "wiced_data_types.h"

typedef enum wiced_hal_scb_peripheral_e{
    WICED_HAL_PUART,
    WICED_HAL_PSPI1,
    WICED_HAL_PSPI2,
    WICED_HAL_I2C,
    WICED_HAL_MAX_PDL_SUPPORT,
    WICED_HAL_NONE = 0xFF
}wiced_hal_scb_peripheral_t;

/**
 * Function         scb_irq_handler
 *
 * Common driver hook IRQ Handler
 *
 * @param[in]      scb : SCB HW block id, eg: 0 - SCB0, 1 - SCB1.
 *
 */
void scb_irq_handler(uint8_t scb);

/**
 * Function         wiced_hal_map_scb
 *
 * To map the peripheral to available SCB hw block.
 *
 * @param[in]      scb_hw : SCB HW block id, eg: 0 - SCB0, 1 - SCB1.
 * @param[in]      peripheral : Peripheral.
 *
 * @return          1 [SUCESS], 0 [FAILURE].
 *
 */
uint8_t wiced_hal_map_scb(uint8_t scb_hw, wiced_hal_scb_peripheral_t peripheral);

#endif // WICED_HAL_SCB_COMMON_H

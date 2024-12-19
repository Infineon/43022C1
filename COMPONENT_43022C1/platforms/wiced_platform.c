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
#if USE_DESIGN_MODUS

#include "wiced_platform.h"
#include "wiced_bt_app_hal_common.h"
#include "platform_mem.h"
#include "spar_utils.h"
#include "wiced_bt_trace.h"

extern wiced_platform_gpio_t platform_gpio_pins[];
extern wiced_platform_led_config_t platform_led[];
extern wiced_platform_button_config_t platform_button[];
extern wiced_platform_gpio_config_t platform_gpio[];
extern size_t platform_gpio_pin_count;
extern size_t led_count;
extern size_t button_count;
extern size_t gpio_count;

void init_cycfg_all(void);
void wiced_platform_warm_up(void);
/* utility functions */

//! The REG32 macro is a convenient way to access a 32-bit hardware register.  It merely casts the
//! integer register address to a pointer to a volatile UINT32 and dereferences it.
#define REG32(address)  ( *(volatile UINT32*)(address) )


/**
 *  \brief Provide utility function for application to register for cb upon button interrupt
 *
 *  \param [in] button select a button from wiced_platform_button_number_t
 *  \param [in] userfn Provide the call back function
 *  \param [in] userdata Data to be provided with the call back
 *  \param [in] trigger_edge To configure interrupt on rising/falling/dual edge
 *
 *  \return none
 *
 */
void wiced_platform_register_button_callback(wiced_platform_button_number_t button, void (*userfn)(void*, UINT8), void* userdata,
                wiced_platform_button_interrupt_edge_t trigger_edge)
{
    if(button < button_count)
    {
        wiced_hal_gpio_register_pin_for_interrupt(*platform_button[button].gpio, userfn, userdata);
        wiced_hal_gpio_configure_pin(*platform_button[button].gpio, (platform_button[button].config | trigger_edge), platform_button[button].default_state);
    }
}

/**
 *  \brief Return state of the pin when button is pressed
 *
 *  \param [in] button select a button from wiced_platform_button_number_t
 *
 *  \return button pressed value
 *
 */
uint32_t wiced_platform_get_button_pressed_value(wiced_platform_button_number_t button)
{
	return platform_button[button].button_pressed_value;
}

#define cr_pad_config_adr0  (0x00320068)
#define cr_pad_config_adr1  (0x0032006c)
#define cr_pad_config_adr4  (0x00320078)
#define cr_pad_config_adr7  (0x003200e4)
#define cr_pad_config_adr8  (0x00320110)
#define cr_pad_fcn_ctl_adr0 (0x00320088)
#define cr_pad_fcn_ctl_adr1 (0x0032008c)
#define cr_pad_fcn_ctl_adr2 (0x00320090)
#define cr_pad_fcn_ctl_adr3 (0x003201a8)

#define FN_PAD_REG_ADD(x)           ((x >> 2) & 0x7f)
#define FN_PAD_CFG_REG_POS(x)       ((x & 0x18) >> 3)
#define FN_PAD_FCN_REG_POS(x)       ((x & 0x1c) >> 2)

#define GPIO_SETUP_GET_ADDR(x)          ((x << 2) | (cr_pad_config_adr0 & 0xfffffe00))

typedef struct tag_GPIO_SETUP_REG_INFO_t
{
    uint8_t config_register_offset;
    uint8_t function_register_offset;
    uint8_t pad_config;
    uint8_t config_register_position : 2;
    uint8_t function_register_position : 3;
} GPIO_SETUP_REG_INFO_t;

GPIO_SETUP_REG_INFO_t core_regs[15] =
{
    // gpio 0 BT_DEV_WAKE  WICED_GPIO_00 (40)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr0),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr0),
        .pad_config = 0x09,
        .config_register_position = FN_PAD_CFG_REG_POS(0),
        .function_register_position = FN_PAD_FCN_REG_POS(0)},
    // gpio 1 BT_HOST_WAKE  WICED_GPIO_01 (41)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr0),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr0),
        .pad_config = 0x09,
        .config_register_position = FN_PAD_CFG_REG_POS(8),
        .function_register_position = FN_PAD_FCN_REG_POS(4)},
    // gpio 2  WICED_GPIO_02 (42)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr0),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr0),
        .pad_config = 0x09,
        .config_register_position = FN_PAD_CFG_REG_POS(16),
        .function_register_position = FN_PAD_FCN_REG_POS(8)},
    // gpio 3  WICED_GPIO_03 (43)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr0),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr0),
        .pad_config = 0x22,
        .config_register_position = FN_PAD_CFG_REG_POS(24),
        .function_register_position = FN_PAD_FCN_REG_POS(12)},
    // gpio 4  WICED_GPIO_04 (44)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr1),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr0),
        .pad_config = 0x02,
        .config_register_position = FN_PAD_CFG_REG_POS(0),
        .function_register_position = FN_PAD_FCN_REG_POS(16)},
    // gpio 5  WICED_GPIO_05 (45)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr1),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr0),
        .pad_config = 0x02,
        .config_register_position = FN_PAD_CFG_REG_POS(8),
        .function_register_position = FN_PAD_FCN_REG_POS(20)},
    // gpio 6 BT_I2S_CLK  WICED_GPIO_06 (46)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr8),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr3),
        .pad_config = 0x60,
        .config_register_position = FN_PAD_CFG_REG_POS(24),
        .function_register_position = FN_PAD_FCN_REG_POS(12)},
    // gpio 7 BT_I2S_WS  WICED_GPIO_07 (47)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr8),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr3),
        .pad_config = 0x09,
        .config_register_position = FN_PAD_CFG_REG_POS(16),
        .function_register_position = FN_PAD_FCN_REG_POS(8)},
    // a gpio 0 BT_PCM_CLK  WICED_GPIO_8 (48)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr4),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr1),
        .pad_config = 0x88,
        .config_register_position = FN_PAD_CFG_REG_POS(16),
        .function_register_position = FN_PAD_FCN_REG_POS(24)},
    // a gpio 1 BT_PCM_SYNC  WICED_GPIO_9 (49)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr4),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr1),
        .pad_config = 0x88,
        .config_register_position = FN_PAD_CFG_REG_POS(24),
        .function_register_position = FN_PAD_FCN_REG_POS(20)},
    // a gpio 2 BT_PCM_OUT  WICED_GPIO_10 (50)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr4),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr1),
        .pad_config = 0x22,
        .config_register_position = FN_PAD_CFG_REG_POS(8),
        .function_register_position = FN_PAD_FCN_REG_POS(16)},
    // a gpio 3 BT_PCM_IN  WICED_GPIO_11 (51)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr4),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr1),
        .pad_config = 0x09,
        .config_register_position = FN_PAD_CFG_REG_POS(0),
        .function_register_position = FN_PAD_FCN_REG_POS(12)},
    // a gpio 4 - not valid (52)
    {   .config_register_offset = 0,
        .function_register_offset = 0,
        .pad_config = 0x09,
        .config_register_position = 0,
        .function_register_position = 0},
    // a gpio 5 BT_I2S_DO  WICED_GPIO_13 (53)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr7),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr2),
        .pad_config = 0x88,
        .config_register_position = FN_PAD_CFG_REG_POS(0),
        .function_register_position = FN_PAD_FCN_REG_POS(16)},
    // a gpio 6 BT_I2S_DI  WICED_GPIO_14 (54)
    {   .config_register_offset = FN_PAD_REG_ADD(cr_pad_config_adr7),
        .function_register_offset = FN_PAD_REG_ADD(cr_pad_fcn_ctl_adr2),
        .pad_config = 0x09,
        .config_register_position = FN_PAD_CFG_REG_POS(16),
        .function_register_position = FN_PAD_FCN_REG_POS(24)},

};

wiced_bt_gpio_select_status_t core_gpio_function_select (wiced_bt_gpio_numbers_t gpio, wiced_bt_gpio_function_t newfunction)
{
    GPIO_SETUP_REG_INFO_t *setup = &core_regs[gpio - WICED_GPIO_00];

    // set new function if a match was found for this pad, or set to gpio input for no match
    REG32(GPIO_SETUP_GET_ADDR(setup->function_register_offset)) &= ~(0xf << (setup->function_register_position << 2));
    REG32(GPIO_SETUP_GET_ADDR(setup->function_register_offset)) |= (newfunction << (setup->function_register_position << 2));

    // take care of default pad setting associated with pins, this can be modified prior to calling
    REG32(GPIO_SETUP_GET_ADDR(setup->config_register_offset)) &= ~(0xff << (setup->config_register_position << 3));
    REG32(GPIO_SETUP_GET_ADDR(setup->config_register_offset)) |= (setup->pad_config << (setup->config_register_position << 3));

    return WICED_GPIO_SUCCESS;
}

// wiced_hal_gpio_configure_pin handles direction and interrupt setup
// this function handles pull up/down, drive strength, and hysteresis
#define GPIO_REG_DIR 0
#define GPIO_REG_SENSE 4
#define GPIO_REG_BOTH 8
#define GPIO_REG_EVENT 12
wiced_bt_gpio_select_status_t core_gpio_pad_configure (wiced_bt_gpio_numbers_t gpio, uint32_t config)
{
    wiced_bt_gpio_select_status_t status = WICED_GPIO_FAILURE;
    GPIO_SETUP_REG_INFO_t *setup;
    uint32_t reg_value, reg_addr;
    do {
        uint8_t register_config = 0;
        uint8_t pin = gpio;
        if(pin < WICED_GPIO_00)
            break;
        if(pin > WICED_GPIO_14)
            break;
        if(pin == (WICED_GPIO_11 + 1))
            break;
        pin -= WICED_GPIO_00;
        setup = &core_regs[pin];

        // clear relevant settings
        reg_value = REG32(GPIO_SETUP_GET_ADDR(setup->config_register_offset));
        register_config = (reg_value >> (setup->config_register_position << 3)) & 0xbe;
        register_config |= (config & GPIO_PULL_UP) ? 2 : 0;
        register_config |= (config & GPIO_PULL_DOWN) ? 4 : 0;
        register_config |= (config & WICED_GPIO_HYSTERESIS_ON) ? 8 : 0;
        register_config |= (config & WICED_GPIO_DRIVE_SEL_MASK) ? 0x80 : 0;
        reg_value &= ~(0xff << (setup->config_register_position << 3));
        reg_value |= register_config << (setup->config_register_position << 3);
        REG32(GPIO_SETUP_GET_ADDR(setup->config_register_offset)) = reg_value;

#if 1 // remove this after patch is updated with fix for single edge trigger
        reg_addr = (pin >> 3) ? 0x32b400 : 0x32c400;
        if(config & GPIO_OUTPUT_ENABLE)
            REG32(reg_addr + GPIO_REG_DIR) |= (1 << (pin & 7));
        else
            REG32(reg_addr + GPIO_REG_DIR) &= ~(1 << (pin & 7));
        if(config & WICED_GPIO_INTERRUPT_ENABLE)
        {
            if(config & WICED_GPIO_EDGE_TRIGGER)
            {
                REG32(reg_addr + GPIO_REG_SENSE) &= ~(1 << (pin & 7));
                if(config & WICED_GPIO_TRIGGER_NEG)
                {
                    REG32(reg_addr + GPIO_REG_EVENT) &= ~(1 << (pin & 7));
                }
                else
                {
                    REG32(reg_addr + GPIO_REG_EVENT) |= (1 << (pin & 7));
                }
                if(config & WICED_GPIO_EDGE_TRIGGER_BOTH)
                {
                    REG32(reg_addr + GPIO_REG_BOTH) |= (1 << (pin & 7));
                }
                else
                {
                    REG32(reg_addr + GPIO_REG_BOTH) &= ~(1 << (pin & 7));
                }
             }
             else
             {
                REG32(reg_addr + GPIO_REG_SENSE) |= (1 << (pin & 7));
                if(config & WICED_GPIO_TRIGGER_NEG)
                {
                    REG32(reg_addr + GPIO_REG_EVENT) &= ~(1 << (pin & 7));
                }
                else
                {
                    REG32(reg_addr + GPIO_REG_EVENT) |= (1 << (pin & 7));
                }
             }
        }
#endif
        status = WICED_GPIO_SUCCESS;
    } while(0);
    return status;
}

#if 0
void platform_report_pins()
{
    uint8_t i,j;
    WICED_BT_TRACE("platform_report_pins\n");
    for(i = 0; i < platform_gpio_pin_count; i++)
    {
        uint8_t pin = platform_gpio_pins[i].gpio_pin;
        if( (pin >= WICED_GPIO_00) && (pin <= WICED_GPIO_14) && (pin != (WICED_GPIO_11 + 1)))
        {
            GPIO_SETUP_REG_INFO_t *setup = &core_regs[pin - WICED_GPIO_00];

            WICED_BT_TRACE("  pin %d fn select at %06x position %d is %d\n", pin,
                    GPIO_SETUP_GET_ADDR(setup->function_register_offset),
                    setup->function_register_position,
                    (REG32(GPIO_SETUP_GET_ADDR(setup->function_register_offset)) >> (setup->function_register_position << 2)) & 0xf);
            WICED_BT_TRACE("    config set at %06x position %d is 0x%x\n",
                    GPIO_SETUP_GET_ADDR(setup->config_register_offset),
                    setup->config_register_position,
                    (REG32(GPIO_SETUP_GET_ADDR(setup->config_register_offset)) >> (setup->config_register_position << 3)) & 0xff);
        }
    }
}
void platform_report_buttons()
{
    uint8_t i,j;
    WICED_BT_TRACE("platform_report_buttons\n");
    for (i = 0; i < button_count; i++)
    {
        uint8_t pin = *platform_button[i].gpio;
        if( (pin >= WICED_GPIO_00) && (pin <= WICED_GPIO_14) && (pin != (WICED_GPIO_11 + 1)))
        {
            GPIO_SETUP_REG_INFO_t *setup = &core_regs[pin - WICED_GPIO_00];
            uint32_t gpio_reg_adr;

            WICED_BT_TRACE("  pin %d fn select at %06x position %d is %d\n", pin,
                    GPIO_SETUP_GET_ADDR(setup->function_register_offset),
                    setup->function_register_position,
                    (REG32(GPIO_SETUP_GET_ADDR(setup->function_register_offset)) >> (setup->function_register_position << 2)) & 0xf);
            pin -= WICED_GPIO_00;
            gpio_reg_adr = (pin >> 3) ? 0x32b400 : 0x32c400;

            WICED_BT_TRACE("    config set at %06x position %d is 0x%x\n",
                    GPIO_SETUP_GET_ADDR(setup->config_register_offset),
                    setup->config_register_position,
                    (REG32(GPIO_SETUP_GET_ADDR(setup->config_register_offset)) >> (setup->config_register_position << 3)) & 0xff);
            WICED_BT_TRACE("    direction set at %06x position %d is 0x%x\n",
                    gpio_reg_adr,
                    pin & 7,
                    (REG32(gpio_reg_adr) & (1 << (pin & 7))));
            WICED_BT_TRACE("    int sense set at %06x position %d is 0x%x\n",
                    gpio_reg_adr+GPIO_REG_SENSE,
                    pin & 7,
                    (REG32(gpio_reg_adr+GPIO_REG_SENSE) & (1 << (pin & 7))));
            WICED_BT_TRACE("    int both edge set at %06x position %d is 0x%x\n",
                    gpio_reg_adr+GPIO_REG_BOTH,
                    pin & 7,
                    (REG32(gpio_reg_adr+GPIO_REG_BOTH) & (1 << (pin & 7))));
        }
    }
}
#endif

wiced_bt_gpio_select_status_t gpio_function_Set_GPIO_Function(wiced_bt_gpio_numbers_t gpio, wiced_bt_gpio_function_t newfunction);
void wiced_hal_gpio_select_function_local(wiced_bt_gpio_numbers_t gpio, wiced_bt_gpio_function_t newfunction);

wiced_bt_gpio_select_status_t wiced_hal_gpio_select_function(wiced_bt_gpio_numbers_t pin, wiced_bt_gpio_function_t function)
{
    wiced_bt_gpio_select_status_t status = WICED_GPIO_FAILURE;
    if(pin < WICED_GPIO_00)
    {
        status = gpio_function_Set_GPIO_Function(pin, function);
    }
    else if( (pin <= WICED_GPIO_14) && (pin != (WICED_GPIO_11+1)))
    {
        status = core_gpio_function_select(pin, function);
    }
    return status;
}

// needed for SPI
#define cr_pad_fcn_ctl_lhl_0_adr                       0x0032021c
#define cr_pad_fcn_ctl_lhl_1_adr                       0x0032023c
void wiced_hal_gpio_select_function_local(wiced_bt_gpio_numbers_t gpio, wiced_bt_gpio_function_t function);

/**
 *  \brief Initialize all the required pins and configure their functionality
 *
 *  \return none
 *
 */
void wiced_platform_init(void)
{
    uint32_t i = 0;

    /* Initialize modules required for GPIO */
    wiced_bt_app_hal_init();
    wiced_platform_warm_up();

    /* Configure pins available on the platform with the chosen functionality */
    for (i = 0; i < platform_gpio_pin_count; i++)
    {
        // the patched version _FillinFunctionInfo is called by wiced_hal_gpio_select_function_local
        // use this for SPI until patch entry availability is improved for 43012
        if( ((platform_gpio_pins[i].functionality == WICED_SPI_1_CLK) ||
            (platform_gpio_pins[i].functionality == WICED_SPI_1_CS) ||
            (platform_gpio_pins[i].functionality == WICED_SPI_1_MISO) ||
            (platform_gpio_pins[i].functionality == WICED_SPI_1_MOSI)) &&
            (platform_gpio_pins[i].gpio_pin < WICED_GPIO_00) )
        {
            wiced_hal_gpio_select_function_local(platform_gpio_pins[i].gpio_pin, platform_gpio_pins[i].functionality);
            // just one time, need for SPI
            if(platform_gpio_pins[i].functionality == WICED_SPI_1_CLK)
            {
                REG32(cr_pad_fcn_ctl_lhl_0_adr) = REG32(cr_pad_fcn_ctl_lhl_0_adr) | 0x80000000;
                REG32(cr_pad_fcn_ctl_lhl_1_adr) = (REG32(cr_pad_fcn_ctl_lhl_1_adr) & 0xfffffff0) | 0x0d;
            }
        }
        else
        {
            wiced_hal_gpio_select_function(platform_gpio_pins[i].gpio_pin, platform_gpio_pins[i].functionality);
        }
    }

    /* Initialize LEDs and turn off by default */
    for (i = 0; i < led_count; i++)
    {
        wiced_hal_gpio_configure_pin(*platform_led[i].gpio, platform_led[i].config, platform_led[i].default_state);
        core_gpio_pad_configure(*platform_led[i].gpio, (platform_led[i].config));
    }

    /* Initialize buttons with the default configuration */
    for (i = 0; i < button_count; i++)
    {
        wiced_hal_gpio_configure_pin(*platform_button[i].gpio, (platform_button[i].config), platform_button[i].default_state);
        core_gpio_pad_configure(*platform_button[i].gpio, (platform_button[i].config));
    }

    /* Initialize GPIOs with the default configuration */
    for (i = 0; i < gpio_count; i++)
    {
        wiced_hal_gpio_configure_pin(*platform_gpio[i].gpio, (platform_gpio[i].config), platform_gpio[i].default_state);
        core_gpio_pad_configure(*platform_gpio[i].gpio, (platform_gpio[i].config));
    }

    /* any other personality-based initialization */
 //   init_cycfg_all();

    platform_mem_init();

    /* disable watchdog, set up SWD, wait for attach if ENABLE_DEBUG */
    SETUP_APP_FOR_DEBUG_IF_DEBUG_ENABLED();
    BUSY_WAIT_UNTIL_MANUAL_CONTINUE_IF_DEBUG_ENABLED();
}
#endif

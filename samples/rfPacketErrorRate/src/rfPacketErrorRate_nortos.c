/*
 * Copyright (c) 2016-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DeviceFamily_CC13X2
#define DeviceFamily_CC13X2
#endif

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>

/* TI Drivers */
//#include <ti/display/Display.h>
//#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/rf/RF.h>
//#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* Application specific Header files */
#include "menu.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

/***** Variable declarations *****/

/* Pin driver objects and handles */
//PIN_Handle buttonPinHandle;
//static PIN_State buttonPinState;

extern void menu_runTask(); 

/*
Application button pin configuration table.
Buttons interrupts are configured to trigger on falling edge.
*/

/* change this to use another GPIO port */
#ifndef DT_ALIAS_SW0_GPIOS_CONTROLLER
#ifdef SW0_GPIO_NAME
#define DT_ALIAS_SW0_GPIOS_CONTROLLER SW0_GPIO_NAME
#else
#error SW0_GPIO_NAME or DT_ALIAS_SW0_GPIOS_CONTROLLER needs to be set in board.h
#endif
#endif
#define PORT    DT_ALIAS_SW0_GPIOS_CONTROLLER

/* change this to use another GPIO pin */
#ifdef DT_ALIAS_SW0_GPIOS_PIN
#define PIN1     DT_ALIAS_SW0_GPIOS_PIN
#else
#error DT_ALIAS_SW0_GPIOS_PIN needs to be set in board.h
#endif

/* change this to use another GPIO pin */
#ifdef DT_ALIAS_SW1_GPIOS_PIN
#define PIN2     DT_ALIAS_SW1_GPIOS_PIN
#else
#error DT_ALIAS_SW1_GPIOS_PIN needs to be set in board.h
#endif

/* change to use another GPIO pin interrupt config */
#ifdef DT_ALIAS_SW0_GPIOS_FLAGS
#define EDGE1    (DT_ALIAS_SW0_GPIOS_FLAGS | GPIO_INT_EDGE)
#else
/*
 * If DT_ALIAS_SW0_GPIOS_FLAGS not defined used default EDGE value.
 * Change this to use a different interrupt trigger
 */
#define EDGE1    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#endif

/* change to use another GPIO pin interrupt config */
#ifdef DT_ALIAS_SW1_GPIOS_FLAGS
#define EDGE2    (DT_ALIAS_SW1_GPIOS_FLAGS | GPIO_INT_EDGE)
#else
/*
 * If DT_ALIAS_SW0_GPIOS_FLAGS not defined used default EDGE value.
 * Change this to use a different interrupt trigger
 */
#define EDGE2    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#endif

/* change this to enable pull-up/pull-down */
#ifndef DT_ALIAS_SW0_GPIOS_FLAGS
#ifdef DT_ALIAS_SW0_GPIOS_PIN_PUD
#define DT_ALIAS_SW0_GPIOS_FLAGS DT_ALIAS_SW0_GPIOS_PIN_PUD
#else
#define DT_ALIAS_SW0_GPIOS_FLAGS 0
#endif
#endif
#define PULL_UP1 DT_ALIAS_SW0_GPIOS_FLAGS

/* change this to enable pull-up/pull-down */
#ifndef DT_ALIAS_SW1_GPIOS_FLAGS
#ifdef DT_ALIAS_SW1_GPIOS_PIN_PUD
#define DT_ALIAS_SW1_GPIOS_FLAGS DT_ALIAS_SW1_GPIOS_PIN_PUD
#else
#define DT_ALIAS_SW1_GPIOS_FLAGS 0
#endif
#endif
#define PULL_UP2 DT_ALIAS_SW1_GPIOS_FLAGS


/*
Interrupt handler for the button pints.
*/
static struct gpio_callback gpio_cb1;
static struct gpio_callback gpio_cb2;

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
                    u32_t pins)
{
    //printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());

    if (pins == BIT(DT_ALIAS_SW0_GPIOS_PIN)) {
        menu_notifyButtonPressed(Button_Select);
    } else if (pins == BIT(DT_ALIAS_SW1_GPIOS_PIN)) {
        menu_notifyButtonPressed(Button_Navigate);
    }
}

void mainThread(void *arg0)
{
    /* Initialize the radio */
//    RF_Params rfParams;
//    RF_Params_init(&rfParams);

    /* Initialize the UART and SPI for the display driver. */
//    Display_init();
//    UART_init();
//    SPI_init();
	

       struct device *gpiob;

        gpiob = device_get_binding(PORT);
        if (!gpiob) {
                printk("%s(): %d: error\n", __func__, __LINE__);
                return;
        }

        gpio_pin_configure(gpiob, PIN1,
                           GPIO_DIR_IN | GPIO_INT |  PULL_UP1 | EDGE1);

        gpio_init_callback(&gpio_cb1, button_pressed, BIT(PIN1));

        gpio_add_callback(gpiob, &gpio_cb1);
        gpio_pin_enable_callback(gpiob, PIN1);

        gpio_pin_configure(gpiob, PIN2,
                           GPIO_DIR_IN | GPIO_INT |  PULL_UP2 | EDGE2);

        gpio_init_callback(&gpio_cb2, button_pressed, BIT(PIN2));
        gpio_add_callback(gpiob, &gpio_cb2);
        gpio_pin_enable_callback(gpiob, PIN2);

    menu_init(); 

    /* Start task execution */
    
	menu_runTask();
}

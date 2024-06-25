/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USB Device CDC echo Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>

#include "cyabs_rtos.h"
#include "usbd_cdc_task.h"
#include "cy_debug.h"
#include "cy_memtrack.h"

/*******************************************************************************
* Macros
********************************************************************************/

/*******************************************************************************
* Function Prototypes
********************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;


int main(void)
{
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;

    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    CY_MEMTRACK_INITIALIZE();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    DEBUG_PRINT(("\x1b[2J\x1b[;H"));

    DEBUG_PRINT(("****************** "
                 "emUSB Device: CDC echo application "
                 "****************** \n\n"));

    xTaskCreate((void *)usbd_cdc_read_task,
                "USBD Read Task",
                USBD_CDC_TASK_STACK_SIZE,
                NULL,
                USBD_CDC_TASK_PRIORITY,
                NULL);

    xTaskCreate((void *)usbd_cdc_write_task,
                "USBD Write Task",
                USBD_CDC_TASK_STACK_SIZE,
                NULL,
                USBD_CDC_TASK_PRIORITY,
                NULL);

    /* Start the scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);

    return 0;
}


/* [] END OF FILE */

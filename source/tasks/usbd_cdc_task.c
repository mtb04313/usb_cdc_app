/******************************************************************************
* File Name: usbd_cdc_task.c
*
* Description: Implements USB device CDC communication
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2020-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/* Include header files */
#include <stdio.h>
#include "feature_config.h"

#include "usbd_cdc_task.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_pdl.h"

#include "USB.h"
#include "USB_CDC.h"

//#include "cy_time_misc.h"
#include "cy_debug.h"


/****************************************************************************
 * Local Definition
 *****************************************************************************/
#define USB_CONFIG_DELAY          (50U) /* In milliseconds */

/****************************************************************************
 * Local Data
 *****************************************************************************/
static const char *TAG = "usbd_cdc";

static const USB_DEVICE_INFO s_usb_deviceInfo = {
    0x058B,                       /* VendorId    */
    0x027D,                       /* ProductId    */
    "Infineon Technologies",      /* VendorName   */
    "CDC Code Example",           /* ProductName  */
    "12345678"                    /* SerialNumber */
};



/****************************************************************************
 * Local Functions
 *****************************************************************************/

/*********************************************************************
* Function Name: USBD_CDC_Echo_Init
**********************************************************************
* Summary:
*  Add communication device class to USB stack
*
* Parameters:
*  void
*
* Return:
*  void
**********************************************************************/

static USB_CDC_HANDLE usb_add_cdc(void)
{
    static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA     InitData;
    USB_ADD_EP_INFO       EPBulkIn;
    USB_ADD_EP_INFO       EPBulkOut;
    USB_ADD_EP_INFO       EPIntIn;

    memset(&InitData, 0, sizeof(InitData));
    EPBulkIn.Flags          = 0;                             /* Flags not used */
    EPBulkIn.InDir          = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPBulkIn.Interval       = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPIn  = USBD_AddEPEx(&EPBulkIn, NULL, 0);

    EPBulkOut.Flags         = 0;                             /* Flags not used */
    EPBulkOut.InDir         = USB_DIR_OUT;                   /* OUT direction (Host to Device) */
    EPBulkOut.Interval      = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPOut = USBD_AddEPEx(&EPBulkOut, OutBuffer, sizeof(OutBuffer));

    EPIntIn.Flags           = 0;                             /* Flags not used */
    EPIntIn.InDir           = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPIntIn.Interval        = 64;                            /* Interval of 8 ms (64 * 125us) */
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE ;   /* Maximum packet size (64 for Interrupt) */
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         /* Endpoint type - Interrupt */
    InitData.EPInt = USBD_AddEPEx(&EPIntIn, NULL, 0);

    return USBD_CDC_Add(&InitData);
}

/****************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 * Function Name: iso7816_flush_task
 ******************************************************************************
 * Summary:
 *  This task initializes the LED and DFU middleware and then waits for host
 *  to start the DFU operation.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void usbd_cdc_task(void)
{
    USB_CDC_HANDLE usb_cdcHandle;
    char read_buffer[USB_FS_BULK_MAX_PACKET_SIZE];
    char write_buffer[USB_FS_BULK_MAX_PACKET_SIZE];

    cy_rslt_t result;
    int num_bytes_received;
    int num_bytes_to_write = 0;

    CY_LOGD(TAG, "%s [%d]\n", __FUNCTION__, __LINE__);

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED,
                             CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG,
                             CYBSP_LED_STATE_OFF);

    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    /* Initializes the USB stack */
    USBD_Init();

    /* Endpoint Initialization for CDC class */
    usb_cdcHandle = usb_add_cdc();

    /* Set device info used in enumeration */
    USBD_SetDeviceInfo(&s_usb_deviceInfo);

    /* Start the USB stack */
    USBD_Start();

    /* Turning the LED on to indicate device is active */
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

    for (;;)
    {
        /* Wait for configuration */
        while ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED)
        {
            cyhal_system_delay_ms(USB_CONFIG_DELAY);
        }

        num_bytes_received = USBD_CDC_Receive(usb_cdcHandle, read_buffer, sizeof(read_buffer), 0);
        if (num_bytes_received) {
            DEBUG_PRINT(("%d received\n", num_bytes_received));
        }

        memcpy(write_buffer + num_bytes_to_write, read_buffer, num_bytes_received);
        num_bytes_to_write +=  num_bytes_received;


        if ((num_bytes_to_write > 0) &&
            ((read_buffer[num_bytes_received - 1] == '\r') ||
             (read_buffer[num_bytes_received - 1] == '\n')))
        {
            int retVal;
            /* Sending one packet to host */
            retVal = USBD_CDC_Write(usb_cdcHandle, write_buffer, num_bytes_to_write, 0);

            /* Waits for specified number of bytes to be written to host */
            USBD_CDC_WaitForTX(usb_cdcHandle, 0);
            DEBUG_PRINT(("[%d] %d written\n", __LINE__, retVal));

            /* If the last sent packet is exactly the maximum packet
            *  size, it is followed by a zero-length packet to assure
            *  that the end of the segment is properly identified by
            *  the terminal.
            */

            if (num_bytes_to_write == sizeof(write_buffer))
            {
                /* Sending zero-length packet to host */
                retVal = USBD_CDC_Write(usb_cdcHandle, NULL, 0, 0);

                /* Waits for specified number of bytes to be written to host */
                USBD_CDC_WaitForTX(usb_cdcHandle, 0);
                DEBUG_PRINT(("[%d] %d written\n", __LINE__, retVal));
            }

            num_bytes_to_write = 0;
        }

        cyhal_syspm_sleep();
    }
}

/* [] END OF FILE */

/*******************************************************************************
* File Name: feature_config.h
*
* Description: This file defines whether features are enabled / disabled
*
* Related Document: See README.md
*
*
*********************************************************************************
 Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef SOURCE_FEATURE_CONFIG_H_
#define SOURCE_FEATURE_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif


/*-- Public Definitions -------------------------------------------------*/

#define ENABLE_FEATURE                  1
#define DISABLE_FEATURE                 2

// core features
#define FEATURE_PPP                     DISABLE_FEATURE // was ENABLE_FEATURE
#define FEATURE_WIFI                    DISABLE_FEATURE
#define FEATURE_CONSOLE                 ENABLE_FEATURE
#define FEATURE_ESIM_LPA_MENU           DISABLE_FEATURE // unused option
#define FEATURE_APPS                    ENABLE_FEATURE
#define FEATURE_IO_MENU                 DISABLE_FEATURE //ENABLE_FEATURE
#define FEATURE_MQTT                    DISABLE_FEATURE //ENABLE_FEATURE
#define FEATURE_ATMODEM_MQTT            ENABLE_FEATURE
#define FEATURE_MODEM_LITE              DISABLE_FEATURE //ENABLE_FEATURE

#if defined (TARGET_APP_CY8CKIT_062_WIFI_BT) // 062 WIFI BT Pioneer Kit
// disable to conserve SRAM
#define FEATURE_BLE_MODEM               DISABLE_FEATURE
#else
#define FEATURE_BLE_MODEM               DISABLE_FEATURE // was ENABLE_FEATURE
#endif

#define FEATURE_SNTP_TIME               DISABLE_FEATURE
#define FEATURE_FLASH_EEPROM            DISABLE_FEATURE // unused option

// eSIM LPA menu features (only takes effect if FEATURE_ESIM_LPA_MENU is enabled)
#define FEATURE_ADD_PROFILE             DISABLE_FEATURE // unused option
#define FEATURE_ADVANCED_OPTIONS        DISABLE_FEATURE // unused option
#define FEATURE_SWITCH_PROFILE          DISABLE_FEATURE // unused option
#define FEATURE_DELETE_PROFILE          DISABLE_FEATURE // unused option
#define FEATURE_SET_PROFILE_NICKNAME    DISABLE_FEATURE // unused option

// unit tests
#define FEATURE_UNIT_TEST_CURL          DISABLE_FEATURE // unused option
#define FEATURE_UNIT_TEST_ESIM_LPA      DISABLE_FEATURE // unused option
#define FEATURE_UNIT_TEST_RTOS          DISABLE_FEATURE // unused option

#ifdef __cplusplus
}
#endif

#endif      /* SOURCE_FEATURE_CONFIG_H_ */

/* [] END OF FILE */

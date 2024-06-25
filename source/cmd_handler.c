/******************************************************************************
* File Name: cmd_handler.c
*
* Description: Implements commands supported by the device
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
#include "feature_config.h"
#include "product_config.h"

#include "cmd_handler.h"
#include "command_set.h"
#include "cy_debug.h"
#include "cy_memtrack.h"

/****************************************************************************
 * Local Definition
 *****************************************************************************/
typedef bool (*handler_fn)(const cy_message_t* send_p, cy_message_t* reply_p);

#define CRLF                    "\r\n"
#define AT_RSP_OK_DECORATED     CRLF AT_RSP_OK CRLF
#define AT_RSP_ERROR_DECORATED  CRLF AT_RSP_ERROR CRLF

/****************************************************************************
 * Local Data
 *****************************************************************************/
static bool s_is_ate_on = false;

/****************************************************************************
 * Local Functions
 *****************************************************************************/
static bool format_reply(const cy_message_t* send_p,
                         cy_message_t* reply_p,
                         const char* msg_p)
{
    ReturnAssert(msg_p != NULL, false);

    uint16_t rsp_len = strlen(msg_p);
    uint16_t rsp_total_len = (s_is_ate_on? send_p->length : 0) + rsp_len;
    uint16_t i = 0;

    reply_p->buf_p = (uint8_t*)CY_MEMTRACK_MALLOC(rsp_total_len);
    ReturnAssert(reply_p->buf_p != NULL, false);

    reply_p->length = rsp_total_len;
    if (s_is_ate_on) {
        memcpy(reply_p->buf_p, send_p->buf_p, send_p->length);
        i += send_p->length;
    }
    memcpy(&reply_p->buf_p[i], msg_p, rsp_len);

    return true;
}

#if 0 // unused
static bool handler_echo(const cy_message_t* send_p,
                         cy_message_t* reply_p)
{
    reply_p->buf_p = (uint8_t*)CY_MEMTRACK_MALLOC(send_p->length);
    ReturnAssert(reply_p->buf_p != NULL, false);

    reply_p->length = send_p->length;
    memcpy(reply_p->buf_p, send_p->buf_p, reply_p->length);

    return true;
}
#endif

static bool handler_ok(const cy_message_t* send_p,
                           cy_message_t* reply_p)
{
    return (format_reply(send_p,
                         reply_p,
                         AT_RSP_OK_DECORATED));
}

static bool handler_error(const cy_message_t* send_p,
                           cy_message_t* reply_p)
{
    return (format_reply(send_p,
                         reply_p,
                         AT_RSP_ERROR_DECORATED));
}

static bool handler_atdata(const cy_message_t* send_p,
                           cy_message_t* reply_p)
{
    bool result = false;
    bool is_ok = false;
    const char* atdata = AT_CMD_DATA;
    uint16_t min_expected_len = strlen(atdata) + 1; // add 1 for terminating '\r'

    if ((send_p->length >= min_expected_len) &&
        (send_p->buf_p[send_p->length - 1] == '\r')) {
        is_ok = true;
    }

    if (is_ok) {
        result = (handler_ok(send_p,
                             reply_p));
    }
    else {
        result = (handler_error(send_p,
                             reply_p));
    }

    return result;
}


static bool handler_ate_helper(const cy_message_t* send_p,
                               cy_message_t* reply_p,
                               bool set_ate_value)
{
    s_is_ate_on = set_ate_value;

    return (handler_ok(send_p,
                         reply_p));
}

static bool handler_ate0(const cy_message_t* send_p,
                         cy_message_t* reply_p)
{
    return handler_ate_helper(send_p, reply_p, false);
}

static bool handler_ate1(const cy_message_t* send_p,
                         cy_message_t* reply_p)
{
    return handler_ate_helper(send_p, reply_p, true);
}

static bool handler_atcgmm(const cy_message_t* send_p,
                           cy_message_t* reply_p)
{
    return (format_reply(send_p,
                         reply_p,
                         CRLF MY_PRODUCT_NAME AT_RSP_OK_DECORATED));
}

static bool handler_atcgsn(const cy_message_t* send_p,
                           cy_message_t* reply_p)
{
    return (format_reply(send_p,
                         reply_p,
                         CRLF MY_SERIAL_NUMBER AT_RSP_OK_DECORATED));
}


/****************************************************************************
 * Local Data
 *****************************************************************************/
/* Structure used to map command and handler functions */
typedef struct
{
    const char* cmd_p;
    handler_fn  fn;
    bool partial_match;
} command_fn_t;

static command_fn_t s_command_map[] = {
    {AT_CMD_ECHO_OFF,   handler_ate0,       false},
    {AT_CMD_ECHO_ON,    handler_ate1,       false},
    {AT_CMD_MODEL,      handler_atcgmm,     false},
    {AT_CMD_IMEI,       handler_atcgsn,     false},
    {AT_CMD_DATA,       handler_atdata,     true},
};


/****************************************************************************
 * Local Functions
 *****************************************************************************/

// return true if the given buf_p matches an entry in the map
static bool lookup_cmd_handler(const cy_message_t* send_p,
                               handler_fn *fn_p)
{
    int i;
    ReturnAssert(fn_p != NULL, false);

    for (i = 0; i < sizeof(s_command_map)/sizeof(s_command_map[0]); i++) {
        uint16_t expected_len = strlen(s_command_map[i].cmd_p);

        if (((send_p->length == expected_len) ||
             ((send_p->length > expected_len) && s_command_map[i].partial_match)) &&
            (memcmp(s_command_map[i].cmd_p, send_p->buf_p, expected_len) == 0)) {
            *fn_p = s_command_map[i].fn;
            return true;
        }
    }

    return false;
}

/****************************************************************************
 * Public Functions
 *****************************************************************************/

bool process_message(const cy_message_t* send_p,
                     cy_message_t* reply_p)
{
    ReturnAssert(send_p != NULL, false);
    ReturnAssert(reply_p != NULL, false);
    ReturnAssert(send_p->buf_p != NULL, false);

    handler_fn fn = NULL;
    bool result = lookup_cmd_handler(send_p, &fn);

    if (result) {
        result = fn(send_p, reply_p);
    }
    else {
        result = handler_error(send_p, reply_p);
    }

    return result;
}

/* [] END OF FILE */

/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
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


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* WDT time out for reset mode, in milliseconds. Max limit is given by
 * CYHAL_WDT_MAX_TIMEOUT_MS */
#define WDT_TIME_OUT_MS                     4000
#define ENABLE_BLOCKING_FUNCTION            1

/* Match count =  Desired interrupt interval in seconds x ILO Frequency in Hz */
#define WDT_MATCH_COUNT                     (WDT_TIME_OUT_MS*32000)/1000

/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void initialize_wdt();

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Check the reason for device restart */
    if(CY_SYSLIB_RESET_HWWDT == Cy_SysLib_GetResetReason())
    {
        /* It's WDT reset event - blink LED twice */
        Cy_GPIO_Clr(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        Cy_SysLib_Delay(100);
        Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        Cy_SysLib_Delay(200);
        Cy_GPIO_Clr(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        Cy_SysLib_Delay(100);
        Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
    }
    else
    {
        /* It's Power-On reset or XRES event - blink LED once */
        Cy_GPIO_Clr(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        Cy_SysLib_Delay(100);
        Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        Cy_SysLib_Delay(100);
    }

    /* Clears the reset cause register */
    Cy_SysLib_ClearResetReason();

    /* Initialize WDT */
    initialize_wdt();

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
#if (ENABLE_BLOCKING_FUNCTION)
         while(1);
#else
         /* Reset WDT */
         Cy_WDT_ClearWatchdog();

        /* Constant delay of 1000ms */
        Cy_SysLib_Delay(1000);

        /* Invert the state of LED */
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
#endif

    }
}

/*******************************************************************************
* Function Name: InitializeWDT
********************************************************************************
* Summary:
* This function initializes the WDT block
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void initialize_wdt()
{
   /* Step 1- Unlock WDT */
   Cy_WDT_Unlock();

   /* Step 2- Write the ignore bits - operate with only 14 bits */
   Cy_WDT_SetIgnoreBits(16);

   /* Step 3- Write match value */
   Cy_WDT_SetMatch(WDT_MATCH_COUNT);

   /* Step 4- Clear match event interrupt, if any */
   Cy_WDT_ClearInterrupt();

   /* Step 5- Enable WDT */
   Cy_WDT_Enable();

   /* Step 6- Lock WDT configuration */
   Cy_WDT_Lock();
}
/* [] END OF FILE */

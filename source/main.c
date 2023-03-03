/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 MCU SCB UART Transmit and
*              Receive with DMA for ModusToolbox.
*
* Related Document: See README.md
* 
*******************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cycfg.h"
#include "cybsp.h"
#include "UartDma.h"
#include "stdio.h"
#include <inttypes.h>

/*******************************************************************************
*            Macros
*******************************************************************************/
#define UART_INT_PRIORITY     (3u)
#define DMA_IRQ               (cpuss_interrupt_dma_IRQn)
#define DMA_INT_PRIORITY      (3u)
#define BUFFER_SIZE           (1u)

/* Debug print macro to enable UART print */
#define DEBUG_PRINT           (0u)

/*******************************************************************************
*            Forward declaration
*******************************************************************************/
void Isr_UART(void);


/*******************************************************************************
*            Global variables
*******************************************************************************/
/* Variable set by the UART ISR to indicate UART transfer error */
bool uart_error;

/* Variables set by the DMA ISR to indicate DMA transfer status */
extern bool rx_dma_done;
extern bool tx_dma_error;
extern bool rx_dma_error;

/* Structure for UART Context */
cy_stc_scb_uart_context_t CYBSP_UART_context;

#if DEBUG_PRINT
/* Variable used for tracking the print status */
volatile bool ENTER_LOOP = true;

/*******************************************************************************
* Function Name: check_status
********************************************************************************
* Summary:
*  Prints the error message.
*
* Parameters:
*  error_msg - message to print if any error encountered.
*  status - status obtained after evaluation.
*
* Return:
*  void
*
*******************************************************************************/
void check_status(char *message, cy_rslt_t status)
{
    char error_msg[50];

    sprintf(error_msg, "Error Code: 0x%08" PRIX32 "\n", status);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\nFAIL: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, error_msg);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
}
#endif


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
* The main function performs the following actions:
*  1. Configures RX and TX DMAs to handle UART RX+TX direction.
*  2. Configures UART interface.
*  3. Sends text header to the UART serial terminal.
*  5. Waits in an infinite loop (for DMA or UART error interrupt)
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_en_sysint_status_t intr_result;

    /* Variables to hold ping and pong buffer */
    uint8_t rx_dma_uart_buffer_a[BUFFER_SIZE];
    uint8_t rx_dma_uart_buffer_b[BUFFER_SIZE];

    cy_en_scb_uart_status_t init_status;

    /* Flag to control which descriptor to use */
    cy_en_dmac_descriptor_t active_descr = CY_DMAC_DESCRIPTOR_PING;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Halt the CPU if device initialization failed */
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Start UART operation */
    init_status = Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    if (init_status != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable the SCB block for the UART operation */
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Transmit header to the terminal */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "************************************************************\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "PMG1 MCU UART Transmit and Receive using DMA\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "************************************************************\r\n\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, ">> Start typing to see the echo on the screen \r\n\n");

    /* UART interrupt initialization structure  */
    cy_stc_sysint_t CYBSP_UART_INT_cfg =
    {
        .intrSrc      = CYBSP_UART_IRQ,
        .intrPriority = UART_INT_PRIORITY
    };

    /* DMA interrupt initialization structure */
    cy_stc_sysint_t DMA_INT_cfg =
    {
        .intrSrc      = (IRQn_Type)DMA_IRQ,
        .intrPriority = DMA_INT_PRIORITY,
    };

    /* Configure DMA Rx and Tx channels for operation */
    configure_rx_dma(rx_dma_uart_buffer_a, rx_dma_uart_buffer_b);
    configure_tx_dma(rx_dma_uart_buffer_a);

    /* Enable interrupt for TxDma channel and RxDma channel */
    Cy_DMAC_SetInterruptMask(TxDma_HW, TXDMA_CHANNEL_INT_MASK | RXDMA_CHANNEL_INT_MASK);

    /* Initialize and enable the DMA interrupt */
    intr_result = Cy_SysInt_Init(&DMA_INT_cfg, &Isr_DMA);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_SysInt_Init failed with error code", intr_result);
#endif
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    NVIC_EnableIRQ(DMA_INT_cfg.intrSrc);

    /* Initialize and enable the UART interrupt */
    intr_result = Cy_SysInt_Init(&CYBSP_UART_INT_cfg, &Isr_UART);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_SysInt_Init failed with error code", intr_result);
#endif
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    
    NVIC_EnableIRQ(CYBSP_UART_INT_cfg.intrSrc);

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Indicate status if RxDma error or TxDma error or UART error occurs */
        if(uart_error | tx_dma_error | rx_dma_error)
        {
            CY_ASSERT(CY_ASSERT_FAILED);
        }
        /* Handle RxDma complete */
        if(rx_dma_done == true)
        {
            /* Ping Pong between rx_dma_uart_buffer_a and rx_dma_uart_buffer_b */
            if (active_descr == CY_DMAC_DESCRIPTOR_PING)
            {
                /* Set source RX Buffer A as source for TxDMA */
                Cy_DMAC_Descriptor_SetSrcAddress(TxDma_HW, TxDma_CHANNEL,
                                                 CY_DMAC_DESCRIPTOR_PING,
                                                 (void *)rx_dma_uart_buffer_a);

                active_descr = CY_DMAC_DESCRIPTOR_PONG;
            }
            else
            {
                /* Set source RX Buffer B as source for TxDMA */
                Cy_DMAC_Descriptor_SetSrcAddress(TxDma_HW, TxDma_CHANNEL,
                                                 CY_DMAC_DESCRIPTOR_PING,
                                                 (void *)rx_dma_uart_buffer_b);

                active_descr = CY_DMAC_DESCRIPTOR_PING;
            }

            /* Set PING descriptor as current descriptor for TxDma channel  */
            Cy_DMAC_Channel_SetCurrentDescriptor(TxDma_HW, TxDma_CHANNEL,
                                                 CY_DMAC_DESCRIPTOR_PING);
            /* Validate the PING descriptor */
            Cy_DMAC_Descriptor_SetState(TxDma_HW, TxDma_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING, true);
            /* Enable TxDma channel */
            Cy_DMAC_Channel_Enable(TxDma_HW, TxDma_CHANNEL);
            rx_dma_done = false;
        }
#if DEBUG_PRINT
        if (ENTER_LOOP)
        {
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Entered for loop\r\n");
            ENTER_LOOP = false;
        }
#endif
    }
}

/*******************************************************************************
* Function Name: Isr_UART
********************************************************************************
*
* Summary:
* Handles UART Rx underflow and overflow conditions. This conditions must never
* occur.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Isr_UART(void)
{
    uint32_t rx_intr_src;
    uint32_t tx_intr_src;

    /* Get RX interrupt sources */
    rx_intr_src =  Cy_SCB_UART_GetRxFifoStatus(CYBSP_UART_HW);
    Cy_SCB_UART_ClearRxFifoStatus(CYBSP_UART_HW, rx_intr_src);

    /* Get TX interrupt sources */
    tx_intr_src =  Cy_SCB_UART_GetTxFifoStatus(CYBSP_UART_HW);
    Cy_SCB_UART_ClearTxFifoStatus(CYBSP_UART_HW, tx_intr_src);

    /* RX overflow or RX underflow or TX overflow occurred */
    uart_error = 1;
}
/* [] END OF FILE */

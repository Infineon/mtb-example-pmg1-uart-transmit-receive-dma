/******************************************************************************
* File Name: UartDma.c
*
* Description: This file contains all the functions and variables required for
*              proper operation of UART/DMA for this CE
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "UartDma.h"
#include "cy_pdl.h"

/*******************************************************************************
*            Forward declaration
*******************************************************************************/
void handle_error(void);


/*******************************************************************************
*            Global variables
*******************************************************************************/
/* Variables set by the DMA ISR to indicate DMA transfer status */
bool rx_dma_done;
bool tx_dma_error;
bool rx_dma_error;


/*******************************************************************************
* Function Name: configure_rx_dma
********************************************************************************
*
* Summary:
* Configures DMA Rx channel for operation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void configure_rx_dma(uint8_t* buffer_a, uint8_t* buffer_b)
{
    cy_en_dmac_status_t dmac_init_status;

    /* Initialize PING descriptor */
    dmac_init_status = Cy_DMAC_Descriptor_Init(RxDma_HW, RxDma_CHANNEL,
                                               CY_DMAC_DESCRIPTOR_PING,
                                               &RxDma_ping_config);
    if(dmac_init_status != CY_DMAC_SUCCESS)
    {
        handle_error();
    }

    /* Initialize PONG descriptor */
    dmac_init_status = Cy_DMAC_Descriptor_Init(RxDma_HW, RxDma_CHANNEL,
                                               CY_DMAC_DESCRIPTOR_PONG,
                                               &RxDma_pong_config);
    if(dmac_init_status != CY_DMAC_SUCCESS)
    {
        handle_error();
    }

    /* Initialize RxDma channel */
    dmac_init_status = Cy_DMAC_Channel_Init(RxDma_HW, RxDma_CHANNEL,
                                            &RxDma_channel_config);
    if(dmac_init_status != CY_DMAC_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination for PING descriptor */
    Cy_DMAC_Descriptor_SetSrcAddress(RxDma_HW, RxDma_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PING,
                                     (void *) &(KIT_UART_HW->RX_FIFO_RD));
    Cy_DMAC_Descriptor_SetDstAddress(RxDma_HW, RxDma_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PING,(void *) buffer_a);

    /* Set source and destination for PONG descriptor */
    Cy_DMAC_Descriptor_SetSrcAddress(RxDma_HW, RxDma_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PONG,
                                     (void *) &(KIT_UART_HW->RX_FIFO_RD));
    Cy_DMAC_Descriptor_SetDstAddress(RxDma_HW, RxDma_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PONG,(void *) buffer_b);

    /* Validate PING and PONG descriptor of RxDma channel */
    Cy_DMAC_Descriptor_SetState(RxDma_HW, RxDma_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PING, true);
    Cy_DMAC_Descriptor_SetState(RxDma_HW, RxDma_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PONG, true);

    /* Set PING descriptor as current descriptor for RxDma channel  */
    Cy_DMAC_Channel_SetCurrentDescriptor(RxDma_HW, RxDma_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING);

    /* Enable entire DMAC block including the TxDMA  */
    Cy_DMAC_Enable(RxDma_HW);

    /* Enable RxDma channel */
    Cy_DMAC_Channel_Enable(RxDma_HW, RxDma_CHANNEL);
}


/*******************************************************************************
* Function Name: configure_tx_dma
********************************************************************************
*
* Summary:
* Configures DMA Tx channel for operation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void configure_tx_dma(uint8_t* buffer_a)
{
    cy_en_dmac_status_t dmac_init_status;

    /* Initialize PING descriptor */
    dmac_init_status = Cy_DMAC_Descriptor_Init(TxDma_HW, TxDma_CHANNEL,
                                               CY_DMAC_DESCRIPTOR_PING,
                                               &TxDma_ping_config);
    if(dmac_init_status != CY_DMAC_SUCCESS)
    {
        handle_error();
    }

    /* Initialize TxDma channel */
    dmac_init_status = Cy_DMAC_Channel_Init(TxDma_HW, TxDma_CHANNEL,
                                            &TxDma_channel_config);
    if(dmac_init_status != CY_DMAC_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination for PING descriptor */
    Cy_DMAC_Descriptor_SetSrcAddress(TxDma_HW, TxDma_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PING,
                                     (void *) buffer_a);
    Cy_DMAC_Descriptor_SetDstAddress(TxDma_HW, TxDma_CHANNEL,
                                     CY_DMAC_DESCRIPTOR_PING,
                                     (void *) &KIT_UART_HW->TX_FIFO_WR);

   /* Validate the PING descriptor */
    Cy_DMAC_Descriptor_SetState(TxDma_HW, TxDma_CHANNEL,
                                CY_DMAC_DESCRIPTOR_PING, true);

    /* Set PING descriptor as current descriptor for TxDma channel  */
    Cy_DMAC_Channel_SetCurrentDescriptor(TxDma_HW, TxDma_CHANNEL,
                                         CY_DMAC_DESCRIPTOR_PING);
}
/*******************************************************************************
* Function Name: Isr_DMA
********************************************************************************
*
* Summary:
*  Handles Rx Dma descriptor completion interrupt source: triggers Tx Dma to
*  transfer back data received by the Rx Dma descriptor.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Isr_DMA(void)
{
    cy_en_dmac_descriptor_t descriptor;
    cy_en_dmac_response_t dmac_response;

    /* Check if interrupt is set for rxDma channel */
    if((Cy_DMAC_GetInterruptStatusMasked(RxDma_HW) & RXDMA_CHANNEL_INT_MASK) != 0)
    {
        /* Find which is the active descriptor for RxDma channel */
        descriptor = Cy_DMAC_Channel_GetCurrentDescriptor(RxDma_HW, RxDma_CHANNEL);

        /* Check if the RxDma channel response is successful for current transfer */
        if(CY_DMAC_DONE == Cy_DMAC_Descriptor_GetResponse(RxDma_HW, RxDma_CHANNEL,
                                                          (cy_en_dmac_descriptor_t) !descriptor))
        {
            rx_dma_done = true;
        }
        else
        {
            rx_dma_error = true;
        }

        /* Clear RxDma channel interrupt */
        Cy_DMAC_ClearInterrupt(RxDma_HW, RXDMA_CHANNEL_INT_MASK);
    }

    /* Check if interrupt is set for TxDma channel */
    if((Cy_DMAC_GetInterruptStatusMasked(TxDma_HW) & TXDMA_CHANNEL_INT_MASK) != 0)
    {
        dmac_response = Cy_DMAC_Descriptor_GetResponse(TxDma_HW, TxDma_CHANNEL,
                                                       CY_DMAC_DESCRIPTOR_PING);

        /* Check if the RxDma channel response is successful for current
         * transfer. Note that current descriptor is set to invalid after
         * completion */
        if((dmac_response != CY_DMAC_DONE) &&
           (dmac_response != CY_DMAC_INVALID_DESCR))
        {
            tx_dma_error = true;
        }

        /* Clear TxDma channel interrupt */
        Cy_DMAC_ClearInterrupt(TxDma_HW, TXDMA_CHANNEL_INT_MASK);
    }
}

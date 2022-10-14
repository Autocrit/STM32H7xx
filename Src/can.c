/*

  can.c - CAN bus driver code for STM32H7xx ARM processors,
          using FDCAN1 peripheral in Classic mode.

  Part of grblHAL

  Copyright (c) 2022-2024 Jon Escombe

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "can.h"
#include "grbl/hal.h"

#if CANBUS_ENABLE

/*
 * Static function prototypes
 */

/*
 * Static variables
 */
static FDCAN_HandleTypeDef hfdcan;
static FDCAN_FilterTypeDef sFilterConfig;
static FDCAN_TxHeaderTypeDef TxHeader;
static FDCAN_RxHeaderTypeDef RxHeader;
static uint8_t TxData[8];
static uint8_t RxData[8];
static bool rxPendingData;

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan);
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *                     This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

#ifdef CAN_QUEUE_RX_IN_IRQ
      can_get();
#else
      rxPendingData = true;
      HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#endif

  }

}

bool can_rx_pending(void)
{
    return rxPendingData;
}

void can_get(void)
{
    /* May either be called in interrupt context from the RX fifo callback, or from the polling
     * loop of the higher level canbus driver, depending whether CAN_QUEUE_RX_IN_IRQ is defined.
     */

    canbus_message_t message;

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0)) {

        printf("can_get(), adding new data to RX queue..\n");
        if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &RxHeader,RxData) == HAL_OK) {

            /* Attempt to add incoming message to the RX queue.
             *
             * Note: not currently checking for success, if there is no space available we
             * would just end up dropping messages somewhere else (i.e. in the CAN RX fifo)..
             */
            message.id = RxHeader.Identifier;
            // DLC has to be right shifted by 16bits for the FDCAN driver
            message.len = RxHeader.DataLength >> 16;
            memcpy(message.data, RxData, message.len);

            canbus_queue_rx(message);
        }
    }
#ifndef CAN_QUEUE_RX_IN_IRQ
    rxPendingData = false;
    HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
#endif
}

uint8_t can_put(canbus_message_t message)
{
    // DLC has to be left shifted by 16bits for the FDCAN driver, (or use one of the FDCAN_DLC_BYTES_n defines)..
    TxHeader.Identifier = message.id;
    TxHeader.DataLength = message.len << 16;

    memcpy(TxData, message.data, message.len);

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, TxData) != HAL_OK)
    {
        printf("can_put(), error sending message..\n");
        return (0);
    }

    return (1);
}

uint8_t can_stop(void)
{
    HAL_FDCAN_Stop(&hfdcan);
    HAL_FDCAN_DeInit(&hfdcan);

    return (0);
}

uint8_t can_start(uint32_t baud)
{
    uint8_t unknown_rate = 0;

    /* Initialisation */
    hfdcan.Instance = FDCAN1;
    hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan.Init.AutoRetransmission = DISABLE;
    hfdcan.Init.TransmitPause = DISABLE;
    hfdcan.Init.ProtocolException = DISABLE;
    hfdcan.Init.MessageRAMOffset = 0;
    hfdcan.Init.StdFiltersNbr = 1;
    hfdcan.Init.ExtFiltersNbr = 0;
    hfdcan.Init.RxFifo0ElmtsNbr = 1;
    hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan.Init.RxFifo1ElmtsNbr = 0;
    hfdcan.Init.RxBuffersNbr = 0;
    hfdcan.Init.TxEventsNbr = 0;
    hfdcan.Init.TxBuffersNbr = 0;
    hfdcan.Init.TxFifoQueueElmtsNbr = 1;
    hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    /*
     * Can bit time calculations taken from http://www.bittiming.can-wiki.info/ - pre-calculated
     * for supported CAN peripheral clock speeds and baud rates.
     *
     * Unable to query the FDCAN peripheral clock speed until it's initialised, so getting clock speed
     * directly from the RCC_PLL1_DIVQ clock source. This is 48MHz for all currently supported boards..
     */
    PLL1_ClocksTypeDef pll1_clocks;
    HAL_RCCEx_GetPLL1ClockFreq(&pll1_clocks);
    uint32_t pll1_q_freq = pll1_clocks.PLL1_Q_Frequency;
    printf("can_start(), PLL1_Q frequency: %lu\n", pll1_q_freq);

    switch (pll1_q_freq) {

        case 48000000:
            /* FDCAN peripheral running with 48MHz clock, calculated bit timings for all supported baud rates */
            switch (baud) {

                case 125000:
                    hfdcan.Init.NominalPrescaler = 24;
                    hfdcan.Init.NominalTimeSeg1 = 13;
                    hfdcan.Init.NominalTimeSeg2 = 2;
                    hfdcan.Init.NominalSyncJumpWidth = 1;
                    break;

                case 250000:
                    hfdcan.Init.NominalPrescaler = 12;
                    hfdcan.Init.NominalTimeSeg1 = 13;
                    hfdcan.Init.NominalTimeSeg2 = 2;
                    hfdcan.Init.NominalSyncJumpWidth = 1;
                    break;

                case 500000:
                    hfdcan.Init.NominalPrescaler = 6;
                    hfdcan.Init.NominalTimeSeg1 = 13;
                    hfdcan.Init.NominalTimeSeg2 = 2;
                    hfdcan.Init.NominalSyncJumpWidth = 1;
                    break;

                case 1000000:
                    hfdcan.Init.NominalPrescaler = 3;
                    hfdcan.Init.NominalTimeSeg1 = 13;
                    hfdcan.Init.NominalTimeSeg2 = 2;
                    hfdcan.Init.NominalSyncJumpWidth = 1;
                    break;

                default:
                    /* Unsupported baud rate */
                    unknown_rate = 1;
                    break;
            }
            break;

        default:
            /* unsupported CAN peripheral clock frequency */
            unknown_rate = 1;
            break;
    }


    if (unknown_rate) {
        printf("can_start(), error - unable to calculate bit timings\n");
        return(0);
    }

    if (HAL_FDCAN_Init(&hfdcan) != HAL_OK)
    {
        return(0);
    }

    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig) != HAL_OK)
    {
        return(0);
    }

    /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* Start the CAN peripheral (calls the MspInit function) */
    if(HAL_FDCAN_Start(&hfdcan) != HAL_OK)
    {
        return(0);
    }

    /* Add the callback for received data */
    if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        return(0);
    }

    /* Prepare Tx Header */
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    /* Successfully initialised */
    return(1);
}

/**
* @brief FDCAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hfdcan: FDCAN handle pointer
* @retval None
*/
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if(hfdcan->Instance==FDCAN1)
    {
        /* USER CODE BEGIN FDCAN1_MspInit 0 */

        /* USER CODE END FDCAN1_MspInit 0 */

        /** Initializes the peripherals clock
        */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_FDCAN_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**FDCAN1 GPIO Configuration
        PD0     ------> FDCAN1_RX
        PD1     ------> FDCAN1_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* FDCAN1 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

        /* USER CODE BEGIN FDCAN1_MspInit 1 */

        static const periph_pin_t rx = {
            .function = Input_RX,
            .group = PinGroup_CAN,
            .port = GPIOD,
            .pin = 0,
            .mode = { .mask = PINMODE_NONE },
            .description = "CAN"
        };

        static const periph_pin_t tx = {
            .function = Output_TX,
            .group = PinGroup_CAN,
            .port = GPIOD,
            .pin = 1,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "CAN"
        };

        hal.periph_port.register_pin(&rx);
        hal.periph_port.register_pin(&tx);

        /* USER CODE END FDCAN1_MspInit 1 */
    }
}

/**
* @brief FDCAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hfdcan: FDCAN handle pointer
* @retval None
*/
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
    if(hfdcan->Instance==FDCAN1)
    {
        /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

        /* USER CODE END FDCAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_FDCAN_CLK_DISABLE();

        /**FDCAN1 GPIO Configuration
        PD0     ------> FDCAN1_RX
        PD1     ------> FDCAN1_TX
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

        /* FDCAN1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

        /* USER CODE END FDCAN1_MspDeInit 1 */
    }
}

#endif


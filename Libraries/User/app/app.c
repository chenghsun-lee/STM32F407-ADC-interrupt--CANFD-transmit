/*******************************************************************************
  Application:  Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    Implementation of application.

  Description:
    .
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and 
any derivatives exclusively with Microchip products. It is your responsibility 
to comply with third party license terms applicable to your use of third party 
software (including open source software) that may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, 
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES 
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER 
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF 
THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED 
BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID 
DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
//DOM-IGNORE-END


// Include files
#include "app.h"
#include "drv_canfdspi_api.h"
#include "drv_spi.h"
#include "stdio.h"
#include "main.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

APP_DATA appData;

CAN_CONFIG config;
CAN_OPERATION_MODE opMode;

// Transmit objects
CAN_TX_FIFO_CONFIG txConfig;
CAN_TX_FIFO_EVENT txFlags;
CAN_TX_MSGOBJ txObj;
uint8_t txd[MAX_DATA_BYTES];



REG_t reg;

APP_Payload payload;

uint8_t i;

CAN_BUS_DIAGNOSTIC busDiagnostics;
uint8_t tec;
uint8_t rec;
CAN_ERROR_STATE errorFlags;



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void APP_Initialize(void)
{
    DRV_SPI_Initialize();

		GPIO_InitTypeDef GPIO_InitStructure;
	
		//20230726 YW
		//not sure 
		RCC_AHB1PeriphClockCmd( RCC_INT_IN|RCC_INT_TX_IN|RCC_INT_RX_IN|RCC_LED0_PIN|RCC_S1_PIN , ENABLE); 	
		//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);	//RCC_APB1PeriphClockCmd( RCC_APB1Periph_AFIO , ENABLE); 	
		//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
		/**
		*	INT-> PB6 PB8 PB9
		*/					 
		GPIO_InitStructure.GPIO_Pin = INT_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		//20230726 YW
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		
		GPIO_Init(PORT_INT_IN, &GPIO_InitStructure);	

	
		GPIO_InitStructure.GPIO_Pin = INT_TX_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		//20230726 YW
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			
		GPIO_Init(PORT_INT_TX_IN, &GPIO_InitStructure);	
	
	
		GPIO_InitStructure.GPIO_Pin = INT_RX_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		//20230726 YW
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(PORT_INT_RX_IN, &GPIO_InitStructure);	
		
		
#ifdef TEST_SPI
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

    appData.state = APP_STATE_TEST_RAM_ACCESS;

#else
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
#endif

}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void APP_CANFDSPI_Init(void)
{
    // Reset device
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);
	
	
    // Enable ECC and initialize RAM
    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);

    DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);

    // Configure device
    DRV_CANFDSPI_ConfigureObjectReset(&config);
      config.IsoCrcEnable = 1;
      config.StoreInTEF = 0;
	
    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);

    // Setup TX FIFO
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
    txConfig.FifoSize = 7;
    txConfig.PayLoadSize = CAN_PLSIZE_64;
    txConfig.TxPriority = 1;

    DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txConfig);
		
		
    // Setup Bit Time
		//		 DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);
	// DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
		 DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0,  CAN_1000K_8M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
		 
	
						
    // Setup Transmit and Receive Interrupts
    DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
	#ifdef APP_USE_TX_INT
    DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
	#endif
    DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);

    // Select Normal Mode
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
//	DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_CLASSIC_MODE);


}


void APP_TransmitMessageQueue(void)
{

    uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;

    // Check if FIFO is not full
    do {
        DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txFlags);
        if (attempts == 0) {
            Nop();
            Nop();
            DRV_CANFDSPI_ErrorCountStateGet(DRV_CANFDSPI_INDEX_0, &tec, &rec, &errorFlags);
            return;
        }
        attempts--;
    }
    while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));

    // Load message and transmit
    uint8_t n = DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC);

    DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txObj, txd, n, true);

}

extern int ADC_Result;
void APP_TransmitSwitchState(void)
{

			txObj.bF.id.SID = BUTTON_STATUS_ID;  //0x201
      txObj.bF.ctrl.DLC = CAN_DLC_8;
      txObj.bF.ctrl.IDE = 0;
      txObj.bF.ctrl.BRS = 1;
      txObj.bF.ctrl.FDF = 1;
	
      txd[0] = ADC_Result >> 8 ;
	    txd[1] = ADC_Result & 0xFF;

			APP_TransmitMessageQueue();

}



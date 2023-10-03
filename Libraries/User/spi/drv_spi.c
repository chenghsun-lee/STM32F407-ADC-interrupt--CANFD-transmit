/*******************************************************************************
  SPI Driver:  Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi.c

  Summary:
    Implementation of MCU specific SPI functions.

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
#include "drv_spi.h"
//20230726 YW
#include "stm32f4xx.h"
#include "stdint.h"


//20230726 YW 
#define RCC_APB2Periph_GPIO_SPI_FLASH_CS      RCC_AHB1Periph_GPIOA
#define SPI_FALSH_CS_PORT                     GPIOA
#define SPI_FALSH_CS_PIN                      GPIO_Pin_4

#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(SPI_FALSH_CS_PORT, SPI_FALSH_CS_PIN)
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(SPI_FALSH_CS_PORT, SPI_FALSH_CS_PIN)

/* Local function prototypes */
inline void spi_master_init(void);
inline int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);

void DRV_SPI_Initialize(void)
{
	spi_master_init();
}

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
	return spi_master_transfer(SpiTxData, SpiRxData, spiTransferSize);
}

void spi_master_init(void)
{
	
	SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//20230814 SPI1 to SPI3
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_APB2Periph_GPIO_SPI_FLASH_CS, ENABLE); 
	
	
	//20230814 SPI1 to SPI3
	#if 0
	//20230801
	GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOA);
  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);    
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	#endif

	GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOC);
  
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);    
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	
	//20230814 SPI1 to SPI3
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	//20230726
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;	
	//20230814 SPI1 to SPI3
  //GPIO_Init(GPIOA, &GPIO_InitStructure);	
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
  GPIO_InitStructure.GPIO_Pin = SPI_FALSH_CS_PIN;	
	//20230726 
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_FALSH_CS_PORT, &GPIO_InitStructure);
	
	
	SPI_FLASH_CS_LOW();
	
	//SPI_I2S_DeInit(SPI1); //20230801
	//SPI_I2S_DeInit(SPI3);
  /* SPI3 Config -------------------------------------------------------------*/ 								  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//20230814 SPI1 to SPI3
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	//20230814 SPI1 to SPI3
  //SPI_Init(SPI1, &SPI_InitStructure);
  //SPI_Cmd(SPI1, ENABLE); 
	SPI_Init(SPI3, &SPI_InitStructure);
  SPI_Cmd(SPI3, ENABLE); 
}

//#define USE_SPI_FUNCTIONS

int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
	uint16_t pos = 0;
	
		//SPI_FLASH_CS_HIGH();
		 SPI_FLASH_CS_LOW();
	
	while(pos < spiTransferSize)
	{
				/* Loop while DR register in not emplty */
			while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);

			/* Send byte through the SPI2 peripheral */
			SPI_I2S_SendData(SPI3, SpiTxData[pos]);
						
			/* Wait to receive a byte */
			while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);

			/* Return the byte read from the SPI bus */
				 
			SpiRxData[pos] =  SPI_I2S_ReceiveData(SPI3);
			
			pos++;
		
	}
	
	SPI_FLASH_CS_HIGH();
	
	
	return 0;
}


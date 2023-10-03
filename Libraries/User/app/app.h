/*******************************************************************************
  Application:  Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file contains the definitions and declarations for this simple application.

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


#ifndef APP_H_
#define APP_H_

// Include files
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "drv_canfdspi_api.h"


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

//#define TEST_SPI

//! Use RX and TX Interrupt pins to check FIFO status
#define APP_USE_RX_INT

#define MAX_TXQUEUE_ATTEMPTS 50

// Switches
#define APP_DEBOUNCE_TIME  100
#define APP_SWITCH_PRESSED  false
#define APP_SWITCH_RELEASED true

#define RCC_S1_PIN     RCC_AHB1Periph_GPIOB 
#define PORT_S1_PIN     GPIOB
#define S1_PIN 					GPIO_Pin_12   
#define APP_S1_READ() GPIO_ReadInputDataBit(PORT_S1_PIN,S1_PIN)

#define	LED_COUNT			 (1)

// LEDs
#define APP_N_LED LED_COUNT

// Special LEDs
#define APP_INIT_LED    0
#define APP_LED_D1		0
#define APP_TX_LED      0

#define APP_LED_TIME    50000

//20230726 YW
#define RCC_LED0_PIN      RCC_AHB1Periph_GPIOD//RCC_APB2Periph_GPIOA
#define PORT_LED0_PIN     GPIOD//GPIOA
#define LED0_PIN 					GPIO_Pin_12//GPIO_Pin_15                 
#define LED_Off(LED0) 		GPIO_SetBits(PORT_LED0_PIN,LED0##_PIN);
#define LED_On(LED0) 			GPIO_ResetBits(PORT_LED0_PIN,LED0##_PIN);



//20230726 YW
#define RCC_INT_IN      	 RCC_AHB1Periph_GPIOC//RCC_APB2Periph_GPIOC
#define PORT_INT_IN      	 GPIOC


//20230726 YW
#define RCC_INT_TX_IN      RCC_AHB1Periph_GPIOC//RCC_APB2Periph_GPIOC
#define PORT_INT_TX_IN     GPIOC


//20230726 YW
#define RCC_INT_RX_IN      RCC_AHB1Periph_GPIOC//RCC_APB2Periph_GPIOC
#define PORT_INT_RX_IN		 GPIOC

// Interrupts
#define INT_IN			GPIO_Pin_4
#define INT_TX_IN		GPIO_Pin_1 
#define INT_RX_IN		GPIO_Pin_0 

#define APP_INT()		 (!GPIO_ReadInputDataBit(PORT_INT_IN,INT_IN))
#define APP_TX_INT()	(!GPIO_ReadInputDataBit(PORT_INT_TX_IN,INT_TX_IN))
#define APP_RX_INT()	(!GPIO_ReadInputDataBit(PORT_INT_RX_IN,INT_RX_IN))


// Message IDs
#define TX_REQUEST_ID       0x300
#define TX_RESPONSE_ID      0x301
#define BUTTON_STATUS_ID    0x201
#define LED_STATUS_ID       0x200
#define PAYLOAD_ID          0x101

// Transmit Channels
#define APP_TX_FIFO CAN_FIFO_CH2

// Receive Channels
#define APP_RX_FIFO CAN_FIFO_CH1

// Switch states

typedef struct {
	bool S1;
} APP_SwitchState;

// Payload

typedef struct {
	bool On;
	uint8_t Dlc;
	bool Mode;
	uint8_t Counter;
	uint8_t Delay;
	bool BRS;
} APP_Payload;

// *****************************************************************************

/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
 */

typedef enum {
    // Initialization
    APP_STATE_INIT = 0,
    APP_STATE_INIT_TXOBJ,

    // POR signaling
    APP_STATE_FLASH_LEDS,

    // Transmit and Receive
    APP_STATE_TRANSMIT,
    APP_STATE_RECEIVE,
    APP_STATE_PAYLOAD,

    // Test SPI access
    APP_STATE_TEST_RAM_ACCESS,
    APP_STATE_TEST_REGISTER_ACCESS,

    // Switch monitoring
    APP_STATE_SWITCH_CHANGED
} APP_STATES;


// *****************************************************************************

/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct {
    /* The application's current state */
    APP_STATES state;

    /* TODO: Define any additional data used by the application. */


} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

//! Application Initialization

void APP_Initialize(void);

//! Application Tasks
// This routine must be called from SYS_Tasks() routine.

void APP_Tasks(void);

//! Write LEDs based on input byte

void APP_LED_Write(uint8_t led);

//! Clear One LED

void APP_LED_Clear(uint8_t led);

//! Set One LED

void APP_LED_Set(uint8_t led);

//! Initialize CANFDSPI
void APP_CANFDSPI_Init(void);

//! Add message to transmit FIFO
void APP_TransmitMessageQueue(void);

//! Decode received messages
APP_STATES APP_ReceiveMessage_Tasks(void);

//! Transmit switch state
void APP_TransmitSwitchState(void);

//! Periodically send message with requested payload
void APP_PayLoad_Tasks(void);

//! Test SPI access
bool APP_TestRegisterAccess(void);

//! Test RAM access
bool APP_TestRamAccess(void);

#endif /* APP_H_ */

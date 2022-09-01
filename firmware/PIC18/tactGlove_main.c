/****************************************************************
 * capatest pcb usb test file
 * File:		main.c											*
 ****************************************************************
 * Original:    C. Schuermann	/ Bielefeld University				*
 * Date:        26.08.2010										*
 ****************************************************************
 * MCU:  A      PIC18F14j50                             			*
 * Osc. Freq.: 12 MHz (external Quartz)								*
 ****************************************************************
 * PORTA.0				- LED
 * PORTA.1				- CS7
 * PORTA.2				- CS1 					*
 * PORTA.3				- NC
 * PORTA.5              - NC
 * PORTA.6				- CS2
 * PORTA.7              - OSC

 * PORTB.0				- CS5						*
 * PORTB.1    			- CS4
 * PORTB.2              - CS3						*
 * PORTB.3              - CS6
 * PORTB.4              - SCK						*
 * PORTB.5              - SDI
 * PORTB.6              - PGC						*
 * PORTB.7              - PGD
 * PORTB.8              - 						*


 * PORTC.0              - CS8						*
 * PORTC.1              - CS9
 * PORTC.2              - nc
 * PORTC.4              - D-
 * PORTC.5              - D+
 * PORTC.6              - nc
 * PORTC.7              - SDO
**************************************************************/

#include <stdlib.h>
#include <delays.h>
#include <timers.h>
#include <stdio.h>
#include <spi.h>
#include <usart.h>
#include <stdlib.h>
#include <delays.h>
#include <timers.h>
#include <stdio.h>
#include "USB/usb.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"

#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb_ch9.h"
#include "USB/usb_common.h"
#include "USB/usb_function_cdc.h"

#include "USB/usb_device.h"
#include "USB/usb_hal.h"
#include "USB/usb_hal_pic18.h"

#pragma config WDTEN = OFF, PLLDIV = 3, STVREN = OFF, XINST = OFF
#pragma config CPUDIV = OSC1, T1DIG = OFF, OSC = ECPLL, LPT1OSC = OFF
#pragma config FCMEN = OFF, IESO = OFF, DSBOREN = OFF, DSWDTEN = OFF
#pragma config IOL1WAY = OFF

// 4xPLL = 48 Mhz speed

#define LED LATAbits.LATA0

#define TIMER_START_SEC 0  // 0x2472  //0xBFFF

#define CS_1 LATAbits.LATA2
#define CS_2 LATAbits.LATA6
#define CS_3 LATBbits.LATB2
#define CS_4 LATBbits.LATB1
#define CS_5 LATBbits.LATB0
#define CS_6 LATBbits.LATB3

#define CS_7 LATAbits.LATA1
#define CS_8 LATCbits.LATC0
#define CS_9 LATCbits.LATC1

#define AUTOMATIC_SEQUENCE 0x0831 << 4
#define WRITE_REG 0x0831 << 4

void timer0_handler(void);

/** V A R I A B L E S ********************************************************/
#pragma udata

char USB_Head_Buffer[2];
char USB_Out_Buffer[10];
char ledr = 1;
char transmit = 1;
/** P R I V A T E  P R O T O T Y P E S ***************************************/

void ProcessIO(void);
void USBDeviceTasks(void);

#pragma code high_vector = 0x0008
void high_interrupt(void)
{
	_asm GOTO timer0_handler _endasm
}

#pragma code

#pragma interrupt timer0_handler
void timer0_handler(void)
{
	INTCONbits.TMR0IF = 0;
	WriteTimer0(TIMER_START_SEC);
	if (LED == 1)
		LED = 0;
	else
		LED = 1;
	transmit = 1;
	/*	if(ledr==1){
	      LED	= 0;
	      ledr=0;
	   } else {
	       LED=1;
	      ledr=1;
	   }*/
}

/* Main program */
#pragma code

unsigned int spi_transfer(unsigned int nr, unsigned short data)
{
	unsigned char a = (0xFF00 & data) >> 8;
	unsigned char b = (0xFF & data);
	unsigned short a1, b1;

	CS_3 = 1;
	CS_4 = 1;
	CS_9 = 1;
	CS_6 = 1;

	if (nr == 0)
		CS_3 = 0;
	if (nr == 1)
		CS_4 = 0;
	if (nr == 2)
		CS_9 = 0;
	if (nr == 3)
		CS_6 = 0;

	putcSPI(a);
	while (!DataRdySPI()) {
	}
	a1 = SSP1BUF;

	putcSPI(b);
	while (!DataRdySPI()) {
	}
	b1 = SSP1BUF;

	CS_3 = 1;
	CS_4 = 1;
	CS_9 = 1;
	CS_6 = 1;

	data = ((a1 << 8) & 0xFF00) | b1;

	return data;
}

void main(void)
{
	unsigned int pll_startup_counter = 1600;
	// Init ports
	// 1=in, 0=output
	unsigned char adc_nr = 0;

	OSCTUNEbits.PLLEN = 1;  // Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
	while (pll_startup_counter--)
		;

	TRISA = 0b0;  // all out
	TRISB = 0b00100000;  //
	TRISC = 0b0;  //

	//	RPINR6 = 0;

	OpenTimer0(TIMER_INT_ON & T0_SOURCE_INT & T0_16BIT & T0_PS_1_1);
	WriteTimer0(TIMER_START_SEC);
	INTCONbits.GIE = 1;

	LED = 1;

	CS_1 = 1;
	CS_2 = 1;
	CS_3 = 1;
	CS_4 = 1;
	CS_5 = 1;
	CS_6 = 1;
	CS_7 = 1;
	CS_8 = 1;
	CS_9 = 1;

	CS_1 = 0;
	CS_2 = 0;
	CS_3 = 0;
	CS_4 = 0;
	CS_5 = 0;
	CS_6 = 0;
	CS_7 = 0;
	CS_8 = 0;
	CS_9 = 0;

	OpenSPI(SPI_FOSC_16, MODE_00, SMPMID);

	while (adc_nr < 4) {
		spi_transfer(adc_nr, 0xFFFF);
		spi_transfer(adc_nr, (WRITE_REG | (0 << 10)));
		//		spi_transfer(adc_nr, AUTOMATIC_SEQUENCE);
		//		spi_transfer(adc_nr, AUTOMATIC_SEQUENCE);

		adc_nr++;
	}
	adc_nr = 0;

	USB_Out_Buffer[0] = 59;
	USB_Out_Buffer[1] = 1;
	USB_Out_Buffer[2] = 0;
	USB_Out_Buffer[3] = 0;
	USB_Out_Buffer[4] = 0;
	USB_Out_Buffer[5] = 0;

	USBDeviceInit();

	while (1) {
		// mainloop
		USBDeviceTasks();
		if (transmit) {
			ProcessIO();

			transmit = 1;
		}
	}
}

#define PUTHEADER 0
#define VALUES 1
void ProcessIO(void)
{
	unsigned short result;
	static char state = VALUES;
	static char count = -1;
	static char adc_nr = 0;
	static short adcchannel = 0;
	char overflow = 0;

	// User Application USB tasks

	if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
		return;

	if (state == PUTHEADER) {
		if (mUSBUSARTIsTxTrfReady()) {
			putsUSBUSART(USB_Head_Buffer);
			state = VALUES;
		}
	} else {
		if (mUSBUSARTIsTxTrfReady()) {  // Wait for completion

			USB_Out_Buffer[0]++;
			count++;
			if (count >= 64) {  // max channels
				count = 0;
				USB_Out_Buffer[0] = 60;  // start character
			}

			//	if (count<=11) result = read_register(1, (0x0b+count) ); else
			//	result = read_register(2, (0x0b+count-12) );
			adcchannel++;
			if (adcchannel == 16) {
				adcchannel = 0;
				overflow = 1;
			}
			result = spi_transfer(adc_nr, (WRITE_REG | (adcchannel << 10)));

			if (overflow == 1) {
				overflow = 0;
				adc_nr++;
				if (adc_nr == 4)
					adc_nr = 0;
			}

			USB_Out_Buffer[3] = result & 0xFF;
			USB_Out_Buffer[2] = (result & 0xFF00) >> 8;

			putUSBUSART(USB_Out_Buffer, 5);
			LED = !LED;
		}
	}

	CDCTxService();
}  // end ProcessIO

// class spefici request
void USBCBCheckOtherReq(void)
{
	USBCheckCDCRequest();
}  // end

// Init EPS
void USBCBInitEP(void)
{
	CDCInitEP();
}

// USB Event Handler
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
	switch (event) {
		case EVENT_CONFIGURED:
			USBCBInitEP();
			break;
		case EVENT_SET_DESCRIPTOR:
			// USBCBStdSetDscHandler();
			break;
		case EVENT_EP0_REQUEST:
			USBCBCheckOtherReq();
			break;
		case EVENT_SOF:
			// USBCB_SOF_Handler();
			break;
		case EVENT_SUSPEND:
			// USBCBSuspend();
			break;
		case EVENT_RESUME:
			// USBCBWakeFromSuspend();
			break;
		case EVENT_BUS_ERROR:
			// USBCBErrorHandler();
			break;
		case EVENT_TRANSFER:
			Nop();
			break;
		default:
			break;
	}
	return TRUE;
}

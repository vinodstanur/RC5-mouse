/* Name: main.c
 * Project: TV remote controlled HID-mouse 
 * Author: Vinod S (http://blog.vinu.co.in)
 * (Modified Christian Starkjohann's hid mouse example code seen in VUSB project example folder)
 * Creation Date: 12/07/2012
 * License: GNU GPL v2
 */
/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>	/* for sei() */
#include <util/delay.h>		/* for _delay_ms() */
#include <avr/pgmspace.h>	/* required by usbdrv.h */
#include "usbdrv.h"

static uint16_t RC5_DATA;
static uchar RC5_EVENT, RC5_CLICK_EVENT, MOUSE_POINTER_VELOCITY =
    3, RC5_PREVIOUS_DATA = 0;

PROGMEM char usbHidReportDescriptor[52] = {	/* USB report descriptor, size must match usbconfig.h */
	0x05, 0x01,		// USAGE_PAGE (Generic Desktop)
	0x09, 0x02,		// USAGE (Mouse)
	0xa1, 0x01,		// COLLECTION (Application)
	0x09, 0x01,		//   USAGE (Pointer)
	0xA1, 0x00,		//   COLLECTION (Physical)
	0x05, 0x09,		//     USAGE_PAGE (Button)
	0x19, 0x01,		//     USAGE_MINIMUM
	0x29, 0x03,		//     USAGE_MAXIMUM
	0x15, 0x00,		//     LOGICAL_MINIMUM (0)
	0x25, 0x01,		//     LOGICAL_MAXIMUM (1)
	0x95, 0x03,		//     REPORT_COUNT (3)
	0x75, 0x01,		//     REPORT_SIZE (1)
	0x81, 0x02,		//     INPUT (Data,Var,Abs)
	0x95, 0x01,		//     REPORT_COUNT (1)
	0x75, 0x05,		//     REPORT_SIZE (5)
	0x81, 0x03,		//     INPUT (Const,Var,Abs)
	0x05, 0x01,		//     USAGE_PAGE (Generic Desktop)
	0x09, 0x30,		//     USAGE (X)
	0x09, 0x31,		//     USAGE (Y)
	0x09, 0x38,		//     USAGE (Wheel)
	0x15, 0x81,		//     LOGICAL_MINIMUM (-127)
	0x25, 0x7F,		//     LOGICAL_MAXIMUM (127)
	0x75, 0x08,		//     REPORT_SIZE (8)
	0x95, 0x03,		//     REPORT_COUNT (3)
	0x81, 0x06,		//     INPUT (Data,Var,Rel)
	0xC0,			//   END_COLLECTION
	0xC0,			// END COLLECTION
};

/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */
typedef struct {
	uchar buttonMask;
	char dx;
	char dy;
	char dWheel;
} report_t;

static report_t reportBuffer;
static uchar idleRate;		/* repeat rate for keyboards, never used for mice */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;

	/* The following requests are never used. But since they are required by
	 * the specification, we implement them in this example.
	 */
	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {	/* class request type */
		if (rq->bRequest == USBRQ_HID_GET_REPORT) {	/* wValue: ReportType (highbyte), ReportID (lowbyte) */
			/* we only have one report type, so don't look at wValue */
			usbMsgPtr = (void *)&reportBuffer;
			return sizeof(reportBuffer);
		} else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
			usbMsgPtr = &idleRate;
			return 1;
		} else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
			idleRate = rq->wValue.bytes[1];
		}
	} else {
		/* no vendor specific requests implemented */
	}
	return 0;		/* default for not implemented requests: return no data back to host */
}

/*RC5 decoding routine - ISR */
ISR(INT1_vect)
{
	if ((PIND & (1 << PD3)) != 0)
		return;
	_delay_us(10);
	if ((PIND & (1 << PD3)) != 0)
		return;
	_delay_us(10);
	if ((PIND & (1 << PD3)) != 0)
		return;
	_delay_us(10);
	char i = 0;
	RC5_DATA = 0;
	OCR1A = ((double)F_CPU / 1000000) * 1778;
	_delay_us(130);
	TCNT1 = 0;
	for (i = 0; i < 13; i++) {
		while (!(TIFR & (1 << OCF1A))) ;
		TIFR |= 1 << OCF1A;
		RC5_DATA <<= 1;
		if (PIND & (1 << PD3)) {
			_delay_us(10);
			if (PIND & (1 << PD3)) {
				_delay_us(10);
				if (PIND & (1 << PD3)) {
					RC5_DATA++;
				}
			}
		}
	}
	if ((RC5_DATA & 0b1111101100000000) != 0b0000001100000000) {
		GIFR |= (1 << INTF1);
		return;
	}
	RC5_DATA &= (uint16_t) 0b111111;
	RC5_EVENT = 1;
	reportBuffer.buttonMask = 0;
	reportBuffer.dx = 0;
	reportBuffer.dy = 0;
	reportBuffer.dWheel = 0;
	if (RC5_PREVIOUS_DATA == RC5_DATA) {
		MOUSE_POINTER_VELOCITY += 10;
		if (MOUSE_POINTER_VELOCITY > 127)
			MOUSE_POINTER_VELOCITY = 327;
	} else
		MOUSE_POINTER_VELOCITY = 3;
	RC5_PREVIOUS_DATA = RC5_DATA;
	if (RC5_DATA == 8) {
		reportBuffer.dy = MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 4) {
		reportBuffer.dx = -MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 6) {
		reportBuffer.dx = MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 2) {
		reportBuffer.dy = -MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 7) {
		reportBuffer.dx = -MOUSE_POINTER_VELOCITY;
		reportBuffer.dy = MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 9) {
		reportBuffer.dx = MOUSE_POINTER_VELOCITY;
		reportBuffer.dy = MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 1) {
		reportBuffer.dx = -MOUSE_POINTER_VELOCITY;
		reportBuffer.dy = -MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 3) {
		reportBuffer.dx = MOUSE_POINTER_VELOCITY;
		reportBuffer.dy = -MOUSE_POINTER_VELOCITY;
	} else if (RC5_DATA == 5) {
		RC5_CLICK_EVENT = 1;
		reportBuffer.buttonMask = 1;
		_delay_ms(50);
	} else if (RC5_DATA == 0) {
		RC5_CLICK_EVENT = 1;
		reportBuffer.buttonMask = 2;
		_delay_ms(50);
	} else if (RC5_DATA == 16) {
		RC5_CLICK_EVENT = 1;
		reportBuffer.dWheel = 1;
	} else if (RC5_DATA == 17) {
		RC5_CLICK_EVENT = 1;
		reportBuffer.dWheel = -1;
	} else
		RC5_EVENT = 0;
	GIFR |= (1 << INTF1);
}

/*external interrupt enable */
void INT1_init()
{
	PORTD |= 1 << PD3;
	DDRD &= ~(1 << PD3);
	GICR |= (1 << INT1);
}

/* ------------------------------------------------------------------------- */

int __attribute__ ((noreturn)) main(void)
{
	uchar i;
	TCCR1B |= (1 << WGM12) | (1 << CS10);
	volatile uint16_t VIRTUAL_TIMER = 20000;
	wdt_enable(WDTO_1S);
	usbInit();
	usbDeviceDisconnect();
	i = 0;
	while (--i) {
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();
	INT1_init();
	sei();
	for (;;) {		/* main event loop */
		wdt_reset();
		usbPoll();
		if (VIRTUAL_TIMER-- < 5) {
			MOUSE_POINTER_VELOCITY = 3;
			RC5_PREVIOUS_DATA = 0;
			VIRTUAL_TIMER = 1;
		}
		if (usbInterruptIsReady() && RC5_EVENT) {
			RC5_EVENT = 0;
			VIRTUAL_TIMER = 20000;
			usbSetInterrupt((void *)&reportBuffer,
					sizeof(reportBuffer));
			if (RC5_CLICK_EVENT) {
				RC5_CLICK_EVENT = 0;
				reportBuffer.buttonMask = 0;
				reportBuffer.dWheel = 0;
				RC5_EVENT = 1;
			}
		}
	}
}

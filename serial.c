/*
 * serial.c
 *
 * Copyright (c) 2012, 2013, Thomas Buck <xythobuz@me.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "serial.h"
#include "serial_device.h"

/** \addtogroup uart UART Library
 *  UART Library enabling you to control all available
 *  UART Modules. With XON/XOFF Flow Control and buffered
 *  Receiving and Transmitting.
 *  @{
 */

/** \file serial.c
 *  UART Library Implementation
 */

/** If you define this, a '\\r' (CR) will be put in front of a '\\n' (LF) when sending a byte.
 *  Binary Communication will then be impossible!
 */
// #define SERIALINJECTCR

#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE 32 /**< RX Buffer Size in Bytes (Power of 2) */
#endif

#ifndef TX_BUFFER_SIZE
#define TX_BUFFER_SIZE 16 /**< TX Buffer Size in Bytes (Power of 2) */
#endif

/** Defining this enables incoming XON XOFF (sends XOFF if rx buff is full) */
#define FLOWCONTROL

#define FLOWMARK 5 /**< Space remaining to trigger xoff/xon */
#define XON 0x11 /**< XON Value */
#define XOFF 0x13 /**< XOFF Value */

#if (RX_BUFFER_SIZE < 2) || (TX_BUFFER_SIZE < 2)
#error SERIAL BUFFER TOO SMALL!
#endif

#ifdef FLOWCONTROL
#if (RX_BUFFER_SIZE < 8) || (TX_BUFFER_SIZE < 8)
#error SERIAL BUFFER TOO SMALL!
#endif
#endif

#if ((RX_BUFFER_SIZE + TX_BUFFER_SIZE) * UART_COUNT) >= (RAMEND - 0x60)
#error SERIAL BUFFER TOO LARGE!
#endif

#if (RX_BUFFER_SIZE > 65535) || (TX_BUFFER_SIZE > 65535)
#error SERIAL BUFFER INDEX HAS TO FIT 16BIT!
#endif

// serialRegisters
#define SERIALDATA  0
#define SERIALB     1
#define SERIALC     2
#define SERIALA     3
#define SERIALUBRRH 4
#define SERIALUBRRL 5

// serialBits
#define SERIALUCSZ0 0
#define SERIALUCSZ1 1
#define SERIALRXCIE 2
#define SERIALRXEN  3
#define SERIALTXEN  4
#define SERIALUDRIE 5
#define SERIALUDRE  6
#define SERIALUPM0  7
#define SERIALUPM1  8
#define SERIALUSBS  9

static uint8_t volatile rxBuffer[UART_COUNT][RX_BUFFER_SIZE];
static uint8_t volatile txBuffer[UART_COUNT][TX_BUFFER_SIZE];
static uint16_t volatile rxRead[UART_COUNT];
static uint16_t volatile rxWrite[UART_COUNT];
static uint16_t volatile txRead[UART_COUNT];
static uint16_t volatile txWrite[UART_COUNT];
static uint8_t volatile shouldStartTransmission[UART_COUNT];

#ifdef FLOWCONTROL
static uint8_t volatile sendThisNext[UART_COUNT];
static uint8_t volatile flow[UART_COUNT];
static uint16_t volatile rxBufferElements[UART_COUNT];
#endif

uint8_t serialAvailable(void) {
    return UART_COUNT;
}

void serialInit(uint8_t uart, uint16_t baud, uint8_t bits, uint8_t parity, uint8_t stopbits) {
    if ((uart >= UART_COUNT) ||
        (bits < 5)           ||
        (bits > 8)           ||  //no 9bit frames allowed currently
        (parity > 2)         ||  //0: disabled, 1: even parity, 2: odd parity 
        (stopbits > 2)       ||
        (stopbits < 1))
        return;

    // Initialize state variables
    rxRead[uart]  = 0;
    rxWrite[uart] = 0;
    txRead[uart]  = 0;
    txWrite[uart] = 0;
    shouldStartTransmission[uart] = 1;

#ifdef FLOWCONTROL
    sendThisNext[uart] = 0;
    flow[uart] = 1;
    rxBufferElements[uart] = 0;
#endif

    // Set amount of data bits
    switch(bits){
        case 5:
            //UCSZ1:0 == 0b00
            break;
        case 6:
            *serialRegisters[uart][SERIALC] = (1 << serialBits[uart][SERIALUCSZ0]);
            break;
        case 7:
            *serialRegisters[uart][SERIALC] = (1 << serialBits[uart][SERIALUCSZ1]);
            break;
        case 8:
            *serialRegisters[uart][SERIALC] = (1 << serialBits[uart][SERIALUCSZ1]) | (1 << serialBits[uart][SERIALUCSZ0]);
            break;
    }

    // Set parity checking
    switch(parity){
        case 0:
            // UPM1:0 == 0b00
            break;
        case 1:
            *serialRegisters[uart][SERIALC] = (1 << serialBits[uart][SERIALUPM1]);
            break;
        case 2:
            *serialRegisters[uart][SERIALC] = (1 << serialBits[uart][SERIALUPM1]) | (1 << serialBits[uart][SERIALUPM0]);
            break;
    }
    
    // Set number of stopbits
    switch (stopbits){
        case 1: 
            //USBS == 0;
            break;
        case 2:
            *serialRegisters[uart][SERIALC] = (1 << serialBits[uart][SERIALUSBS]);  
            break;
    }

    // Set baudrate
#if SERIALBAUDBIT == 8
    *serialRegisters[uart][SERIALUBRRH] = (BRRVALUE(baud) >> 8);
    *serialRegisters[uart][SERIALUBRRL] =  BRRVALUE(baud);
#else
    *serialBaudRegisters[uart] = BRRVALUE(baud);
#endif

    // Enable Interrupts
    *serialRegisters[uart][SERIALB] = (1 << serialBits[uart][SERIALRXCIE]);

    // Enable Receiver/Transmitter
    *serialRegisters[uart][SERIALB] |= (1 << serialBits[uart][SERIALRXEN]) | (1 << serialBits[uart][SERIALTXEN]);
}

void serialClose(uint8_t uart) {
    if (uart >= UART_COUNT) {
        return;
    }

    uint8_t sreg = SREG;
    sei();
    while (!serialTxBufferEmpty(uart));

    // Wait while Transmit Interrupt is on
    while (*serialRegisters[uart][SERIALB] & (1 << serialBits[uart][SERIALUDRIE]));

    cli();
    *serialRegisters[uart][SERIALB] = 0;
    *serialRegisters[uart][SERIALC] = 0;
    SREG = sreg;
}

#ifdef FLOWCONTROL
void setFlow(uint8_t uart, uint8_t on) {
    if (uart >= UART_COUNT) {
        return;
    }

    if (flow[uart] != on) {
        if (on == 1) {
            // Send XON
            while (sendThisNext[uart] != 0);
            sendThisNext[uart] = XON;
            flow[uart] = 1;
            if (shouldStartTransmission[uart]) {
                shouldStartTransmission[uart] = 0;

                // Enable Interrupt
                *serialRegisters[uart][SERIALB] |= (1 << serialBits[uart][SERIALUDRIE]);

                // Trigger Interrupt
                *serialRegisters[uart][SERIALA] |= (1 << serialBits[uart][SERIALUDRE]);
                // TODO WG: according to the datasheet UDRE is Read-only!!!
            }
        } else {
            // Send XOFF
            sendThisNext[uart] = XOFF;
            flow[uart] = 0;
            if (shouldStartTransmission[uart]) {
                shouldStartTransmission[uart] = 0;

                // Enable Interrupt
                *serialRegisters[uart][SERIALB] |= (1 << serialBits[uart][SERIALUDRIE]);

                // Trigger Interrupt
                *serialRegisters[uart][SERIALA] |= (1 << serialBits[uart][SERIALUDRE]);
                // TODO WG: according to the datasheet UDRE is Read-only!!!
            }
        }

        // Wait until it's transmitted
        while (*serialRegisters[uart][SERIALB] & (1 << serialBits[uart][SERIALUDRIE]));
    }
}
#endif

// ---------------------
// |     Reception     |
// ---------------------

uint8_t serialGetBlocking(uint8_t uart) {
    if (uart >= UART_COUNT) {
        return 0;
    }

    while (serialRxBufferEmpty(uart));
    return serialGet(uart);
}

uint8_t serialGet(uint8_t uart) {
    if ((uart >= UART_COUNT) || (serialRxBufferEmpty(uart))) {
        return 0;
    }
    // TODO WG: how can the caller know if the buffer was empty or the value of the returned byte was actually zero? 

    uint8_t c;

#ifdef FLOWCONTROL
    // This should not underflow as long as the receive buffer is not empty
    rxBufferElements[uart]--;

    if ((flow[uart] == 0) && (rxBufferElements[uart] <= FLOWMARK)) {
        while (sendThisNext[uart] != 0);
        sendThisNext[uart] = XON;
        flow[uart] = 1;
        if (shouldStartTransmission[uart]) {
            shouldStartTransmission[uart] = 0;

            // Enable Interrupt
            *serialRegisters[uart][SERIALB] |= (1 << serialBits[uart][SERIALUDRIE]);

            // Trigger Interrupt
            *serialRegisters[uart][SERIALA] |= (1 << serialBits[uart][SERIALUDRE]);
            // TODO WG: according to the datasheet UDRE is Read-only!!!
        }
    }
#endif
    c = rxBuffer[uart][rxRead[uart]];
    rxBuffer[uart][rxRead[uart]] = 0;
    if (rxRead[uart] < (RX_BUFFER_SIZE - 1)) {
        rxRead[uart]++;
    } else {
        rxRead[uart] = 0;
    }
    return c;
     
}

uint8_t serialRxBufferFull(uint8_t uart) {
    if (uart >= UART_COUNT) {
        return 0;
    }

    return (((rxWrite[uart] + 1) == rxRead[uart])
            || ((rxRead[uart] == 0) && ((rxWrite[uart] + 1) == RX_BUFFER_SIZE)));
}

uint8_t serialRxBufferEmpty(uint8_t uart) {
    if (uart >= UART_COUNT) {
        return 0;
    }

    if (rxRead[uart] == rxWrite[uart]) {
        return 1;
    } else {
        return 0;
    }
}

// ----------------------
// |    Transmission    |
// ----------------------

void serialWrite(uint8_t uart, uint8_t data) {
    if (uart >= UART_COUNT) {
        return;
    }

#ifdef SERIALINJECTCR
    if (data == '\n') {
        serialWrite(uart, '\r');
    }
#endif
    while (serialTxBufferFull(uart));

    txBuffer[uart][txWrite[uart]] = data;
    if (txWrite[uart] < (TX_BUFFER_SIZE - 1)) {
        txWrite[uart]++;
    } else {
        txWrite[uart] = 0;
    }
    if (shouldStartTransmission[uart]) {
        shouldStartTransmission[uart] = 0;

        // Enable Interrupt
        *serialRegisters[uart][SERIALB] |= (1 << serialBits[uart][SERIALUDRIE]);

        // Trigger Interrupt
        *serialRegisters[uart][SERIALA] |= (1 << serialBits[uart][SERIALUDRE]);
        // TODO WG: according to the datasheet UDRE is Read-only!!!
    }
}

uint16_t serialWriteString(uint8_t uart, const char *data) {
    
    uint16_t bytes = 0;
    
    if ((uart >= UART_COUNT) || (data == 0)) {
        return 0;
    }

    while (*data != '\0') {
        serialWrite(uart, *data++);
        bytes++;
        // TODO WG: 'bytes' can overflow
    }
    
    return bytes;
}

uint8_t serialTxBufferFull(uint8_t uart) {
    if (uart >= UART_COUNT) {
        return 0;
    }

    return (((txWrite[uart] + 1) == txRead[uart])
            || ((txRead[uart] == 0) && ((txWrite[uart] + 1) == TX_BUFFER_SIZE)));
}

uint8_t serialTxBufferEmpty(uint8_t uart) {
    if (uart >= UART_COUNT) {
        return 0;
    }

    if (txRead[uart] == txWrite[uart]) {
        return 1;
    } else {
        return 0;
    }
}

// ----------------------
// |      Internal      |
// ----------------------

inline static void serialReceiveInterrupt(uint8_t uart) {
    
    uint8_t C;
    
    // Read UDRE in order to clear RXC (Receive Complete) interrupt flag
    C = *serialRegisters[uart][SERIALDATA];

    // Store recieved byte only if rx buffer is not overflowing
    if (!serialRxBufferFull(uart)) {
        rxBuffer[uart][rxWrite[uart]] = C;
        if (rxWrite[uart] < (RX_BUFFER_SIZE - 1)) {
            rxWrite[uart]++;
        } else {
            rxWrite[uart] = 0;
        }
    } else {
        // Do nothing if the receive buffer is overflowing
    }

#ifdef FLOWCONTROL
    if (rxBufferElements[uart] < 0xFFFF) {
        rxBufferElements[uart]++;
    }

    if ((flow[uart] == 1) && (rxBufferElements[uart] >= (RX_BUFFER_SIZE - FLOWMARK))) {
        sendThisNext[uart] = XOFF;
        flow[uart] = 0;
        if (shouldStartTransmission[uart]) {
            shouldStartTransmission[uart] = 0;

            // Enable Interrupt
            *serialRegisters[uart][SERIALB] |= (1 << serialBits[uart][SERIALUDRIE]);

            // Trigger Interrupt
            *serialRegisters[uart][SERIALA] |= (1 << serialBits[uart][SERIALUDRE]);
            // TODO WG: according to the datasheet UDRE is Read-only!!!
        }
    }
#endif
}

inline static void serialTransmitInterrupt(uint8_t uart) {
#ifdef FLOWCONTROL
    if (sendThisNext[uart]) {
        *serialRegisters[uart][SERIALDATA] = sendThisNext[uart];
        sendThisNext[uart] = 0;
    } else {
#endif
        if (!serialTxBufferEmpty(uart)) {
            *serialRegisters[uart][SERIALDATA] = txBuffer[uart][txRead[uart]];
            if (txRead[uart] < (TX_BUFFER_SIZE -1)) {
                txRead[uart]++;
            } else {
                txRead[uart] = 0;
            }
        } else {
            shouldStartTransmission[uart] = 1;

            // Disable Interrupt
            *serialRegisters[uart][SERIALB] &= ~(1 << serialBits[uart][SERIALUDRIE]);
        }
#ifdef FLOWCONTROL
    }
#endif
}

ISR(SERIALRECIEVEINTERRUPT) {
    // Receive complete
    serialReceiveInterrupt(0);
}

ISR(SERIALTRANSMITINTERRUPT) {
    // Data register empty
    serialTransmitInterrupt(0);
}

#if UART_COUNT > 1
ISR(SERIALRECIEVEINTERRUPT1) {
    // Receive complete
    serialReceiveInterrupt(1);
}

ISR(SERIALTRANSMITINTERRUPT1) {
    // Data register empty
    serialTransmitInterrupt(1);
}
#endif

#if UART_COUNT > 2
ISR(SERIALRECIEVEINTERRUPT2) {
    // Receive complete
    serialReceiveInterrupt(2);
}

ISR(SERIALTRANSMITINTERRUPT2) {
    // Data register empty
    serialTransmitInterrupt(2);
}
#endif

#if UART_COUNT > 3
ISR(SERIALRECIEVEINTERRUPT3) {
    // Receive complete
    serialReceiveInterrupt(3);
}

ISR(SERIALTRANSMITINTERRUPT3) {
    // Data register empty
    serialTransmitInterrupt(3);
}
#endif
/** @} */


/*
 * serial_device.h
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
#ifndef _serial_device_h
#define _serial_device_h

/** \addtogroup uart UART Library
 *  UART Library enabling you to control all available
 *  UART Modules. With XON/XOFF Flow Control and buffered
 *  Receiving and Transmitting.
 *  @{
 */

/** \file serial_device.h
 *  UART Library device-specific configuration.
 *  Contains Register and Bit Positions for different AVR devices.
 */

#if  defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) \
  || defined(__AVR_ATtiny4313__)

#define UART_COUNT 1
#define UART_REGISTERS 6
#define UART_BITS 10
volatile uint8_t * const  serialRegisters[UART_COUNT][UART_REGISTERS] = {{
    &UDR,
    &UCSRB,
    &UCSRC,
    &UCSRA,
    &UBRRH,
    &UBRRL
}};
#define SERIALBAUDBIT 8
uint8_t const serialBits[UART_COUNT][UART_BITS] = {{
    UCSZ0,
    UCSZ1,
    RXCIE,
    RXEN,
    TXEN,
    UDRIE,
    UDRE,
    UPM0,
    UPM1,
    USBS
}};
#define SERIALRECIEVEINTERRUPT USART_RX_vect
#define SERIALTRANSMITINTERRUPT USART_UDRE_vect

#else
#error "AvrSerialLibrary not compatible with your MCU!"
#endif

#endif // _serial_device_h
/** @} */


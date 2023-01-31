/*************************************************************
Jeti Sensor EX Bus Telemetry C++ Library

JetiExBusSamd21Serial - Samd21 serial implementation for half duplex 
single wire operation, compatible with Seeedstudio XIAO m0.

This is an addon from nichtgedacht
Version history: 1.0 initial

JetiExBus Library is:
---------------------------------------------------------------------------
Copyright (C) 2018 Bernd Wokoeck

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.

**************************************************************/

#if defined (__SAMD21__)

#include "JetiExBusSamd21Serial.h"

// #define LED_DEBUG

JetiExBusSamd21Serial * _pInstance = 0;   // instance pointer to find the serial object from ISR

JetiExBusSerial * JetiExBusSerial::CreatePort(int comPort)
{
	return new JetiExBusSamd21Serial();
}

void JetiExBusSamd21Serial::begin(uint32_t baud, uint32_t format)
{
	// connect main clock GCLK0 to USART (sercom0)
  GCLK->CLKCTRL.reg = 
    GCLK_CLKCTRL_CLKEN |
    GCLK_CLKCTRL_GEN_GCLK0 |        // Use GCLK0 (48MHz peconfigured) for UART
    GCLK_CLKCTRL_ID_SERCOM0_CORE;
  while(GCLK->STATUS.bit.SYNCBUSY) {};

  // enable peripheral interface clock 
  // using power mgmnt peripheral
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
  
  // reset USART periphal
  SERCOM0->USART.CTRLA.bit.SWRST = 1;
  while (SERCOM0->USART.SYNCBUSY.bit.SWRST) {};

  // Configure PA6 as RX/TX single Pin
  // See CTRLA register for pad config

  // first set default state as input with pullup
  PORT->Group[0].DIRCLR.reg |= PORT_PA06;

  PORT->Group[0].PINCFG[6].bit.PULLEN = 1;
  PORT->Group[0].PINCFG[6].bit.INEN = 1;
  PORT->Group[0].PINCFG[6].bit.PMUXEN = 1;
  
  // Portmux even pin number, for odd last bit in index would become shifted out
  PORT->Group[0].PMUX[6>>1].bit.PMUXE = PORT_PMUX_PMUXE_D_Val; //SERCOM-alt

// Use 1 Tab = 2 Spaces
/*******************************  default Pin usage and mapping ***************************************************

Serial Interface	                                   Pad	                                           Protocol
                  |         0		   |         1        |         2          |            3           |
ALT-SERCOM0       |  PA04	 A1  - 	 |  PA05  A9  MISO  |  PA06  A10   MOSI  |   PA07    A8    SCK    |   SPI
ALT-SERCOM2       |  PA08	 A4  SDA |  PA09  A5  SCL   |  PA10  A2    -     |   PA11    A3    -      |   I²C
ALT-SERCOM4       |                |                  |  PB08  A6    RX    |   PB09    A7    TX     |   USART
ALT-SERCOM1                                           |  PA30        SWCLK |   PA31          SWDIO  | 	SWD

*******************************************************************************************************************
************************************* we use it this way **********************************************************
Serial Interface	                                   Pad	                                           Protocol
                  |         0      |         1        |         2          |            3           |
ALT-SERCOM0       |  PA04	 A1  - 	 |  PA05  A9    -   |  PA06  A10  RX/TX  |   PA07    A8    -      |   USART
ALT-SERCOM2       |  PA08	 A4  SDA |  PA09  A5  SCL   |  PA10  A2    -     |   PA11    A3    -      |   I²C
ALT-SERCOM4       |                |                  |  PB08  A6    RX    |   PB09    A7    TX     |   USART
*******************************************************************************************************************/

// Configure USART via Control A and Control B

  // implicite config becomes so this is redundant
  SERCOM0->USART.CTRLA.bit.FORM = 0;      // no parity bit
  SERCOM0->USART.CTRLB.bit.SBMODE = 0;    // 1 stop bit
  
  // Control A bits
	SERCOM0->USART.CTRLA.reg =                  // USART is ASYNCHRONOUS
	   SERCOM_USART_CTRLA_DORD |                // Transmit LSB First
	   SERCOM_USART_CTRLA_MODE_USART_INT_CLK |  // Set Internal Clock 
	   SERCOM_USART_CTRLA_RXPO(2) |             // Use SERCOM pad 2 for data reception
	   SERCOM_USART_CTRLA_TXPO(1);              // Set SERCOM pad 2 for data transmission
	
  // Control B bits
	SERCOM0->USART.CTRLB.reg =      // We don't use PARITY
	  SERCOM_USART_CTRLB_RXEN |     // Enable receive when USART is enabled
		//SERCOM_USART_CTRLB_TXEN |   // Enable transmit when USART is enabled
		SERCOM_USART_CTRLB_CHSIZE(0); // Set character size to 8 bits
  while (SERCOM0->USART.SYNCBUSY.bit.CTRLB) {};

  //  Set USART Baud Rate
	// Baud rate is (65536) * (CPU_CLock - 16 * wanted baud) / CPU_Clock
	uint64_t baudRate = (uint64_t)65536 * (F_CPU - 16 * 125000) / F_CPU;
	
	// Set Baud Rate
	// SERCOM0->USART.BAUD.reg = (uint32_t)baudRate;
	REG_SERCOM0_USART_BAUD = (uint32_t)baudRate;
	
  NVIC_EnableIRQ(SERCOM0_IRQn);       // enable SERCOM0 interrupt request
  NVIC_SetPriority(SERCOM0_IRQn,0);   // highest priority

  SERCOM0->USART.INTENSET.bit.RXC = 1;      // enable receive complete interrupt
  // SERCOM0->USART.INTENSET.bit.DRE = 1;   // enable data register empty interrupt
  // SERCOM0->USART.INTENSET.bit.TXC = 1;   // enable TX complete interrupt

  // Finally enable Sercom peripheral
  SERCOM0->USART.CTRLA.bit.ENABLE = 1;
  while(SERCOM0->USART.SYNCBUSY.bit.ENABLE) {};

  // init tx ring buffer 
  m_txHead = 0;
  m_txTail = 0;

  // init rx ring buffer 
  m_rxHead = 0;
  m_rxTail = 0;

  m_bSending = false;

  _pInstance = this; // there is a single instance only

#ifdef LED_DEBUG
  pinMode( 13, OUTPUT);
  digitalWrite(13, HIGH );   
#endif
}

// Byte available?
int JetiExBusSamd21Serial::available(void)
{
   return m_rxHead != m_rxTail;
}

// Read one byte
int JetiExBusSamd21Serial::read(void)
{
  uint8_t c = 0;

  if ( m_rxTail != m_rxHead ) // atomic operation
  {
    NVIC_DisableIRQ(SERCOM0_IRQn);
    __DSB();
    __ISB();

    c = m_rxBuf[m_rxHead];
    m_rxHead = (m_rxHead + 1) % RX_RINGBUF_SIZE;

    NVIC_EnableIRQ(SERCOM0_IRQn);
  }

  return c;
}

// Write one byte  
size_t JetiExBusSamd21Serial::write(const uint8_t *buffer, size_t size)
{

  NVIC_DisableIRQ(SERCOM0_IRQn);
  __DSB();
  __ISB();

  size_t ret = size;

  for( uint8_t i = 0; i < size; i++ )
  {
    if ( (m_txTail + 1) % TX_RINGBUF_SIZE != m_txHead )
	  {
      m_txBuf[m_txTail] = buffer[i];
      m_txTail = (m_txTail + 1) % TX_RINGBUF_SIZE;
	  }
	  else
	  {
		  ret = i;
		  break;
	  }
  }

  // enable transmitter
  if( !m_bSending )
  {
  #ifdef LED_DEBUG
    digitalWrite(13, HIGH ); 
  #endif
    m_bSending    = true;

    SERCOM0->USART.INTENCLR.bit.RXC = 1;  // disable receiver and receiver interrupt
    SERCOM0->USART.CTRLB.bit.RXEN = 0;

    SERCOM0->USART.INTENSET.bit.DRE = 1;  // enable transmitter and tx register empty interrupt
    SERCOM0->USART.CTRLB.bit.TXEN = 1;
    while (SERCOM0->USART.SYNCBUSY.bit.CTRLB) {};
  }

  NVIC_EnableIRQ(SERCOM0_IRQn);

  return ret;
}

void SERCOM0_Handler() // name predefined by startup code 
{

  if (SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE && SERCOM0->USART.INTENSET.reg & SERCOM_USART_INTENSET_DRE)  // if interrupt flag is set AND interrupt is enabled
  {

    // Data Register Empty interrupt
    
    if( _pInstance->m_txTail != _pInstance->m_txHead )
    {
  
      SERCOM0->USART.DATA.reg = _pInstance->m_txBuf[_pInstance->m_txHead];
      _pInstance->m_txHead = (_pInstance->m_txHead + 1) % JetiExBusSamd21Serial::TX_RINGBUF_SIZE;

    }
    else
    {
      // enable TX complete interrupt to get a signal for end of transmission
      SERCOM0->USART.INTENCLR.bit.DRE = 1;   // disable data register empty interrupt
      SERCOM0->USART.INTENSET.bit.TXC = 1;   // enable TX complete interrupt

      // alternative coded
      //SERCOM0->USART.INTENCLR.reg |= SERCOM_USART_INTENCLR_DRE;
      //SERCOM0->USART.INTENSET.reg |= SERCOM_USART_INTENSET_TXC;
      
      _pInstance->m_bSending = false;
    }
  }

  if (SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC && SERCOM0->USART.INTENSET.reg & SERCOM_USART_INTENSET_RXC)  // if interrupt flag is set AND interrupt is enabled
  {

    // Receive Complete interrupt

#ifdef LED_DEBUG
    digitalWrite( 13, LOW ); 
#endif
    //if ((_pInstance->m_rxTail + 1) % _pInstance->RX_RINGBUF_SIZE != _pInstance->m_rxHead ) // if not buffer full
    //{
      _pInstance->m_rxBuf[_pInstance->m_rxTail] = SERCOM0->USART.DATA.reg;
      _pInstance->m_rxTail = (_pInstance->m_rxTail + 1) % _pInstance->RX_RINGBUF_SIZE;
    //}
    //else
    //{
    //  SERCOM0->USART.INTENCLR.bit.RXC = 1;
    //  SERCOM0->USART.CTRLB.bit.RXEN = 0;
    //  while (SERCOM0->USART.SYNCBUSY.bit.CTRLB) {};
    //}
#ifdef LED_DEBUG
    digitalWrite( 13, HIGH ); 
#endif

  }

  if (SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC && SERCOM0->USART.INTENSET.reg & SERCOM_USART_INTENSET_TXC)
  {

    // Transmission Complete interrupt

    // clear TXC interrupt flag ??????????????????
    SERCOM0->USART.INTFLAG.bit.TXC = 1;

    // switch to receive mode
    SERCOM0->USART.INTENCLR.bit.TXC = 1;  // disable TXC interrupt

    SERCOM0->USART.CTRLB.bit.TXEN = 0;    // disable transmitter
    SERCOM0->USART.INTENCLR.bit.TXC = 1;  // disable TXC interrupt
    SERCOM0->USART.CTRLB.bit.RXEN = 1;    // enable receiver
    while (SERCOM0->USART.SYNCBUSY.bit.CTRLB) {};
    SERCOM0->USART.INTENSET.bit.RXC = 1;  // enable RXC interrupt

    // clear receiver buffer
    _pInstance->m_rxHead = 0;
    _pInstance->m_rxTail = 0;

#ifdef LED_DEBUG
    digitalWrite( 13, LOW ); 
#endif

  }
}

/*
extern "C"
{
  void hard_fault_handler_c(unsigned int * hardfault_args, unsigned lr_value)
  {
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;
    stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);
    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);
    printf ("[HardFault handler]\n");
    printf ("R0 = %x\n", stacked_r0);
    printf ("R1 = %x\n", stacked_r1);
    printf ("R2 = %x\n", stacked_r2);
    printf ("R3 = %x\n", stacked_r3);
    printf ("R12 = %x\n", stacked_r12);
    printf ("Stacked LR = %x\n", stacked_lr);
    printf ("Stacked PC = %x\n", stacked_pc);
    printf ("Stacked PSR = %x\n", stacked_psr);
    printf ("Current LR = %x\n", lr_value);

    digitalWrite( 11, LOW );
    digitalWrite( 12, LOW );
    digitalWrite( 13, LOW );

    while(1); // endless loop
  }
}

void HardFault_Handler(void)
{
  __asm__ volatile
  (
    "MOVS r0, #4        \n"
    "MOV r1, LR         \n"
    "TST r0, r1         \n"
    "BEQ stacking_used_MSP \n"
    "MRS R0, PSP        \n" //; first parameter - stacking was using PSP
    "B get_LR_and_branch  \n"
  "stacking_used_MSP:    \n"
    "MRS R0, MSP        \n" //; first parameter - stacking was using MSP
  "get_LR_and_branch:    \n"
    "MOV R1, LR         \n" //; second parameter is LR current value
    "LDR R2, hard_fault_handler_c_addr  \n"
    "BX R2              \n"
  "hard_fault_handler_c_addr:"
    ".word hard_fault_handler_c"  

  );
}
*/

void HardFault_Handler()
{
    digitalWrite( 11, LOW );
    digitalWrite( 12, LOW );
    digitalWrite( 13, LOW );
}

#endif // __SAMD21__
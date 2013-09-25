/**************************************************************************/
/*! 
    @file     gpio.c
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

    @section DESCRIPTION
	
    Controls the general purpose digital IO.

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "gpio.h"

#include <stdbool.h>

#ifdef CFG_CHIBI
#include "drivers/rf/chibi/chb_drvr.h"
volatile uint32_t chibi_counter  = 0;
#endif

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 0 (currently checks pin 0.1)
*/
/**************************************************************************/
#if defined GPIO_ENABLE_IRQ0
void PIOINT0_IRQHandler(void)
{
  uint32_t regVal;

  regVal = gpioIntStatus(0, 1);
  if (regVal)
  {
    gpioIntClear(0, 1);
  }		
  return;
}
#endif

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 1 (currently checks pin 1.1)
*/
/**************************************************************************/
#if defined GPIO_ENABLE_IRQ1
void PIOINT1_IRQHandler(void)
{
  uint32_t regVal;

  regVal = gpioIntStatus(1, 1);
  if ( regVal )
  {
    gpioIntClear(1, 1);
  }

  return;
}
#endif

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 2 (currently checks pin 2.1)
*/
/**************************************************************************/
#if defined GPIO_ENABLE_IRQ2
void PIOINT2_IRQHandler(void)
{
  uint32_t regVal;

  regVal = gpioIntStatus(2, 1);
  if ( regVal )
  {
    gpioIntClear(2, 1);
  }		
  return;
}
#endif

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 3 (currently checks pin 3.1)
*/
/**************************************************************************/
#if defined GPIO_ENABLE_IRQ3
void PIOINT3_IRQHandler(void)
{
  uint32_t regVal;

 #ifdef CFG_CHIBI
  // Check for interrupt on 3.1
  regVal = gpioIntStatus(3, 1);
  if ( regVal )
  {
    chibi_counter++;
    chb_ISR_Handler();
    gpioIntClear(3, 1);
  }		
#else
  regVal = gpioIntStatus(3, 1);
  if ( regVal )
  {
    gpioIntClear(3, 1);
  }		
#endif

 return;
}
#endif

/**************************************************************************/
/*! 
    @brief Disables all of the internal resistors by default, since they
           are set to pullup by default coming out of reset and can waste
           power if they aren't really needed.
*/
/**************************************************************************/
void gpioDisableInternalResistors(void)
{
  // Array of all IOCON addresses for GPIO pins
  uint32_t iocon_addresses[39] = {  0x4004400C, 0x40044010, 0x4004401C, 0x4004402C, 0x4004404C, 0x40044050, 0x40044060, 0x40044064, 0x40044068, 0x40044074,                         // GPIO0
                                    0x40044078, 0x4004407C, 0x40044080, 0x40044094, 0x400440A0, 0x400440A4, 0x400440A8, 0x40044014, 0x40044038, 0x4004406C, 0x40044098,             // GPIO1
                                    0x40044008, 0x40044028, 0x4004405C, 0x4004408C, 0x40044040, 0x40044044, 0x40044000, 0x40044020, 0x40044024, 0x40044054, 0x40044058, 0x40044070, // GPOP2
                                    0x40044084, 0x40044088, 0x4004409C, 0x400440AC, 0x4004403C, 0x40044048 };                                                                       // GPOP3

  uint32_t i;
  for (i = 0; i < 39; i++)
  {
    *((uint32_t*)iocon_addresses[i]) &= ~(IOCON_COMMON_MODE_MASK);
    *((uint32_t*)iocon_addresses[i]) |= IOCON_PIO3_4_MODE_INACTIVE;
  }
}

/**************************************************************************/
/*! 
    @brief Initialises GPIO and enables the GPIO interrupt
           handler for all GPIO ports.
*/
/**************************************************************************/
void gpioInit (void)
{
  /* Enable AHB clock to the GPIO domain. */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_GPIO);
}

/**************************************************************************/
/*! 
    @brief Sets the direction (input/output) for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..11)
    @param[in]  dir
                The pin direction (gpioDirection_Input or
                gpioDirection_Output)
*/
/**************************************************************************/
void gpioSetDir (uint32_t portNum, uint32_t bitPos, gpioDirection_t dir)
{
  // Get the appropriate register (handled this way to optimise code size)
  REG32 *gpiodir = &GPIO_GPIO0DIR;
  switch (portNum)
  {
    case 0:
      gpiodir = &GPIO_GPIO0DIR;
      break;
    case 1:
      gpiodir = &GPIO_GPIO1DIR;
      break;
    case 2:
      gpiodir = &GPIO_GPIO2DIR;
      break;
    case 3:
      gpiodir = &GPIO_GPIO3DIR;
      break;
  }

  // Toggle dir
  dir == gpioDirection_Output ? (*gpiodir |= (1 << bitPos)) : (*gpiodir &= ~(1 << bitPos));
}

/**************************************************************************/
/*! 
    @brief Gets the value for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)

    @return     The current value for the specified port pin (0..1)
*/
/**************************************************************************/
uint32_t gpioGetValue (uint32_t portNum, uint32_t bitPos)
{
  uint32_t value = 0;

  switch (portNum)
  {
    case 0:
      value = (GPIO_GPIO0DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 1:
      value = (GPIO_GPIO1DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 2:
      value = (GPIO_GPIO2DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 3:
      value = (GPIO_GPIO3DATA & (1 << bitPos)) ? 1 : 0;
      break;
    default:
      break;
  }

  return value;
}

/**************************************************************************/
/*! 
    @brief Sets the value for a specific port pin (only relevant when a
           pin is configured as output).

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
    @param[in]  bitValue
                The value to set for the specified bit (0..1).  0 will set
                the pin low and 1 will set the pin high.
*/
/**************************************************************************/
inline void gpioSetValue (const uint32_t portNum, const uint32_t bitPos, const uint32_t bitVal)
{
  // Take advantage of the fact the GPIO registers are bit-banded
  (*(pREG32 ((GPIO_GPIO0_BASE + (portNum << 16)) + ((1 << bitPos) << 2)))) = bitVal ? 0xFFF : 0;
}

/**************************************************************************/
/*! 
    @brief Sets the interrupt sense, event, etc.

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
    @param[in]  sense
                Whether the interrupt should be configured as edge or level
                sensitive.
    @param[in]  edge
                Whether one edge or both trigger an interrupt.
    @param[in]  event
                Whether the rising or the falling edge (high or low)
                should be used to trigger the interrupt.

    @section Example

    @code
    // Initialise gpio
    gpioInit();
    // Set GPIO1.8 to input
    gpioSetDir(1, 8, gpioDirection_Input);
    // Disable the internal pullup/down resistor on P1.8
    gpioSetPullup (&IOCON_PIO1_8, gpioPullupMode_Inactive);
    // Setup an interrupt on GPIO1.8
    gpioSetInterrupt(1,                               // Port
                     8,                               // Pin
                     gpioInterruptSense_Edge,         // Edge/Level Sensitive
                     gpioInterruptEdge_Single,        // Single/Double Edge
                     gpioInterruptEvent_ActiveHigh);  // Rising/Falling
    // Enable the interrupt
    gpioIntEnable(1, 8);
    @endcode
*/
/**************************************************************************/
void gpioSetInterrupt (uint32_t portNum, uint32_t bitPos, gpioInterruptSense_t sense, gpioInterruptEdge_t edge, gpioInterruptEvent_t event)
{
  // Get the appropriate register (handled this way to optimise code size)
  REG32 *gpiois  = &GPIO_GPIO0IS;   // Interrupt sense (edge or level sensitive)
  REG32 *gpioibe = &GPIO_GPIO0IBE;  // Interrupt both edges (0 = int controlled by GPIOIEV, 1 = both edges trigger interrupt)
  REG32 *gpioiev = &GPIO_GPIO0IEV;  // 0 = falling edge or low, 1 = rising edge or high (depending on GPIOIS)
  
  switch (portNum)
  {
    case 0:
      gpiois  = &GPIO_GPIO0IS;
      gpioibe = &GPIO_GPIO0IBE;
      gpioiev = &GPIO_GPIO0IEV;
      break;
    case 1:
      gpiois  = &GPIO_GPIO1IS;
      gpioibe = &GPIO_GPIO1IBE;
      gpioiev = &GPIO_GPIO1IEV;
      break;
    case 2:
      gpiois  = &GPIO_GPIO2IS;
      gpioibe = &GPIO_GPIO2IBE;
      gpioiev = &GPIO_GPIO2IEV;
      break;
    case 3:
      gpiois  = &GPIO_GPIO3IS;
      gpioibe = &GPIO_GPIO3IBE;
      gpioiev = &GPIO_GPIO3IEV;
      break;
  }

  if (sense == gpioInterruptSense_Edge)
  {
    *gpiois &= ~(0x1<<bitPos);
    edge == gpioInterruptEdge_Single ? (*gpioibe &= ~(0x1<<bitPos)) : (*gpioibe |= (0x1<<bitPos));
  }
  else
  {
    *gpiois |= (0x1<<bitPos);
  }

  event == gpioInterruptEvent_ActiveHigh ? (*gpioiev &= ~(0x1<<bitPos)) : (*gpioiev |= (0x1<<bitPos));

  return;
}

/**************************************************************************/
/*! 
    @brief Enables the interrupt mask for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
*/
/**************************************************************************/
void gpioIntEnable (uint32_t portNum, uint32_t bitPos)
{
  switch (portNum)
  {
    case 0:
      GPIO_GPIO0IE |= (0x1<<bitPos);
      break;
    case 1:
      GPIO_GPIO1IE |= (0x1<<bitPos);
      break;
    case 2:
      GPIO_GPIO2IE |= (0x1<<bitPos);
      break;
    case 3:
      GPIO_GPIO3IE |= (0x1<<bitPos);
      break;
    default:
      break;
  }
  return;
}

/**************************************************************************/
/*! 
    @brief Disables the interrupt mask for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
*/
/**************************************************************************/
void gpioIntDisable (uint32_t portNum, uint32_t bitPos)
{
  switch (portNum)
  {
    case 0:
      GPIO_GPIO0IE &= ~(0x1<<bitPos); 
      break;
    case 1:
      GPIO_GPIO1IE &= ~(0x1<<bitPos);	
      break;
    case 2:
      GPIO_GPIO2IE &= ~(0x1<<bitPos);	    
      break;
    case 3:
      GPIO_GPIO3IE &= ~(0x1<<bitPos);	    
      break;
    default:
      break;
  }
  return;
}

/**************************************************************************/
/*! 
    @brief Gets the interrupt status for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)

    @return     The interrupt status for the specified port pin (0..1)
*/
/**************************************************************************/
uint32_t gpioIntStatus (uint32_t portNum, uint32_t bitPos)
{
  uint32_t regVal = 0;

  switch (portNum)
  {
    case 0:
      if (GPIO_GPIO0MIS & (0x1<<bitPos))
      {
        regVal = 1;
      }
      break;
    case 1:
      if (GPIO_GPIO1MIS & (0x1<<bitPos))
      {
        regVal = 1;	
      }
      break;
    case 2:
      if (GPIO_GPIO2MIS & (0x1<<bitPos))
      {
        regVal = 1;
      }		
      break;
    case 3:
      if (GPIO_GPIO3MIS & (0x1<<bitPos))
      {
        regVal = 1;
      }
      break;
    default:
      break;
  }
  return ( regVal );
}

/**************************************************************************/
/*! 
    @brief Clears the interrupt for a port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
*/
/**************************************************************************/
void gpioIntClear (uint32_t portNum, uint32_t bitPos)
{
  switch (portNum)
  {
    case 0:
      GPIO_GPIO0IC |= (0x1<<bitPos); 
    break;
    case 1:
      GPIO_GPIO1IC |= (0x1<<bitPos);	
    break;
    case 2:
      GPIO_GPIO2IC |= (0x1<<bitPos);	    
    break;
    case 3:
      GPIO_GPIO3IC |= (0x1<<bitPos);	    
    break;
    default:
      break;
  }
  return;
}

/**************************************************************************/
/*! 
    @brief Configures the internal pullup/down resistor for GPIO pins
           (only relevant for pins configured as inputs)

    @param[in]  ioconReg
                A pointer to the IOCON registry value corresponding to
                the pin you wish to change (for example: &IOCON_PIO2_0
                for GPIO pin 2.0).
    @param[in]  mode
                The 'mode' that the pin should be set to, which must be
                correspond to a value defined in gpioPullupMode_t
    
    @warning    By default, all GPIO pins have the internal pull-up
                resistor enabled.  This may cause unusual behaviour if
                care isn't taken to set the internal resistor to an
                appropriate state.

    @section Example

    @code
    // Initialise gpio
    gpioInit();
    // Set GPIO1.8 to input
    gpioSetDir(1, 8, gpioDirection_Input);
    // Disable the internal pullup/down resistor on P1.8
    gpioSetPullup(&IOCON_PIO1_8, gpioPullupMode_Inactive);
    @endcode
*/
/**************************************************************************/
void gpioSetPullup (volatile uint32_t *ioconReg, gpioPullupMode_t mode)
{
  // ToDo: Disable interrupts while we are doing this?

  *ioconReg &= ~(IOCON_COMMON_MODE_MASK);
  *ioconReg |= mode;

  // ToDo: Re-enable interrupts?
};

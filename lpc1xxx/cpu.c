/**************************************************************************/
/*! 
    @file     cpu.c
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

    @section DESCRIPTION

    Initialises the CPU and any core clocks.  By default, the core clock
    is set to run at 12MHz.  In order to reduce power consumption all pins
    are set to GPIO and input by cpuInit.

    @section EXAMPLE
    @code
    #include "lpc111x.h"
    #include "core/cpu/cpu.h"

    int main (void)
    {
      // Initialise the CPU and setup the PLL
      cpuInit();
      
      while(1)
      {
      }
    }
    @endcode 
    
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

#include "cpu.h"

/**************************************************************************/
/*! 
    @brief Configures the PLL and main system clock

    The speed at which the MCU operates is set here using the SCB_PLLCTRL
    register, and the SCB_PLLCLKSEL register can be used to select which
    oscillator to use to generate the system clocks (the internal 12MHz
    oscillator or an external crystal).
*/
/**************************************************************************/
void cpu_pll_setup(uint32_t m, uint32_t p)
{
  uint32_t i;

  // Power up system oscillator
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_SYSOSC_MASK);

  // Setup the crystal input (bypass disabled, 1-20MHz crystal)
  SCB_SYSOSCCTRL = (SCB_SYSOSCCTRL_BYPASS_DISABLED | SCB_SYSOSCCTRL_FREQRANGE_15TO25MHZ);

  for (i = 0; i < 200; i++)
  {
    __asm volatile ("NOP");
  }

  // Configure PLL
  SCB_PLLCLKSEL = SCB_CLKSEL_SOURCE_MAINOSC;    // Use the external crystal
  SCB_PLLCLKUEN = SCB_PLLCLKUEN_UPDATE;         // Update clock source
  SCB_PLLCLKUEN = SCB_PLLCLKUEN_DISABLE;        // Toggle update register once
  SCB_PLLCLKUEN = SCB_PLLCLKUEN_UPDATE;         // Update clock source again
  
  // Wait until the clock is updated
  while (!(SCB_PLLCLKUEN & SCB_PLLCLKUEN_UPDATE));

  /*
  // Set clock speed
  switch (multiplier)
  {
    // Fclkout = M * Fclkin = FCCO / (2 * P)
    // FCCO should be in the range of 156-320MHz 
    // (see Table 44 of the LPC1114 usermanual for examples)
    case CPU_MULTIPLIER_2:
      // Fclkout = 24.0MHz
      // FCCO = 2 * 4 * 24 = 192MHz
      SCB_PLLCTRL = (SCB_PLLCTRL_MSEL_2 | SCB_PLLCTRL_PSEL_4);
      break;
    case CPU_MULTIPLIER_3:
      // Fclkout = 36.0MHz
      // FCCO = 2 * 4 * 36 = 288MHz
      SCB_PLLCTRL = (SCB_PLLCTRL_MSEL_3 | SCB_PLLCTRL_PSEL_4);
      break;
    case CPU_MULTIPLIER_4:
      // Fclkout = 48.0MHz
      // FCCO = 2 * 2 * 48 = 192MHz
      SCB_PLLCTRL = (SCB_PLLCTRL_MSEL_4 | SCB_PLLCTRL_PSEL_2);
      break;
    case CPU_MULTIPLIER_1:
    default:
      // Fclkout = 12.0MHz
      // FCCO = 2 * 8 * 12 = 192MHz
      SCB_PLLCTRL = (SCB_PLLCTRL_MSEL_1 | SCB_PLLCTRL_PSEL_8);
      break;
  }
  */
  SCB_PLLCTRL = m|p;

  // Enable system PLL
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_SYSPLL_MASK);

  // Wait for PLL to lock
  while (!(SCB_PLLSTAT & SCB_PLLSTAT_LOCK));
  
  // Setup main clock
  SCB_MAINCLKSEL = SCB_MAINCLKSEL_SOURCE_SYSPLLCLKOUT;
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_UPDATE;       // Update clock source
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_DISABLE;      // Toggle update register once
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_UPDATE;

  // Wait until the clock is updated
  while (!(SCB_MAINCLKUEN & SCB_MAINCLKUEN_UPDATE));

  // Set system AHB clock
  SCB_SYSAHBCLKDIV = SCB_SYSAHBCLKDIV_DIV1;

  // Enabled IOCON clock for I/O related peripherals
  SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_IOCON;
}


/**************************************************************************/
/*! 
    @brief Enables main clock output on PIO0_1.
*/
/**************************************************************************/
void cpu_enable_clkout(void)
{
  SCB_CLKOUTCLKSEL = 3;
  SCB_CLKOUTCLKDIV = SCB_CLKOUTCLKDIV_DIV1;
  SCB_CLKOUTCLKUEN = 0;
  SCB_CLKOUTCLKUEN = 1;
  IOCON_PIO0_1 = IOCON_PIO0_1_FUNC_CLKOUT;
}

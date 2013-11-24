/**************************************************************************/
/*! 
    @file     cpu.c
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

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

    Modified by Matt Sarnoff (msarnoff.org)
    September 24, 2013
*/
/**************************************************************************/

#include "cpu.h"


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


/* Enables main clock output on PIO0_1. */
void cpu_enable_clkout(void)
{
  SCB_CLKOUTCLKSEL = 3;
  SCB_CLKOUTCLKDIV = SCB_CLKOUTCLKDIV_DIV1;
  SCB_CLKOUTCLKUEN = 0;
  SCB_CLKOUTCLKUEN = 1;
  IOCON_PIO0_1 = IOCON_PIO0_1_FUNC_CLKOUT;
}


void cpu_reset(void)
{
  /* writing the magic value to the AIRCR register resets the CPU */
  SCB_AIRCR = SCB_AIRCR_VECTKEY_VALUE | SCB_AIRCR_SYSRESETREQ;
  while (1) {}
} 

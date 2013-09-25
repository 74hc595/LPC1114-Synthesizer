/**************************************************************************/
/*! 
    @file     lpc111x.h
    @author   K. Townsend (microBuilder.eu)
    @version  1.0

    LPC1114 header file

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

#ifndef _LPC111X_H_
#define _LPC111X_H_

#include <stdint.h>

typedef volatile uint8_t REG8;
typedef volatile uint16_t REG16;
typedef volatile uint32_t REG32;
typedef unsigned char byte_t;

#define pREG8  (REG8 *)
#define pREG16 (REG16 *)
#define pREG32 (REG32 *)

/*##############################################################################
## System Control Block
##############################################################################*/

#define SCB_BASE_ADDRESS                          (*(pREG32 (0x40048000)))    // System control block base address

/*  SCB_MEMREMAP (System memory remap register)
    The system memory remap register selects whether the ARM interrupt vectors are read
    from the boot ROM, the flash, or the SRAM.  */

#define SCB_MEMREMAP                              (*(pREG32 (0x40048000)))    // System memory remap
#define SCB_MEMREMAP_MODE_BOOTLOADER              ((unsigned int) 0x00000000) // Interrupt vectors are remapped to Boot ROM
#define SCB_MEMREMAP_MODE_RAM                     ((unsigned int) 0x00000001) // Interrupt vectors are remapped to Static ROM
#define SCB_MEMREMAP_MODE_FLASH                   ((unsigned int) 0x00000002) // Interrupt vectors are not remapped and reside in Flash
#define SCB_MEMREMAP_MASK                         ((unsigned int) 0x00000003)

/*  PRESETCTRL (Peripheral reset control register) */

#define SCB_PRESETCTRL                            (*(pREG32 (0x40048004)))    // Peripheral reset control
#define SCB_PRESETCTRL_SSP0_RESETENABLED          ((unsigned int) 0x00000000)
#define SCB_PRESETCTRL_SSP0_RESETDISABLED         ((unsigned int) 0x00000001)
#define SCB_PRESETCTRL_SSP0_MASK                  ((unsigned int) 0x00000001)
#define SCB_PRESETCTRL_I2C_RESETENABLED           ((unsigned int) 0x00000000)
#define SCB_PRESETCTRL_I2C_RESETDISABLED          ((unsigned int) 0x00000002)
#define SCB_PRESETCTRL_I2C_MASK                   ((unsigned int) 0x00000002)
#define SCB_PRESETCTRL_SSP1_RESETENABLED          ((unsigned int) 0x00000000)
#define SCB_PRESETCTRL_SSP1_RESETDISABLED         ((unsigned int) 0x00000004)
#define SCB_PRESETCTRL_SSP1_MASK                  ((unsigned int) 0x00000004)
#define SCB_PRESETCTRL_CAN_RESETENABLED           ((unsigned int) 0x00000000)
#define SCB_PRESETCTRL_CAN_RESETDISABLED          ((unsigned int) 0x00000008)
#define SCB_PRESETCTRL_CAN_MASK                   ((unsigned int) 0x00000008)

/*  SYSPLLCTRL (System PLL control register)
    This register connects and enables the system PLL and configures the PLL multiplier and
    divider values. The PLL accepts an input frequency from 10 MHz to 25 MHz from various
    clock sources. The input frequency is multiplied up to a high frequency, then divided down
    to provide the actual clock used by the CPU, peripherals, and optionally the USB
    subsystem. Note that the USB subsystem has its own dedicated PLL. The PLL can
    produce a clock up to the maximum allowed for the CPU, which is 50 MHz. */

#define SCB_PLLCTRL                               (*(pREG32 (0x40048008)))    // System PLL control
#define SCB_PLLCTRL_MSEL_1                        ((unsigned int) 0x00000000)
#define SCB_PLLCTRL_MSEL_2                        ((unsigned int) 0x00000001)
#define SCB_PLLCTRL_MSEL_3                        ((unsigned int) 0x00000002)
#define SCB_PLLCTRL_MSEL_4                        ((unsigned int) 0x00000003)
#define SCB_PLLCTRL_MSEL_5                        ((unsigned int) 0x00000004)
#define SCB_PLLCTRL_MSEL_6                        ((unsigned int) 0x00000005)
#define SCB_PLLCTRL_MSEL_7                        ((unsigned int) 0x00000006)
#define SCB_PLLCTRL_MSEL_8                        ((unsigned int) 0x00000007)
#define SCB_PLLCTRL_MSEL_9                        ((unsigned int) 0x00000008)
#define SCB_PLLCTRL_MSEL_10                       ((unsigned int) 0x00000009)
#define SCB_PLLCTRL_MSEL_11                       ((unsigned int) 0x0000000A)
#define SCB_PLLCTRL_MSEL_12                       ((unsigned int) 0x0000000B)
#define SCB_PLLCTRL_MSEL_13                       ((unsigned int) 0x0000000C)
#define SCB_PLLCTRL_MSEL_14                       ((unsigned int) 0x0000000D)
#define SCB_PLLCTRL_MSEL_15                       ((unsigned int) 0x0000000E)
#define SCB_PLLCTRL_MSEL_16                       ((unsigned int) 0x0000000F)
#define SCB_PLLCTRL_MSEL_17                       ((unsigned int) 0x00000010)
#define SCB_PLLCTRL_MSEL_18                       ((unsigned int) 0x00000011)
#define SCB_PLLCTRL_MSEL_19                       ((unsigned int) 0x00000012)
#define SCB_PLLCTRL_MSEL_20                       ((unsigned int) 0x00000013)
#define SCB_PLLCTRL_MSEL_21                       ((unsigned int) 0x00000014)
#define SCB_PLLCTRL_MSEL_22                       ((unsigned int) 0x00000015)
#define SCB_PLLCTRL_MSEL_23                       ((unsigned int) 0x00000016)
#define SCB_PLLCTRL_MSEL_24                       ((unsigned int) 0x00000017)
#define SCB_PLLCTRL_MSEL_25                       ((unsigned int) 0x00000018)
#define SCB_PLLCTRL_MSEL_26                       ((unsigned int) 0x00000019)
#define SCB_PLLCTRL_MSEL_27                       ((unsigned int) 0x0000001A)
#define SCB_PLLCTRL_MSEL_28                       ((unsigned int) 0x0000001B)
#define SCB_PLLCTRL_MSEL_29                       ((unsigned int) 0x0000001C)
#define SCB_PLLCTRL_MSEL_30                       ((unsigned int) 0x0000001D)
#define SCB_PLLCTRL_MSEL_31                       ((unsigned int) 0x0000001E)
#define SCB_PLLCTRL_MSEL_32                       ((unsigned int) 0x0000001F)
#define SCB_PLLCTRL_MSEL_MASK                     ((unsigned int) 0x0000001F)    
#define SCB_PLLCTRL_PSEL_2                        ((unsigned int) 0x00000000)       
#define SCB_PLLCTRL_PSEL_4                        ((unsigned int) 0x00000020)       
#define SCB_PLLCTRL_PSEL_8                        ((unsigned int) 0x00000040)       
#define SCB_PLLCTRL_PSEL_16                       ((unsigned int) 0x00000060)
#define SCB_PLLCTRL_PSEL_BIT                      (5)
#define SCB_PLLCTRL_PSEL_MASK                     ((unsigned int) 0x00000060)    
#define SCB_PLLCTRL_MASK                          ((unsigned int) 0x0000007F)    

/*  SYSPLLSTAT (System PLL status register)
    This register is a Read-only register and supplies the PLL lock status */

#define SCB_PLLSTAT                               (*(pREG32 (0x4004800C)))    // System PLL status
#define SCB_PLLSTAT_LOCK                          ((unsigned int) 0x00000001) // 0 = PLL not locked, 1 = PLL locked
#define SCB_PLLSTAT_LOCK_MASK                     ((unsigned int) 0x00000001)    

/*  SYSOSCCTRL (System oscillator control register)
    This register configures the frequency range for the system oscillator. */

#define SCB_SYSOSCCTRL                            (*(pREG32 (0x40048020)))    // System oscillator control
#define SCB_SYSOSCCTRL_BYPASS_DISABLED            ((unsigned int) 0x00000000) // Oscillator is not bypassed.
#define SCB_SYSOSCCTRL_BYPASS_ENABLED             ((unsigned int) 0x00000001) // Bypass enabled
#define SCB_SYSOSCCTRL_BYPASS_MASK                ((unsigned int) 0x00000001)
#define SCB_SYSOSCCTRL_FREQRANGE_1TO20MHZ         ((unsigned int) 0x00000000) // 1-20 MHz frequency range
#define SCB_SYSOSCCTRL_FREQRANGE_15TO25MHZ        ((unsigned int) 0x00000002) // 15-25 MHz frequency range
#define SCB_SYSOSCCTRL_FREQRANGE_MASK             ((unsigned int) 0x00000002)

/*  WDTOSCTRL (Watchdog oscillator control register)
    This register configures the watchdog oscillator. The oscillator consists of an analog and a
    digital part. The analog part contains the oscillator function and generates an analog clock
    (Fclkana). With the digital part, the analog output clock (Fclkana) can be divided to the
    required output clock frequency wdt_osc_clk. The analog output frequency (Fclkana) can
    be adjusted with the FREQSEL bits between 500 kHz and 3.7 MHz. With the digital part
    Fclkana will be divided (divider ratios = 2, 4,...,64) to wdt_osc_clk using the DIVSEL bits.*/

#define SCB_WDTOSCCTRL                            (*(pREG32 (0x40048024)))    // Watchdog oscillator control
#define SCB_WDTOSCCTRL_DIVSEL_DIV2                ((unsigned int) 0x00000000) // Reset value
#define SCB_WDTOSCCTRL_DIVSEL_DIV4                ((unsigned int) 0x00000001)
#define SCB_WDTOSCCTRL_DIVSEL_DIV6                ((unsigned int) 0x00000002)
#define SCB_WDTOSCCTRL_DIVSEL_DIV8                ((unsigned int) 0x00000003)
#define SCB_WDTOSCCTRL_DIVSEL_DIV10               ((unsigned int) 0x00000004)
#define SCB_WDTOSCCTRL_DIVSEL_DIV12               ((unsigned int) 0x00000005)
#define SCB_WDTOSCCTRL_DIVSEL_DIV14               ((unsigned int) 0x00000006)
#define SCB_WDTOSCCTRL_DIVSEL_DIV16               ((unsigned int) 0x00000007)
#define SCB_WDTOSCCTRL_DIVSEL_DIV18               ((unsigned int) 0x00000008)
#define SCB_WDTOSCCTRL_DIVSEL_DIV20               ((unsigned int) 0x00000009)
#define SCB_WDTOSCCTRL_DIVSEL_DIV22               ((unsigned int) 0x0000000A)
#define SCB_WDTOSCCTRL_DIVSEL_DIV24               ((unsigned int) 0x0000000B)
#define SCB_WDTOSCCTRL_DIVSEL_DIV26               ((unsigned int) 0x0000000C)
#define SCB_WDTOSCCTRL_DIVSEL_DIV28               ((unsigned int) 0x0000000D)
#define SCB_WDTOSCCTRL_DIVSEL_DIV30               ((unsigned int) 0x0000000E)
#define SCB_WDTOSCCTRL_DIVSEL_DIV32               ((unsigned int) 0x0000000F)
#define SCB_WDTOSCCTRL_DIVSEL_DIV34               ((unsigned int) 0x00000010)
#define SCB_WDTOSCCTRL_DIVSEL_DIV36               ((unsigned int) 0x00000011)
#define SCB_WDTOSCCTRL_DIVSEL_DIV38               ((unsigned int) 0x00000012)
#define SCB_WDTOSCCTRL_DIVSEL_DIV40               ((unsigned int) 0x00000013)
#define SCB_WDTOSCCTRL_DIVSEL_DIV42               ((unsigned int) 0x00000014)
#define SCB_WDTOSCCTRL_DIVSEL_DIV44               ((unsigned int) 0x00000015)
#define SCB_WDTOSCCTRL_DIVSEL_DIV46               ((unsigned int) 0x00000016)
#define SCB_WDTOSCCTRL_DIVSEL_DIV48               ((unsigned int) 0x00000017)
#define SCB_WDTOSCCTRL_DIVSEL_DIV50               ((unsigned int) 0x00000018)
#define SCB_WDTOSCCTRL_DIVSEL_DIV52               ((unsigned int) 0x00000019)
#define SCB_WDTOSCCTRL_DIVSEL_DIV54               ((unsigned int) 0x0000001A)
#define SCB_WDTOSCCTRL_DIVSEL_DIV56               ((unsigned int) 0x0000001B)
#define SCB_WDTOSCCTRL_DIVSEL_DIV58               ((unsigned int) 0x0000001C)
#define SCB_WDTOSCCTRL_DIVSEL_DIV60               ((unsigned int) 0x0000001D)
#define SCB_WDTOSCCTRL_DIVSEL_DIV62               ((unsigned int) 0x0000001E)
#define SCB_WDTOSCCTRL_DIVSEL_DIV64               ((unsigned int) 0x0000001F)
#define SCB_WDTOSCCTRL_DIVSEL_MASK                ((unsigned int) 0x0000001F)
#define SCB_WDTOSCCTRL_FREQSEL_0_5MHZ             ((unsigned int) 0x00000020)
#define SCB_WDTOSCCTRL_FREQSEL_0_8MHZ             ((unsigned int) 0x00000040)
#define SCB_WDTOSCCTRL_FREQSEL_1_1MHZ             ((unsigned int) 0x00000060)
#define SCB_WDTOSCCTRL_FREQSEL_1_4MHZ             ((unsigned int) 0x00000080)
#define SCB_WDTOSCCTRL_FREQSEL_1_6MHZ             ((unsigned int) 0x000000A0) // Reset value
#define SCB_WDTOSCCTRL_FREQSEL_1_8MHZ             ((unsigned int) 0x000000C0)
#define SCB_WDTOSCCTRL_FREQSEL_2_0MHZ             ((unsigned int) 0x000000E0)
#define SCB_WDTOSCCTRL_FREQSEL_2_2MHZ             ((unsigned int) 0x00000100)
#define SCB_WDTOSCCTRL_FREQSEL_2_4MHZ             ((unsigned int) 0x00000120)
#define SCB_WDTOSCCTRL_FREQSEL_2_6MHZ             ((unsigned int) 0x00000140)
#define SCB_WDTOSCCTRL_FREQSEL_2_7MHZ             ((unsigned int) 0x00000160)
#define SCB_WDTOSCCTRL_FREQSEL_2_9MHZ             ((unsigned int) 0x00000180)
#define SCB_WDTOSCCTRL_FREQSEL_3_1MHZ             ((unsigned int) 0x000001A0)
#define SCB_WDTOSCCTRL_FREQSEL_3_2MHZ             ((unsigned int) 0x000001C0)
#define SCB_WDTOSCCTRL_FREQSEL_3_4MHZ             ((unsigned int) 0x000001E0)
#define SCB_WDTOSCCTRL_FREQSEL_MASK               ((unsigned int) 0x000001E0)

/*  IRCCTRL (Internal resonant crystal control register)
    This register is used to trim the on-chip 12 MHz oscillator. The trim value is factory-preset
    and written by the boot code on start-up. */

#define SCB_IRCCTRL                               (*(pREG32 (0x40048028)))    // IRC control
#define SCB_IRCCTRL_MASK                          ((unsigned int) 0x000000FF)

/*  SYSRSTSTAT (System reset status register)
    The SYSRSTSTAT register shows the source of the latest reset event. The bits are
    cleared by writing a one to any of the bits. The POR event clears all other bits in this
    register, but if another reset signal (e.g., EXTRST) remains asserted after the POR signal
    is negated, then its bit is set to detected. */

#define SCB_RESETSTAT                             (*(pREG32 (0x40048030)))    // System reset status register
#define SCB_RESETSTAT_POR_MASK                    ((unsigned int) 0x00000001) // POR reset status
#define SCB_RESETSTAT_EXTRST_MASK                 ((unsigned int) 0x00000002) // Status of the external reset pin
#define SCB_RESETSTAT_WDT_MASK                    ((unsigned int) 0x00000004) // Status of the watchdog reset
#define SCB_RESETSTAT_BOD_MASK                    ((unsigned int) 0x00000008) // Status of the brown-out detect reset
#define SCB_RESETSTAT_SYSRST_MASK                 ((unsigned int) 0x00000010) // Status of the software system reset
#define SCB_RESETSTAT_MASK                        ((unsigned int) 0x00000010)

/*  SYSPLLCLKSEL (System PLL clock source select register)
    This register selects the clock source for the system PLL. The SYSPLLCLKUEN register
    must be toggled from LOW to HIGH for the update to take effect.
*/

#define SCB_PLLCLKSEL                             (*(pREG32 (0x40048040)))    // System PLL clock source select
#define SCB_CLKSEL_SOURCE_INTERNALOSC             ((unsigned int) 0x00000000)
#define SCB_CLKSEL_SOURCE_MAINOSC                 ((unsigned int) 0x00000001)
#define SCB_CLKSEL_SOURCE_MASK                    ((unsigned int) 0x00000002)

/*  SYSPLLUEN (System PLL clock source update enable register)
    This register updates the clock source of the system PLL with the new input clock after the
    SYSPLLCLKSEL register has been written to. In order for the update to take effect, first
    write a zero to the SYSPLLUEN register and then write a one to SYSPLLUEN. */

#define SCB_PLLCLKUEN                             (*(pREG32 (0x40048044)))    // System PLL clock source update enable
#define SCB_PLLCLKUEN_DISABLE                     ((unsigned int) 0x00000000)
#define SCB_PLLCLKUEN_UPDATE                      ((unsigned int) 0x00000001)
#define SCB_PLLCLKUEN_MASK                        ((unsigned int) 0x00000001)

/*  MAINCLKSEL (Main clock source select register)
    This register selects the main system clock which can be either the output from the
    system PLL or the IRC, system, or Watchdog oscillators directly. The main system clock
    clocks the core, the peripherals, and optionally the USB block.
    The MAINCLKUEN register must be toggled from LOW to HIGH for the update to take effect.*/

#define SCB_MAINCLKSEL                            (*(pREG32 (0x40048070)))    // Main clock source select
#define SCB_MAINCLKSEL_SOURCE_INTERNALOSC         ((unsigned int) 0x00000000) // Use IRC oscillator for main clock source
#define SCB_MAINCLKSEL_SOURCE_INPUTCLOCK          ((unsigned int) 0x00000001) // Use Input clock to system PLL for main clock source
#define SCB_MAINCLKSEL_SOURCE_WDTOSC              ((unsigned int) 0x00000002) // Use watchdog oscillator for main clock source
#define SCB_MAINCLKSEL_SOURCE_SYSPLLCLKOUT        ((unsigned int) 0x00000003) // Use system PLL clock out for main clock source
#define SCB_MAINCLKSEL_MASK                       ((unsigned int) 0x00000003)

/*  MAINCLKUEN (Main clock source update enable register)
    This register updates the clock source of the main clock with the new input clock after the
    MAINCLKSEL register has been written to. In order for the update to take effect, first write
    a zero to the MAINUEN register and then write a one to MAINCLKUEN. */

#define SCB_MAINCLKUEN                            (*(pREG32 (0x40048074)))    // Main clock source update enable
#define SCB_MAINCLKUEN_DISABLE                    ((unsigned int) 0x00000000)
#define SCB_MAINCLKUEN_UPDATE                     ((unsigned int) 0x00000001)
#define SCB_MAINCLKUEN_MASK                       ((unsigned int) 0x00000001)

/*  SYSAHBCLKDIV (System AHB clock divider register)
    This register divides the main clock to provide the system clock to the core, memories,
    and the peripherals. The system clock can be shut down completely by setting the DIV
    bits to 0x0. */

#define SCB_SYSAHBCLKDIV                          (*(pREG32 (0x40048078)))    // System AHB clock divider
#define SCB_SYSAHBCLKDIV_DISABLE                  ((unsigned int) 0x00000000) // 0 will shut the system clock down completely
#define SCB_SYSAHBCLKDIV_DIV1                     ((unsigned int) 0x00000001) // 1, 2 or 4 are the most common values
#define SCB_SYSAHBCLKDIV_DIV2                     ((unsigned int) 0x00000002)    
#define SCB_SYSAHBCLKDIV_DIV4                     ((unsigned int) 0x00000004)    
#define SCB_SYSAHBCLKDIV_MASK                     ((unsigned int) 0x000000FF) // AHB clock divider can be from 0 to 255

/*  AHBCLKCTRL (System AHB clock control register)
    The AHBCLKCTRL register enables the clocks to individual system and peripheral blocks.
    The system clock (sys_ahb_clk[0], bit 0 in the AHBCLKCTRL register) provides the clock
    for the AHB to APB bridge, the AHB matrix, the ARM Cortex-M0, the Syscon block, and
    the PMU. This clock cannot be disabled. */

#define SCB_SYSAHBCLKCTRL                         (*(pREG32 (0x40048080)))    // System AHB clock control
#define SCB_SYSAHBCLKCTRL_SYS                     ((unsigned int) 0x00000001) // Enables clock for AHB and APB bridges, FCLK, HCLK, SysCon and PMU
#define SCB_SYSAHBCLKCTRL_SYS_MASK                ((unsigned int) 0x00000001)
#define SCB_SYSAHBCLKCTRL_ROM                     ((unsigned int) 0x00000002) // Enables clock for ROM
#define SCB_SYSAHBCLKCTRL_ROM_MASK                ((unsigned int) 0x00000002)
#define SCB_SYSAHBCLKCTRL_RAM                     ((unsigned int) 0x00000004) // Enables clock for SRAM
#define SCB_SYSAHBCLKCTRL_RAM_MASK                ((unsigned int) 0x00000004)
#define SCB_SYSAHBCLKCTRL_FLASHREG                ((unsigned int) 0x00000008) // Enables clock for flash register interface
#define SCB_SYSAHBCLKCTRL_FLASHREG_MASK           ((unsigned int) 0x00000008)
#define SCB_SYSAHBCLKCTRL_FLASHARRAY              ((unsigned int) 0x00000010) // Enables clock for flash array interface
#define SCB_SYSAHBCLKCTRL_FLASHARRAY_MASK         ((unsigned int) 0x00000010)
#define SCB_SYSAHBCLKCTRL_I2C                     ((unsigned int) 0x00000020) // Enables clock for I2C
#define SCB_SYSAHBCLKCTRL_I2C_MASK                ((unsigned int) 0x00000020)
#define SCB_SYSAHBCLKCTRL_GPIO                    ((unsigned int) 0x00000040) // Enables clock for GPIO
#define SCB_SYSAHBCLKCTRL_GPIO_MASK               ((unsigned int) 0x00000040)
#define SCB_SYSAHBCLKCTRL_CT16B0                  ((unsigned int) 0x00000080) // Enables clock for 16-bit counter/timer 0
#define SCB_SYSAHBCLKCTRL_CT16B0_MASK             ((unsigned int) 0x00000080)
#define SCB_SYSAHBCLKCTRL_CT16B1                  ((unsigned int) 0x00000100) // Enables clock for 16-bit counter/timer 1
#define SCB_SYSAHBCLKCTRL_CT16B1_MASK             ((unsigned int) 0x00000100)
#define SCB_SYSAHBCLKCTRL_CT32B0                  ((unsigned int) 0x00000200) // Enables clock for 32-bit counter/timer 0
#define SCB_SYSAHBCLKCTRL_CT32B0_MASK             ((unsigned int) 0x00000200)
#define SCB_SYSAHBCLKCTRL_CT32B1                  ((unsigned int) 0x00000400) // Enables clock for 32-bit counter/timer 1
#define SCB_SYSAHBCLKCTRL_CT32B1_MASK             ((unsigned int) 0x00000400)
#define SCB_SYSAHBCLKCTRL_SSP0                    ((unsigned int) 0x00000800) // Enables clock for SSP0
#define SCB_SYSAHBCLKCTRL_SSP0_MASK               ((unsigned int) 0x00000800)
#define SCB_SYSAHBCLKCTRL_UART                    ((unsigned int) 0x00001000) // Enables clock for UART.  UART pins must be configured
#define SCB_SYSAHBCLKCTRL_UART_MASK               ((unsigned int) 0x00001000) // in the IOCON block before the UART clock can be enabled.
#define SCB_SYSAHBCLKCTRL_ADC                     ((unsigned int) 0x00002000) // Enables clock for ADC
#define SCB_SYSAHBCLKCTRL_ADC_MASK                ((unsigned int) 0x00002000)
#define SCB_SYSAHBCLKCTRL_WDT                     ((unsigned int) 0x00008000) // Enables clock for watchdog timer
#define SCB_SYSAHBCLKCTRL_WDT_MASK                ((unsigned int) 0x00008000)
#define SCB_SYSAHBCLKCTRL_IOCON                   ((unsigned int) 0x00010000) // Enables clock for IO configuration block
#define SCB_SYSAHBCLKCTRL_IOCON_MASK              ((unsigned int) 0x00010000)
#define SCB_SYSAHBCLKCTRL_CAN                     ((unsigned int) 0x00020000) // Enables clock for CAN
#define SCB_SYSAHBCLKCTRL_CAN_MASK                ((unsigned int) 0x00020000)
#define SCB_SYSAHBCLKCTRL_SSP1                    ((unsigned int) 0x00040000) // Enables clock for SSP1
#define SCB_SYSAHBCLKCTRL_SSP1_MASK               ((unsigned int) 0x00040000)
#define SCB_SYSAHBCLKCTRL_ALL_MASK                ((unsigned int) 0x0007FFFF)

/*  SSP0CLKDIV (SSP0 clock divider register)
    This register configures the SSP0 peripheral clock SSP0_PCLK. The SSP0_PCLK can be
    shut down by setting the DIV bits to 0x0.  It can be set from 1..255. */

#define SCB_SSP0CLKDIV                            (*(pREG32 (0x40048094)))    // SSP0 clock divider
#define SCB_SSP0CLKDIV_DISABLE                    ((unsigned int) 0x00000000)
#define SCB_SSP0CLKDIV_DIV1                       ((unsigned int) 0x00000001) // Divide SSP0 clock by 1 (can be set from 1..255)
#define SCB_SSP0CLKDIV_DIV2                       ((unsigned int) 0x00000002)
#define SCB_SSP0CLKDIV_DIV3                       ((unsigned int) 0x00000003)
#define SCB_SSP0CLKDIV_DIV4                       ((unsigned int) 0x00000004)
#define SCB_SSP0CLKDIV_DIV5                       ((unsigned int) 0x00000005)
#define SCB_SSP0CLKDIV_DIV6                       ((unsigned int) 0x00000006)
#define SCB_SSP0CLKDIV_DIV10                      ((unsigned int) 0x0000000A)
#define SCB_SSP0CLKDIV_MASK                       ((unsigned int) 0x000000FF)

/*  UARTCLKDIV (UART clock divider register)
    This register configures the UART peripheral. The UART_PCLK can be shut down by
    setting the DIV bits to 0x0.
    Remark: Note that the UART pins must be configured in the IOCON block before the
    UART clock can be enabled.  */

#define SCB_UARTCLKDIV                            (*(pREG32 (0x40048098)))    // UART clock divider
#define SCB_UARTCLKDIV_DISABLE                    ((unsigned int) 0x00000000)
#define SCB_UARTCLKDIV_DIV1                       ((unsigned int) 0x00000001) // Divide UART clock by 1 (can be set from 1..255)
#define SCB_UARTCLKDIV_DIV2                       ((unsigned int) 0x00000002)
#define SCB_UARTCLKDIV_DIV4                       ((unsigned int) 0x00000004)
#define SCB_UARTCLKDIV_MASK                       ((unsigned int) 0x000000FF)

/*  SSP1CLKDIV (SSP1 clock divider register)
    This register configures the SSP1 peripheral clock SSP1_PCLK. The SSP1_PCLK can be
    shut down by setting the DIV bits to 0x0.  It can be set from 1..255. */

#define SCB_SSP1CLKDIV                            (*(pREG32 (0x4004809C)))    // SSP1 clock divider
#define SCB_SSP1CLKDIV_DISABLE                    ((unsigned int) 0x00000000)
#define SCB_SSP1CLKDIV_DIV1                       ((unsigned int) 0x00000001) // Divide SSP1 clock by 1 (can be set from 1..255)
#define SCB_SSP1CLKDIV_DIV2                       ((unsigned int) 0x00000002)
#define SCB_SSP1CLKDIV_DIV3                       ((unsigned int) 0x00000003)
#define SCB_SSP1CLKDIV_DIV4                       ((unsigned int) 0x00000004)
#define SCB_SSP1CLKDIV_DIV5                       ((unsigned int) 0x00000005)
#define SCB_SSP1CLKDIV_DIV6                       ((unsigned int) 0x00000006)
#define SCB_SSP1CLKDIV_DIV10                      ((unsigned int) 0x0000000A)
#define SCB_SSP1CLKDIV_MASK                       ((unsigned int) 0x000000FF)

/*  WDTCLKSEL (WDT clock source select register)
    This register selects the clock source for the watchdog timer. The WDTCLKUEN register
    must be toggled from LOW to HIGH for the update to take effect.  */

#define SCB_WDTCLKSEL                             (*(pREG32 (0x400480D0)))    // Watchdog clock source select
#define SCB_WDTCLKSEL_SOURCE_INTERNALOSC          ((unsigned int) 0x00000000) // Use the internal oscillator
#define SCB_WDTCLKSEL_SOURCE_INPUTCLOCK           ((unsigned int) 0x00000001) // Use the main clock
#define SCB_WDTCLKSEL_SOURCE_WATCHDOGOSC          ((unsigned int) 0x00000002) // Use the watchdog oscillator
#define SCB_WDTCLKSEL_MASK                        ((unsigned int) 0x00000003)    

/*  WDTCLKUEN (WDT clock source update enable register)
    This register updates the clock source of the watchdog timer with the new input clock after
    the WDTCLKSEL register has been written to. In order for the update to take effect at the
    input of the watchdog timer, first write a zero to the WDTCLKUEN register and then write
    a one to WDTCLKUEN.  */

#define SCB_WDTCLKUEN                             (*(pREG32 (0x400480D4)))    // Watchdog clock source update enable
#define SCB_WDTCLKUEN_DISABLE                     ((unsigned int) 0x00000000)
#define SCB_WDTCLKUEN_UPDATE                      ((unsigned int) 0x00000001)
#define SCB_WDTCLKUEN_MASK                        ((unsigned int) 0x00000001)

/*  WDTCLKDIV (WDT clock divider register)
    This register determines the divider values for the watchdog clock wdt_clk. */

#define SCB_WDTCLKDIV                             (*(pREG32 (0x400480D8)))    // Watchdog clock divider
#define SCB_WDTCLKDIV_DISABLE                     ((unsigned int) 0x00000000)
#define SCB_WDTCLKDIV_DIV1                        ((unsigned int) 0x00000001) // Divide clock by 1 (can be set from 1..255)
#define SCB_WDTCLKDIV_MASK                        ((unsigned int) 0x000000FF)

/*  CLKOUTCLKSEL (CLKOUT clock source select register)
    This register configures the clkout_clk signal to be output on the CLKOUT pin. All three
    oscillators and the main clock can be selected for the clkout_clk clock.
    The CLKOUTCLKUEN register must be toggled from LOW to HIGH for the update to take effect. */

#define SCB_CLKOUTCLKSEL                          (*(pREG32 (0x400480E0)))    // CLKOUT clock source select
#define SCB_CLKOUTCLKSEL_SOURCE_USBPLLOUT         ((unsigned int) 0x00000000) // USB PLL output
#define SCB_CLKOUTCLKSEL_SOURCE_INPUTCLOCK        ((unsigned int) 0x00000001) // Use the main clock
#define SCB_CLKOUTCLKSEL_SOURCE_WATCHDOGOSC       ((unsigned int) 0x00000002) // Use the watchdog oscillator
#define SCB_CLKOUTCLKSEL_MASK                     ((unsigned int) 0x00000003)

/*  CLKOUTUEN (CLKOUT clock source update enable register)
    This register updates the clock source of the CLKOUT pin with the new clock after the
    CLKOUTCLKSEL register has been written to. In order for the update to take effect at the
    input of the CLKOUT pin, first write a zero to the CLKCLKUEN register and then write a
    one to CLKCLKUEN. */

#define SCB_CLKOUTCLKUEN                          (*(pREG32 (0x400480E4)))    // CLKOUT clock source update enable
#define SCB_CLKOUTCLKUEN_DISABLE                  ((unsigned int) 0x00000000)
#define SCB_CLKOUTCLKUEN_UPDATE                   ((unsigned int) 0x00000001)
#define SCB_CLKOUTCLKUEN_MASK                     ((unsigned int) 0x00000001)

/*  CLKOUTCLKDIV (CLKOUT clock divider register)
    This register determines the divider value for the clkout_clk signal on the CLKOUT pin. */

#define SCB_CLKOUTCLKDIV                          (*(pREG32 (0x400480E8)))    // CLKOUT clock divider
#define SCB_CLKOUTCLKDIV_DISABLE                  ((unsigned int) 0x00000000)
#define SCB_CLKOUTCLKDIV_DIV1                     ((unsigned int) 0x00000001) // Divide clock by 1 (can be set from 1..255)
#define SCB_CLKOUTCLKDIV_MASK                     ((unsigned int) 0x000000FF)

/*  PIOPORCAP0 (POR captured PIO status register 0)
    The PIOPORCAP0 register captures the state (HIGH or LOW) of the PIO pins of ports 0,1,
    and 2 (pins PIO2_0 to PIO2_7) at power-on-reset. Each bit represents the reset state of
    one GPIO pin. This register is a read-only status register.  */

#define SCB_PIOPORCAP0                            (*(pREG32 (0x40048100)))    // POR captured PIO status 0
#define SCB_PIOPORCAP0_PIO0_0                     ((unsigned int) 0x00000001)
#define SCB_PIOPORCAP0_PIO0_0_MASK                ((unsigned int) 0x00000001)
#define SCB_PIOPORCAP0_PIO0_1                     ((unsigned int) 0x00000002)
#define SCB_PIOPORCAP0_PIO0_1_MASK                ((unsigned int) 0x00000002)
#define SCB_PIOPORCAP0_PIO0_2                     ((unsigned int) 0x00000004)
#define SCB_PIOPORCAP0_PIO0_2_MASK                ((unsigned int) 0x00000004)
#define SCB_PIOPORCAP0_PIO0_3                     ((unsigned int) 0x00000008)
#define SCB_PIOPORCAP0_PIO0_3_MASK                ((unsigned int) 0x00000008)
#define SCB_PIOPORCAP0_PIO0_4                     ((unsigned int) 0x00000010)
#define SCB_PIOPORCAP0_PIO0_4_MASK                ((unsigned int) 0x00000010)
#define SCB_PIOPORCAP0_PIO0_5                     ((unsigned int) 0x00000020)
#define SCB_PIOPORCAP0_PIO0_5_MASK                ((unsigned int) 0x00000020)
#define SCB_PIOPORCAP0_PIO0_6                     ((unsigned int) 0x00000040)
#define SCB_PIOPORCAP0_PIO0_6_MASK                ((unsigned int) 0x00000040)
#define SCB_PIOPORCAP0_PIO0_7                     ((unsigned int) 0x00000080)
#define SCB_PIOPORCAP0_PIO0_7_MASK                ((unsigned int) 0x00000080)
#define SCB_PIOPORCAP0_PIO0_8                     ((unsigned int) 0x00000100)
#define SCB_PIOPORCAP0_PIO0_8_MASK                ((unsigned int) 0x00000100)
#define SCB_PIOPORCAP0_PIO0_9                     ((unsigned int) 0x00000200)
#define SCB_PIOPORCAP0_PIO0_9_MASK                ((unsigned int) 0x00000200)
#define SCB_PIOPORCAP0_PIO0_10                    ((unsigned int) 0x00000400)
#define SCB_PIOPORCAP0_PIO0_10_MASK               ((unsigned int) 0x00000400)
#define SCB_PIOPORCAP0_PIO0_11                    ((unsigned int) 0x00000800)
#define SCB_PIOPORCAP0_PIO0_11_MASK               ((unsigned int) 0x00000800)
#define SCB_PIOPORCAP0_PIO1_0                     ((unsigned int) 0x00001000)
#define SCB_PIOPORCAP0_PIO1_0_MASK                ((unsigned int) 0x00001000)
#define SCB_PIOPORCAP0_PIO1_1                     ((unsigned int) 0x00002000)
#define SCB_PIOPORCAP0_PIO1_1_MASK                ((unsigned int) 0x00002000)
#define SCB_PIOPORCAP0_PIO1_2                     ((unsigned int) 0x00004000)
#define SCB_PIOPORCAP0_PIO1_2_MASK                ((unsigned int) 0x00004000)
#define SCB_PIOPORCAP0_PIO1_3                     ((unsigned int) 0x00008000)
#define SCB_PIOPORCAP0_PIO1_3_MASK                ((unsigned int) 0x00008000)
#define SCB_PIOPORCAP0_PIO1_4                     ((unsigned int) 0x00010000)
#define SCB_PIOPORCAP0_PIO1_4_MASK                ((unsigned int) 0x00010000)
#define SCB_PIOPORCAP0_PIO1_5                     ((unsigned int) 0x00020000)
#define SCB_PIOPORCAP0_PIO1_5_MASK                ((unsigned int) 0x00020000)
#define SCB_PIOPORCAP0_PIO1_6                     ((unsigned int) 0x00040000)
#define SCB_PIOPORCAP0_PIO1_6_MASK                ((unsigned int) 0x00040000)
#define SCB_PIOPORCAP0_PIO1_7                     ((unsigned int) 0x00080000)
#define SCB_PIOPORCAP0_PIO1_7_MASK                ((unsigned int) 0x00080000)
#define SCB_PIOPORCAP0_PIO1_8                     ((unsigned int) 0x00100000)
#define SCB_PIOPORCAP0_PIO1_8_MASK                ((unsigned int) 0x00100000)
#define SCB_PIOPORCAP0_PIO1_9                     ((unsigned int) 0x00200000)
#define SCB_PIOPORCAP0_PIO1_9_MASK                ((unsigned int) 0x00200000)
#define SCB_PIOPORCAP0_PIO1_10                    ((unsigned int) 0x00400000)
#define SCB_PIOPORCAP0_PIO1_10_MASK               ((unsigned int) 0x00400000)
#define SCB_PIOPORCAP0_PIO1_11                    ((unsigned int) 0x00800000)
#define SCB_PIOPORCAP0_PIO1_11_MASK               ((unsigned int) 0x00800000)
#define SCB_PIOPORCAP0_PIO2_0                     ((unsigned int) 0x01000000)
#define SCB_PIOPORCAP0_PIO2_0_MASK                ((unsigned int) 0x01000000)
#define SCB_PIOPORCAP0_PIO2_1                     ((unsigned int) 0x02000000)
#define SCB_PIOPORCAP0_PIO2_1_MASK                ((unsigned int) 0x02000000)
#define SCB_PIOPORCAP0_PIO2_2                     ((unsigned int) 0x04000000)
#define SCB_PIOPORCAP0_PIO2_2_MASK                ((unsigned int) 0x04000000)
#define SCB_PIOPORCAP0_PIO2_3                     ((unsigned int) 0x08000000)
#define SCB_PIOPORCAP0_PIO2_3_MASK                ((unsigned int) 0x08000000)
#define SCB_PIOPORCAP0_PIO2_4                     ((unsigned int) 0x10000000)
#define SCB_PIOPORCAP0_PIO2_4_MASK                ((unsigned int) 0x10000000)
#define SCB_PIOPORCAP0_PIO2_5                     ((unsigned int) 0x20000000)
#define SCB_PIOPORCAP0_PIO2_5_MASK                ((unsigned int) 0x20000000)
#define SCB_PIOPORCAP0_PIO2_6                     ((unsigned int) 0x40000000)
#define SCB_PIOPORCAP0_PIO2_6_MASK                ((unsigned int) 0x40000000)
#define SCB_PIOPORCAP0_PIO2_7                     ((unsigned int) 0x80000000)
#define SCB_PIOPORCAP0_PIO2_7_MASK                ((unsigned int) 0x80000000)

/*  PIOPORCAP1 (POR captured PIO status register 1)
    The PIOPORCAP1 register captures the state (HIGH or LOW) of the PIO pins of port 2
    (PIO2_8 to PIO2_11) and port 3 at power-on-reset. Each bit represents the reset state of
    one PIO pin. This register is a read-only status register.  */

#define SCB_PIOPORCAP1                            (*(pREG32 (0x40048104)))    // POR captured PIO status 1
#define SCB_PIOPORCAP1_PIO2_8                     ((unsigned int) 0x00000001)
#define SCB_PIOPORCAP1_PIO2_8_MASK                ((unsigned int) 0x00000001)
#define SCB_PIOPORCAP1_PIO2_9                     ((unsigned int) 0x00000002)
#define SCB_PIOPORCAP1_PIO2_9_MASK                ((unsigned int) 0x00000002)
#define SCB_PIOPORCAP1_PIO2_10                    ((unsigned int) 0x00000004)
#define SCB_PIOPORCAP1_PIO2_10_MASK               ((unsigned int) 0x00000004)
#define SCB_PIOPORCAP1_PIO2_11                    ((unsigned int) 0x00000008)
#define SCB_PIOPORCAP1_PIO2_11_MASK               ((unsigned int) 0x00000008)
#define SCB_PIOPORCAP1_PIO3_0                     ((unsigned int) 0x00000010)
#define SCB_PIOPORCAP1_PIO3_0_MASK                ((unsigned int) 0x00000010)
#define SCB_PIOPORCAP1_PIO3_1                     ((unsigned int) 0x00000020)
#define SCB_PIOPORCAP1_PIO3_1_MASK                ((unsigned int) 0x00000020)
#define SCB_PIOPORCAP1_PIO3_2                     ((unsigned int) 0x00000040)
#define SCB_PIOPORCAP1_PIO3_2_MASK                ((unsigned int) 0x00000040)
#define SCB_PIOPORCAP1_PIO3_3                     ((unsigned int) 0x00000080)
#define SCB_PIOPORCAP1_PIO3_3_MASK                ((unsigned int) 0x00000080)
#define SCB_PIOPORCAP1_PIO3_4                     ((unsigned int) 0x00000100)
#define SCB_PIOPORCAP1_PIO3_4_MASK                ((unsigned int) 0x00000100)
#define SCB_PIOPORCAP1_PIO3_5                     ((unsigned int) 0x00000200)
#define SCB_PIOPORCAP1_PIO3_5_MASK                ((unsigned int) 0x00000200)

/*  BODCTRL (Brown-out detection control register)
    The BOD control register selects four separate threshold values for sending a BOD
    interrupt to the NVIC. Only one level is allowed for forced reset.  */

#define SCB_BODCTRL                               (*(pREG32 (0x40048150)))    // Brown-out detector control
#define SCB_BODCTRL_RSTLEVEL_1_46V_1_63V          ((unsigned int) 0x00000000)
#define SCB_BODCTRL_RSTLEVEL_2_06V_2_15V          ((unsigned int) 0x00000001)
#define SCB_BODCTRL_RSTLEVEL_2_35V_2_43V          ((unsigned int) 0x00000002)
#define SCB_BODCTRL_RSTLEVEL_2_63V_2_71V          ((unsigned int) 0x00000003)
#define SCB_BODCTRL_RSTLEVEL_MASK                 ((unsigned int) 0x00000003)
#define SCB_BODCTRL_INTLEVEL_1_65V_1_80V          ((unsigned int) 0x00000000)
#define SCB_BODCTRL_INTLEVEL_2_22V_2_35V          ((unsigned int) 0x00000004)
#define SCB_BODCTRL_INTLEVEL_2_52V_2_66V          ((unsigned int) 0x00000008)
#define SCB_BODCTRL_INTLEVEL_2_80V_2_90V          ((unsigned int) 0x0000000C)
#define SCB_BODCTRL_INTLEVEL_MASK                 ((unsigned int) 0x0000000C)
#define SCB_BODCTRL_RSTENABLE_DISABLE             ((unsigned int) 0x00000000)
#define SCB_BODCTRL_RSTENABLE_ENABLE              ((unsigned int) 0x00000010)
#define SCB_BODCTRL_RSTENABLE_MASK                ((unsigned int) 0x00000010)

/*  SYSTCKCAL (System tick counter calibration register) */

#define SCB_SYSTICKCCAL                           (*(pREG32 (0x40048158)))    // System tick counter calibration
#define SCB_SYSTICKCCAL_MASK                      ((unsigned int) 0x03FFFFFF) // Undefined as of v0.10 of the LPC1114 User Manual

/*  STARTAPRP0 (Start logic edge control register 0)
    The STARTAPRP0 register controls the start logic inputs of ports 0 (PIO0_0 to PIO0_11)
    and 1 (PIO1_0). This register selects a falling or rising edge on the corresponding PIO
    input to produce a falling or rising clock edge, respectively, for the start logic (see
    Section 38.3).
    Every bit in the STARTAPRP0 register controls one port input and is connected to one
    wake-up interrupt in the NVIC. Bit 0 in the STARTAPRP0 register corresponds to interrupt
    0, bit 1 to interrupt 1, etc. (see Table 549), up to a total of 13 interrupts.
    Remark: Each interrupt connected to a start logic input must be enabled in the NVIC if the
    corresponding PIO pin is used to wake up the chip from Deep-sleep mode. */

#define SCB_STARTAPRP0                            (*(pREG32 (0x40048200)))    // Start logic edge control register 0; bottom 32 interrupts
#define SCB_STARTAPRP0_APRPIO0_0                  ((unsigned int) 0x00000001)
#define SCB_STARTAPRP0_APRPIO0_0_MASK             ((unsigned int) 0x00000001)
#define SCB_STARTAPRP0_APRPIO0_1                  ((unsigned int) 0x00000002)
#define SCB_STARTAPRP0_APRPIO0_1_MASK             ((unsigned int) 0x00000002)
#define SCB_STARTAPRP0_APRPIO0_2                  ((unsigned int) 0x00000004)
#define SCB_STARTAPRP0_APRPIO0_2_MASK             ((unsigned int) 0x00000004)
#define SCB_STARTAPRP0_APRPIO0_3                  ((unsigned int) 0x00000008)
#define SCB_STARTAPRP0_APRPIO0_3_MASK             ((unsigned int) 0x00000008)
#define SCB_STARTAPRP0_APRPIO0_4                  ((unsigned int) 0x00000010)
#define SCB_STARTAPRP0_APRPIO0_4_MASK             ((unsigned int) 0x00000010)
#define SCB_STARTAPRP0_APRPIO0_5                  ((unsigned int) 0x00000020)
#define SCB_STARTAPRP0_APRPIO0_5_MASK             ((unsigned int) 0x00000020)
#define SCB_STARTAPRP0_APRPIO0_6                  ((unsigned int) 0x00000040)
#define SCB_STARTAPRP0_APRPIO0_6_MASK             ((unsigned int) 0x00000040)
#define SCB_STARTAPRP0_APRPIO0_7                  ((unsigned int) 0x00000080)
#define SCB_STARTAPRP0_APRPIO0_7_MASK             ((unsigned int) 0x00000080)
#define SCB_STARTAPRP0_APRPIO0_8                  ((unsigned int) 0x00000100)
#define SCB_STARTAPRP0_APRPIO0_8_MASK             ((unsigned int) 0x00000100)
#define SCB_STARTAPRP0_APRPIO0_9                  ((unsigned int) 0x00000200)
#define SCB_STARTAPRP0_APRPIO0_9_MASK             ((unsigned int) 0x00000200)
#define SCB_STARTAPRP0_APRPIO0_10                 ((unsigned int) 0x00000400)
#define SCB_STARTAPRP0_APRPIO0_10_MASK            ((unsigned int) 0x00000400)
#define SCB_STARTAPRP0_APRPIO0_11                 ((unsigned int) 0x00000800)
#define SCB_STARTAPRP0_APRPIO0_11_MASK            ((unsigned int) 0x00000800)
#define SCB_STARTAPRP0_APRPIO1_0                  ((unsigned int) 0x00001000)
#define SCB_STARTAPRP0_APRPIO1_0_MASK             ((unsigned int) 0x00001000)
#define SCB_STARTAPRP0_MASK                       ((unsigned int) 0xFFFFFFFF)

/*  STARTERP0 (Start logic signal enable register 0)
    This STARTERP0 register enables or disables the start signal bits in the start logic.  */

#define SCB_STARTERP0                             (*(pREG32 (0x40048204)))    // Start logic signal enable register 0; bottom 32 interrupts
#define SCB_STARTERP0_ERPIO0_0                    ((unsigned int) 0x00000001)
#define SCB_STARTERP0_ERPIO0_0_MASK               ((unsigned int) 0x00000001)
#define SCB_STARTERP0_ERPIO0_1                    ((unsigned int) 0x00000002)
#define SCB_STARTERP0_ERPIO0_1_MASK               ((unsigned int) 0x00000002)
#define SCB_STARTERP0_ERPIO0_2                    ((unsigned int) 0x00000004)
#define SCB_STARTERP0_ERPIO0_2_MASK               ((unsigned int) 0x00000004)
#define SCB_STARTERP0_ERPIO0_3                    ((unsigned int) 0x00000008)
#define SCB_STARTERP0_ERPIO0_3_MASK               ((unsigned int) 0x00000008)
#define SCB_STARTERP0_ERPIO0_4                    ((unsigned int) 0x00000010)
#define SCB_STARTERP0_ERPIO0_4_MASK               ((unsigned int) 0x00000010)
#define SCB_STARTERP0_ERPIO0_5                    ((unsigned int) 0x00000020)
#define SCB_STARTERP0_ERPIO0_5_MASK               ((unsigned int) 0x00000020)
#define SCB_STARTERP0_ERPIO0_6                    ((unsigned int) 0x00000040)
#define SCB_STARTERP0_ERPIO0_6_MASK               ((unsigned int) 0x00000040)
#define SCB_STARTERP0_ERPIO0_7                    ((unsigned int) 0x00000080)
#define SCB_STARTERP0_ERPIO0_7_MASK               ((unsigned int) 0x00000080)
#define SCB_STARTERP0_ERPIO0_8                    ((unsigned int) 0x00000100)
#define SCB_STARTERP0_ERPIO0_8_MASK               ((unsigned int) 0x00000100)
#define SCB_STARTERP0_ERPIO0_9                    ((unsigned int) 0x00000200)
#define SCB_STARTERP0_ERPIO0_9_MASK               ((unsigned int) 0x00000200)
#define SCB_STARTERP0_ERPIO0_10                   ((unsigned int) 0x00000400)
#define SCB_STARTERP0_ERPIO0_10_MASK              ((unsigned int) 0x00000400)
#define SCB_STARTERP0_ERPIO0_11                   ((unsigned int) 0x00000800)
#define SCB_STARTERP0_ERPIO0_11_MASK              ((unsigned int) 0x00000800)
#define SCB_STARTERP0_ERPIO1_0                    ((unsigned int) 0x00001000)
#define SCB_STARTERP0_ERPIO1_0_MASK               ((unsigned int) 0x00001000)
#define SCB_STARTERP0_MASK                        ((unsigned int) 0xFFFFFFFF)

/*  STARTRSRP0CLR (Start logic reset register 0)
    Writing a one to a bit in the STARTRSRP0CLR register resets the start logic state. The
    start-up logic uses the input signals to generate a clock edge for registering a start
    signal. This clock edge (falling or rising) sets the interrupt for waking up from
    Deep-sleep mode. Therefore, the start-up logic states must be cleared before being used. */

#define SCB_STARTRSRP0CLR                         (*(pREG32 (0x40048208)))    // Start logic reset register 0; bottom 32 interrupts
#define SCB_STARTRSRP0CLR_RSRPIO0_0               ((unsigned int) 0x00000001)
#define SCB_STARTRSRP0CLR_RSRPIO0_0_MASK          ((unsigned int) 0x00000001)
#define SCB_STARTRSRP0CLR_RSRPIO0_1               ((unsigned int) 0x00000002)
#define SCB_STARTRSRP0CLR_RSRPIO0_1_MASK          ((unsigned int) 0x00000002)
#define SCB_STARTRSRP0CLR_RSRPIO0_2               ((unsigned int) 0x00000004)
#define SCB_STARTRSRP0CLR_RSRPIO0_2_MASK          ((unsigned int) 0x00000004)
#define SCB_STARTRSRP0CLR_RSRPIO0_3               ((unsigned int) 0x00000008)
#define SCB_STARTRSRP0CLR_RSRPIO0_3_MASK          ((unsigned int) 0x00000008)
#define SCB_STARTRSRP0CLR_RSRPIO0_4               ((unsigned int) 0x00000010)
#define SCB_STARTRSRP0CLR_RSRPIO0_4_MASK          ((unsigned int) 0x00000010)
#define SCB_STARTRSRP0CLR_RSRPIO0_5               ((unsigned int) 0x00000020)
#define SCB_STARTRSRP0CLR_RSRPIO0_5_MASK          ((unsigned int) 0x00000020)
#define SCB_STARTRSRP0CLR_RSRPIO0_6               ((unsigned int) 0x00000040)
#define SCB_STARTRSRP0CLR_RSRPIO0_6_MASK          ((unsigned int) 0x00000040)
#define SCB_STARTRSRP0CLR_RSRPIO0_7               ((unsigned int) 0x00000080)
#define SCB_STARTRSRP0CLR_RSRPIO0_7_MASK          ((unsigned int) 0x00000080)
#define SCB_STARTRSRP0CLR_RSRPIO0_8               ((unsigned int) 0x00000100)
#define SCB_STARTRSRP0CLR_RSRPIO0_8_MASK          ((unsigned int) 0x00000100)
#define SCB_STARTRSRP0CLR_RSRPIO0_9               ((unsigned int) 0x00000200)
#define SCB_STARTRSRP0CLR_RSRPIO0_9_MASK          ((unsigned int) 0x00000200)
#define SCB_STARTRSRP0CLR_RSRPIO0_10              ((unsigned int) 0x00000400)
#define SCB_STARTRSRP0CLR_RSRPIO0_10_MASK         ((unsigned int) 0x00000400)
#define SCB_STARTRSRP0CLR_RSRPIO0_11              ((unsigned int) 0x00000800)
#define SCB_STARTRSRP0CLR_RSRPIO0_11_MASK         ((unsigned int) 0x00000800)
#define SCB_STARTRSRP0CLR_RSRPIO1_0               ((unsigned int) 0x00001000)
#define SCB_STARTRSRP0CLR_RSRPIO1_0_MASK          ((unsigned int) 0x00001000)
#define SCB_STARTRSRP0CLR_MASK                    ((unsigned int) 0xFFFFFFFF)

/*  (Start logic status register 0)
    This register reflects the status of the enabled start signal bits. Each bit
    (if enabled) reflects the state of the start logic, i.e. whether or not a
    wake-up signal has been received for a given pin.  */

#define SCB_STARTSRP0                             (*(pREG32 (0x4004820C)))    // Start logic status register 0; bottom 32 interrupts
#define SCB_STARTSRP0_SRPIO0_0                    ((unsigned int) 0x00000001)
#define SCB_STARTSRP0_SRPIO0_0_MASK               ((unsigned int) 0x00000001)
#define SCB_STARTSRP0_SRPIO0_1                    ((unsigned int) 0x00000002)
#define SCB_STARTSRP0_SRPIO0_1_MASK               ((unsigned int) 0x00000002)
#define SCB_STARTSRP0_SRPIO0_2                    ((unsigned int) 0x00000004)
#define SCB_STARTSRP0_SRPIO0_2_MASK               ((unsigned int) 0x00000004)
#define SCB_STARTSRP0_SRPIO0_3                    ((unsigned int) 0x00000008)
#define SCB_STARTSRP0_SRPIO0_3_MASK               ((unsigned int) 0x00000008)
#define SCB_STARTSRP0_SRPIO0_4                    ((unsigned int) 0x00000010)
#define SCB_STARTSRP0_SRPIO0_4_MASK               ((unsigned int) 0x00000010)
#define SCB_STARTSRP0_SRPIO0_5                    ((unsigned int) 0x00000020)
#define SCB_STARTSRP0_SRPIO0_5_MASK               ((unsigned int) 0x00000020)
#define SCB_STARTSRP0_SRPIO0_6                    ((unsigned int) 0x00000040)
#define SCB_STARTSRP0_SRPIO0_6_MASK               ((unsigned int) 0x00000040)
#define SCB_STARTSRP0_SRPIO0_7                    ((unsigned int) 0x00000080)
#define SCB_STARTSRP0_SRPIO0_7_MASK               ((unsigned int) 0x00000080)
#define SCB_STARTSRP0_SRPIO0_8                    ((unsigned int) 0x00000100)
#define SCB_STARTSRP0_SRPIO0_8_MASK               ((unsigned int) 0x00000100)
#define SCB_STARTSRP0_SRPIO0_9                    ((unsigned int) 0x00000200)
#define SCB_STARTSRP0_SRPIO0_9_MASK               ((unsigned int) 0x00000200)
#define SCB_STARTSRP0_SRPIO0_10                   ((unsigned int) 0x00000400)
#define SCB_STARTSRP0_SRPIO0_10_MASK              ((unsigned int) 0x00000400)
#define SCB_STARTSRP0_SRPIO0_11                   ((unsigned int) 0x00000800)
#define SCB_STARTSRP0_SRPIO0_11_MASK              ((unsigned int) 0x00000800)
#define SCB_STARTSRP0_SRPIO1_0                    ((unsigned int) 0x00001000)
#define SCB_STARTSRP0_SRPIO1_0_MASK               ((unsigned int) 0x00001000)
#define SCB_STARTSRP0_MASK                        ((unsigned int) 0xFFFFFFFF)

/*  PDSLEEPCFG (Deep-sleep mode configuration register)
    The bits in this register can be programmed to indicate the state the chip must enter when
    the Deep-sleep mode is asserted by the ARM. The value of the PDSLEEPCFG register
    will be automatically loaded into the PDRUNCFG register when the Sleep mode is
    entered. */

/*  Note: The latest version of the UM (10 March 2011) indicates that only four 
    values can be assigned to the PDSLEEPCFG register.  This differs from early
    versions of the UM which contained seperate values, but values have been
    added for the four 'valid' values included in the latest UM. */

#define SCB_PDSLEEPCFG                            (*(pREG32 (0x40048230)))    // Power-down states in Deep-sleep mode
#define SCB_PDSLEEPCFG_BOD_ON_WDOSC_ON            ((unsigned int) 0x000018B7)
#define SCB_PDSLEEPCFG_BOD_ON_WDOSC_OFF           ((unsigned int) 0x000018F7)
#define SCB_PDSLEEPCFG_BOD_OFF_WDOSC_ON           ((unsigned int) 0x000018BF)
#define SCB_PDSLEEPCFG_BOD_OFF_WDOSC_OFF          ((unsigned int) 0x000018FF)
//#define SCB_PDSLEEPCFG_IRCOUT_PD                  ((unsigned int) 0x00000001)
//#define SCB_PDSLEEPCFG_IRCOUT_PD_MASK             ((unsigned int) 0x00000001)
//#define SCB_PDSLEEPCFG_IRC_PD                     ((unsigned int) 0x00000002)
//#define SCB_PDSLEEPCFG_IRC_PD_MASK                ((unsigned int) 0x00000002)
//#define SCB_PDSLEEPCFG_FLASH_PD                   ((unsigned int) 0x00000004)
//#define SCB_PDSLEEPCFG_FLASH_PD_MASK              ((unsigned int) 0x00000004)
//#define SCB_PDSLEEPCFG_BOD_PD                     ((unsigned int) 0x00000008)
//#define SCB_PDSLEEPCFG_BOD_PD_MASK                ((unsigned int) 0x00000008)
//#define SCB_PDSLEEPCFG_ADC_PD                     ((unsigned int) 0x00000010)
//#define SCB_PDSLEEPCFG_ADC_PD_MASK                ((unsigned int) 0x00000010)
//#define SCB_PDSLEEPCFG_SYSOSC_PD                  ((unsigned int) 0x00000020)
//#define SCB_PDSLEEPCFG_SYSOSC_PD_MASK             ((unsigned int) 0x00000020)
//#define SCB_PDSLEEPCFG_WDTOSC_PD                  ((unsigned int) 0x00000040)
//#define SCB_PDSLEEPCFG_WDTOSC_PD_MASK             ((unsigned int) 0x00000040)
//#define SCB_PDSLEEPCFG_SYSPLL_PD                  ((unsigned int) 0x00000080)
//#define SCB_PDSLEEPCFG_SYSPLL_PD_MASK             ((unsigned int) 0x00000080)

/*  PDAWAKECFG (Wake-up configuration register)
    The bits in this register can be programmed to indicate the state the chip must enter when
    it is waking up from Deep-sleep mode. */

#define SCB_PDAWAKECFG                            (*(pREG32 (0x40048234)))    // Power-down states after wake-up from Deep-sleep mode
#define SCB_PDAWAKECFG_IRCOUT_PD                  ((unsigned int) 0x00000001)
#define SCB_PDAWAKECFG_IRCOUT_PD_MASK             ((unsigned int) 0x00000001)
#define SCB_PDAWAKECFG_IRC_PD                     ((unsigned int) 0x00000002)
#define SCB_PDAWAKECFG_IRC_PD_MASK                ((unsigned int) 0x00000002)
#define SCB_PDAWAKECFG_FLASH_PD                   ((unsigned int) 0x00000004)
#define SCB_PDAWAKECFG_FLASH_PD_MASK              ((unsigned int) 0x00000004)
#define SCB_PDAWAKECFG_BOD_PD                     ((unsigned int) 0x00000008)
#define SCB_PDAWAKECFG_BOD_PD_MASK                ((unsigned int) 0x00000008)
#define SCB_PDAWAKECFG_ADC_PD                     ((unsigned int) 0x00000010)
#define SCB_PDAWAKECFG_ADC_PD_MASK                ((unsigned int) 0x00000010)
#define SCB_PDAWAKECFG_SYSOSC_PD                  ((unsigned int) 0x00000020)
#define SCB_PDAWAKECFG_SYSOSC_PD_MASK             ((unsigned int) 0x00000020)
#define SCB_PDAWAKECFG_WDTOSC_PD                  ((unsigned int) 0x00000040)
#define SCB_PDAWAKECFG_WDTOSC_PD_MASK             ((unsigned int) 0x00000040)
#define SCB_PDAWAKECFG_SYSPLL_PD                  ((unsigned int) 0x00000080)
#define SCB_PDAWAKECFG_SYSPLL_PD_MASK             ((unsigned int) 0x00000080)

/*  PDRUNCFG (Power-down configuration register)
    The bits in the PDRUNCFG register control the power to the various analog blocks. This
    register can be written to at any time while the chip is running, and a write will take effect
    immediately with the exception of the power-down signal to the IRC.  Setting a 1 powers-down
    a peripheral and 0 enables it. */

#define SCB_PDRUNCFG                              (*(pREG32 (0x40048238)))    // Power-down configuration register
#define SCB_PDRUNCFG_IRCOUT                       ((unsigned int) 0x00000001) // IRC oscillator output power-down
#define SCB_PDRUNCFG_IRCOUT_MASK                  ((unsigned int) 0x00000001)    
#define SCB_PDRUNCFG_IRC                          ((unsigned int) 0x00000002) // IRC oscillator power-down
#define SCB_PDRUNCFG_IRC_MASK                     ((unsigned int) 0x00000002)
#define SCB_PDRUNCFG_FLASH                        ((unsigned int) 0x00000004) // Flash power-down
#define SCB_PDRUNCFG_FLASH_MASK                   ((unsigned int) 0x00000004)
#define SCB_PDRUNCFG_BOD                          ((unsigned int) 0x00000008) // Brown-out detector power-down
#define SCB_PDRUNCFG_BOD_MASK                     ((unsigned int) 0x00000008)
#define SCB_PDRUNCFG_ADC                          ((unsigned int) 0x00000010) // ADC power-down
#define SCB_PDRUNCFG_ADC_MASK                     ((unsigned int) 0x00000010)
#define SCB_PDRUNCFG_SYSOSC                       ((unsigned int) 0x00000020) // System oscillator power-down
#define SCB_PDRUNCFG_SYSOSC_MASK                  ((unsigned int) 0x00000020)
#define SCB_PDRUNCFG_WDTOSC                       ((unsigned int) 0x00000040) // Watchdog oscillator power-down
#define SCB_PDRUNCFG_WDTOSC_MASK                  ((unsigned int) 0x00000040)
#define SCB_PDRUNCFG_SYSPLL                       ((unsigned int) 0x00000080) // System PLL power-down
#define SCB_PDRUNCFG_SYSPLL_MASK                  ((unsigned int) 0x00000080)

/*  DEVICE_ID (Device ID Register)
    This device ID register is a read-only register and contains the device ID for each
    LPC111x part. This register is also read by the ISP/IAP commands. */

#define SCB_DEVICEID                              (*(pREG32 (0x400483F4)))    // Device ID
#define SCB_DEVICEID_LPC1111_101                  ((unsigned int) 0x041E502B)
#define SCB_DEVICEID_LPC1111_102                  ((unsigned int) 0x2516902B)
#define SCB_DEVICEID_LPC1111_201                  ((unsigned int) 0x0416502B)
#define SCB_DEVICEID_LPC1111_202                  ((unsigned int) 0x2516D02B) 
#define SCB_DEVICEID_LPC1112_101                  ((unsigned int) 0x042D502B)
#define SCB_DEVICEID_LPC1112_102                  ((unsigned int) 0x2524D02B)
#define SCB_DEVICEID_LPC1112_201                  ((unsigned int) 0x0425502B)
#define SCB_DEVICEID_LPC1112_202                  ((unsigned int) 0x2524902B)
#define SCB_DEVICEID_LPC1113_201                  ((unsigned int) 0x0434502B)
#define SCB_DEVICEID_LPC1113_202                  ((unsigned int) 0x2532902B)
#define SCB_DEVICEID_LPC1113_301                  ((unsigned int) 0x0434102B)
#define SCB_DEVICEID_LPC1113_302                  ((unsigned int) 0x2532102B)
#define SCB_DEVICEID_LPC1114_201                  ((unsigned int) 0x0444502B)
#define SCB_DEVICEID_LPC1114_202                  ((unsigned int) 0x2540902B)
#define SCB_DEVICEID_LPC1114_301                  ((unsigned int) 0x0444102B)
#define SCB_DEVICEID_LPC1114_302                  ((unsigned int) 0x2540102B)
#define SCB_DEVICEID_LPC11C12_301                 ((unsigned int) 0x1421102B)
#define SCB_DEVICEID_LPC11C14_301                 ((unsigned int) 0x1440102B)
#define SCB_DEVICEID_LPC11C22_301                 ((unsigned int) 0x1431102B)
#define SCB_DEVICEID_LPC11C24_301                 ((unsigned int) 0X1430102B)

/*  CPU ID Base Register */

#define SCB_CPUID                                 (*(pREG32 (0xE000ED00)))
#define SCB_CPUID_REVISION_MASK                   ((unsigned int) 0x0000000F) // Revision Code
#define SCB_CPUID_PARTNO_MASK                     ((unsigned int) 0x0000FFF0) // Part Number
#define SCB_CPUID_CONSTANT_MASK                   ((unsigned int) 0x000F0000) // Constant
#define SCB_CPUID_VARIANT_MASK                    ((unsigned int) 0x00F00000) // Variant
#define SCB_CPUID_IMPLEMENTER_MASK                ((unsigned int) 0xFF000000) // Implementer

#define SCB_ICSR                                  (*(pREG32 (0xE000ED04)))
#define SCB_ICSR_NMIPENDSET_MASK                  ((unsigned int) 0x80000000)
#define SCB_ICSR_NMIPENDSET                       ((unsigned int) 0x80000000)
#define SCB_ICSR_PENDSVSET_MASK                   ((unsigned int) 0x10000000)
#define SCB_ICSR_PENDSVSET                        ((unsigned int) 0x10000000)
#define SCB_ICSR_PENDSVCLR_MASK                   ((unsigned int) 0x08000000)
#define SCB_ICSR_PENDSVCLR                        ((unsigned int) 0x08000000)
#define SCB_ICSR_PENDSTSET_MASK                   ((unsigned int) 0x04000000)
#define SCB_ICSR_PENDSTSET                        ((unsigned int) 0x04000000)
#define SCB_ICSR_PENDSTCLR_MASK                   ((unsigned int) 0x02000000)
#define SCB_ICSR_PENDSTCLR                        ((unsigned int) 0x02000000)
#define SCB_ICSR_ISRPREEMPT_MASK                  ((unsigned int) 0x00800000)
#define SCB_ICSR_ISRPREEMPT                       ((unsigned int) 0x00800000)
#define SCB_ICSR_ISRPENDING_MASK                  ((unsigned int) 0x00400000)
#define SCB_ICSR_ISRPENDING                       ((unsigned int) 0x00400000)
#define SCB_ICSR_VECTPENDING_MASK                 ((unsigned int) 0x001FF000)
#define SCB_ICSR_VECTACTIVE_MASK                  ((unsigned int) 0x000001FF)

/*  Application Interrupt and Reset Control Register */

#define SCB_AIRCR                                 (*(pREG32 (0xE000ED0C)))
#define SCB_AIRCR_VECTKEY_VALUE                   ((unsigned int) 0x05FA0000) // Vect key needs to be set to 05FA for reset to work
#define SCB_AIRCR_VECTKEY_MASK                    ((unsigned int) 0xFFFF0000)
#define SCB_AIRCR_ENDIANESS                       ((unsigned int) 0x00008000) // Read Endianness (1=Big, 0=Little)
#define SCB_AIRCR_ENDIANESS_MASK                  ((unsigned int) 0x00008000)
#define SCB_AIRCR_SYSRESETREQ                     ((unsigned int) 0x00000004) // Request system reset
#define SCB_AIRCR_SYSRESETREQ_MASK                ((unsigned int) 0x00000004)
#define SCB_AIRCR_VECTCLRACTIVE                   ((unsigned int) 0x00000002) // Used to prevent accidental reset
#define SCB_AIRCR_VECTCLRACTIVE_MASK              ((unsigned int) 0x00000002)

/*  System Control Register */

#define SCB_SCR                                   (*(pREG32 (0xE000ED10)))
#define SCB_SCR_SLEEPONEXIT_MASK                  ((unsigned int) 0x00000002) // Enable sleep on exit
#define SCB_SCR_SLEEPONEXIT                       ((unsigned int) 0x00000002)
#define SCB_SCR_SLEEPDEEP_MASK                    ((unsigned int) 0x00000004)
#define SCB_SCR_SLEEPDEEP                         ((unsigned int) 0x00000004) // Enable deep sleep
#define SCB_SCR_SEVONPEND_MASK                    ((unsigned int) 0x00000010) // Wake up from WFE is new int is pended regardless of priority
#define SCB_SCR_SEVONPEND                         ((unsigned int) 0x00000010)

/*##############################################################################
## Power Management Unit (PMU)
##############################################################################*/

#define PMU_BASE_ADDRESS                          (0x40038000)

#define PMU_PMUCTRL                               (*(pREG32 (0x40038000)))    // Power control register
#define PMU_PMUCTRL_DPDEN_MASK                    ((unsigned int) 0x00000002) // Deep power-down enable
#define PMU_PMUCTRL_DPDEN_DEEPPOWERDOWN           ((unsigned int) 0x00000002) // WFI will enter deep power-down mode
#define PMU_PMUCTRL_DPDEN_SLEEP                   ((unsigned int) 0x00000000) // WFI will enter sleep mode
#define PMU_PMUCTRL_SLEEPFLAG_MASK                ((unsigned int) 0x00000100) // Read-only ... indicates if the device is in active or sleep/deep-sleep/deep-power-down mode
#define PMU_PMUCTRL_SLEEPFLAG                     ((unsigned int) 0x00000100)
#define PMU_PMUCTRL_DPDFLAG_MASK                  ((unsigned int) 0x00000800) // Deep power-down flag
#define PMU_PMUCTRL_DPDFLAG                       ((unsigned int) 0x00000800)

/*  GPREG0..3 (General purpose registers 0 to 3)
    The general purpose registers retain data through the Deep power-down mode when
    power is still applied to the VDD(3V3) pin but the chip has entered Deep power-down mode.
    Only a cold boot when all power has been completely removed from the chip will reset
    the general purpose registers.  */

#define PMU_GPREG0                                (*(pREG32 (0x40038004)))    // General purpose register 0
#define PMU_GPREG0_GPDATA_MASK                    ((unsigned int) 0xFFFFFFFF)

#define PMU_GPREG1                                (*(pREG32 (0x40038008)))    // General purpose register 1
#define PMU_GPREG1_GPDATA_MASK                    ((unsigned int) 0xFFFFFFFF)

#define PMU_GPREG2                                (*(pREG32 (0x4003800C)))    // General purpose register 2
#define PMU_GPREG2_GPDATA_MASK                    ((unsigned int) 0xFFFFFFFF)

#define PMU_GPREG3                                (*(pREG32 (0x40038010)))    // General purpose register 3
#define PMU_GPREG3_GPDATA_MASK                    ((unsigned int) 0xFFFFFFFF)

/*  GPREG4 (General purpose register 4)
    The general purpose register 4 retains data through the Deep power-down mode when
    power is still applied to the VDD(3V3) pin but the chip has entered Deep power-down mode.
    Only a cold boot, when all power has been completely removed from the chip, will reset
    the general purpose registers.
      
    Remark: If the external voltage applied on pin VDD(3V3) drops below 2.2V, the
    hysteresis of the WAKEUP input pin has to be disabled in order for the chip to wake up
    from Deep power-down mode.  */

#define PMU_GPREG4                                (*(pREG32 (0x40038014)))    // General purpose register 4
#define PMU_GPREG4_GPDATA_MASK                    ((unsigned int) 0xFFFFF800)
#define PMU_GPREG4_WAKEUPHYS_MASK                 ((unsigned int) 0x00000400)
#define PMU_GPREG4_WAKEUPHYS_HYSTERESISENABLED    ((unsigned int) 0x00000400)
#define PMU_GPREG4_WAKEUPHYS_HYSTERESISDISABLED   ((unsigned int) 0x00000000)
#define PMU_GPREG4_GPDATA_MASK                    ((unsigned int) 0xFFFFF800)

/*##############################################################################
## I/O Control (IOCON)
##############################################################################*/

#define IOCON_BASE_ADDRESS                        (0x40044000)

#define IOCON_COMMON_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_COMMON_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_COMMON_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_COMMON_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_COMMON_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_COMMON_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_COMMON_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_COMMON_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_COMMON_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_nRESET_PIO0_0                       (*(pREG32 (0x4004400C)))
#define IOCON_nRESET_PIO0_0_FUNC_MASK             ((unsigned int) 0x00000007)
#define IOCON_nRESET_PIO0_0_FUNC_RESET            ((unsigned int) 0x00000000)
#define IOCON_nRESET_PIO0_0_FUNC_GPIO             ((unsigned int) 0x00000001)
#define IOCON_nRESET_PIO0_0_MODE_MASK             ((unsigned int) 0x00000018)
#define IOCON_nRESET_PIO0_0_MODE_INACTIVE         ((unsigned int) 0x00000000)
#define IOCON_nRESET_PIO0_0_MODE_PULLDOWN         ((unsigned int) 0x00000008)
#define IOCON_nRESET_PIO0_0_MODE_PULLUP           ((unsigned int) 0x00000010)
#define IOCON_nRESET_PIO0_0_MODE_REPEATER         ((unsigned int) 0x00000018)
#define IOCON_nRESET_PIO0_0_HYS_MASK              ((unsigned int) 0x00000020)
#define IOCON_nRESET_PIO0_0_HYS_DISABLE           ((unsigned int) 0x00000000)
#define IOCON_nRESET_PIO0_0_HYS_ENABLE            ((unsigned int) 0x00000020)

#define IOCON_PIO0_1                              (*(pREG32 (0x40044010)))
#define IOCON_PIO0_1_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_1_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_1_FUNC_CLKOUT                  ((unsigned int) 0x00000001)
#define IOCON_PIO0_1_FUNC_CT32B0_MAT2             ((unsigned int) 0x00000002)
#define IOCON_PIO0_1_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO0_1_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO0_1_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO0_1_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO0_1_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO0_1_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO0_1_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO0_1_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO0_2                              (*(pREG32 (0x4004401C)))
#define IOCON_PIO0_2_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_2_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_2_FUNC_SSEL                    ((unsigned int) 0x00000001)
#define IOCON_PIO0_2_FUNC_CT16B0_CAP0             ((unsigned int) 0x00000002)
#define IOCON_PIO0_2_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO0_2_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO0_2_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO0_2_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO0_2_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO0_2_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO0_2_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO0_2_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO0_3                              (*(pREG32 (0x4004402C)))
#define IOCON_PIO0_3_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_3_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_3_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO0_3_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO0_3_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO0_3_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO0_3_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO0_3_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO0_3_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO0_3_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO0_4                              (*(pREG32 (0x40044030)))
#define IOCON_PIO0_4_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_4_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_4_FUNC_I2CSCL                  ((unsigned int) 0x00000001)
#define IOCON_PIO0_4_I2CMODE_MASK                 ((unsigned int) 0x00000300)
#define IOCON_PIO0_4_I2CMODE_STANDARDI2C          ((unsigned int) 0x00000000)
#define IOCON_PIO0_4_I2CMODE_STANDARDIO           ((unsigned int) 0x00000100)
#define IOCON_PIO0_4_I2CMODE_FASTPLUSI2C          ((unsigned int) 0x00000200)

#define IOCON_PIO0_5                              (*(pREG32 (0x40044034)))
#define IOCON_PIO0_5_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_5_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_5_FUNC_I2CSDA                  ((unsigned int) 0x00000001)
#define IOCON_PIO0_5_I2CMODE_MASK                 ((unsigned int) 0x00000300)
#define IOCON_PIO0_5_I2CMODE_STANDARDI2C          ((unsigned int) 0x00000000)
#define IOCON_PIO0_5_I2CMODE_STANDARDIO           ((unsigned int) 0x00000100)
#define IOCON_PIO0_5_I2CMODE_FASTPLUSI2C          ((unsigned int) 0x00000200)

#define IOCON_PIO0_6                              (*(pREG32 (0x4004404C)))
#define IOCON_PIO0_6_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_6_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_6_FUNC_SCK                     ((unsigned int) 0x00000002)
#define IOCON_PIO0_6_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO0_6_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO0_6_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO0_6_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO0_6_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO0_6_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO0_6_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO0_6_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO0_7                              (*(pREG32 (0x40044050)))
#define IOCON_PIO0_7_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_7_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_7_FUNC_CTS                     ((unsigned int) 0x00000001)
#define IOCON_PIO0_7_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO0_7_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO0_7_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO0_7_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO0_7_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO0_7_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO0_7_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO0_7_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO0_8                              (*(pREG32 (0x40044060)))
#define IOCON_PIO0_8_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_8_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_8_FUNC_MISO0                   ((unsigned int) 0x00000001)
#define IOCON_PIO0_8_FUNC_CT16B0_MAT0             ((unsigned int) 0x00000002)
#define IOCON_PIO0_8_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO0_8_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO0_8_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO0_8_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO0_8_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO0_8_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO0_8_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO0_8_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO0_9                              (*(pREG32 (0x40044064)))
#define IOCON_PIO0_9_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO0_9_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO0_9_FUNC_MOSI0                   ((unsigned int) 0x00000001)
#define IOCON_PIO0_9_FUNC_CT16B0_MAT1             ((unsigned int) 0x00000002)
#define IOCON_PIO0_9_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO0_9_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO0_9_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO0_9_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO0_9_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO0_9_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO0_9_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO0_9_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_JTAG_TCK_PIO0_10                    (*(pREG32 (0x40044068)))
#define IOCON_JTAG_TCK_PIO0_10_FUNC_MASK          ((unsigned int) 0x00000007)
#define IOCON_JTAG_TCK_PIO0_10_FUNC_SWCLK         ((unsigned int) 0x00000000)
#define IOCON_JTAG_TCK_PIO0_10_FUNC_GPIO          ((unsigned int) 0x00000001)
#define IOCON_JTAG_TCK_PIO0_10_FUNC_SCK           ((unsigned int) 0x00000002)
#define IOCON_JTAG_TCK_PIO0_10_FUNC_CT16B0_MAT2   ((unsigned int) 0x00000003)
#define IOCON_JTAG_TCK_PIO0_10_MODE_MASK          ((unsigned int) 0x00000018)
#define IOCON_JTAG_TCK_PIO0_10_MODE_INACTIVE      ((unsigned int) 0x00000000)
#define IOCON_JTAG_TCK_PIO0_10_MODE_PULLDOWN      ((unsigned int) 0x00000008)
#define IOCON_JTAG_TCK_PIO0_10_MODE_PULLUP        ((unsigned int) 0x00000010)
#define IOCON_JTAG_TCK_PIO0_10_MODE_REPEATER      ((unsigned int) 0x00000018)
#define IOCON_JTAG_TCK_PIO0_10_HYS_MASK           ((unsigned int) 0x00000020)
#define IOCON_JTAG_TCK_PIO0_10_HYS_DISABLE        ((unsigned int) 0x00000000)
#define IOCON_JTAG_TCK_PIO0_10_HYS_ENABLE         ((unsigned int) 0x00000020)

#define IOCON_JTAG_TDI_PIO0_11                    (*(pREG32 (0x40044074)))
#define IOCON_JTAG_TDI_PIO0_11_FUNC_MASK          ((unsigned int) 0x00000007)
#define IOCON_JTAG_TDI_PIO0_11_FUNC_TDI           ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDI_PIO0_11_FUNC_GPIO          ((unsigned int) 0x00000001)
#define IOCON_JTAG_TDI_PIO0_11_FUNC_AD0           ((unsigned int) 0x00000002)
#define IOCON_JTAG_TDI_PIO0_11_FUNC_CT32B0_MAT3   ((unsigned int) 0x00000003)
#define IOCON_JTAG_TDI_PIO0_11_MODE_MASK          ((unsigned int) 0x00000018)
#define IOCON_JTAG_TDI_PIO0_11_MODE_INACTIVE      ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDI_PIO0_11_MODE_PULLDOWN      ((unsigned int) 0x00000008)
#define IOCON_JTAG_TDI_PIO0_11_MODE_PULLUP        ((unsigned int) 0x00000010)
#define IOCON_JTAG_TDI_PIO0_11_MODE_REPEATER      ((unsigned int) 0x00000018)
#define IOCON_JTAG_TDI_PIO0_11_HYS_MASK           ((unsigned int) 0x00000020)
#define IOCON_JTAG_TDI_PIO0_11_HYS_DISABLE        ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDI_PIO0_11_HYS_ENABLE         ((unsigned int) 0x00000020)
#define IOCON_JTAG_TDI_PIO0_11_ADMODE_MASK        ((unsigned int) 0x00000080)
#define IOCON_JTAG_TDI_PIO0_11_ADMODE_ANALOG      ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDI_PIO0_11_ADMODE_DIGITAL     ((unsigned int) 0x00000080)

#define IOCON_JTAG_TMS_PIO1_0                     (*(pREG32 (0x40044078)))
#define IOCON_JTAG_TMS_PIO1_0_FUNC_MASK           ((unsigned int) 0x00000007)
#define IOCON_JTAG_TMS_PIO1_0_FUNC_TMS            ((unsigned int) 0x00000000)
#define IOCON_JTAG_TMS_PIO1_0_FUNC_GPIO           ((unsigned int) 0x00000001)
#define IOCON_JTAG_TMS_PIO1_0_FUNC_AD1            ((unsigned int) 0x00000002)
#define IOCON_JTAG_TMS_PIO1_0_FUNC_CT32B1_CAP0    ((unsigned int) 0x00000003)
#define IOCON_JTAG_TMS_PIO1_0_MODE_MASK           ((unsigned int) 0x00000018)
#define IOCON_JTAG_TMS_PIO1_0_MODE_INACTIVE       ((unsigned int) 0x00000000)
#define IOCON_JTAG_TMS_PIO1_0_MODE_PULLDOWN       ((unsigned int) 0x00000008)
#define IOCON_JTAG_TMS_PIO1_0_MODE_PULLUP         ((unsigned int) 0x00000010)
#define IOCON_JTAG_TMS_PIO1_0_MODE_REPEATER       ((unsigned int) 0x00000018)
#define IOCON_JTAG_TMS_PIO1_0_HYS_MASK            ((unsigned int) 0x00000020)
#define IOCON_JTAG_TMS_PIO1_0_HYS_DISABLE         ((unsigned int) 0x00000000)
#define IOCON_JTAG_TMS_PIO1_0_HYS_ENABLE          ((unsigned int) 0x00000020)
#define IOCON_JTAG_TMS_PIO1_0_ADMODE_MASK         ((unsigned int) 0x00000080)
#define IOCON_JTAG_TMS_PIO1_0_ADMODE_ANALOG       ((unsigned int) 0x00000000)
#define IOCON_JTAG_TMS_PIO1_0_ADMODE_DIGITAL      ((unsigned int) 0x00000080)

#define IOCON_JTAG_TDO_PIO1_1                     (*(pREG32 (0x4004407C)))
#define IOCON_JTAG_TDO_PIO1_1_FUNC_MASK           ((unsigned int) 0x00000007)
#define IOCON_JTAG_TDO_PIO1_1_FUNC_TDO            ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDO_PIO1_1_FUNC_GPIO           ((unsigned int) 0x00000001)
#define IOCON_JTAG_TDO_PIO1_1_FUNC_AD2            ((unsigned int) 0x00000002)
#define IOCON_JTAG_TDO_PIO1_1_FUNC_CT32B1_MAT0    ((unsigned int) 0x00000003)
#define IOCON_JTAG_TDO_PIO1_1_MODE_MASK           ((unsigned int) 0x00000018)
#define IOCON_JTAG_TDO_PIO1_1_MODE_INACTIVE       ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDO_PIO1_1_MODE_PULLDOWN       ((unsigned int) 0x00000008)
#define IOCON_JTAG_TDO_PIO1_1_MODE_PULLUP         ((unsigned int) 0x00000010)
#define IOCON_JTAG_TDO_PIO1_1_MODE_REPEATER       ((unsigned int) 0x00000018)
#define IOCON_JTAG_TDO_PIO1_1_HYS_MASK            ((unsigned int) 0x00000020)
#define IOCON_JTAG_TDO_PIO1_1_HYS_DISABLE         ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDO_PIO1_1_HYS_ENABLE          ((unsigned int) 0x00000020)
#define IOCON_JTAG_TDO_PIO1_1_ADMODE_MASK         ((unsigned int) 0x00000080)
#define IOCON_JTAG_TDO_PIO1_1_ADMODE_ANALOG       ((unsigned int) 0x00000000)
#define IOCON_JTAG_TDO_PIO1_1_ADMODE_DIGITAL      ((unsigned int) 0x00000080)

#define IOCON_JTAG_nTRST_PIO1_2                   (*(pREG32 (0x40044080)))
#define IOCON_JTAG_nTRST_PIO1_2_FUNC_MASK         ((unsigned int) 0x00000007)
#define IOCON_JTAG_nTRST_PIO1_2_FUNC_TRST         ((unsigned int) 0x00000000)
#define IOCON_JTAG_nTRST_PIO1_2_FUNC_GPIO         ((unsigned int) 0x00000001)
#define IOCON_JTAG_nTRST_PIO1_2_FUNC_AD3          ((unsigned int) 0x00000002)
#define IOCON_JTAG_nTRST_PIO1_2_FUNC_CT32B1_MAT1  ((unsigned int) 0x00000003)
#define IOCON_JTAG_nTRST_PIO1_2_MODE_MASK         ((unsigned int) 0x00000018)
#define IOCON_JTAG_nTRST_PIO1_2_MODE_INACTIVE     ((unsigned int) 0x00000000)
#define IOCON_JTAG_nTRST_PIO1_2_MODE_PULLDOWN     ((unsigned int) 0x00000008)
#define IOCON_JTAG_nTRST_PIO1_2_MODE_PULLUP       ((unsigned int) 0x00000010)
#define IOCON_JTAG_nTRST_PIO1_2_MODE_REPEATER     ((unsigned int) 0x00000018)
#define IOCON_JTAG_nTRST_PIO1_2_HYS_MASK          ((unsigned int) 0x00000020)
#define IOCON_JTAG_nTRST_PIO1_2_HYS_DISABLE       ((unsigned int) 0x00000000)
#define IOCON_JTAG_nTRST_PIO1_2_HYS_ENABLE        ((unsigned int) 0x00000020)
#define IOCON_JTAG_nTRST_PIO1_2_ADMODE_MASK       ((unsigned int) 0x00000080)
#define IOCON_JTAG_nTRST_PIO1_2_ADMODE_ANALOG     ((unsigned int) 0x00000000)
#define IOCON_JTAG_nTRST_PIO1_2_ADMODE_DIGITAL    ((unsigned int) 0x00000080)

#define IOCON_SWDIO_PIO1_3                        (*(pREG32 (0x40044090)))
#define IOCON_SWDIO_PIO1_3_FUNC_MASK              ((unsigned int) 0x00000007)
#define IOCON_SWDIO_PIO1_3_FUNC_SWDIO             ((unsigned int) 0x00000000)
#define IOCON_SWDIO_PIO1_3_FUNC_GPIO              ((unsigned int) 0x00000001)
#define IOCON_SWDIO_PIO1_3_FUNC_AD4               ((unsigned int) 0x00000002)
#define IOCON_SWDIO_PIO1_3_FUNC_CT32B1_MAT2       ((unsigned int) 0x00000003)
#define IOCON_SWDIO_PIO1_3_HYS_MASK               ((unsigned int) 0x00000020)
#define IOCON_SWDIO_PIO1_3_HYS_DISABLE            ((unsigned int) 0x00000000)
#define IOCON_SWDIO_PIO1_3_HYS_ENABLE             ((unsigned int) 0x00000020)
#define IOCON_SWDIO_PIO1_3_ADMODE_MASK            ((unsigned int) 0x00000080)
#define IOCON_SWDIO_PIO1_3_ADMODE_ANALOG          ((unsigned int) 0x00000000)
#define IOCON_SWDIO_PIO1_3_ADMODE_DIGITAL         ((unsigned int) 0x00000080)

#define IOCON_PIO1_4                              (*(pREG32 (0x40044094)))
#define IOCON_PIO1_4_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO1_4_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO1_4_FUNC_AD5                     ((unsigned int) 0x00000001)
#define IOCON_PIO1_4_FUNC_CT32B1_MAT3             ((unsigned int) 0x00000002)
#define IOCON_PIO1_4_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO1_4_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO1_4_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO1_4_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO1_4_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO1_4_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO1_4_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO1_4_HYS_ENABLE                   ((unsigned int) 0x00000020)
#define IOCON_PIO1_4_ADMODE_MASK                  ((unsigned int) 0x00000080)
#define IOCON_PIO1_4_ADMODE_ANALOG                ((unsigned int) 0x00000000)
#define IOCON_PIO1_4_ADMODE_DIGITAL               ((unsigned int) 0x00000080)

#define IOCON_PIO1_5                              (*(pREG32 (0x400440A0)))
#define IOCON_PIO1_5_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO1_5_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO1_5_FUNC_RTS                     ((unsigned int) 0x00000001)
#define IOCON_PIO1_5_FUNC_CT32B0_CAP0             ((unsigned int) 0x00000002)
#define IOCON_PIO1_5_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO1_5_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO1_5_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO1_5_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO1_5_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO1_5_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO1_5_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO1_5_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO1_6                              (*(pREG32 (0x400440A4)))
#define IOCON_PIO1_6_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO1_6_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO1_6_FUNC_UART_RXD                ((unsigned int) 0x00000001)
#define IOCON_PIO1_6_FUNC_CT32B0_MAT0             ((unsigned int) 0x00000002)
#define IOCON_PIO1_6_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO1_6_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO1_6_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO1_6_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO1_6_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO1_6_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO1_6_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO1_6_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO1_7                              (*(pREG32 (0x400440A8)))
#define IOCON_PIO1_7_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO1_7_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO1_7_FUNC_UART_TXD                ((unsigned int) 0x00000001)
#define IOCON_PIO1_7_FUNC_CT32B0_MAT1             ((unsigned int) 0x00000002)
#define IOCON_PIO1_7_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO1_7_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO1_7_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO1_7_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO1_7_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO1_7_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO1_7_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO1_7_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO1_8                              (*(pREG32 (0x40044014)))
#define IOCON_PIO1_8_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO1_8_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO1_8_FUNC_CT16B1_CAP0             ((unsigned int) 0x00000001)
#define IOCON_PIO1_8_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO1_8_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO1_8_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO1_8_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO1_8_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO1_8_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO1_8_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO1_8_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO1_9                              (*(pREG32 (0x40044038)))
#define IOCON_PIO1_9_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO1_9_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO1_9_FUNC_CT16B1_MAT0             ((unsigned int) 0x00000001)
#define IOCON_PIO1_9_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO1_9_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO1_9_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO1_9_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO1_9_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO1_9_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO1_9_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO1_9_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO1_10                             (*(pREG32 (0x4004406C)))
#define IOCON_PIO1_10_FUNC_MASK                   ((unsigned int) 0x00000007)
#define IOCON_PIO1_10_FUNC_GPIO                   ((unsigned int) 0x00000000)
#define IOCON_PIO1_10_FUNC_AD6                    ((unsigned int) 0x00000001)
#define IOCON_PIO1_10_FUNC_CT16B1_MAT1            ((unsigned int) 0x00000002)
#define IOCON_PIO1_10_MODE_MASK                   ((unsigned int) 0x00000018)
#define IOCON_PIO1_10_MODE_INACTIVE               ((unsigned int) 0x00000000)
#define IOCON_PIO1_10_MODE_PULLDOWN               ((unsigned int) 0x00000008)
#define IOCON_PIO1_10_MODE_PULLUP                 ((unsigned int) 0x00000010)
#define IOCON_PIO1_10_MODE_REPEATER               ((unsigned int) 0x00000018)
#define IOCON_PIO1_10_HYS_MASK                    ((unsigned int) 0x00000020)
#define IOCON_PIO1_10_HYS_DISABLE                 ((unsigned int) 0x00000000)
#define IOCON_PIO1_10_HYS_ENABLE                  ((unsigned int) 0x00000020)
#define IOCON_PIO1_10_ADMODE_MASK                 ((unsigned int) 0x00000080)
#define IOCON_PIO1_10_ADMODE_ANALOG               ((unsigned int) 0x00000000)
#define IOCON_PIO1_10_ADMODE_DIGITAL              ((unsigned int) 0x00000080)

#define IOCON_PIO1_11                             (*(pREG32 (0x40044098)))
#define IOCON_PIO1_11_FUNC_MASK                   ((unsigned int) 0x00000007)
#define IOCON_PIO1_11_FUNC_GPIO                   ((unsigned int) 0x00000000)
#define IOCON_PIO1_11_FUNC_AD7                    ((unsigned int) 0x00000001)
#define IOCON_PIO1_11_MODE_MASK                   ((unsigned int) 0x00000018)
#define IOCON_PIO1_11_MODE_INACTIVE               ((unsigned int) 0x00000000)
#define IOCON_PIO1_11_MODE_PULLDOWN               ((unsigned int) 0x00000008)
#define IOCON_PIO1_11_MODE_PULLUP                 ((unsigned int) 0x00000010)
#define IOCON_PIO1_11_MODE_REPEATER               ((unsigned int) 0x00000018)
#define IOCON_PIO1_11_HYS_MASK                    ((unsigned int) 0x00000020)
#define IOCON_PIO1_11_HYS_DISABLE                 ((unsigned int) 0x00000000)
#define IOCON_PIO1_11_HYS_ENABLE                  ((unsigned int) 0x00000020)
#define IOCON_PIO1_11_ADMODE_MASK                 ((unsigned int) 0x00000080)
#define IOCON_PIO1_11_ADMODE_ANALOG               ((unsigned int) 0x00000000)
#define IOCON_PIO1_11_ADMODE_DIGITAL              ((unsigned int) 0x00000080)

#define IOCON_PIO2_0                              (*(pREG32 (0x40044008)))
#define IOCON_PIO2_0_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_0_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_0_FUNC_DTR                     ((unsigned int) 0x00000001)
#define IOCON_PIO2_0_FUNC_SSEL1                   ((unsigned int) 0x00000002)
#define IOCON_PIO2_0_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_0_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_0_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_0_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_0_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_0_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_0_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_0_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_1                              (*(pREG32 (0x40044028)))
#define IOCON_PIO2_1_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_1_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_1_FUNC_DSR                     ((unsigned int) 0x00000001)
#define IOCON_PIO2_1_FUNC_SCK1                    ((unsigned int) 0x00000002)
#define IOCON_PIO2_1_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_1_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_1_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_1_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_1_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_1_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_1_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_1_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_2                              (*(pREG32 (0x4004405C)))
#define IOCON_PIO2_2_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_2_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_2_FUNC_DCD                     ((unsigned int) 0x00000001)
#define IOCON_PIO2_2_FUNC_MISO1                   ((unsigned int) 0x00000002)
#define IOCON_PIO2_2_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_2_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_2_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_2_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_2_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_2_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_2_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_2_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_3                              (*(pREG32 (0x4004408C)))
#define IOCON_PIO2_3_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_3_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_3_FUNC_RI                      ((unsigned int) 0x00000001)
#define IOCON_PIO2_3_FUNC_MOSI1                   ((unsigned int) 0x00000002)
#define IOCON_PIO2_3_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_3_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_3_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_3_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_3_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_3_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_3_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_3_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_4                              (*(pREG32 (0x40044040)))
#define IOCON_PIO2_4_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_4_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_4_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_4_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_4_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_4_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_4_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_4_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_4_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_4_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_5                              (*(pREG32 (0x40044044)))
#define IOCON_PIO2_5_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_5_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_5_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_5_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_5_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_5_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_5_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_5_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_5_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_5_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_6                              (*(pREG32 (0x40044000)))
#define IOCON_PIO2_6_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_6_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_6_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_6_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_6_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_6_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_6_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_6_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_6_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_6_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_7                              (*(pREG32 (0x40044020)))
#define IOCON_PIO2_7_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_7_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_7_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_7_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_7_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_7_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_7_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_7_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_7_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_7_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_8                              (*(pREG32 (0x40044024)))
#define IOCON_PIO2_8_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_8_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_8_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_8_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_8_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_8_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_8_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_8_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_8_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_8_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_9                              (*(pREG32 (0x40044054)))
#define IOCON_PIO2_9_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_9_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_9_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_9_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_9_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_9_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_9_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_9_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_9_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_9_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_10                              (*(pREG32 (0x40044058)))
#define IOCON_PIO2_10_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO2_10_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO2_10_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO2_10_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO2_10_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO2_10_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO2_10_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO2_10_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO2_10_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO2_10_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO2_11                             (*(pREG32 (0x40044070)))
#define IOCON_PIO2_11_FUNC_MASK                   ((unsigned int) 0x00000007)
#define IOCON_PIO2_11_FUNC_GPIO                   ((unsigned int) 0x00000000)
#define IOCON_PIO2_11_FUNC_SCK                    ((unsigned int) 0x00000001)
#define IOCON_PIO2_11_MODE_MASK                   ((unsigned int) 0x00000018)
#define IOCON_PIO2_11_MODE_INACTIVE               ((unsigned int) 0x00000000)
#define IOCON_PIO2_11_MODE_PULLDOWN               ((unsigned int) 0x00000008)
#define IOCON_PIO2_11_MODE_PULLUP                 ((unsigned int) 0x00000010)
#define IOCON_PIO2_11_MODE_REPEATER               ((unsigned int) 0x00000018)
#define IOCON_PIO2_11_HYS_MASK                    ((unsigned int) 0x00000020)
#define IOCON_PIO2_11_HYS_DISABLE                 ((unsigned int) 0x00000000)
#define IOCON_PIO2_11_HYS_ENABLE                  ((unsigned int) 0x00000020)

#define IOCON_PIO3_0                              (*(pREG32 (0x40044084)))
#define IOCON_PIO3_0_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO3_0_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO3_0_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO3_0_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO3_0_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO3_0_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO3_0_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO3_0_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO3_0_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO3_0_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO3_1                              (*(pREG32 (0x40044088)))
#define IOCON_PIO3_1_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO3_1_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO3_1_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO3_1_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO3_1_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO3_1_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO3_1_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO3_1_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO3_1_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO3_1_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO3_2                              (*(pREG32 (0x4004409C)))
#define IOCON_PIO3_2_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO3_2_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO3_2_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO3_2_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO3_2_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO3_2_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO3_2_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO3_2_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO3_2_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO3_2_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO3_3                              (*(pREG32 (0x400440AC)))
#define IOCON_PIO3_3_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO3_3_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO3_3_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO3_3_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO3_3_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO3_3_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO3_3_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO3_3_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO3_3_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO3_3_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO3_4                              (*(pREG32 (0x4004403C)))
#define IOCON_PIO3_4_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO3_4_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO3_4_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO3_4_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO3_4_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO3_4_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO3_4_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO3_4_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO3_4_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO3_4_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_PIO3_5                              (*(pREG32 (0x40044048)))
#define IOCON_PIO3_5_FUNC_MASK                    ((unsigned int) 0x00000007)
#define IOCON_PIO3_5_FUNC_GPIO                    ((unsigned int) 0x00000000)
#define IOCON_PIO3_5_MODE_MASK                    ((unsigned int) 0x00000018)
#define IOCON_PIO3_5_MODE_INACTIVE                ((unsigned int) 0x00000000)
#define IOCON_PIO3_5_MODE_PULLDOWN                ((unsigned int) 0x00000008)
#define IOCON_PIO3_5_MODE_PULLUP                  ((unsigned int) 0x00000010)
#define IOCON_PIO3_5_MODE_REPEATER                ((unsigned int) 0x00000018)
#define IOCON_PIO3_5_HYS_MASK                     ((unsigned int) 0x00000020)
#define IOCON_PIO3_5_HYS_DISABLE                  ((unsigned int) 0x00000000)
#define IOCON_PIO3_5_HYS_ENABLE                   ((unsigned int) 0x00000020)

#define IOCON_SCKLOC                              (*(pREG32 (0x400440B0)))    // (*(pREG32 (0x40044110)))
#define IOCON_SCKLOC_SCKPIN_PIO0_10               ((unsigned int) 0x00000000) // Set SCK function to pin 0.10
#define IOCON_SCKLOC_SCKPIN_PIO2_11               ((unsigned int) 0x00000001) // Set SCK function to pin 2.11
#define IOCON_SCKLOC_SCKPIN_PIO0_6                ((unsigned int) 0x00000003) // Set SCK function to pin 0.6
#define IOCON_SCKLOC_SCKPIN_MASK                  ((unsigned int) 0x00000003)

#define IOCON_DSR_LOC                             (*(pREG32 (0x400440B4)))
#define IOCON_DSR_LOC_DSRLOC_PIO2_1               ((unsigned int) 0x00000001)
#define IOCON_DSR_LOC_DSRLOC_PIO3_1               ((unsigned int) 0x00000002) // Set DSR function to pin 2.1
#define IOCON_DSR_LOC_DSRLOC_MASK                 ((unsigned int) 0x00000003) // Set DSR function to pin 3.1

#define IOCON_DCD_LOC                             (*(pREG32 (0x400440B8)))
#define IOCON_DCD_LOC_DCDLOC_PIO2_2               ((unsigned int) 0x00000001) // Set DCD function to pin 2.2
#define IOCON_DCD_LOC_DCDLOC_PIO3_2               ((unsigned int) 0x00000002) // Set DCD function to pin 3.2
#define IOCON_DCD_LOC_DCDLOC_MASK                 ((unsigned int) 0x00000003)

#define IOCON_RI_LOC                              (*(pREG32 (0x400440BC)))
#define IOCON_RI_LOC_RILOC_PIO2_3                 ((unsigned int) 0x00000001) // Set RI function to pin 2.3
#define IOCON_RI_LOC_RILOC_PIO3_3                 ((unsigned int) 0x00000002) // Set RI function to pin 3.3
#define IOCON_RI_LOC_RILOC_MASK                   ((unsigned int) 0x00000003)

/*##############################################################################
## Nested Vectored Interrupt Controller
##############################################################################*/

#define NVIC_BASE_ADDRESS                         (0xE000E100)

typedef struct
{
  volatile uint32_t ISER[8];                      /*!< Offset: 0x000  Interrupt Set Enable Register           */
	 uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                      /*!< Offset: 0x080  Interrupt Clear Enable Register         */
	  uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                      /*!< Offset: 0x100  Interrupt Set Pending Register          */
	 uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                      /*!< Offset: 0x180  Interrupt Clear Pending Register        */
	 uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                      /*!< Offset: 0x200  Interrupt Active bit Register           */
	 uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                      /*!< Offset: 0x300  Interrupt Priority Register (8Bit wide) */
	uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                        /*!< Offset: 0xE00  Software Trigger Interrupt Register     */
}  NVIC_Type;                                               

#define NVIC                                      ((NVIC_Type *) NVIC_BASE_ADDRESS)

static inline void __enable_irq()                 { __asm volatile ("cpsie i"); }
static inline void __disable_irq()                { __asm volatile ("cpsid i"); }

typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn           = -14,    /*!< 2 Non Maskable Interrupt                           */
  HardFault_IRQn                = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                   */
  SVCall_IRQn                   = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                     */
  PendSV_IRQn                   = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                     */
  SysTick_IRQn                  = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                 */

/******  LPC11xx Specific Interrupt Numbers *******************************************************/
  WAKEUP0_IRQn                  = 0,        /*!< All I/O pins can be used as wakeup source.       */
  WAKEUP1_IRQn                  = 1,        /*!< There are 13 pins in total for LPC11xx           */
  WAKEUP2_IRQn                  = 2,
  WAKEUP3_IRQn                  = 3,
  WAKEUP4_IRQn                  = 4,   
  WAKEUP5_IRQn                  = 5,        
  WAKEUP6_IRQn                  = 6,        
  WAKEUP7_IRQn                  = 7,        
  WAKEUP8_IRQn                  = 8,        
  WAKEUP9_IRQn                  = 9,        
  WAKEUP10_IRQn                 = 10,       
  WAKEUP11_IRQn                 = 11,       
  WAKEUP12_IRQn                 = 12,       
  SSP1_IRQn                     = 14,       /*!< SSP1 Interrupt                                   */
  I2C_IRQn                      = 15,       /*!< I2C Interrupt                                    */
  TIMER_16_0_IRQn               = 16,       /*!< 16-bit Timer0 Interrupt                          */
  TIMER_16_1_IRQn               = 17,       /*!< 16-bit Timer1 Interrupt                          */
  TIMER_32_0_IRQn               = 18,       /*!< 32-bit Timer0 Interrupt                          */
  TIMER_32_1_IRQn               = 19,       /*!< 32-bit Timer1 Interrupt                          */
  SSP0_IRQn                     = 20,       /*!< SSP0 Interrupt                                   */
  UART_IRQn                     = 21,       /*!< UART Interrupt                                   */
  ADC_IRQn                      = 24,       /*!< A/D Converter Interrupt                          */
  WDT_IRQn                      = 25,       /*!< Watchdog timer Interrupt                         */  
  BOD_IRQn                      = 26,       /*!< Brown Out Detect(BOD) Interrupt                  */
  EINT3_IRQn                    = 28,       /*!< External Interrupt 3 Interrupt                   */
  EINT2_IRQn                    = 29,       /*!< External Interrupt 2 Interrupt                   */
  EINT1_IRQn                    = 30,       /*!< External Interrupt 1 Interrupt                   */
  EINT0_IRQn                    = 31,       /*!< External Interrupt 0 Interrupt                   */
} IRQn_t;

static inline void NVIC_EnableIRQ(IRQn_t IRQn)
{
  NVIC->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

static inline void NVIC_DisableIRQ(IRQn_t IRQn)
{
  NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*##############################################################################
## GPIO - General Purpose I/O
##############################################################################*/

#define GPIO_GPIO0_BASE                           (0x50000000)
#define GPIO_GPIO1_BASE                           (0x50010000)
#define GPIO_GPIO2_BASE                           (0x50020000)
#define GPIO_GPIO3_BASE                           (0x50030000)

#define GPIO_GPIO0DATA                            (*(pREG32 (0x50003FFC)))    // Port data register
#define GPIO_GPIO0DIR                             (*(pREG32 (0x50008000)))    // Data direction register
#define GPIO_GPIO0IS                              (*(pREG32 (0x50008004)))    // Interrupt sense register
#define GPIO_GPIO0IBE                             (*(pREG32 (0x50008008)))    // Interrupt both edges register
#define GPIO_GPIO0IEV                             (*(pREG32 (0x5000800C)))    // Interrupt event register
#define GPIO_GPIO0IE                              (*(pREG32 (0x50008010)))    // Interrupt mask register
#define GPIO_GPIO0RIS                             (*(pREG32 (0x50008014)))    // Raw interrupt status register
#define GPIO_GPIO0MIS                             (*(pREG32 (0x50008018)))    // Masked interrupt status register
#define GPIO_GPIO0IC                              (*(pREG32 (0x5000801C)))    // Interrupt clear register

#define GPIO_GPIO1DATA                            (*(pREG32 (0x50013FFC)))    // Port data register
#define GPIO_GPIO1DIR                             (*(pREG32 (0x50018000)))    // Data direction register
#define GPIO_GPIO1IS                              (*(pREG32 (0x50018004)))    // Interrupt sense register
#define GPIO_GPIO1IBE                             (*(pREG32 (0x50018008)))    // Interrupt both edges register
#define GPIO_GPIO1IEV                             (*(pREG32 (0x5001800C)))    // Interrupt event register
#define GPIO_GPIO1IE                              (*(pREG32 (0x50018010)))    // Interrupt mask register
#define GPIO_GPIO1RIS                             (*(pREG32 (0x50018014)))    // Raw interrupt status register
#define GPIO_GPIO1MIS                             (*(pREG32 (0x50018018)))    // Masked interrupt status register
#define GPIO_GPIO1IC                              (*(pREG32 (0x5001801C)))    // Interrupt clear register

#define GPIO_GPIO2DATA                            (*(pREG32 (0x50023FFC)))    // Port data register
#define GPIO_GPIO2DIR                             (*(pREG32 (0x50028000)))    // Data direction register
#define GPIO_GPIO2IS                              (*(pREG32 (0x50028004)))    // Interrupt sense register
#define GPIO_GPIO2IBE                             (*(pREG32 (0x50028008)))    // Interrupt both edges register
#define GPIO_GPIO2IEV                             (*(pREG32 (0x5002800C)))    // Interrupt event register
#define GPIO_GPIO2IE                              (*(pREG32 (0x50028010)))    // Interrupt mask register
#define GPIO_GPIO2RIS                             (*(pREG32 (0x50028014)))    // Raw interrupt status register
#define GPIO_GPIO2MIS                             (*(pREG32 (0x50028018)))    // Masked interrupt status register
#define GPIO_GPIO2IC                              (*(pREG32 (0x5002801C)))    // Interrupt clear register

#define GPIO_GPIO3DATA                            (*(pREG32 (0x50033FFC)))    // Port data register
#define GPIO_GPIO3DIR                             (*(pREG32 (0x50038000)))    // Data direction register
#define GPIO_GPIO3IS                              (*(pREG32 (0x50038004)))    // Interrupt sense register
#define GPIO_GPIO3IBE                             (*(pREG32 (0x50038008)))    // Interrupt both edges register
#define GPIO_GPIO3IEV                             (*(pREG32 (0x5003800C)))    // Interrupt event register
#define GPIO_GPIO3IE                              (*(pREG32 (0x50038010)))    // Interrupt mask register
#define GPIO_GPIO3RIS                             (*(pREG32 (0x50038014)))    // Raw interrupt status register
#define GPIO_GPIO3MIS                             (*(pREG32 (0x50038018)))    // Masked interrupt status register
#define GPIO_GPIO3IC                              (*(pREG32 (0x5003801C)))    // Interrupt clear register

#define GPIO_IO_P0                                ((unsigned int) 0x00000001)
#define GPIO_IO_P1                                ((unsigned int) 0x00000002)
#define GPIO_IO_P2                                ((unsigned int) 0x00000004)
#define GPIO_IO_P3                                ((unsigned int) 0x00000008)
#define GPIO_IO_P4                                ((unsigned int) 0x00000010)
#define GPIO_IO_P5                                ((unsigned int) 0x00000020)
#define GPIO_IO_P6                                ((unsigned int) 0x00000040)
#define GPIO_IO_P7                                ((unsigned int) 0x00000080)
#define GPIO_IO_P8                                ((unsigned int) 0x00000100)
#define GPIO_IO_P9                                ((unsigned int) 0x00000200)
#define GPIO_IO_P10                               ((unsigned int) 0x00000400)
#define GPIO_IO_P11                               ((unsigned int) 0x00000800)
#define GPIO_IO_ALL                               ((unsigned int) 0x00000FFF)

/*##############################################################################
## UART
##############################################################################*/

#define UART_BASE_ADDRESS                         (0x40008000)

/*  Receive buffer */
#define UART_U0RBR                                (*(pREG32 (0x40008000)))
#define UART_U0RBR_MASK                           ((unsigned int) 0x000000FF)

/*  Transmitter holding register */
#define UART_U0THR                                (*(pREG32 (0x40008000)))

/*  Divisor latch LSB */
#define UART_U0DLL                                (*(pREG32 (0x40008000)))

/*  Divisor latch MSB */
#define UART_U0DLM                                (*(pREG32 (0x40008004)))

/* Interrupt enable */
#define UART_U0IER                                (*(pREG32 (0x40008004)))
#define UART_U0IER_RBR_Interrupt_MASK             ((unsigned int) 0x00000001) // Enables the received data available interrupt
#define UART_U0IER_RBR_Interrupt_Enabled          ((unsigned int) 0x00000001)
#define UART_U0IER_RBR_Interrupt_Disabled         ((unsigned int) 0x00000000)
#define UART_U0IER_THRE_Interrupt_MASK            ((unsigned int) 0x00000002) // Enables the THRE interrupt
#define UART_U0IER_THRE_Interrupt_Enabled         ((unsigned int) 0x00000002)
#define UART_U0IER_THRE_Interrupt_Disabled        ((unsigned int) 0x00000000)
#define UART_U0IER_RLS_Interrupt_MASK             ((unsigned int) 0x00000004) // Enables the Rx line status interrupt
#define UART_U0IER_RLS_Interrupt_Enabled          ((unsigned int) 0x00000004)
#define UART_U0IER_RLS_Interrupt_Disabled         ((unsigned int) 0x00000000)
#define UART_U0IER_ABEOIntEn_MASK                 ((unsigned int) 0x00000100) // End of auto-baud interrupt
#define UART_U0IER_ABEOIntEn_Enabled              ((unsigned int) 0x00000100)
#define UART_U0IER_ABEOIntEn_Disabled             ((unsigned int) 0x00000000)
#define UART_U0IER_ABTOIntEn_MASK                 ((unsigned int) 0x00000200) // Auto-baud timeout interrupt
#define UART_U0IER_ABTOIntEn_Enabled              ((unsigned int) 0x00000200)
#define UART_U0IER_ABTOIntEn_Disabled             ((unsigned int) 0x00000000)

/*  Interrupt identification */
#define UART_U0IIR                                (*(pREG32 (0x40008008)))
#define UART_U0IIR_IntStatus_MASK                 ((unsigned int) 0x00000001) // Interrupt status
#define UART_U0IIR_IntStatus_InterruptPending     ((unsigned int) 0x00000001)
#define UART_U0IIR_IntStatus_NoInterruptPending   ((unsigned int) 0x00000000)
#define UART_U0IIR_IntId_MASK                     ((unsigned int) 0x0000000E) // Interrupt identification
#define UART_U0IIR_IntId_RLS                      ((unsigned int) 0x00000006) // Receive line status
#define UART_U0IIR_IntId_RDA                      ((unsigned int) 0x00000004) // Receive data available
#define UART_U0IIR_IntId_CTI                      ((unsigned int) 0x0000000C) // Character time-out indicator
#define UART_U0IIR_IntId_THRE                     ((unsigned int) 0x00000002) // THRE interrupt
#define UART_U0IIR_IntId_MODEM                    ((unsigned int) 0x00000000) // Modem interrupt
#define UART_U0IIR_FIFO_Enable_MASK               ((unsigned int) 0x000000C0)
#define UART_U0IIR_ABEOInt_MASK                   ((unsigned int) 0x00000100) // End of auto-baud interrupt
#define UART_U0IIR_ABEOInt                        ((unsigned int) 0x00000100)
#define UART_U0IIR_ABTOInt_MASK                   ((unsigned int) 0x00000200) // Auto-baud time-out interrupt
#define UART_U0IIR_ABTOInt                        ((unsigned int) 0x00000200)

/*  FIFO control */
#define UART_U0FCR                                (*(pREG32 (0x40008008)))
#define UART_U0FCR_FIFO_Enable_MASK               ((unsigned int) 0x00000001) // UART FIFOs enabled/disabled
#define UART_U0FCR_FIFO_Enabled                   ((unsigned int) 0x00000001)
#define UART_U0FCR_FIFO_Disabled                  ((unsigned int) 0x00000000)
#define UART_U0FCR_Rx_FIFO_Reset_MASK             ((unsigned int) 0x00000002)
#define UART_U0FCR_Rx_FIFO_Reset                  ((unsigned int) 0x00000002) // Clear Rx FIFO
#define UART_U0FCR_Tx_FIFO_Reset_MASK             ((unsigned int) 0x00000004)
#define UART_U0FCR_Tx_FIFO_Reset                  ((unsigned int) 0x00000004) // Clear Tx FIFO
#define UART_U0FCR_Rx_Trigger_Level_Select_MASK   ((unsigned int) 0x000000C0) // Chars written before before interrupt
#define UART_U0FCR_Rx_Trigger_Level_Select_1Char  ((unsigned int) 0x00000000) 
#define UART_U0FCR_Rx_Trigger_Level_Select_4Char  ((unsigned int) 0x00000040) 
#define UART_U0FCR_Rx_Trigger_Level_Select_8Char  ((unsigned int) 0x00000080) 
#define UART_U0FCR_Rx_Trigger_Level_Select_12Char ((unsigned int) 0x000000C0) 

/*  Modem control */
#define UART_U0MCR                                (*(pREG32 (0x40008010)))
#define UART_U0MCR_DTR_Control_MASK               ((unsigned int) 0x00000001) // Source for modem output pin DTR
#define UART_U0MCR_DTR_Control                    ((unsigned int) 0x00000001)
#define UART_U0MCR_RTS_Control_MASK               ((unsigned int) 0x00000002) // Source for modem output pin RTS
#define UART_U0MCR_RTS_Control                    ((unsigned int) 0x00000002)
#define UART_U0MCR_Loopback_Mode_Select_MASK      ((unsigned int) 0x00000010) // Diagnostic loopback mode
#define UART_U0MCR_Loopback_Mode_Select_Enabled   ((unsigned int) 0x00000010)
#define UART_U0MCR_Loopback_Mode_Select_Disabled  ((unsigned int) 0x00000000)
#define UART_U0MCR_RTSen_MASK                     ((unsigned int) 0x00000040) // Disable auto-rts flow control
#define UART_U0MCR_RTSen_Enabled                  ((unsigned int) 0x00000040)
#define UART_U0MCR_RTSen_Disabled                 ((unsigned int) 0x00000000)
#define UART_U0MCR_CTSen_MASK                     ((unsigned int) 0x00000080) // Disable auto-cts flow control
#define UART_U0MCR_CTSen_Enabled                  ((unsigned int) 0x00000080)
#define UART_U0MCR_CTSen_Disabled                 ((unsigned int) 0x00000000)

/*  Line control */
#define UART_U0LCR                                (*(pREG32 (0x4000800C)))
#define UART_U0LCR_Word_Length_Select_MASK        ((unsigned int) 0x00000003) // Word Length Selector
#define UART_U0LCR_Word_Length_Select_5Chars      ((unsigned int) 0x00000000)
#define UART_U0LCR_Word_Length_Select_6Chars      ((unsigned int) 0x00000001)
#define UART_U0LCR_Word_Length_Select_7Chars      ((unsigned int) 0x00000002)
#define UART_U0LCR_Word_Length_Select_8Chars      ((unsigned int) 0x00000003)
#define UART_U0LCR_Stop_Bit_Select_MASK           ((unsigned int) 0x00000004) // Stop bit select
#define UART_U0LCR_Stop_Bit_Select_1Bits          ((unsigned int) 0x00000000)
#define UART_U0LCR_Stop_Bit_Select_2Bits          ((unsigned int) 0x00000004)
#define UART_U0LCR_Parity_Enable_MASK             ((unsigned int) 0x00000008) // Parity enable
#define UART_U0LCR_Parity_Enabled                 ((unsigned int) 0x00000008)
#define UART_U0LCR_Parity_Disabled                ((unsigned int) 0x00000000)
#define UART_U0LCR_Parity_Select_MASK             ((unsigned int) 0x00000030) // Parity select
#define UART_U0LCR_Parity_Select_OddParity        ((unsigned int) 0x00000000)
#define UART_U0LCR_Parity_Select_EvenParity       ((unsigned int) 0x00000010)
#define UART_U0LCR_Parity_Select_Forced1          ((unsigned int) 0x00000020)
#define UART_U0LCR_Parity_Select_Forced0          ((unsigned int) 0x00000030)
#define UART_U0LCR_Break_Control_MASK             ((unsigned int) 0x00000040) // Break transmission control
#define UART_U0LCR_Break_Control_Enabled          ((unsigned int) 0x00000040)
#define UART_U0LCR_Break_Control_Disabled         ((unsigned int) 0x00000000)
#define UART_U0LCR_Divisor_Latch_Access_MASK      ((unsigned int) 0x00000080) // Divisor latch access
#define UART_U0LCR_Divisor_Latch_Access_Enabled   ((unsigned int) 0x00000080)
#define UART_U0LCR_Divisor_Latch_Access_Disabled  ((unsigned int) 0x00000000)

/*  Line status */
#define UART_U0LSR                                (*(pREG32 (0x40008014)))
#define UART_U0LSR_RDR_MASK                       ((unsigned int) 0x00000001) // Receiver data ready
#define UART_U0LSR_RDR_EMPTY                      ((unsigned int) 0x00000000) // U0RBR is empty
#define UART_U0LSR_RDR_DATA                       ((unsigned int) 0x00000001) // U0RBR contains valid data
#define UART_U0LSR_OE_MASK                        ((unsigned int) 0x00000002) // Overrun error
#define UART_U0LSR_OE                             ((unsigned int) 0x00000002)
#define UART_U0LSR_PE_MASK                        ((unsigned int) 0x00000004) // Parity error
#define UART_U0LSR_PE                             ((unsigned int) 0x00000004)
#define UART_U0LSR_FE_MASK                        ((unsigned int) 0x00000008) // Framing error
#define UART_U0LSR_FE                             ((unsigned int) 0x00000008)
#define UART_U0LSR_BI_MASK                        ((unsigned int) 0x00000010) // Break interrupt
#define UART_U0LSR_BI                             ((unsigned int) 0x00000010)
#define UART_U0LSR_THRE_MASK                      ((unsigned int) 0x00000020) // Transmitter holding register empty
#define UART_U0LSR_THRE                           ((unsigned int) 0x00000020)
#define UART_U0LSR_TEMT_MASK                      ((unsigned int) 0x00000040) // Transmitter empty
#define UART_U0LSR_TEMT                           ((unsigned int) 0x00000040)
#define UART_U0LSR_RXFE_MASK                      ((unsigned int) 0x00000080) // Error in Rx FIFO
#define UART_U0LSR_RXFE                           ((unsigned int) 0x00000080)

/*  Modem status */
#define UART_U0MSR                                (*(pREG32 (0x40008018)))
#define UART_U0MSR_Delta_CTS_MASK                 ((unsigned int) 0x00000001) // State change of input CTS
#define UART_U0MSR_Delta_CTS                      ((unsigned int) 0x00000001)
#define UART_U0MSR_Delta_DSR_MASK                 ((unsigned int) 0x00000002) // State change of input DSR
#define UART_U0MSR_Delta_DSR                      ((unsigned int) 0x00000002)
#define UART_U0MSR_Trailing_Edge_RI_MASK          ((unsigned int) 0x00000004) // Low to high transition of input RI
#define UART_U0MSR_Trailing_Edge_RI               ((unsigned int) 0x00000004)
#define UART_U0MSR_Delta_DCD_MASK                 ((unsigned int) 0x00000008) // State change of input DCD
#define UART_U0MSR_Delta_DCD                      ((unsigned int) 0x00000008)
#define UART_U0MSR_CTS_MASK                       ((unsigned int) 0x00000010) // Clear to send state
#define UART_U0MSR_CTS                            ((unsigned int) 0x00000010)
#define UART_U0MSR_DSR_MASK                       ((unsigned int) 0x00000020) // Data set ready state
#define UART_U0MSR_DSR                            ((unsigned int) 0x00000020)
#define UART_U0MSR_RI_MASK                        ((unsigned int) 0x00000040) // Ring indicator state
#define UART_U0MSR_RI                             ((unsigned int) 0x00000040)
#define UART_U0MSR_DCD_MASK                       ((unsigned int) 0x00000080) // Data carrier detect state
#define UART_U0MSR_DCD                            ((unsigned int) 0x00000080)

/*  Scratch pad */
#define UART_U0SCR                                (*(pREG32 (0x4000801C)))

/*  Auto-baud control */
#define UART_U0ACR                                (*(pREG32 (0x40008020)))
#define UART_U0ACR_Start_MASK                     ((unsigned int) 0x00000001) // Auto-baud start/stop
#define UART_U0ACR_Start                          ((unsigned int) 0x00000001)
#define UART_U0ACR_Stop                           ((unsigned int) 0x00000000)
#define UART_U0ACR_Mode_MASK                      ((unsigned int) 0x00000002) // Auto-baud mode select
#define UART_U0ACR_Mode_Mode1                     ((unsigned int) 0x00000000)
#define UART_U0ACR_Mode_Mode2                     ((unsigned int) 0x00000002)
#define UART_U0ACR_AutoRestart_MASK               ((unsigned int) 0x00000004)
#define UART_U0ACR_AutoRestart_NoRestart          ((unsigned int) 0x00000000)
#define UART_U0ACR_AutoRestart_Restart            ((unsigned int) 0x00000004) // Restart in case of time-out
#define UART_U0ACR_ABEOIntClr_MASK                ((unsigned int) 0x00000100) // End of auto-baud interrupt clear bit
#define UART_U0ACR_ABEOIntClr                     ((unsigned int) 0x00000100) 
#define UART_U0ACR_ABTOIntClr_MASK                ((unsigned int) 0x00000200) // Auto-baud timeout interrupt clear bit
#define UART_U0ACR_ABTOIntClr                     ((unsigned int) 0x00000200)

/*  Fractional divider */
#define UART_U0FDR                                (*(pREG32 (0x40008028)))
#define UART_U0FDR_DIVADDVAL_MASK                 ((unsigned int) 0x0000000F) // Fractional divider: prescaler register
#define UART_U0FDR_MULVAL_MASK                    ((unsigned int) 0x000000F0) // Fractional divider: prescaler multiplier

/*  Transmit enable */
#define UART_U0TER                                (*(pREG32 (0x40008030)))
#define UART_U0TER_TXEN_MASK                      ((unsigned int) 0x00000080) // UART transmit enable
#define UART_U0TER_TXEN_Enabled                   ((unsigned int) 0x00000080)
#define UART_U0TER_TXEN_Disabled                  ((unsigned int) 0x00000000)

/*  RS485 control register */
#define UART_U0RS485CTRL                          (*(pREG32 (0x4000804C)))
#define UART_U0RS485CTRL_NMMEN_MASK               ((unsigned int) 0x00000001) // Normal multi-drop mode
#define UART_U0RS485CTRL_NMMEN                    ((unsigned int) 0x00000001)
#define UART_U0RS485CTRL_RXDIS_MASK               ((unsigned int) 0x00000002) // Receiver
#define UART_U0RS485CTRL_RXDIS                    ((unsigned int) 0x00000002)
#define UART_U0RS485CTRL_AADEN_MASK               ((unsigned int) 0x00000004) // Auto-address detect
#define UART_U0RS485CTRL_AADEN                    ((unsigned int) 0x00000004)
#define UART_U0RS485CTRL_SEL_MASK                 ((unsigned int) 0x00000008) 
#define UART_U0RS485CTRL_SEL_RTS                  ((unsigned int) 0x00000000) // Use RTS for direction control
#define UART_U0RS485CTRL_SEL_DTS                  ((unsigned int) 0x00000008) // Use DTS for direction control
#define UART_U0RS485CTRL_DCTRL_MASK               ((unsigned int) 0x00000010) // Enable/Disable auto-direction control
#define UART_U0RS485CTRL_DCTRL_Disabled           ((unsigned int) 0x00000000)
#define UART_U0RS485CTRL_DCTRL_Enabled            ((unsigned int) 0x00000010)
#define UART_U0RS485CTRL_OINV_MASK                ((unsigned int) 0x00000020) // Reverse polarity of direction control signal on RTS/DTR pin
#define UART_U0RS485CTRL_OINV_Normal              ((unsigned int) 0x00000000)
#define UART_U0RS485CTRL_OINV_Inverted            ((unsigned int) 0x00000020)

/*  RS485 address match */
#define UART_U0RS485ADRMATCH                      (*(pREG32 (0x40008050)))

/*  RS485 delay value */
#define UART_U0RS485DLY                           (*(pREG32 (0x40008054)))

/*  UART FIFO level */
#define UART_U0FIFOLVL                            (*(pREG32 (0x40008058)))
#define UART_U0FIFOLVL_RXFIFOLVL_MASK             ((unsigned int) 0x0000000F)
#define UART_U0FIFOLVL_RXFIFOLVL_Empty            ((unsigned int) 0x00000000)
#define UART_U0FIFOLVL_RXFIFOLVL_Full             ((unsigned int) 0x0000000F)
#define UART_U0FIFOLVL_TXFIFOLVL_MASK             ((unsigned int) 0x00000F00)
#define UART_U0FIFOLVL_TXFIFOLVL_Empty            ((unsigned int) 0x00000000)
#define UART_U0FIFOLVL_TXFIFOLVL_Full             ((unsigned int) 0x00000F00)

/*##############################################################################
## I2C
##############################################################################*/

#define I2C_BASE_ADDRESS                          (0x40000000)

/*  I2CCONSET (I2C Control Set register)
    The I2CONSET registers control setting of bits in the I2CON register that controls
    operation of the I2C interface. Writing a one to a bit of this register causes the
    corresponding bit in the I2C control register to be set. Writing a zero has no effect. */

#define I2C_I2CCONSET                             (*(pREG32 (0x40000000)))    // I2C control set register
#define I2C_I2CCONSET_AA_MASK                     ((unsigned int) 0x00000004)
#define I2C_I2CCONSET_AA                          ((unsigned int) 0x00000004) // Asset acknowlegde flag
#define I2C_I2CCONSET_SI_MASK                     ((unsigned int) 0x00000008)
#define I2C_I2CCONSET_SI                          ((unsigned int) 0x00000008) // I2C interrupt flag
#define I2C_I2CCONSET_STO_MASK                    ((unsigned int) 0x00000010)
#define I2C_I2CCONSET_STO                         ((unsigned int) 0x00000010) // Stop flag
#define I2C_I2CCONSET_STA_MASK                    ((unsigned int) 0x00000020)
#define I2C_I2CCONSET_STA                         ((unsigned int) 0x00000020) // Start flag
#define I2C_I2CCONSET_I2EN_MASK                   ((unsigned int) 0x00000040)
#define I2C_I2CCONSET_I2EN                        ((unsigned int) 0x00000040) // I2C interface enable

/*  I2CSTAT (I2C Status register)
    Each I2C Status register reflects the condition of the corresponding I2C interface. The I2C
    Status register is Read-Only. */

#define I2C_I2CSTAT                               (*(pREG32 (0x40000004)))    // I2C status register
#define I2C_I2CSTAT_Status_MASK                   ((unsigned int) 0x000000F8) // Status information

/*  I2CDAT (I2C Data register)
    This register contains the data to be transmitted or the data just received. The CPU can
    read and write to this register only while it is not in the process of shifting a byte, when the
    SI bit is set. Data in I2DAT remains stable as long as the SI bit is set. Data in I2DAT is
    always shifted from right to left: the first bit to be transmitted is the MSB (bit 7), and after a
    byte has been received, the first bit of received data is located at the MSB of I2DAT. */

#define I2C_I2CDAT                                (*(pREG32 (0x40000008)))    // I2C data register

/*  I2CADR0 (I2C Slave Address register)
    These registers are readable and writable and are only used when an I2C interface is set
    to slave mode.  */

#define I2C_I2CADR0                               (*(pREG32 (0x4000000C)))    // I2C slave address register
#define I2C_I2CADR0_GC_MASK                       ((unsigned int) 0x00000001)
#define I2C_I2CADR0_GC                            ((unsigned int) 0x00000001) // General call enable bit
#define I2C_I2CADR0_Address_MASK                  ((unsigned int) 0x000000FE) // I2C device address for slave mode

/*  I2CSCLH (I2C SCL HIGH duty cycle register) */

#define I2C_I2CSCLH                               (*(pREG32 (0x40000010)))

/*  I2CSCLL (I2C SCL LOW duty cycle register) */

#define I2C_I2CSCLL                               (*(pREG32 (0x40000014)))

/*  I2CCONCLR (I2C Control Clear register)
    The I2CONCLR registers control clearing of bits in the I2CON register that controls
    operation of the I2C interface. Writing a one to a bit of this register causes the
    corresponding bit in the I2C control register to be cleared. Writing a zero has no effect.  */

#define I2C_I2CCONCLR                             (*(pREG32 (0x40000018)))    // I2C control clear register
#define I2C_I2CCONCLR_AAC_MASK                    ((unsigned int) 0x00000004) // Assert acknowledge clear bit
#define I2C_I2CCONCLR_AAC                         ((unsigned int) 0x00000004)
#define I2C_I2CCONCLR_SIC_MASK                    ((unsigned int) 0x00000008) // I2C interrupt clear bit
#define I2C_I2CCONCLR_SIC                         ((unsigned int) 0x00000008)
#define I2C_I2CCONCLR_STAC_MASK                   ((unsigned int) 0x00000020) // Start flag clear bit
#define I2C_I2CCONCLR_STAC                        ((unsigned int) 0x00000020)
#define I2C_I2CCONCLR_I2ENC_MASK                  ((unsigned int) 0x00000040) // I2C interface disable bit
#define I2C_I2CCONCLR_I2ENC                       ((unsigned int) 0x00000040)

/*  I2CMMCTRL (I2C Monitor mode control register)
    This register controls the Monitor mode which allows the I2C module to monitor traffic on
    the I2C bus without actually participating in traffic or interfering with the I2C bus.  */

#define I2C_I2CMMCTRL                             (*(pREG32 (0x4000001C)))    // I2C monitor control register
#define I2C_I2CMMCTRL_MM_ENA_MASK                 ((unsigned int) 0x00000001) // Monitor mode enable
#define I2C_I2CMMCTRL_MM_ENA_ENABLED              ((unsigned int) 0x00000001)
#define I2C_I2CMMCTRL_MM_ENA_DISABLED             ((unsigned int) 0x00000000)
#define I2C_I2CMMCTRL_ENA_SCL_MASK                ((unsigned int) 0x00000002) // SCL output enable
#define I2C_I2CMMCTRL_ENA_SCL_HOLDLOW             ((unsigned int) 0x00000002)
#define I2C_I2CMMCTRL_ENA_SCL_FORCEHIGH           ((unsigned int) 0x00000000)
#define I2C_I2CMMCTRL_MATCH_ALL_MASK              ((unsigned int) 0x00000008) // Select interrupt register match
#define I2C_I2CMMCTRL_MATCH_ALL_NORMAL            ((unsigned int) 0x00000000)
#define I2C_I2CMMCTRL_MATCH_ALL_ANYADDRESS        ((unsigned int) 0x00000008)

/*  I2CADR1..3 (I2C Slave Address registers)
    These registers are readable and writable and are only used when an I2C interface is set
    to slave mode. In master mode, this register has no effect. The LSB of I2ADR is the
    General Call bit. When this bit is set, the General Call address (0x00) is recognized. */

#define I2C_I2CADR1                               (*(pREG32 (0x40000020)))    // I2C slave address register 1
#define I2C_I2CADR1_GC_MASK                       ((unsigned int) 0x00000001) // General call enable bit
#define I2C_I2CADR1_GC                            ((unsigned int) 0x00000001)
#define I2C_I2CADR1_Address_MASK                  ((unsigned int) 0x000000FE)

#define I2C_I2CADR2                               (*(pREG32 (0x40000024)))    // I2C slave address register 2
#define I2C_I2CADR2_GC_MASK                       ((unsigned int) 0x00000001) // General call enable bit
#define I2C_I2CADR2_GC                            ((unsigned int) 0x00000001)
#define I2C_I2CADR2_Address_MASK                  ((unsigned int) 0x000000FE)

#define I2C_I2CADR3                               (*(pREG32 (0x40000028)))    // I2C slave address register 3
#define I2C_I2CADR3_GC_MASK                       ((unsigned int) 0x00000001) // General call enable bit
#define I2C_I2CADR3_GC                            ((unsigned int) 0x00000001)
#define I2C_I2CADR3_Address_MASK                  ((unsigned int) 0x000000FE)

/*  I2CDATA_BUFFER (I2C Data buffer register) */

#define I2C_I2CDATA_BUFFER                        (*(pREG32 (0x4000002C)))    // I2C data buffer register

/*  I2CMASK0..3 (I2C Mask registers) */

#define I2C_I2CMASK0                              (*(pREG32 (0x40000030)))    // I2C mask register 0
#define I2C_I2CMASK0_MASK_MASK                    ((unsigned int) 0x000000FE)

#define I2C_I2CMASK1                              (*(pREG32 (0x40000034)))    // I2C mask register 1
#define I2C_I2CMASK1_MASK_MASK                    ((unsigned int) 0x000000FE)

#define I2C_I2CMASK2                              (*(pREG32 (0x40000038)))    // I2C mask register 2
#define I2C_I2CMASK2_MASK_MASK                    ((unsigned int) 0x000000FE)

#define I2C_I2CMASK3                              (*(pREG32 (0x4000003C)))    // I2C mask register 3
#define I2C_I2CMASK3_MASK_MASK                    ((unsigned int) 0x000000FE)

/*##############################################################################
## SSP0/1 - Synchronous Serial Ports
##############################################################################*/

#define SSP_SSP0_BASE_ADDRESS                     (0x40040000)

/*  SSP0CR0 (SSP0 Control Register 0)
    This register controls the basic operation of the SSP controller. */

#define SSP_SSP0CR0                               (*(pREG32 (0x40040000)))    // Control register 0
#define SSP_SSP0CR0_DSS_MASK                      ((unsigned int) 0x0000000F) // Data size select
#define SSP_SSP0CR0_DSS_4BIT                      ((unsigned int) 0x00000003)
#define SSP_SSP0CR0_DSS_5BIT                      ((unsigned int) 0x00000004)
#define SSP_SSP0CR0_DSS_6BIT                      ((unsigned int) 0x00000005)
#define SSP_SSP0CR0_DSS_7BIT                      ((unsigned int) 0x00000006)
#define SSP_SSP0CR0_DSS_8BIT                      ((unsigned int) 0x00000007)
#define SSP_SSP0CR0_DSS_9BIT                      ((unsigned int) 0x00000008)
#define SSP_SSP0CR0_DSS_10BIT                     ((unsigned int) 0x00000009)
#define SSP_SSP0CR0_DSS_11BIT                     ((unsigned int) 0x0000000A)
#define SSP_SSP0CR0_DSS_12BIT                     ((unsigned int) 0x0000000B)
#define SSP_SSP0CR0_DSS_13BIT                     ((unsigned int) 0x0000000C)
#define SSP_SSP0CR0_DSS_14BIT                     ((unsigned int) 0x0000000D)
#define SSP_SSP0CR0_DSS_15BIT                     ((unsigned int) 0x0000000E)
#define SSP_SSP0CR0_DSS_16BIT                     ((unsigned int) 0x0000000F)
#define SSP_SSP0CR0_FRF_MASK                      ((unsigned int) 0x00000030) // Frame format
#define SSP_SSP0CR0_FRF_SPI                       ((unsigned int) 0x00000000)
#define SSP_SSP0CR0_FRF_TI                        ((unsigned int) 0x00000010)
#define SSP_SSP0CR0_FRF_MWIRE                     ((unsigned int) 0x00000020)
#define SSP_SSP0CR0_CPOL_MASK                     ((unsigned int) 0x00000040) // Clock out polarity
#define SSP_SSP0CR0_CPOL_LOW                      ((unsigned int) 0x00000000)
#define SSP_SSP0CR0_CPOL_HIGH                     ((unsigned int) 0x00000040)
#define SSP_SSP0CR0_CPHA_MASK                     ((unsigned int) 0x00000080) // Clock out phase
#define SSP_SSP0CR0_CPHA_FIRST                    ((unsigned int) 0x00000000)
#define SSP_SSP0CR0_CPHA_SECOND                   ((unsigned int) 0x00000080)

/*  Serial Clock Rate. The number of prescaler-output clocks per
    bit on the bus, minus one. Given that CPSDVSR is the
    prescale divider, and the APB clock PCLK clocks the
    prescaler, the bit frequency is PCLK / (CPSDVSR × [SCR+1]). */

#define SSP_SSP0CR0_SCR_MASK                      ((unsigned int) 0x0000FF00) // Serial clock rate
#define SSP_SSP0CR0_SCR_1                         ((unsigned int) 0x00000100)
#define SSP_SSP0CR0_SCR_2                         ((unsigned int) 0x00000200)
#define SSP_SSP0CR0_SCR_3                         ((unsigned int) 0x00000300)
#define SSP_SSP0CR0_SCR_4                         ((unsigned int) 0x00000400)
#define SSP_SSP0CR0_SCR_5                         ((unsigned int) 0x00000500)
#define SSP_SSP0CR0_SCR_6                         ((unsigned int) 0x00000600)
#define SSP_SSP0CR0_SCR_7                         ((unsigned int) 0x00000700)
#define SSP_SSP0CR0_SCR_8                         ((unsigned int) 0x00000800)
#define SSP_SSP0CR0_SCR_9                         ((unsigned int) 0x00000900)
#define SSP_SSP0CR0_SCR_10                        ((unsigned int) 0x00000A00)
#define SSP_SSP0CR0_SCR_11                        ((unsigned int) 0x00000B00)
#define SSP_SSP0CR0_SCR_12                        ((unsigned int) 0x00000C00)
#define SSP_SSP0CR0_SCR_13                        ((unsigned int) 0x00000D00)
#define SSP_SSP0CR0_SCR_14                        ((unsigned int) 0x00000E00)
#define SSP_SSP0CR0_SCR_15                        ((unsigned int) 0x00000F00)
#define SSP_SSP0CR0_SCR_16                        ((unsigned int) 0x00001000)

/*  SSP0CR1 (SSP0 Control Register 1)
    This register controls certain aspects of the operation of the SSP controller.  */

#define SSP_SSP0CR1                               (*(pREG32 (0x40040004)))    // Control register 1
#define SSP_SSP0CR1_LBM_MASK                      ((unsigned int) 0x00000001) // Loop back mode
#define SSP_SSP0CR1_LBM_NORMAL                    ((unsigned int) 0x00000000)
#define SSP_SSP0CR1_LBM_INVERTED                  ((unsigned int) 0x00000001) // MISO/MOSI are reversed
#define SSP_SSP0CR1_SSE_MASK                      ((unsigned int) 0x00000002) // SSP enable
#define SSP_SSP0CR1_SSE_DISABLED                  ((unsigned int) 0x00000000)
#define SSP_SSP0CR1_SSE_ENABLED                   ((unsigned int) 0x00000002)
#define SSP_SSP0CR1_MS_MASK                       ((unsigned int) 0x00000004) // Master/Slave Mode
#define SSP_SSP0CR1_MS_MASTER                     ((unsigned int) 0x00000000)
#define SSP_SSP0CR1_MS_SLAVE                      ((unsigned int) 0x00000004)
#define SSP_SSP0CR1_SOD_MASK                      ((unsigned int) 0x00000008) // Slave output disable

/*  SSP0DR (SSP0 Data Register)
    Software can write data to be transmitted to this register, and read data that has been
    received. */

#define SSP_SSP0DR                                (*(pREG32 (0x40040008)))    // Data register
#define SSP_SSP0DR_MASK                           ((unsigned int) 0x0000FFFF) // Data

/*  SSP0SR (SSP0 Status Register)
    This read-only register reflects the current status of the SSP controller.  */

#define SSP_SSP0SR                                (*(pREG32 (0x4004000C)))    // Status register
#define SSP_SSP0SR_TFE_MASK                       ((unsigned int) 0x00000001) // Transmit FIFO empty
#define SSP_SSP0SR_TFE_EMPTY                      ((unsigned int) 0x00000001)
#define SSP_SSP0SR_TFE_NOTEMPTY                   ((unsigned int) 0x00000000)
#define SSP_SSP0SR_TNF_MASK                       ((unsigned int) 0x00000002) // Transmit FIFO not full
#define SSP_SSP0SR_TNF_NOTFULL                    ((unsigned int) 0x00000002)
#define SSP_SSP0SR_TNF_FULL                       ((unsigned int) 0x00000000)
#define SSP_SSP0SR_RNE_MASK                       ((unsigned int) 0x00000004) // Receive FIFO not empty
#define SSP_SSP0SR_RNE_NOTEMPTY                   ((unsigned int) 0x00000004)
#define SSP_SSP0SR_RNE_EMPTY                      ((unsigned int) 0x00000000)
#define SSP_SSP0SR_RFF_MASK                       ((unsigned int) 0x00000008) // Receive FIFO full
#define SSP_SSP0SR_RFF_FULL                       ((unsigned int) 0x00000008)
#define SSP_SSP0SR_RFF_NOTFULL                    ((unsigned int) 0x00000000)
#define SSP_SSP0SR_BSY_MASK                       ((unsigned int) 0x00000010) // Busy Flag
#define SSP_SSP0SR_BSY_IDLE                       ((unsigned int) 0x00000000)
#define SSP_SSP0SR_BSY_BUSY                       ((unsigned int) 0x00000010)

/*  SSP0CPSR (SSP0 Clock Prescale Register)
    This register controls the factor by which the Prescaler divides the SSP peripheral clock
    SSP_PCLK to yield the prescaler clock that is, in turn, divided by the SCR factor in
    SSP0CR0, to determine the bit clock.  */

#define SSP_SSP0CPSR                              (*(pREG32 (0x40040010)))    // Clock prescale register
#define SSP_SSP0CPSR_CPSDVSR_MASK                 ((unsigned int) 0x000000FF)
#define SSP_SSP0CPSR_CPSDVSR_DIV2                 ((unsigned int) 0x00000002)
#define SSP_SSP0CPSR_CPSDVSR_DIV4                 ((unsigned int) 0x00000004)

/*  SSP0IMSC (SSP0 Interrupt Mask Set/Clear Register)
    This register controls whether each of the four possible interrupt conditions in the SSP
    controller are enabled. Note that ARM uses the word masked in the opposite sense from
    classic computer terminology, in which masked meant disabled. ARM uses the word
    masked to mean enabled. To avoid confusion we will not use the word masked. */

#define SSP_SSP0IMSC                              (*(pREG32 (0x40040014)))    // Interrupt mask set/clear register
#define SSP_SSP0IMSC_RORIM_MASK                   ((unsigned int) 0x00000001) // Receive overrun interrupt
#define SSP_SSP0IMSC_RORIM_ENBL                   ((unsigned int) 0x00000001)
#define SSP_SSP0IMSC_RORIM_DSBL                   ((unsigned int) 0x00000000)
#define SSP_SSP0IMSC_RTIM_MASK                    ((unsigned int) 0x00000002) // Receive timeout interrupt
#define SSP_SSP0IMSC_RTIM_ENBL                    ((unsigned int) 0x00000002)
#define SSP_SSP0IMSC_RTIM_DSBL                    ((unsigned int) 0x00000000)
#define SSP_SSP0IMSC_RXIM_MASK                    ((unsigned int) 0x00000004) // Rx FIFO >= 1/2 full interrupt
#define SSP_SSP0IMSC_RXIM_ENBL                    ((unsigned int) 0x00000004)
#define SSP_SSP0IMSC_RXIM_DSBL                    ((unsigned int) 0x00000000)
#define SSP_SSP0IMSC_TXIM_MASK                    ((unsigned int) 0x00000008) // Tx FIFO >= 1/2 empty interrupt
#define SSP_SSP0IMSC_TXIM_ENBL                    ((unsigned int) 0x00000008)
#define SSP_SSP0IMSC_TXIM_DSBL                    ((unsigned int) 0x00000000)

/*  SSP0RIS (SSP0 Raw Interrupt Status Register)
    This read-only register contains a 1 for each interrupt condition that is asserted,
    regardless of whether or not the interrupt is enabled in the SSP0IMSC.  */

#define SSP_SSP0RIS                               (*(pREG32 (0x40040018)))    // Raw interrupt status register
#define SSP_SSP0RIS_RORRIS_MASK                   ((unsigned int) 0x00000001) // Frame received while Rx FIFO full
#define SSP_SSP0RIS_RORRIS_RCVD                   ((unsigned int) 0x00000001)
#define SSP_SSP0RIS_RTRIS_MASK                    ((unsigned int) 0x00000002) // Rx FIFO not empty no read within timeout
#define SSP_SSP0RIS_RTRIS_NOTEMPTY                ((unsigned int) 0x00000002)
#define SSP_SSP0RIS_RXRIS_MASK                    ((unsigned int) 0x00000004) // Rx FIFO >= half full
#define SSP_SSP0RIS_RXRIS_HALFFULL                ((unsigned int) 0x00000004)
#define SSP_SSP0RIS_TXRIS_MASK                    ((unsigned int) 0x00000008) // Tx FIF0 >= half-empty
#define SSP_SSP0RIS_TXRIS_HALFEMPTY               ((unsigned int) 0x00000008)

/*  SSP0MIS (SSP0 Masked Interrupt Status Register)
    This read-only register contains a 1 for each interrupt condition that is asserted and
    enabled in the SSP0IMSC. When an SSP interrupt occurs, the interrupt service routine
    should read this register to determine the cause(s) of the interrupt. */

#define SSP_SSP0MIS                               (*(pREG32 (0x4004001C)))    // Masked interrupt status register
#define SSP_SSP0MIS_RORMIS_MASK                   ((unsigned int) 0x00000001) // Frame received while Rx FIFO full
#define SSP_SSP0MIS_RORMIS_FRMRCVD                ((unsigned int) 0x00000001)
#define SSP_SSP0MIS_RTMIS_MASK                    ((unsigned int) 0x00000002) // Rx FIFO not empty no read withing timeout
#define SSP_SSP0MIS_RTMIS_NOTEMPTY                ((unsigned int) 0x00000002)
#define SSP_SSP0MIS_RXMIS_MASK                    ((unsigned int) 0x00000004) // Rx FIFO >= half full
#define SSP_SSP0MIS_RXMIS_HALFFULL                ((unsigned int) 0x00000004)
#define SSP_SSP0MIS_TXMIS_MASK                    ((unsigned int) 0x00000008) // Tx FIFO >= half-empty
#define SSP_SSP0MIS_TXMIS_HALFEMPTY               ((unsigned int) 0x00000008)

/*  SSP0ICR (SSP0 Interrupt Clear Register)
    Software can write one or more one(s) to this write-only register, to clear the
    corresponding interrupt condition(s) in the SSP controller. Note that the other two interrupt
    conditions can be cleared by writing or reading the appropriate FIFO, or disabled by
    clearing the corresponding bit in SSP0IMSC. */

#define SSP_SSP0ICR                               (*(pREG32 (0x40040020)))    // SSPICR interrupt clear register
#define SSP_SSP0ICR_RORIC_MASK                    ((unsigned int) 0x00000001) // Clears RORIC interrupt flag
#define SSP_SSP0ICR_RORIC_CLEAR                   ((unsigned int) 0x00000001)
#define SSP_SSP0ICR_RTIC_MASK                     ((unsigned int) 0x00000002) // Clear Rx FIFO not empty/no read flag
#define SSP_SSP0ICR_RTIC_CLEAR                    ((unsigned int) 0x00000002)

#define SSP_SSP1_BASE_ADDRESS                     (0x40058000)

/*  SSP1CR0 (SSP1 Control Register 0)
    This register controls the basic operation of the SSP controller. */

#define SSP_SSP1CR0                               (*(pREG32 (0x40058000)))    // Control register 0
#define SSP_SSP1CR0_DSS_MASK                      ((unsigned int) 0x0000000F) // Data size select
#define SSP_SSP1CR0_DSS_4BIT                      ((unsigned int) 0x00000003)
#define SSP_SSP1CR0_DSS_5BIT                      ((unsigned int) 0x00000004)
#define SSP_SSP1CR0_DSS_6BIT                      ((unsigned int) 0x00000005)
#define SSP_SSP1CR0_DSS_7BIT                      ((unsigned int) 0x00000006)
#define SSP_SSP1CR0_DSS_8BIT                      ((unsigned int) 0x00000007)
#define SSP_SSP1CR0_DSS_9BIT                      ((unsigned int) 0x00000008)
#define SSP_SSP1CR0_DSS_10BIT                     ((unsigned int) 0x00000009)
#define SSP_SSP1CR0_DSS_11BIT                     ((unsigned int) 0x0000000A)
#define SSP_SSP1CR0_DSS_12BIT                     ((unsigned int) 0x0000000B)
#define SSP_SSP1CR0_DSS_13BIT                     ((unsigned int) 0x0000000C)
#define SSP_SSP1CR0_DSS_14BIT                     ((unsigned int) 0x0000000D)
#define SSP_SSP1CR0_DSS_15BIT                     ((unsigned int) 0x0000000E)
#define SSP_SSP1CR0_DSS_16BIT                     ((unsigned int) 0x0000000F)
#define SSP_SSP1CR0_FRF_MASK                      ((unsigned int) 0x00000030) // Frame format
#define SSP_SSP1CR0_FRF_SPI                       ((unsigned int) 0x00000000)
#define SSP_SSP1CR0_FRF_TI                        ((unsigned int) 0x00000010)
#define SSP_SSP1CR0_FRF_MWIRE                     ((unsigned int) 0x00000020)
#define SSP_SSP1CR0_CPOL_MASK                     ((unsigned int) 0x00000040) // Clock out polarity
#define SSP_SSP1CR0_CPOL_LOW                      ((unsigned int) 0x00000000)
#define SSP_SSP1CR0_CPOL_HIGH                     ((unsigned int) 0x00000040)
#define SSP_SSP1CR0_CPHA_MASK                     ((unsigned int) 0x00000080) // Clock out phase
#define SSP_SSP1CR0_CPHA_FIRST                    ((unsigned int) 0x00000000)
#define SSP_SSP1CR0_CPHA_SECOND                   ((unsigned int) 0x00000080)

/*  Serial Clock Rate. The number of prescaler-output clocks per
    bit on the bus, minus one. Given that CPSDVSR is the
    prescale divider, and the APB clock PCLK clocks the
    prescaler, the bit frequency is PCLK / (CPSDVSR × [SCR+1]). */

#define SSP_SSP1CR0_SCR_MASK                      ((unsigned int) 0x0000FF00) // Serial clock rate
#define SSP_SSP1CR0_SCR_1                         ((unsigned int) 0x00000100)
#define SSP_SSP1CR0_SCR_2                         ((unsigned int) 0x00000200)
#define SSP_SSP1CR0_SCR_3                         ((unsigned int) 0x00000300)
#define SSP_SSP1CR0_SCR_4                         ((unsigned int) 0x00000400)
#define SSP_SSP1CR0_SCR_5                         ((unsigned int) 0x00000500)
#define SSP_SSP1CR0_SCR_6                         ((unsigned int) 0x00000600)
#define SSP_SSP1CR0_SCR_7                         ((unsigned int) 0x00000700)
#define SSP_SSP1CR0_SCR_8                         ((unsigned int) 0x00000800)
#define SSP_SSP1CR0_SCR_9                         ((unsigned int) 0x00000900)
#define SSP_SSP1CR0_SCR_10                        ((unsigned int) 0x00000A00)
#define SSP_SSP1CR0_SCR_11                        ((unsigned int) 0x00000B00)
#define SSP_SSP1CR0_SCR_12                        ((unsigned int) 0x00000C00)
#define SSP_SSP1CR0_SCR_13                        ((unsigned int) 0x00000D00)
#define SSP_SSP1CR0_SCR_14                        ((unsigned int) 0x00000E00)
#define SSP_SSP1CR0_SCR_15                        ((unsigned int) 0x00000F00)
#define SSP_SSP1CR0_SCR_16                        ((unsigned int) 0x00001000)

/*  SSP1CR1 (SSP1 Control Register 1)
    This register controls certain aspects of the operation of the SSP controller.  */

#define SSP_SSP1CR1                               (*(pREG32 (0x40058004)))    // Control register 1
#define SSP_SSP1CR1_LBM_MASK                      ((unsigned int) 0x00000001) // Loop back mode
#define SSP_SSP1CR1_LBM_NORMAL                    ((unsigned int) 0x00000000)
#define SSP_SSP1CR1_LBM_INVERTED                  ((unsigned int) 0x00000001) // MISO/MOSI are reversed
#define SSP_SSP1CR1_SSE_MASK                      ((unsigned int) 0x00000002) // SSP enable
#define SSP_SSP1CR1_SSE_DISABLED                  ((unsigned int) 0x00000000)
#define SSP_SSP1CR1_SSE_ENABLED                   ((unsigned int) 0x00000002)
#define SSP_SSP1CR1_MS_MASK                       ((unsigned int) 0x00000004) // Master/Slave Mode
#define SSP_SSP1CR1_MS_MASTER                     ((unsigned int) 0x00000000)
#define SSP_SSP1CR1_MS_SLAVE                      ((unsigned int) 0x00000004)
#define SSP_SSP1CR1_SOD_MASK                      ((unsigned int) 0x00000008) // Slave output disable

/*  SSP1DR (SSP1 Data Register)
    Software can write data to be transmitted to this register, and read data that has been
    received. */

#define SSP_SSP1DR                                (*(pREG32 (0x40058008)))    // Data register
#define SSP_SSP1DR_MASK                           ((unsigned int) 0x0000FFFF) // Data

/*  SSP1SR (SSP1 Status Register)
    This read-only register reflects the current status of the SSP controller.  */

#define SSP_SSP1SR                                (*(pREG32 (0x4005800C)))    // Status register
#define SSP_SSP1SR_TFE_MASK                       ((unsigned int) 0x00000001) // Transmit FIFO empty
#define SSP_SSP1SR_TFE_EMPTY                      ((unsigned int) 0x00000001)
#define SSP_SSP1SR_TFE_NOTEMPTY                   ((unsigned int) 0x00000000)
#define SSP_SSP1SR_TNF_MASK                       ((unsigned int) 0x00000002) // Transmit FIFO not full
#define SSP_SSP1SR_TNF_NOTFULL                    ((unsigned int) 0x00000002)
#define SSP_SSP1SR_TNF_FULL                       ((unsigned int) 0x00000000)
#define SSP_SSP1SR_RNE_MASK                       ((unsigned int) 0x00000004) // Receive FIFO not empty
#define SSP_SSP1SR_RNE_NOTEMPTY                   ((unsigned int) 0x00000004)
#define SSP_SSP1SR_RNE_EMPTY                      ((unsigned int) 0x00000000)
#define SSP_SSP1SR_RFF_MASK                       ((unsigned int) 0x00000008) // Receive FIFO full
#define SSP_SSP1SR_RFF_FULL                       ((unsigned int) 0x00000008)
#define SSP_SSP1SR_RFF_NOTFULL                    ((unsigned int) 0x00000000)
#define SSP_SSP1SR_BSY_MASK                       ((unsigned int) 0x00000010) // Busy Flag
#define SSP_SSP1SR_BSY_IDLE                       ((unsigned int) 0x00000000)
#define SSP_SSP1SR_BSY_BUSY                       ((unsigned int) 0x00000010)

/*  SSP1CPSR (SSP1 Clock Prescale Register)
    This register controls the factor by which the Prescaler divides the SSP peripheral clock
    SSP_PCLK to yield the prescaler clock that is, in turn, divided by the SCR factor in
    SSP1CR0, to determine the bit clock.  */

#define SSP_SSP1CPSR                              (*(pREG32 (0x40058010)))    // Clock prescale register
#define SSP_SSP1CPSR_CPSDVSR_MASK                 ((unsigned int) 0x000000FF)
#define SSP_SSP1CPSR_CPSDVSR_DIV2                 ((unsigned int) 0x00000002)
#define SSP_SSP1CPSR_CPSDVSR_DIV4                 ((unsigned int) 0x00000004)

/*  SSP1IMSC (SSP1 Interrupt Mask Set/Clear Register)
    This register controls whether each of the four possible interrupt conditions in the SSP
    controller are enabled. Note that ARM uses the word masked in the opposite sense from
    classic computer terminology, in which masked meant disabled. ARM uses the word
    masked to mean enabled. To avoid confusion we will not use the word masked. */

#define SSP_SSP1IMSC                              (*(pREG32 (0x40058014)))    // Interrupt mask set/clear register
#define SSP_SSP1IMSC_RORIM_MASK                   ((unsigned int) 0x00000001) // Receive overrun interrupt
#define SSP_SSP1IMSC_RORIM_ENBL                   ((unsigned int) 0x00000001)
#define SSP_SSP1IMSC_RORIM_DSBL                   ((unsigned int) 0x00000000)
#define SSP_SSP1IMSC_RTIM_MASK                    ((unsigned int) 0x00000002) // Receive timeout interrupt
#define SSP_SSP1IMSC_RTIM_ENBL                    ((unsigned int) 0x00000002)
#define SSP_SSP1IMSC_RTIM_DSBL                    ((unsigned int) 0x00000000)
#define SSP_SSP1IMSC_RXIM_MASK                    ((unsigned int) 0x00000004) // Rx FIFO >= 1/2 full interrupt
#define SSP_SSP1IMSC_RXIM_ENBL                    ((unsigned int) 0x00000004)
#define SSP_SSP1IMSC_RXIM_DSBL                    ((unsigned int) 0x00000000)
#define SSP_SSP1IMSC_TXIM_MASK                    ((unsigned int) 0x00000008) // Tx FIFO >= 1/2 empty interrupt
#define SSP_SSP1IMSC_TXIM_ENBL                    ((unsigned int) 0x00000008)
#define SSP_SSP1IMSC_TXIM_DSBL                    ((unsigned int) 0x00000000)

/*  SSP1RIS (SSP1 Raw Interrupt Status Register)
    This read-only register contains a 1 for each interrupt condition that is asserted,
    regardless of whether or not the interrupt is enabled in the SSP1IMSC.  */

#define SSP_SSP1RIS                               (*(pREG32 (0x40058018)))    // Raw interrupt status register
#define SSP_SSP1RIS_RORRIS_MASK                   ((unsigned int) 0x00000001) // Frame received while Rx FIFO full
#define SSP_SSP1RIS_RORRIS_RCVD                   ((unsigned int) 0x00000001)
#define SSP_SSP1RIS_RTRIS_MASK                    ((unsigned int) 0x00000002) // Rx FIFO not empty no read within timeout
#define SSP_SSP1RIS_RTRIS_NOTEMPTY                ((unsigned int) 0x00000002)
#define SSP_SSP1RIS_RXRIS_MASK                    ((unsigned int) 0x00000004) // Rx FIFO >= half full
#define SSP_SSP1RIS_RXRIS_HALFFULL                ((unsigned int) 0x00000004)
#define SSP_SSP1RIS_TXRIS_MASK                    ((unsigned int) 0x00000008) // Tx FIF0 >= half-empty
#define SSP_SSP1RIS_TXRIS_HALFEMPTY               ((unsigned int) 0x00000008)

/*  SSP1MIS (SSP1 Masked Interrupt Status Register)
    This read-only register contains a 1 for each interrupt condition that is asserted and
    enabled in the SSP1IMSC. When an SSP interrupt occurs, the interrupt service routine
    should read this register to determine the cause(s) of the interrupt. */

#define SSP_SSP1MIS                               (*(pREG32 (0x4005801C)))    // Masked interrupt status register
#define SSP_SSP1MIS_RORMIS_MASK                   ((unsigned int) 0x00000001) // Frame received while Rx FIFO full
#define SSP_SSP1MIS_RORMIS_FRMRCVD                ((unsigned int) 0x00000001)
#define SSP_SSP1MIS_RTMIS_MASK                    ((unsigned int) 0x00000002) // Rx FIFO not empty no read withing timeout
#define SSP_SSP1MIS_RTMIS_NOTEMPTY                ((unsigned int) 0x00000002)
#define SSP_SSP1MIS_RXMIS_MASK                    ((unsigned int) 0x00000004) // Rx FIFO >= half full
#define SSP_SSP1MIS_RXMIS_HALFFULL                ((unsigned int) 0x00000004)
#define SSP_SSP1MIS_TXMIS_MASK                    ((unsigned int) 0x00000008) // Tx FIFO >= half-empty
#define SSP_SSP1MIS_TXMIS_HALFEMPTY               ((unsigned int) 0x00000008)

/*  SSP1ICR (SSP1 Interrupt Clear Register)
    Software can write one or more one(s) to this write-only register, to clear the
    corresponding interrupt condition(s) in the SSP controller. Note that the other two interrupt
    conditions can be cleared by writing or reading the appropriate FIFO, or disabled by
    clearing the corresponding bit in SSP1IMSC. */

#define SSP_SSP1ICR                               (*(pREG32 (0x40058020)))    // SSPICR interrupt clear register
#define SSP_SSP1ICR_RORIC_MASK                    ((unsigned int) 0x00000001) // Clears RORIC interrupt flag
#define SSP_SSP1ICR_RORIC_CLEAR                   ((unsigned int) 0x00000001)
#define SSP_SSP1ICR_RTIC_MASK                     ((unsigned int) 0x00000002) // Clear Rx FIFO not empty/no read flag
#define SSP_SSP1ICR_RTIC_CLEAR                    ((unsigned int) 0x00000002)

/*##############################################################################
## 16-Bit Timers (CT16B0/1)
##############################################################################*/

#define TMR_CT16B0_BASE_ADDRESS                   (0x4000C000)

/*  Interrupt register */
#define TMR_TMR16B0IR                             (*(pREG32 (0x4000C000)))
#define TMR_TMR16B0IR_MR0_MASK                    ((unsigned int) 0x00000001) // Interrupt flag for match channel 0
#define TMR_TMR16B0IR_MR0                         ((unsigned int) 0x00000001)
#define TMR_TMR16B0IR_MR1_MASK                    ((unsigned int) 0x00000002) // Interrupt flag for match channel 1
#define TMR_TMR16B0IR_MR1                         ((unsigned int) 0x00000002)
#define TMR_TMR16B0IR_MR2_MASK                    ((unsigned int) 0x00000004) // Interrupt flag for match channel 2
#define TMR_TMR16B0IR_MR2                         ((unsigned int) 0x00000004)
#define TMR_TMR16B0IR_MR3_MASK                    ((unsigned int) 0x00000008) // Interrupt flag for match channel 3
#define TMR_TMR16B0IR_MR3                         ((unsigned int) 0x00000008)
#define TMR_TMR16B0IR_CR0_MASK                    ((unsigned int) 0x00000010) // Interrupt flag for capture channel 0 event
#define TMR_TMR16B0IR_CR0                         ((unsigned int) 0x00000010)
#define TMR_TMR16B0IR_MASK_ALL                    ((unsigned int) 0x0000001F)

/*  Timer control register */
#define TMR_TMR16B0TCR                            (*(pREG32 (0x4000C004)))
#define TMR_TMR16B0TCR_COUNTERENABLE_MASK         ((unsigned int) 0x00000001) // Counter enable
#define TMR_TMR16B0TCR_COUNTERENABLE_ENABLED      ((unsigned int) 0x00000001)
#define TMR_TMR16B0TCR_COUNTERENABLE_DISABLED     ((unsigned int) 0x00000000)
#define TMR_TMR16B0TCR_COUNTERRESET_MASK          ((unsigned int) 0x00000002)
#define TMR_TMR16B0TCR_COUNTERRESET_ENABLED       ((unsigned int) 0x00000002)
#define TMR_TMR16B0TCR_COUNTERRESET_DISABLED      ((unsigned int) 0x00000002)

/*  Timer counter */
#define TMR_TMR16B0TC                             (*(pREG32 (0x4000C008)))

/*  Prescale register */
#define TMR_TMR16B0PR                             (*(pREG32 (0x4000C00C)))

/*  Prescale counter register */
#define TMR_TMR16B0PC                             (*(pREG32 (0x4000C010)))

/*  Match control register */
#define TMR_TMR16B0MCR                            (*(pREG32 (0x4000C014)))
#define TMR_TMR16B0MR0                            (*(pREG32 (0x4000C018)))    // Match register 0
#define TMR_TMR16B0MR1                            (*(pREG32 (0x4000C01C)))    // Match register 1
#define TMR_TMR16B0MR2                            (*(pREG32 (0x4000C020)))    // Match register 2
#define TMR_TMR16B0MR3                            (*(pREG32 (0x4000C024)))    // Match register 3
#define TMR_TMR16B0MCR_MR0_INT_MASK               ((unsigned int) 0x00000001) // Interrupt on MRO
#define TMR_TMR16B0MCR_MR0_INT_ENABLED            ((unsigned int) 0x00000001)
#define TMR_TMR16B0MCR_MR0_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR0_RESET_MASK             ((unsigned int) 0x00000002) // Reset on MR0
#define TMR_TMR16B0MCR_MR0_RESET_ENABLED          ((unsigned int) 0x00000002)
#define TMR_TMR16B0MCR_MR0_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR0_STOP_MASK              ((unsigned int) 0x00000004) // Stop on MR0
#define TMR_TMR16B0MCR_MR0_STOP_ENABLED           ((unsigned int) 0x00000004)
#define TMR_TMR16B0MCR_MR0_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR1_INT_MASK               ((unsigned int) 0x00000008) // Interrupt on MR1
#define TMR_TMR16B0MCR_MR1_INT_ENABLED            ((unsigned int) 0x00000008)
#define TMR_TMR16B0MCR_MR1_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR1_RESET_MASK             ((unsigned int) 0x00000010) // Reset on MR1
#define TMR_TMR16B0MCR_MR1_RESET_ENABLED          ((unsigned int) 0x00000010)
#define TMR_TMR16B0MCR_MR1_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR1_STOP_MASK              ((unsigned int) 0x00000020) // Stop on MR1
#define TMR_TMR16B0MCR_MR1_STOP_ENABLED           ((unsigned int) 0x00000020)
#define TMR_TMR16B0MCR_MR1_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR2_INT_MASK               ((unsigned int) 0x00000040) // Interrupt on MR2
#define TMR_TMR16B0MCR_MR2_INT_ENABLED            ((unsigned int) 0x00000040)
#define TMR_TMR16B0MCR_MR2_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR2_RESET_MASK             ((unsigned int) 0x00000080) // Reset on MR2
#define TMR_TMR16B0MCR_MR2_RESET_ENABLED          ((unsigned int) 0x00000080)
#define TMR_TMR16B0MCR_MR2_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR2_STOP_MASK              ((unsigned int) 0x00000100) // Stop on MR2
#define TMR_TMR16B0MCR_MR2_STOP_ENABLED           ((unsigned int) 0x00000100)
#define TMR_TMR16B0MCR_MR2_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR3_INT_MASK               ((unsigned int) 0x00000200) // Interrupt on MR3
#define TMR_TMR16B0MCR_MR3_INT_ENABLED            ((unsigned int) 0x00000200)
#define TMR_TMR16B0MCR_MR3_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR3_RESET_MASK             ((unsigned int) 0x00000400) // Reset on MR3
#define TMR_TMR16B0MCR_MR3_RESET_ENABLED          ((unsigned int) 0x00000400)
#define TMR_TMR16B0MCR_MR3_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B0MCR_MR3_STOP_MASK              ((unsigned int) 0x00000800) // Stop on MR3
#define TMR_TMR16B0MCR_MR3_STOP_ENABLED           ((unsigned int) 0x00000800)
#define TMR_TMR16B0MCR_MR3_STOP_DISABLED          ((unsigned int) 0x00000000)

/*  Capture control register */
#define TMR_TMR16B0CCR                            (*(pREG32 (0x4000C028)))
#define TMR_TMR16B0CCR_CAP0RE_MASK                ((unsigned int) 0x00000001) // Capture on rising edge
#define TMR_TMR16B0CCR_CAP0RE_ENABLED             ((unsigned int) 0x00000001)
#define TMR_TMR16B0CCR_CAP0RE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR16B0CCR_CAP0FE_MASK                ((unsigned int) 0x00000002) // Capture on falling edge
#define TMR_TMR16B0CCR_CAP0FE_ENABLED             ((unsigned int) 0x00000002)
#define TMR_TMR16B0CCR_CAP0FE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR16B0CCR_CAP0I_MASK                 ((unsigned int) 0x00000004) // Interrupt on CAP0 event
#define TMR_TMR16B0CCR_CAP0I_ENABLED              ((unsigned int) 0x00000004)
#define TMR_TMR16B0CCR_CAP0I_DISABLED             ((unsigned int) 0x00000000)

/*  Capture register */
#define TMR_TMR16B0CR0                            (*(pREG32 (0x4000C02C)))

/*  External match register */
#define TMR_TMR16B0EMR                            (*(pREG32 (0x4000C03C)))
#define TMR_TMR16B0EMR_EM0_MASK                   ((unsigned int) 0x00000001) // External match 0
#define TMR_TMR16B0EMR_EM0                        ((unsigned int) 0x00000001)
#define TMR_TMR16B0EMR_EMC0_MASK                  ((unsigned int) 0x00000030)
#define TMR_TMR16B0EMR_EMC0_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B0EMR_EMC0_LOW                   ((unsigned int) 0x00000010)
#define TMR_TMR16B0EMR_EMC0_HIGH                  ((unsigned int) 0x00000020)
#define TMR_TMR16B0EMR_EMC0_TOGGLE                ((unsigned int) 0x00000030)
#define TMR_TMR16B0EMR_EM1_MASK                   ((unsigned int) 0x00000002) // External match 1
#define TMR_TMR16B0EMR_EM1                        ((unsigned int) 0x00000002)
#define TMR_TMR16B0EMR_EMC1_MASK                  ((unsigned int) 0x000000C0)
#define TMR_TMR16B0EMR_EMC1_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B0EMR_EMC1_LOW                   ((unsigned int) 0x00000040)
#define TMR_TMR16B0EMR_EMC1_HIGH                  ((unsigned int) 0x00000080)
#define TMR_TMR16B0EMR_EMC1_TOGGLE                ((unsigned int) 0x000000C0)
#define TMR_TMR16B0EMR_EM2_MASK                   ((unsigned int) 0x00000004) // External match 2
#define TMR_TMR16B0EMR_EM2                        ((unsigned int) 0x00000004)
#define TMR_TMR16B0EMR_EMC2_MASK                  ((unsigned int) 0x00000300)
#define TMR_TMR16B0EMR_EMC2_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B0EMR_EMC2_LOW                   ((unsigned int) 0x00000100)
#define TMR_TMR16B0EMR_EMC2_HIGH                  ((unsigned int) 0x00000200)
#define TMR_TMR16B0EMR_EMC2_TOGGLE                ((unsigned int) 0x00000300)
#define TMR_TMR16B0EMR_EM3_MASK                   ((unsigned int) 0x00000008) // External match 3
#define TMR_TMR16B0EMR_EM3                        ((unsigned int) 0x00000008)
#define TMR_TMR16B0EMR_EMC3_MASK                  ((unsigned int) 0x00000C00)
#define TMR_TMR16B0EMR_EMC3_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B0EMR_EMC3_LOW                   ((unsigned int) 0x00000400)
#define TMR_TMR16B0EMR_EMC3_HIGH                  ((unsigned int) 0x00000800)
#define TMR_TMR16B0EMR_EMC3_TOGGLE                ((unsigned int) 0x00000C00)

/*  Count control register */
#define TMR_TMR16B0CTCR                           (*(pREG32 (0x4000C070)))
#define TMR_TMR16B0CTCR_CTMODE_MASK               ((unsigned int) 0x00000003) // Counter/Timer mode
#define TMR_TMR16B0CTCR_CTMODE_TIMER              ((unsigned int) 0x00000000) // Timer Mode: Every rising PCLK edge
#define TMR_TMR16B0CTCR_CTMODE_COUNTERRISING      ((unsigned int) 0x00000001) // Counter: TC increments on rising edge of input
#define TMR_TMR16B0CTCR_CTMODE_COUNTERFALLING     ((unsigned int) 0x00000002) // Counter: TC increments on falling edge of input
#define TMR_TMR16B0CTCR_CTMODE_COUNTERBOTH        ((unsigned int) 0x00000003) // Counter: TC increments on both edges of input
#define TMR_TMR16B0CTCR_CINPUTSELECT_MASK         ((unsigned int) 0x0000000C)   
#define TMR_TMR16B0CTCR_CINPUTSELECT              ((unsigned int) 0x00000000) // CINPUTSELECT must be set to 00

/*  PWM control register */
#define TMR_TMR16B0PWMC                           (*(pREG32 (0x4000C074)))
#define TMR_TMR16B0PWMC_PWM0_MASK                 ((unsigned int) 0x00000001)   
#define TMR_TMR16B0PWMC_PWM0_ENABLED              ((unsigned int) 0x00000001) // PWM mode is enabled for CT16Bn_MAT0
#define TMR_TMR16B0PWMC_PWM0_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR16B0PWMC_PWM1_MASK                 ((unsigned int) 0x00000002)   
#define TMR_TMR16B0PWMC_PWM1_ENABLED              ((unsigned int) 0x00000002) // PWM mode is enabled for CT16Bn_MAT1
#define TMR_TMR16B0PWMC_PWM1_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR16B0PWMC_PWM2_MASK                 ((unsigned int) 0x00000004)   
#define TMR_TMR16B0PWMC_PWM2_ENABLED              ((unsigned int) 0x00000004) // PWM mode is enabled for CT16Bn_MAT2
#define TMR_TMR16B0PWMC_PWM2_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR16B0PWMC_PWM3_MASK                 ((unsigned int) 0x00000008)   
#define TMR_TMR16B0PWMC_PWM3_ENABLED              ((unsigned int) 0x00000008)
#define TMR_TMR16B0PWMC_PWM3_DISABLED             ((unsigned int) 0x00000000)

#define TMR_CT16B1_BASE_ADDRESS                   (0x40010000)

/*  Interrupt register */
#define TMR_TMR16B1IR                             (*(pREG32 (0x40010000)))
#define TMR_TMR16B1IR_MR0_MASK                    ((unsigned int) 0x00000001) // Interrupt flag for match channel 0
#define TMR_TMR16B1IR_MR0                         ((unsigned int) 0x00000001)
#define TMR_TMR16B1IR_MR1_MASK                    ((unsigned int) 0x00000002) // Interrupt flag for match channel 1
#define TMR_TMR16B1IR_MR1                         ((unsigned int) 0x00000002)
#define TMR_TMR16B1IR_MR2_MASK                    ((unsigned int) 0x00000004) // Interrupt flag for match channel 2
#define TMR_TMR16B1IR_MR2                         ((unsigned int) 0x00000004)
#define TMR_TMR16B1IR_MR3_MASK                    ((unsigned int) 0x00000008) // Interrupt flag for match channel 3
#define TMR_TMR16B1IR_MR3                         ((unsigned int) 0x00000008)
#define TMR_TMR16B1IR_CR0_MASK                    ((unsigned int) 0x00000010) // Interrupt flag for capture channel 0 event
#define TMR_TMR16B1IR_CR0                         ((unsigned int) 0x00000010)
#define TMR_TMR16B1IR_MASK_ALL                    ((unsigned int) 0x0000001F)

/*  Timer control register */
#define TMR_TMR16B1TCR                            (*(pREG32 (0x40010004)))
#define TMR_TMR16B1TCR_COUNTERENABLE_MASK         ((unsigned int) 0x00000001) // Counter enable
#define TMR_TMR16B1TCR_COUNTERENABLE_ENABLED      ((unsigned int) 0x00000001)
#define TMR_TMR16B1TCR_COUNTERENABLE_DISABLED     ((unsigned int) 0x00000000)
#define TMR_TMR16B1TCR_COUNTERRESET_MASK          ((unsigned int) 0x00000002)
#define TMR_TMR16B1TCR_COUNTERRESET_ENABLED       ((unsigned int) 0x00000002)
#define TMR_TMR16B1TCR_COUNTERRESET_DISABLED      ((unsigned int) 0x00000002)

/*  Timer counter */
#define TMR_TMR16B1TC                             (*(pREG32 (0x40010008)))

/*  Prescale register */
#define TMR_TMR16B1PR                             (*(pREG32 (0x4001000C)))

/*  Prescale counter register */
#define TMR_TMR16B1PC                             (*(pREG32 (0x40010010)))

/*  Match control register */
#define TMR_TMR16B1MCR                            (*(pREG32 (0x40010014)))
#define TMR_TMR16B1MR0                            (*(pREG32 (0x40010018)))    // Match register 0
#define TMR_TMR16B1MR1                            (*(pREG32 (0x4001001C)))    // Match register 1
#define TMR_TMR16B1MR2                            (*(pREG32 (0x40010020)))    // Match register 2
#define TMR_TMR16B1MR3                            (*(pREG32 (0x40010024)))    // Match register 3
#define TMR_TMR16B1MCR_MR0_INT_MASK               ((unsigned int) 0x00000001) // Interrupt on MRO
#define TMR_TMR16B1MCR_MR0_INT_ENABLED            ((unsigned int) 0x00000001)
#define TMR_TMR16B1MCR_MR0_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR0_RESET_MASK             ((unsigned int) 0x00000002) // Reset on MR0
#define TMR_TMR16B1MCR_MR0_RESET_ENABLED          ((unsigned int) 0x00000002)
#define TMR_TMR16B1MCR_MR0_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR0_STOP_MASK              ((unsigned int) 0x00000004) // Stop on MR0
#define TMR_TMR16B1MCR_MR0_STOP_ENABLED           ((unsigned int) 0x00000004)
#define TMR_TMR16B1MCR_MR0_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR1_INT_MASK               ((unsigned int) 0x00000008) // Interrupt on MR1
#define TMR_TMR16B1MCR_MR1_INT_ENABLED            ((unsigned int) 0x00000008)
#define TMR_TMR16B1MCR_MR1_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR1_RESET_MASK             ((unsigned int) 0x00000010) // Reset on MR1
#define TMR_TMR16B1MCR_MR1_RESET_ENABLED          ((unsigned int) 0x00000010)
#define TMR_TMR16B1MCR_MR1_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR1_STOP_MASK              ((unsigned int) 0x00000020) // Stop on MR1
#define TMR_TMR16B1MCR_MR1_STOP_ENABLED           ((unsigned int) 0x00000020)
#define TMR_TMR16B1MCR_MR1_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR2_INT_MASK               ((unsigned int) 0x00000040) // Interrupt on MR2
#define TMR_TMR16B1MCR_MR2_INT_ENABLED            ((unsigned int) 0x00000040)
#define TMR_TMR16B1MCR_MR2_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR2_RESET_MASK             ((unsigned int) 0x00000080) // Reset on MR2
#define TMR_TMR16B1MCR_MR2_RESET_ENABLED          ((unsigned int) 0x00000080)
#define TMR_TMR16B1MCR_MR2_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR2_STOP_MASK              ((unsigned int) 0x00000100) // Stop on MR2
#define TMR_TMR16B1MCR_MR2_STOP_ENABLED           ((unsigned int) 0x00000100)
#define TMR_TMR16B1MCR_MR2_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR3_INT_MASK               ((unsigned int) 0x00000200) // Interrupt on MR3
#define TMR_TMR16B1MCR_MR3_INT_ENABLED            ((unsigned int) 0x00000200)
#define TMR_TMR16B1MCR_MR3_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR3_RESET_MASK             ((unsigned int) 0x00000400) // Reset on MR3
#define TMR_TMR16B1MCR_MR3_RESET_ENABLED          ((unsigned int) 0x00000400)
#define TMR_TMR16B1MCR_MR3_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR16B1MCR_MR3_STOP_MASK              ((unsigned int) 0x00000800) // Stop on MR3
#define TMR_TMR16B1MCR_MR3_STOP_ENABLED           ((unsigned int) 0x00000800)
#define TMR_TMR16B1MCR_MR3_STOP_DISABLED          ((unsigned int) 0x00000000)

/*  Capture control register */
#define TMR_TMR16B1CCR                            (*(pREG32 (0x40010028)))
#define TMR_TMR16B1CCR_CAP0RE_MASK                ((unsigned int) 0x00000001) // Capture on rising edge
#define TMR_TMR16B1CCR_CAP0RE_ENABLED             ((unsigned int) 0x00000001)
#define TMR_TMR16B1CCR_CAP0RE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR16B1CCR_CAP0FE_MASK                ((unsigned int) 0x00000002) // Capture on falling edge
#define TMR_TMR16B1CCR_CAP0FE_ENABLED             ((unsigned int) 0x00000002)
#define TMR_TMR16B1CCR_CAP0FE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR16B1CCR_CAP0I_MASK                 ((unsigned int) 0x00000004) // Interrupt on CAP0 event
#define TMR_TMR16B1CCR_CAP0I_ENABLED              ((unsigned int) 0x00000004)
#define TMR_TMR16B1CCR_CAP0I_DISABLED             ((unsigned int) 0x00000000)

/*  Capture register */
#define TMR_TMR16B1CR0                            (*(pREG32 (0x4001002C)))

/*  External match register */
#define TMR_TMR16B1EMR                            (*(pREG32 (0x4001003C)))
#define TMR_TMR16B1EMR_EM0_MASK                   ((unsigned int) 0x00000001) // External match 0
#define TMR_TMR16B1EMR_EM0                        ((unsigned int) 0x00000001)
#define TMR_TMR16B1EMR_EMC0_MASK                  ((unsigned int) 0x00000030)
#define TMR_TMR16B1EMR_EMC0_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B1EMR_EMC0_LOW                   ((unsigned int) 0x00000010)
#define TMR_TMR16B1EMR_EMC0_HIGH                  ((unsigned int) 0x00000020)
#define TMR_TMR16B1EMR_EMC0_TOGGLE                ((unsigned int) 0x00000030)
#define TMR_TMR16B1EMR_EM1_MASK                   ((unsigned int) 0x00000002) // External match 1
#define TMR_TMR16B1EMR_EM1                        ((unsigned int) 0x00000002)
#define TMR_TMR16B1EMR_EMC1_MASK                  ((unsigned int) 0x000000C0)
#define TMR_TMR16B1EMR_EMC1_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B1EMR_EMC1_LOW                   ((unsigned int) 0x00000040)
#define TMR_TMR16B1EMR_EMC1_HIGH                  ((unsigned int) 0x00000080)
#define TMR_TMR16B1EMR_EMC1_TOGGLE                ((unsigned int) 0x000000C0)
#define TMR_TMR16B1EMR_EM2_MASK                   ((unsigned int) 0x00000004) // External match 2
#define TMR_TMR16B1EMR_EM2                        ((unsigned int) 0x00000004)
#define TMR_TMR16B1EMR_EMC2_MASK                  ((unsigned int) 0x00000300)
#define TMR_TMR16B1EMR_EMC2_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B1EMR_EMC2_LOW                   ((unsigned int) 0x00000100)
#define TMR_TMR16B1EMR_EMC2_HIGH                  ((unsigned int) 0x00000200)
#define TMR_TMR16B1EMR_EMC2_TOGGLE                ((unsigned int) 0x00000300)
#define TMR_TMR16B1EMR_EM3_MASK                   ((unsigned int) 0x00000008) // External match 3
#define TMR_TMR16B1EMR_EM3                        ((unsigned int) 0x00000008)
#define TMR_TMR16B1EMR_EMC3_MASK                  ((unsigned int) 0x00000C00)
#define TMR_TMR16B1EMR_EMC3_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR16B1EMR_EMC3_LOW                   ((unsigned int) 0x00000400)
#define TMR_TMR16B1EMR_EMC3_HIGH                  ((unsigned int) 0x00000800)
#define TMR_TMR16B1EMR_EMC3_TOGGLE                ((unsigned int) 0x00000C00)

/*  Count control register */
#define TMR_TMR16B1CTCR                           (*(pREG32 (0x40010070)))
#define TMR_TMR16B1CTCR_CTMODE_MASK               ((unsigned int) 0x00000003) // Counter/Timer mode
#define TMR_TMR16B1CTCR_CTMODE_TIMER              ((unsigned int) 0x00000000) // Timer Mode: Every rising PCLK edge
#define TMR_TMR16B1CTCR_CTMODE_COUNTERRISING      ((unsigned int) 0x00000001) // Counter: TC increments on rising edge of input
#define TMR_TMR16B1CTCR_CTMODE_COUNTERFALLING     ((unsigned int) 0x00000002) // Counter: TC increments on falling edge of input
#define TMR_TMR16B1CTCR_CTMODE_COUNTERBOTH        ((unsigned int) 0x00000003) // Counter: TC increments on both edges of input
#define TMR_TMR16B1CTCR_CINPUTSELECT_MASK         ((unsigned int) 0x0000000C)   
#define TMR_TMR16B1CTCR_CINPUTSELECT              ((unsigned int) 0x00000000) // CINPUTSELECT must be set to 00

/*  PWM control register */
#define TMR_TMR16B1PWMC                           (*(pREG32 (0x40010074)))
#define TMR_TMR16B1PWMC_PWM0_MASK                 ((unsigned int) 0x00000001)   
#define TMR_TMR16B1PWMC_PWM0_ENABLED              ((unsigned int) 0x00000001) // PWM mode is enabled for CT16Bn_MAT0
#define TMR_TMR16B1PWMC_PWM0_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR16B1PWMC_PWM1_MASK                 ((unsigned int) 0x00000002)   
#define TMR_TMR16B1PWMC_PWM1_ENABLED              ((unsigned int) 0x00000002) // PWM mode is enabled for CT16Bn_MAT1
#define TMR_TMR16B1PWMC_PWM1_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR16B1PWMC_PWM2_MASK                 ((unsigned int) 0x00000004)   
#define TMR_TMR16B1PWMC_PWM2_ENABLED              ((unsigned int) 0x00000004) // PWM mode is enabled for CT16Bn_MAT2
#define TMR_TMR16B1PWMC_PWM2_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR16B1PWMC_PWM3_MASK                 ((unsigned int) 0x00000008)   
#define TMR_TMR16B1PWMC_PWM3_ENABLED              ((unsigned int) 0x00000008)
#define TMR_TMR16B1PWMC_PWM3_DISABLED             ((unsigned int) 0x00000000)

/*##############################################################################
## 32-Bit Timers (CT32B0/1)
##############################################################################*/

#define TMR_CT32B0_BASE_ADDRESS                   (0x40014000)

/*  Interrupt register */
#define TMR_TMR32B0IR                             (*(pREG32 (0x40014000)))
#define TMR_TMR32B0IR_MR0_MASK                    ((unsigned int) 0x00000001) // Interrupt flag for match channel 0
#define TMR_TMR32B0IR_MR0                         ((unsigned int) 0x00000001)
#define TMR_TMR32B0IR_MR1_MASK                    ((unsigned int) 0x00000002) // Interrupt flag for match channel 1
#define TMR_TMR32B0IR_MR1                         ((unsigned int) 0x00000002)
#define TMR_TMR32B0IR_MR2_MASK                    ((unsigned int) 0x00000004) // Interrupt flag for match channel 2
#define TMR_TMR32B0IR_MR2                         ((unsigned int) 0x00000004)
#define TMR_TMR32B0IR_MR3_MASK                    ((unsigned int) 0x00000008) // Interrupt flag for match channel 3
#define TMR_TMR32B0IR_MR3                         ((unsigned int) 0x00000008)
#define TMR_TMR32B0IR_CR0_MASK                    ((unsigned int) 0x00000010) // Interrupt flag for capture channel 0 event
#define TMR_TMR32B0IR_CR0                         ((unsigned int) 0x00000010)
#define TMR_TMR32B0IR_MASK_ALL                    ((unsigned int) 0x0000001F)

/*  Timer control register */
#define TMR_TMR32B0TCR                            (*(pREG32 (0x40014004)))
#define TMR_TMR32B0TCR_COUNTERENABLE_MASK         ((unsigned int) 0x00000001) // Counter enable
#define TMR_TMR32B0TCR_COUNTERENABLE_ENABLED      ((unsigned int) 0x00000001)
#define TMR_TMR32B0TCR_COUNTERENABLE_DISABLED     ((unsigned int) 0x00000000)
#define TMR_TMR32B0TCR_COUNTERRESET_MASK          ((unsigned int) 0x00000002)
#define TMR_TMR32B0TCR_COUNTERRESET_ENABLED       ((unsigned int) 0x00000002)
#define TMR_TMR32B0TCR_COUNTERRESET_DISABLED      ((unsigned int) 0x00000002)

/*  Timer counter */
#define TMR_TMR32B0TC                             (*(pREG32 (0x40014008)))

/*  Prescale register */
#define TMR_TMR32B0PR                             (*(pREG32 (0x4001400C)))

/*  Prescale counter register */
#define TMR_TMR32B0PC                             (*(pREG32 (0x40014010)))

/*  Match control register */
#define TMR_TMR32B0MCR                            (*(pREG32 (0x40014014)))
#define TMR_TMR32B0MR0                            (*(pREG32 (0x40014018)))    // Match register 0
#define TMR_TMR32B0MR1                            (*(pREG32 (0x4001401C)))    // Match register 1
#define TMR_TMR32B0MR2                            (*(pREG32 (0x40014020)))    // Match register 2
#define TMR_TMR32B0MR3                            (*(pREG32 (0x40014024)))    // Match register 3
#define TMR_TMR32B0MCR_MR0_INT_MASK               ((unsigned int) 0x00000001) // Interrupt on MRO
#define TMR_TMR32B0MCR_MR0_INT_ENABLED            ((unsigned int) 0x00000001)
#define TMR_TMR32B0MCR_MR0_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR0_RESET_MASK             ((unsigned int) 0x00000002) // Reset on MR0
#define TMR_TMR32B0MCR_MR0_RESET_ENABLED          ((unsigned int) 0x00000002)
#define TMR_TMR32B0MCR_MR0_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR0_STOP_MASK              ((unsigned int) 0x00000004) // Stop on MR0
#define TMR_TMR32B0MCR_MR0_STOP_ENABLED           ((unsigned int) 0x00000004)
#define TMR_TMR32B0MCR_MR0_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR1_INT_MASK               ((unsigned int) 0x00000008) // Interrupt on MR1
#define TMR_TMR32B0MCR_MR1_INT_ENABLED            ((unsigned int) 0x00000008)
#define TMR_TMR32B0MCR_MR1_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR1_RESET_MASK             ((unsigned int) 0x00000010) // Reset on MR1
#define TMR_TMR32B0MCR_MR1_RESET_ENABLED          ((unsigned int) 0x00000010)
#define TMR_TMR32B0MCR_MR1_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR1_STOP_MASK              ((unsigned int) 0x00000020) // Stop on MR1
#define TMR_TMR32B0MCR_MR1_STOP_ENABLED           ((unsigned int) 0x00000020)
#define TMR_TMR32B0MCR_MR1_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR2_INT_MASK               ((unsigned int) 0x00000040) // Interrupt on MR2
#define TMR_TMR32B0MCR_MR2_INT_ENABLED            ((unsigned int) 0x00000040)
#define TMR_TMR32B0MCR_MR2_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR2_RESET_MASK             ((unsigned int) 0x00000080) // Reset on MR2
#define TMR_TMR32B0MCR_MR2_RESET_ENABLED          ((unsigned int) 0x00000080)
#define TMR_TMR32B0MCR_MR2_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR2_STOP_MASK              ((unsigned int) 0x00000100) // Stop on MR2
#define TMR_TMR32B0MCR_MR2_STOP_ENABLED           ((unsigned int) 0x00000100)
#define TMR_TMR32B0MCR_MR2_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR3_INT_MASK               ((unsigned int) 0x00000200) // Interrupt on MR3
#define TMR_TMR32B0MCR_MR3_INT_ENABLED            ((unsigned int) 0x00000200)
#define TMR_TMR32B0MCR_MR3_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR3_RESET_MASK             ((unsigned int) 0x00000400) // Reset on MR3
#define TMR_TMR32B0MCR_MR3_RESET_ENABLED          ((unsigned int) 0x00000400)
#define TMR_TMR32B0MCR_MR3_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B0MCR_MR3_STOP_MASK              ((unsigned int) 0x00000800) // Stop on MR3
#define TMR_TMR32B0MCR_MR3_STOP_ENABLED           ((unsigned int) 0x00000800)
#define TMR_TMR32B0MCR_MR3_STOP_DISABLED          ((unsigned int) 0x00000000)

/*  Capture control register */
#define TMR_TMR32B0CCR                            (*(pREG32 (0x40014028)))
#define TMR_TMR32B0CCR_CAP0RE_MASK                ((unsigned int) 0x00000001) // Capture on rising edge
#define TMR_TMR32B0CCR_CAP0RE_ENABLED             ((unsigned int) 0x00000001)
#define TMR_TMR32B0CCR_CAP0RE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR32B0CCR_CAP0FE_MASK                ((unsigned int) 0x00000002) // Capture on falling edge
#define TMR_TMR32B0CCR_CAP0FE_ENABLED             ((unsigned int) 0x00000002)
#define TMR_TMR32B0CCR_CAP0FE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR32B0CCR_CAP0I_MASK                 ((unsigned int) 0x00000004) // Interrupt on CAP0 event
#define TMR_TMR32B0CCR_CAP0I_ENABLED              ((unsigned int) 0x00000004)
#define TMR_TMR32B0CCR_CAP0I_DISABLED             ((unsigned int) 0x00000000)

/*  Capture register */
#define TMR_TMR32B0CR0                            (*(pREG32 (0x4001402C)))

/*  External match register */
#define TMR_TMR32B0EMR                            (*(pREG32 (0x4001403C)))
#define TMR_TMR32B0EMR_EM0_MASK                   ((unsigned int) 0x00000001) // External match 0
#define TMR_TMR32B0EMR_EM0                        ((unsigned int) 0x00000001)
#define TMR_TMR32B0EMR_EMC0_MASK                  ((unsigned int) 0x00000030)
#define TMR_TMR32B0EMR_EMC0_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B0EMR_EMC0_LOW                   ((unsigned int) 0x00000010)
#define TMR_TMR32B0EMR_EMC0_HIGH                  ((unsigned int) 0x00000020)
#define TMR_TMR32B0EMR_EMC0_TOGGLE                ((unsigned int) 0x00000030)
#define TMR_TMR32B0EMR_EM1_MASK                   ((unsigned int) 0x00000002) // External match 1
#define TMR_TMR32B0EMR_EM1                        ((unsigned int) 0x00000002)
#define TMR_TMR32B0EMR_EMC1_MASK                  ((unsigned int) 0x000000C0)
#define TMR_TMR32B0EMR_EMC1_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B0EMR_EMC1_LOW                   ((unsigned int) 0x00000040)
#define TMR_TMR32B0EMR_EMC1_HIGH                  ((unsigned int) 0x00000080)
#define TMR_TMR32B0EMR_EMC1_TOGGLE                ((unsigned int) 0x000000C0)
#define TMR_TMR32B0EMR_EM2_MASK                   ((unsigned int) 0x00000004) // External match 2
#define TMR_TMR32B0EMR_EM2                        ((unsigned int) 0x00000004)
#define TMR_TMR32B0EMR_EMC2_MASK                  ((unsigned int) 0x00000300)
#define TMR_TMR32B0EMR_EMC2_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B0EMR_EMC2_LOW                   ((unsigned int) 0x00000100)
#define TMR_TMR32B0EMR_EMC2_HIGH                  ((unsigned int) 0x00000200)
#define TMR_TMR32B0EMR_EMC2_TOGGLE                ((unsigned int) 0x00000300)
#define TMR_TMR32B0EMR_EM3_MASK                   ((unsigned int) 0x00000008) // External match 3
#define TMR_TMR32B0EMR_EM3                        ((unsigned int) 0x00000008)
#define TMR_TMR32B0EMR_EMC3_MASK                  ((unsigned int) 0x00000C00)
#define TMR_TMR32B0EMR_EMC3_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B0EMR_EMC3_LOW                   ((unsigned int) 0x00000400)
#define TMR_TMR32B0EMR_EMC3_HIGH                  ((unsigned int) 0x00000800)
#define TMR_TMR32B0EMR_EMC3_TOGGLE                ((unsigned int) 0x00000C00)

/*  Count control register */
#define TMR_TMR32B0CTCR                           (*(pREG32 (0x40014070)))
#define TMR_TMR32B0CTCR_CTMODE_MASK               ((unsigned int) 0x00000003) // Counter/Timer mode
#define TMR_TMR32B0CTCR_CTMODE_TIMER              ((unsigned int) 0x00000000) // Timer Mode: Every rising PCLK edge
#define TMR_TMR32B0CTCR_CTMODE_COUNTERRISING      ((unsigned int) 0x00000001) // Counter: TC increments on rising edge of input
#define TMR_TMR32B0CTCR_CTMODE_COUNTERFALLING     ((unsigned int) 0x00000002) // Counter: TC increments on falling edge of input
#define TMR_TMR32B0CTCR_CTMODE_COUNTERBOTH        ((unsigned int) 0x00000003) // Counter: TC increments on both edges of input
#define TMR_TMR32B0CTCR_CINPUTSELECT_MASK         ((unsigned int) 0x0000000C)   
#define TMR_TMR32B0CTCR_CINPUTSELECT              ((unsigned int) 0x00000000) // CINPUTSELECT must be set to 00

/*  PWM control register */
#define TMR_TMR32B0PWMC                           (*(pREG32 (0x40014074)))
#define TMR_TMR32B0PWMC_PWM0_MASK                 ((unsigned int) 0x00000001)   
#define TMR_TMR32B0PWMC_PWM0_ENABLED              ((unsigned int) 0x00000001) // PWM mode is enabled for CT32Bn_MAT0
#define TMR_TMR32B0PWMC_PWM0_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR32B0PWMC_PWM1_MASK                 ((unsigned int) 0x00000002)   
#define TMR_TMR32B0PWMC_PWM1_ENABLED              ((unsigned int) 0x00000002) // PWM mode is enabled for CT32Bn_MAT1
#define TMR_TMR32B0PWMC_PWM1_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR32B0PWMC_PWM2_MASK                 ((unsigned int) 0x00000004)   
#define TMR_TMR32B0PWMC_PWM2_ENABLED              ((unsigned int) 0x00000004) // PWM mode is enabled for CT32Bn_MAT2
#define TMR_TMR32B0PWMC_PWM2_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR32B0PWMC_PWM3_MASK                 ((unsigned int) 0x00000008)   
#define TMR_TMR32B0PWMC_PWM3_ENABLED              ((unsigned int) 0x00000008)
#define TMR_TMR32B0PWMC_PWM3_DISABLED             ((unsigned int) 0x00000000)

#define TMR_CT32B1_BASE_ADDRESS                   (0x40018000)

/*  Interrupt register */
#define TMR_TMR32B1IR                             (*(pREG32 (0x40018000)))
#define TMR_TMR32B1IR_MR0_MASK                    ((unsigned int) 0x00000001) // Interrupt flag for match channel 0
#define TMR_TMR32B1IR_MR0                         ((unsigned int) 0x00000001)
#define TMR_TMR32B1IR_MR1_MASK                    ((unsigned int) 0x00000002) // Interrupt flag for match channel 1
#define TMR_TMR32B1IR_MR1                         ((unsigned int) 0x00000002)
#define TMR_TMR32B1IR_MR2_MASK                    ((unsigned int) 0x00000004) // Interrupt flag for match channel 2
#define TMR_TMR32B1IR_MR2                         ((unsigned int) 0x00000004)
#define TMR_TMR32B1IR_MR3_MASK                    ((unsigned int) 0x00000008) // Interrupt flag for match channel 3
#define TMR_TMR32B1IR_MR3                         ((unsigned int) 0x00000008)
#define TMR_TMR32B1IR_CR0_MASK                    ((unsigned int) 0x00000010) // Interrupt flag for capture channel 0 event
#define TMR_TMR32B1IR_CR0                         ((unsigned int) 0x00000010)
#define TMR_TMR32B1IR_MASK_ALL                    ((unsigned int) 0x0000001F)

/*  Timer control register */
#define TMR_TMR32B1TCR                            (*(pREG32 (0x40018004)))
#define TMR_TMR32B1TCR_COUNTERENABLE_MASK         ((unsigned int) 0x00000001) // Counter enable
#define TMR_TMR32B1TCR_COUNTERENABLE_ENABLED      ((unsigned int) 0x00000001)
#define TMR_TMR32B1TCR_COUNTERENABLE_DISABLED     ((unsigned int) 0x00000000)
#define TMR_TMR32B1TCR_COUNTERRESET_MASK          ((unsigned int) 0x00000002)
#define TMR_TMR32B1TCR_COUNTERRESET_ENABLED       ((unsigned int) 0x00000002)
#define TMR_TMR32B1TCR_COUNTERRESET_DISABLED      ((unsigned int) 0x00000002)

/*  Timer counter */
#define TMR_TMR32B1TC                             (*(pREG32 (0x40018008)))

/*  Prescale register */
#define TMR_TMR32B1PR                             (*(pREG32 (0x4001800C)))

/*  Prescale counter register */
#define TMR_TMR32B1PC                             (*(pREG32 (0x40018010)))

/*  Match control register */
#define TMR_TMR32B1MCR                            (*(pREG32 (0x40018014)))
#define TMR_TMR32B1MR0                            (*(pREG32 (0x40018018)))    // Match register 0
#define TMR_TMR32B1MR1                            (*(pREG32 (0x4001801C)))    // Match register 1
#define TMR_TMR32B1MR2                            (*(pREG32 (0x40018020)))    // Match register 2
#define TMR_TMR32B1MR3                            (*(pREG32 (0x40018024)))    // Match register 3
#define TMR_TMR32B1MCR_MR0_INT_MASK               ((unsigned int) 0x00000001) // Interrupt on MRO
#define TMR_TMR32B1MCR_MR0_INT_ENABLED            ((unsigned int) 0x00000001)
#define TMR_TMR32B1MCR_MR0_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR0_RESET_MASK             ((unsigned int) 0x00000002) // Reset on MR0
#define TMR_TMR32B1MCR_MR0_RESET_ENABLED          ((unsigned int) 0x00000002)
#define TMR_TMR32B1MCR_MR0_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR0_STOP_MASK              ((unsigned int) 0x00000004) // Stop on MR0
#define TMR_TMR32B1MCR_MR0_STOP_ENABLED           ((unsigned int) 0x00000004)
#define TMR_TMR32B1MCR_MR0_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR1_INT_MASK               ((unsigned int) 0x00000008) // Interrupt on MR1
#define TMR_TMR32B1MCR_MR1_INT_ENABLED            ((unsigned int) 0x00000008)
#define TMR_TMR32B1MCR_MR1_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR1_RESET_MASK             ((unsigned int) 0x00000010) // Reset on MR1
#define TMR_TMR32B1MCR_MR1_RESET_ENABLED          ((unsigned int) 0x00000010)
#define TMR_TMR32B1MCR_MR1_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR1_STOP_MASK              ((unsigned int) 0x00000020) // Stop on MR1
#define TMR_TMR32B1MCR_MR1_STOP_ENABLED           ((unsigned int) 0x00000020)
#define TMR_TMR32B1MCR_MR1_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR2_INT_MASK               ((unsigned int) 0x00000040) // Interrupt on MR2
#define TMR_TMR32B1MCR_MR2_INT_ENABLED            ((unsigned int) 0x00000040)
#define TMR_TMR32B1MCR_MR2_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR2_RESET_MASK             ((unsigned int) 0x00000080) // Reset on MR2
#define TMR_TMR32B1MCR_MR2_RESET_ENABLED          ((unsigned int) 0x00000080)
#define TMR_TMR32B1MCR_MR2_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR2_STOP_MASK              ((unsigned int) 0x00000100) // Stop on MR2
#define TMR_TMR32B1MCR_MR2_STOP_ENABLED           ((unsigned int) 0x00000100)
#define TMR_TMR32B1MCR_MR2_STOP_DISABLED          ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR3_INT_MASK               ((unsigned int) 0x00000200) // Interrupt on MR3
#define TMR_TMR32B1MCR_MR3_INT_ENABLED            ((unsigned int) 0x00000200)
#define TMR_TMR32B1MCR_MR3_INT_DISABLED           ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR3_RESET_MASK             ((unsigned int) 0x00000400) // Reset on MR3
#define TMR_TMR32B1MCR_MR3_RESET_ENABLED          ((unsigned int) 0x00000400)
#define TMR_TMR32B1MCR_MR3_RESET_DISABLED         ((unsigned int) 0x00000000)
#define TMR_TMR32B1MCR_MR3_STOP_MASK              ((unsigned int) 0x00000800) // Stop on MR3
#define TMR_TMR32B1MCR_MR3_STOP_ENABLED           ((unsigned int) 0x00000800)
#define TMR_TMR32B1MCR_MR3_STOP_DISABLED          ((unsigned int) 0x00000000)

/*  Capture control register */
#define TMR_TMR32B1CCR                            (*(pREG32 (0x40018028)))
#define TMR_TMR32B1CCR_CAP0RE_MASK                ((unsigned int) 0x00000001) // Capture on rising edge
#define TMR_TMR32B1CCR_CAP0RE_ENABLED             ((unsigned int) 0x00000001)
#define TMR_TMR32B1CCR_CAP0RE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR32B1CCR_CAP0FE_MASK                ((unsigned int) 0x00000002) // Capture on falling edge
#define TMR_TMR32B1CCR_CAP0FE_ENABLED             ((unsigned int) 0x00000002)
#define TMR_TMR32B1CCR_CAP0FE_DISABLED            ((unsigned int) 0x00000000)
#define TMR_TMR32B1CCR_CAP0I_MASK                 ((unsigned int) 0x00000004) // Interrupt on CAP0 event
#define TMR_TMR32B1CCR_CAP0I_ENABLED              ((unsigned int) 0x00000004)
#define TMR_TMR32B1CCR_CAP0I_DISABLED             ((unsigned int) 0x00000000)

/*  Capture register */
#define TMR_TMR32B1CR0                            (*(pREG32 (0x4001802C)))

/*  External match register */
#define TMR_TMR32B1EMR                            (*(pREG32 (0x4001803C)))
#define TMR_TMR32B1EMR_EM0_MASK                   ((unsigned int) 0x00000001) // External match 0
#define TMR_TMR32B1EMR_EM0                        ((unsigned int) 0x00000001)
#define TMR_TMR32B1EMR_EMC0_MASK                  ((unsigned int) 0x00000030)
#define TMR_TMR32B1EMR_EMC0_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B1EMR_EMC0_LOW                   ((unsigned int) 0x00000010)
#define TMR_TMR32B1EMR_EMC0_HIGH                  ((unsigned int) 0x00000020)
#define TMR_TMR32B1EMR_EMC0_TOGGLE                ((unsigned int) 0x00000030)
#define TMR_TMR32B1EMR_EM1_MASK                   ((unsigned int) 0x00000002) // External match 1
#define TMR_TMR32B1EMR_EM1                        ((unsigned int) 0x00000002)
#define TMR_TMR32B1EMR_EMC1_MASK                  ((unsigned int) 0x000000C0)
#define TMR_TMR32B1EMR_EMC1_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B1EMR_EMC1_LOW                   ((unsigned int) 0x00000040)
#define TMR_TMR32B1EMR_EMC1_HIGH                  ((unsigned int) 0x00000080)
#define TMR_TMR32B1EMR_EMC1_TOGGLE                ((unsigned int) 0x000000C0)
#define TMR_TMR32B1EMR_EM2_MASK                   ((unsigned int) 0x00000004) // External match 2
#define TMR_TMR32B1EMR_EM2                        ((unsigned int) 0x00000004)
#define TMR_TMR32B1EMR_EMC2_MASK                  ((unsigned int) 0x00000300)
#define TMR_TMR32B1EMR_EMC2_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B1EMR_EMC2_LOW                   ((unsigned int) 0x00000100)
#define TMR_TMR32B1EMR_EMC2_HIGH                  ((unsigned int) 0x00000200)
#define TMR_TMR32B1EMR_EMC2_TOGGLE                ((unsigned int) 0x00000300)
#define TMR_TMR32B1EMR_EM3_MASK                   ((unsigned int) 0x00000008) // External match 3
#define TMR_TMR32B1EMR_EM3                        ((unsigned int) 0x00000008)
#define TMR_TMR32B1EMR_EMC3_MASK                  ((unsigned int) 0x00000C00)
#define TMR_TMR32B1EMR_EMC3_DONOTHING             ((unsigned int) 0x00000000)
#define TMR_TMR32B1EMR_EMC3_LOW                   ((unsigned int) 0x00000400)
#define TMR_TMR32B1EMR_EMC3_HIGH                  ((unsigned int) 0x00000800)
#define TMR_TMR32B1EMR_EMC3_TOGGLE                ((unsigned int) 0x00000C00)

/*  Count control register */
#define TMR_TMR32B1CTCR                           (*(pREG32 (0x40018070)))
#define TMR_TMR32B1CTCR_CTMODE_MASK               ((unsigned int) 0x00000003) // Counter/Timer mode
#define TMR_TMR32B1CTCR_CTMODE_TIMER              ((unsigned int) 0x00000000) // Timer Mode: Every rising PCLK edge
#define TMR_TMR32B1CTCR_CTMODE_COUNTERRISING      ((unsigned int) 0x00000001) // Counter: TC increments on rising edge of input
#define TMR_TMR32B1CTCR_CTMODE_COUNTERFALLING     ((unsigned int) 0x00000002) // Counter: TC increments on falling edge of input
#define TMR_TMR32B1CTCR_CTMODE_COUNTERBOTH        ((unsigned int) 0x00000003) // Counter: TC increments on both edges of input
#define TMR_TMR32B1CTCR_CINPUTSELECT_MASK         ((unsigned int) 0x0000000C)   
#define TMR_TMR32B1CTCR_CINPUTSELECT              ((unsigned int) 0x00000000) // CINPUTSELECT must be set to 00

/*  PWM control register */
#define TMR_TMR32B1PWMC                           (*(pREG32 (0x40018074)))
#define TMR_TMR32B1PWMC_PWM0_MASK                 ((unsigned int) 0x00000001)   
#define TMR_TMR32B1PWMC_PWM0_ENABLED              ((unsigned int) 0x00000001) // PWM mode is enabled for CT32Bn_MAT0
#define TMR_TMR32B1PWMC_PWM0_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR32B1PWMC_PWM1_MASK                 ((unsigned int) 0x00000002)   
#define TMR_TMR32B1PWMC_PWM1_ENABLED              ((unsigned int) 0x00000002) // PWM mode is enabled for CT32Bn_MAT1
#define TMR_TMR32B1PWMC_PWM1_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR32B1PWMC_PWM2_MASK                 ((unsigned int) 0x00000004)   
#define TMR_TMR32B1PWMC_PWM2_ENABLED              ((unsigned int) 0x00000004) // PWM mode is enabled for CT32Bn_MAT2
#define TMR_TMR32B1PWMC_PWM2_DISABLED             ((unsigned int) 0x00000000)
#define TMR_TMR32B1PWMC_PWM3_MASK                 ((unsigned int) 0x00000008)   
#define TMR_TMR32B1PWMC_PWM3_ENABLED              ((unsigned int) 0x00000008)
#define TMR_TMR32B1PWMC_PWM3_DISABLED             ((unsigned int) 0x00000000)

/*##############################################################################
## WDT - Watchdog Timer
##############################################################################*/

#define WDT_BASE_ADDRESS                          (0x40004000)

/*  WDMOD (Watchdog Mode register)
    The WDMOD register controls the operation of the Watchdog through the combination of
    WDEN and RESET bits. Note that a watchdog feed must be performed before any
    changes to the WDMOD register take effect.  */

#define WDT_WDMOD                                 (*(pREG32 (0x40004000)))    // Watchdog mode register
#define WDT_WDMOD_WDEN_DISABLED                   (0x00000000)    // Watchdog enable bit
#define WDT_WDMOD_WDEN_ENABLED                    (0x00000001)
#define WDT_WDMOD_WDEN_MASK                       (0x00000001)
#define WDT_WDMOD_WDRESET_DISABLED                (0x00000000)    // Watchdog reset enable bit
#define WDT_WDMOD_WDRESET_ENABLED                 (0x00000002)
#define WDT_WDMOD_WDRESET_MASK                    (0x00000002)
#define WDT_WDMOD_WDTOF                           (0x00000004)    // Watchdog time-out interrupt flag
#define WDT_WDMOD_WDTOF_MASK                      (0x00000004)    // Set when the watchdog times out
#define WDT_WDMOD_WDINT                           (0x00000008)    // Watchdog timer interrupt flag
#define WDT_WDMOD_WDINT_MASK                      (0x00000008)

/*  WDTC (Watchdog timer constant register) */
#define WDT_WDTC                                  (*(pREG32 (0x40004004)))

/*  WDFEED (Watchdog Feed register)
    Writing 0xAA followed by 0x55 to this register will reload the Watchdog timer with the
    WDTC value. This operation will also start the Watchdog if it is enabled via the WDMOD
    register. Setting the WDEN bit in the WDMOD register is not sufficient to enable the
    Watchdog. A valid feed sequence must be completed after setting WDEN before the
    Watchdog is capable of generating a reset. Until then, the Watchdog will ignore feed
    errors. After writing 0xAA to WDFEED, access to any Watchdog register other than writing
    0x55 to WDFEED causes an immediate reset/interrupt when the Watchdog is enabled.
    The reset will be generated during the second PCLK following an incorrect access to a
    Watchdog register during a feed sequence.
    Interrupts should be disabled during the feed sequence. An abort condition will occur if an
    interrupt happens during the feed sequence. */

#define WDT_WDFEED                                (*(pREG32 (0x40004008)))    // Watchdog feed sequence register
#define WDT_WDFEED_FEED1                          (0x000000AA)
#define WDT_WDFEED_FEED2                          (0x00000055)

/*  WDTV (Watchdog timer value register) */
#define WDT_WDTV                                  (*(pREG32 (0x4000400C)))

/*##############################################################################
## System Tick Timer
##############################################################################*/

#define SYSTICK_BASE_ADDRESS                      (0xE000E000)

/*  STCTRL (System Timer Control and status register)
    The STCTRL register contains control information for the System Tick Timer, and provides
    a status flag.  */

#define SYSTICK_STCTRL                            (*(pREG32 (0xE000E010)))    // System tick control
#define SYSTICK_STCTRL_ENABLE                     (0x00000001)    // System tick counter enable
#define SYSTICK_STCTRL_TICKINT                    (0x00000002)    // System tick interrupt enable
#define SYSTICK_STCTRL_CLKSOURCE                  (0x00000004)    // NOTE: This isn't documented but is based on NXP examples
#define SYSTICK_STCTRL_COUNTFLAG                  (0x00010000)    // System tick counter flag

/*  STRELOAD (System Timer Reload value register)
    The STRELOAD register is set to the value that will be loaded into the System Tick Timer
    whenever it counts down to zero. This register is loaded by software as part of timer
    initialization. The STCALIB register may be read and used as the value for STRELOAD if
    the CPU or external clock is running at the frequency intended for use with the STCALIB
    value.  */

#define SYSTICK_STRELOAD                          (*(pREG32 (0xE000E014)))    // System timer reload
#define SYSTICK_STRELOAD_MASK                     (0x00FFFFFF)

/*  STCURR (System Timer Current value register)
    The STCURR register returns the current count from the System Tick counter when it is
    read by software. */

#define SYSTICK_STCURR                            (*(pREG32 (0xE000E018)))
#define SYSTICK_STCURR_MASK                       (0x00FFFFFF)

/*  STCALIB (System Timer Calibration value register) */

#define SYSTICK_STCALIB                           (*(pREG32 (0xE000E01C)))    // System timer calibration
#define SYSTICK_STCALIB_TENMS_MASK                (0x00FFFFFF)
#define SYSTICK_STCALIB_SKEW_MASK                 (0x40000000)
#define SYSTICK_STCALIB_NOREF_MASK                (0x80000000)

/*##############################################################################
## ADC
##############################################################################*/

#define ADC_AD0_BASE_ADDRESS                      (0x4001C000)

/*  AD0CR (ADC control register) */

#define ADC_AD0CR                                 (*(pREG32 (0x4001C000)))
#define ADC_AD0CR_SEL_MASK                        (0x000000FF)
#define ADC_AD0CR_SEL_AD0                         (0x00000001)
#define ADC_AD0CR_SEL_AD1                         (0x00000002)
#define ADC_AD0CR_SEL_AD2                         (0x00000004)
#define ADC_AD0CR_SEL_AD3                         (0x00000008)
#define ADC_AD0CR_SEL_AD4                         (0x00000010)
#define ADC_AD0CR_SEL_AD5                         (0x00000020)
#define ADC_AD0CR_SEL_AD6                         (0x00000040)
#define ADC_AD0CR_SEL_AD7                         (0x00000080)
#define ADC_AD0CR_CLKDIV_MASK                     (0x0000FF00)
#define ADC_AD0CR_BURST_MASK                      (0x00010000)
#define ADC_AD0CR_BURST_SWMODE                    (0x00000000)
#define ADC_AD0CR_BURST_HWSCANMODE                (0x00010000)
#define ADC_AD0CR_CLKS_MASK                       (0x000E0000)
#define ADC_AD0CR_CLKS_10BITS                     (0x00000000)
#define ADC_AD0CR_CLKS_9BITS                      (0x00020000)
#define ADC_AD0CR_CLKS_8BITS                      (0x00040000)
#define ADC_AD0CR_CLKS_7BITS                      (0x00060000)
#define ADC_AD0CR_CLKS_6BITS                      (0x00080000)
#define ADC_AD0CR_CLKS_5BITS                      (0x000A0000)
#define ADC_AD0CR_CLKS_4BITS                      (0x000C0000)
#define ADC_AD0CR_CLKS_3BITS                      (0x000E0000)
#define ADC_AD0CR_START_MASK                      (0x07000000)
#define ADC_AD0CR_START_NOSTART                   (0x00000000)
#define ADC_AD0CR_START_STARTNOW                  (0x01000000)
#define ADC_AD0CR_EDGE_MASK                       (0x08000000)
#define ADC_AD0CR_EDGE_FALLING                    (0x08000000)
#define ADC_AD0CR_EDGE_RISING                     (0x00000000)

/*  AD9GDR (A/D Global Data Register)
    The A/D Global Data Register contains the result of the most recent A/D conversion. This
    includes the data, DONE, and Overrun flags, and the number of the A/D channel to which
    the data relates. */

#define ADC_AD0GDR                                ((unsigned int) 0x4001C004)
#define ADC_AD0GDR_RESULT_MASK                    (0x0000FFC0)
#define ADC_AD0GDR_CHN_MASK                       (0x07000000)    // Channel from which the results were converted
#define ADC_AD0GDR_OVERUN_MASK                    (0x40000000)
#define ADC_AD0GDR_OVERUN                         (0x40000000)
#define ADC_AD0GDR_DONE_MASK                      (0x80000000)
#define ADC_AD0GDR_DONE                           (0x80000000)

/*  AD0STAT (A/D Status Register)
    The A/D Status register allows checking the status of all A/D channels simultaneously.
    The DONE and OVERRUN flags appearing in the ADDRn register for each A/D channel
    are mirrored in ADSTAT. The interrupt flag (the logical OR of all DONE flags) is also found
    in ADSTAT.  */

#define ADC_AD0STAT                               ((unsigned int) 0x4001C030)
#define ADC_AD0STAT_DONE0_MASK                    (0x00000001)
#define ADC_AD0STAT_DONE0                         (0x00000001)
#define ADC_AD0STAT_DONE1_MASK                    (0x00000002)
#define ADC_AD0STAT_DONE1                         (0x00000002)
#define ADC_AD0STAT_DONE2_MASK                    (0x00000004)
#define ADC_AD0STAT_DONE2                         (0x00000004)
#define ADC_AD0STAT_DONE3_MASK                    (0x00000008)
#define ADC_AD0STAT_DONE3                         (0x00000008)
#define ADC_AD0STAT_DONE4_MASK                    (0x00000010)
#define ADC_AD0STAT_DONE4                         (0x00000010)
#define ADC_AD0STAT_DONE5_MASK                    (0x00000020)
#define ADC_AD0STAT_DONE5                         (0x00000020)
#define ADC_AD0STAT_DONE6_MASK                    (0x00000040)
#define ADC_AD0STAT_DONE6                         (0x00000040)
#define ADC_AD0STAT_DONE7_MASK                    (0x00000080)
#define ADC_AD0STAT_DONE7                         (0x00000080)
#define ADC_AD0STAT_OVERRUN0_MASK                 (0x00000100)
#define ADC_AD0STAT_OVERRUN0                      (0x00000100)
#define ADC_AD0STAT_OVERRUN1_MASK                 (0x00000200)
#define ADC_AD0STAT_OVERRUN1                      (0x00000200)
#define ADC_AD0STAT_OVERRUN2_MASK                 (0x00000400)
#define ADC_AD0STAT_OVERRUN2                      (0x00000400)
#define ADC_AD0STAT_OVERRUN3_MASK                 (0x00000800)
#define ADC_AD0STAT_OVERRUN3                      (0x00000800)
#define ADC_AD0STAT_OVERRUN4_MASK                 (0x00001000)
#define ADC_AD0STAT_OVERRUN4                      (0x00001000)
#define ADC_AD0STAT_OVERRUN5_MASK                 (0x00002000)
#define ADC_AD0STAT_OVERRUN5                      (0x00002000)
#define ADC_AD0STAT_OVERRUN6_MASK                 (0x00004000)
#define ADC_AD0STAT_OVERRUN6                      (0x00004000)
#define ADC_AD0STAT_OVERRUN7_MASK                 (0x00008000)
#define ADC_AD0STAT_OVERRUN7                      (0x00008000)
#define ADC_AD0STAT_ADINT_MASK                    (0x00010000)
#define ADC_AD0STAT_ADINT                         (0x00010000)

/*  ADINTEN0 (A/D Interrupt Enable Register)
    This register allows control over which A/D channels generate an interrupt when a
    conversion is complete. For example, it may be desirable to use some A/D channels to
    monitor sensors by continuously performing conversions on them. The most recent
    results are read by the application program whenever they are needed. In this case, an
    interrupt is not desirable at the end of each conversion for some A/D channels.  */

#define ADC_AD0INTEN                              ((unsigned int) 0x4001C00C)   // ADC Interrupt Enable Register
#define ADC_AD0INTEN_ADINTEN0_MASK                (0x00000001)
#define ADC_AD0INTEN_ADINTEN0                     (0x00000001)
#define ADC_AD0INTEN_ADINTEN1_MASK                (0x00000002)
#define ADC_AD0INTEN_ADINTEN1                     (0x00000002)
#define ADC_AD0INTEN_ADINTEN2_MASK                (0x00000004)
#define ADC_AD0INTEN_ADINTEN2                     (0x00000004)
#define ADC_AD0INTEN_ADINTEN3_MASK                (0x00000008)
#define ADC_AD0INTEN_ADINTEN3                     (0x00000008)
#define ADC_AD0INTEN_ADINTEN4_MASK                (0x00000010)
#define ADC_AD0INTEN_ADINTEN4                     (0x00000010)
#define ADC_AD0INTEN_ADINTEN5_MASK                (0x00000020)
#define ADC_AD0INTEN_ADINTEN5                     (0x00000020)
#define ADC_AD0INTEN_ADINTEN6_MASK                (0x00000040)
#define ADC_AD0INTEN_ADINTEN6                     (0x00000040)
#define ADC_AD0INTEN_ADINTEN7_MASK                (0x00000080)
#define ADC_AD0INTEN_ADINTEN7                     (0x00000080)
#define ADC_AD0INTEN_ADGINTEN_MASK                (0x00000100)
#define ADC_AD0INTEN_ADGINTEN_ENABLE              (0x00000100)
#define ADC_AD0INTEN_ADGINTEN_DISABLE             (0x00000000)

/*  AD0DR0..7 (A/D Data Registers)
    The A/D Data Registers hold the result when an A/D conversion is complete, and also
    include the flags that indicate when a conversion has been completed and when a
    conversion overrun has occurred. */

#define ADC_AD0DR0                                ((unsigned int) 0x4001C010)   // ADC Data Register 0
#define ADC_AD0DR1                                ((unsigned int) 0x4001C014)   // ADC Data Register 1
#define ADC_AD0DR2                                ((unsigned int) 0x4001C018)   // ADC Data Register 2
#define ADC_AD0DR3                                ((unsigned int) 0x4001C01C)   // ADC Data Register 3
#define ADC_AD0DR4                                ((unsigned int) 0x4001C020)   // ADC Data Register 4
#define ADC_AD0DR5                                ((unsigned int) 0x4001C024)   // ADC Data Register 5
#define ADC_AD0DR6                                ((unsigned int) 0x4001C028)   // ADC Data Register 6
#define ADC_AD0DR7                                ((unsigned int) 0x4001C02C)   // ADC Data Register 7
#define ADC_DR_V_MASK                             (0x0000FFC0)
#define ADC_DR_OVERRUN_MASK                       (0x40000000)
#define ADC_DR_OVERRUN                            (0x40000000)
#define ADC_DR_DONE_MASK                          (0x80000000)
#define ADC_DR_DONE                               (0x80000000)

/*##############################################################################
## Misc. Inline Functions
##############################################################################*/

/**************************************************************************/
/*! 
    @brief  Causes a system reset

    Resets the system using the AIRCR register, and waits in a loop until reset
    occurs (since there may be a delay since the write to the register and the
*/
/**************************************************************************/
static inline void __reset()      { __disable_irq(); SCB_AIRCR = SCB_AIRCR_VECTKEY_VALUE | SCB_AIRCR_SYSRESETREQ; while(1); }


#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/types.h>
#include <MFRC522.h>
#include "bcm2835.h"
#include <security/pam_appl.h>
#include <security/pam_modules.h>

#define RSTPIN RPI_V2_GPIO_P1_22
#define _POSIX_C_SOURCE 200809L
#define BCK2835_LIBRARY_BUILD


// [BEGIN] Code region from BCM2835/RPi_RFID library
/* Physical address and size of the peripherals block
// May be overridden on RPi2
*/
off_t bcm2835_peripherals_base = BCM2835_PERI_BASE;
size_t bcm2835_peripherals_size = BCM2835_PERI_SIZE;

/* Virtual memory address of the mapped peripherals block 
 */
uint32_t *bcm2835_peripherals = (uint32_t *)MAP_FAILED;

/* And the register bases within the peripherals block
 */
volatile uint32_t *bcm2835_gpio        = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_pwm         = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_clk         = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_pads        = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_spi0        = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_bsc0        = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_bsc1        = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_st	       = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_aux	       = (uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_spi1        = (uint32_t *)MAP_FAILED;
/* BEB*/
volatile uint32_t *bcm2835_smi         = (uint32_t *)MAP_FAILED;



/* This variable allows us to test on hardware other than RPi.
// It prevents access to the kernel memory, and does not do any peripheral access
// Instead it prints out what it _would_ do if debug were 0
 */
static uint8_t debug = 0;

/* RPI 4 has different pullup registers - we need to know if we have that type */

static uint8_t pud_type_rpi4 = 0;

/* RPI 4 has different pullup operation - make backwards compat */

static uint8_t pud_compat_setting = BCM2835_GPIO_PUD_OFF;

/* I2C The time needed to transmit one byte. In microseconds.
 */
static int i2c_byte_wait_us = 0;

/* SPI bit order. BCM2835 SPI0 only supports MSBFIRST, so we instead 
 * have a software based bit reversal, based on a contribution by Damiano Benedetti
 */
static uint8_t bcm2835_spi_bit_order = BCM2835_SPI_BIT_ORDER_MSBFIRST;
static uint8_t bcm2835_byte_reverse_table[] = 
{
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
};

static uint8_t bcm2835_correct_order(uint8_t b)
{
    if (bcm2835_spi_bit_order == BCM2835_SPI_BIT_ORDER_LSBFIRST)
	return bcm2835_byte_reverse_table[b];
    else
	return b;
}

#ifdef BCM2835_HAVE_LIBCAP
#include <sys/capability.h>
static int bcm2835_has_capability(cap_value_t capability)
{
    int ok = 0;
    cap_t cap = cap_get_proc();
    if (cap)
    {
        cap_flag_value_t value;
        if (cap_get_flag(cap,capability,CAP_EFFECTIVE,&value) == 0 && value == CAP_SET)
            ok = 1;
       cap_free(cap);
    }
    return ok;
}
#endif

/*
// Low level register access functions
*/

/* Function to return the pointers to the hardware register bases */
uint32_t* bcm2835_regbase(uint8_t regbase)
{
    switch (regbase)
    {
	case BCM2835_REGBASE_ST:
	    return (uint32_t *)bcm2835_st;
	case BCM2835_REGBASE_GPIO:
	    return (uint32_t *)bcm2835_gpio;
	case BCM2835_REGBASE_PWM:
	    return (uint32_t *)bcm2835_pwm;
	case BCM2835_REGBASE_CLK:
	    return (uint32_t *)bcm2835_clk;
	case BCM2835_REGBASE_PADS:
	    return (uint32_t *)bcm2835_pads;
	case BCM2835_REGBASE_SPI0:
	    return (uint32_t *)bcm2835_spi0;
	case BCM2835_REGBASE_BSC0:
	    return (uint32_t *)bcm2835_bsc0;
	case BCM2835_REGBASE_BSC1:
	    return (uint32_t *)bcm2835_st;
	case BCM2835_REGBASE_AUX:
	    return (uint32_t *)bcm2835_aux;
	case BCM2835_REGBASE_SPI1:
	    return (uint32_t *)bcm2835_spi1;
        /* BEB */
        case BCM2835_REGBASE_SMI:
	    return (uint32_t *)bcm2835_smi;


    }
    return (uint32_t *)MAP_FAILED;
}

void  bcm2835_set_debug(uint8_t d)
{
    debug = d;
}

unsigned int bcm2835_version(void) 
{
    return BCM2835_VERSION;
}

/* Read with memory barriers from peripheral
 *
 */
uint32_t bcm2835_peri_read(volatile uint32_t* paddr)
{
    uint32_t ret;
    if (debug)
    {
		printf("bcm2835_peri_read  paddr %p\n", (void *) paddr);
		return 0;
    }
    else
    {
       __sync_synchronize();
       ret = *paddr;
       __sync_synchronize();
       return ret;
    }
}

/* read from peripheral without the read barrier
 * This can only be used if more reads to THE SAME peripheral
 * will follow.  The sequence must terminate with memory barrier
 * before any read or write to another peripheral can occur.
 * The MB can be explicit, or one of the barrier read/write calls.
 */
uint32_t bcm2835_peri_read_nb(volatile uint32_t* paddr)
{
    if (debug)
    {
	printf("bcm2835_peri_read_nb  paddr %p\n", paddr);
	return 0;
    }
    else
    {
	return *paddr;
    }
}

/* Write with memory barriers to peripheral
 */

void bcm2835_peri_write(volatile uint32_t* paddr, uint32_t value)
{
    if (debug)
    {
	printf("bcm2835_peri_write paddr %p, value %08X\n", paddr, value);
    }
    else
    {
        __sync_synchronize();
        *paddr = value;
        __sync_synchronize();
    }
}

/* write to peripheral without the write barrier */
void bcm2835_peri_write_nb(volatile uint32_t* paddr, uint32_t value)
{
    if (debug)
    {
	printf("bcm2835_peri_write_nb paddr %p, value %08X\n",
                paddr, value);
    }
    else
    {
	*paddr = value;
    }
}

/* Set/clear only the bits in value covered by the mask
 * This is not atomic - can be interrupted.
 */
void bcm2835_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask)
{
    uint32_t v = bcm2835_peri_read(paddr);
    v = (v & ~mask) | (value & mask);
    bcm2835_peri_write(paddr, v);
}

/*
// Low level convenience functions
*/

/* Function select
// pin is a BCM2835 GPIO pin number NOT RPi pin number
//      There are 6 control registers, each control the functions of a block
//      of 10 pins.
//      Each control register has 10 sets of 3 bits per GPIO pin:
//
//      000 = GPIO Pin X is an input
//      001 = GPIO Pin X is an output
//      100 = GPIO Pin X takes alternate function 0
//      101 = GPIO Pin X takes alternate function 1
//      110 = GPIO Pin X takes alternate function 2
//      111 = GPIO Pin X takes alternate function 3
//      011 = GPIO Pin X takes alternate function 4
//      010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//      X / 10 + ((X % 10) * 3)
*/
void bcm2835_gpio_fsel(uint8_t pin, uint8_t mode)
{
    /* Function selects are 10 pins per 32 bit word, 3 bits per pin */
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPFSEL0/4 + (pin/10);
    uint8_t   shift = (pin % 10) * 3;
    uint32_t  mask = BCM2835_GPIO_FSEL_MASK << shift;
    uint32_t  value = mode << shift;
    bcm2835_peri_set_bits(paddr, value, mask);
}

/* Set output pin */
void bcm2835_gpio_set(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPSET0/4 + pin/32;
    uint8_t shift = pin % 32;
    bcm2835_peri_write(paddr, 1 << shift);
}

/* Clear output pin */
void bcm2835_gpio_clr(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPCLR0/4 + pin/32;
    uint8_t shift = pin % 32;
    bcm2835_peri_write(paddr, 1 << shift);
}

/* Set all output pins in the mask */
void bcm2835_gpio_set_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPSET0/4;
    bcm2835_peri_write(paddr, mask);
}

/* Clear all output pins in the mask */
void bcm2835_gpio_clr_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPCLR0/4;
    bcm2835_peri_write(paddr, mask);
}

/* Read input pin */
uint8_t bcm2835_gpio_lev(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPLEV0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = bcm2835_peri_read(paddr);
    return (value & (1 << shift)) ? HIGH : LOW;
}

/* See if an event detection bit is set
// Sigh cant support interrupts yet
*/
uint8_t bcm2835_gpio_eds(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPEDS0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = bcm2835_peri_read(paddr);
    return (value & (1 << shift)) ? HIGH : LOW;
}

uint32_t bcm2835_gpio_eds_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPEDS0/4;
    uint32_t value = bcm2835_peri_read(paddr);
    return (value & mask);
}

/* Write a 1 to clear the bit in EDS */
void bcm2835_gpio_set_eds(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPEDS0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_write(paddr, value);
}

void bcm2835_gpio_set_eds_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPEDS0/4;
    bcm2835_peri_write(paddr, mask);
}

/* Rising edge detect enable */
void bcm2835_gpio_ren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, value, value);
}
void bcm2835_gpio_clr_ren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, 0, value);
}

/* Falling edge detect enable */
void bcm2835_gpio_fen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, value, value);
}
void bcm2835_gpio_clr_fen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, 0, value);
}

/* High detect enable */
void bcm2835_gpio_hen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPHEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, value, value);
}
void bcm2835_gpio_clr_hen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPHEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, 0, value);
}

/* Low detect enable */
void bcm2835_gpio_len(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPLEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, value, value);
}
void bcm2835_gpio_clr_len(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPLEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, 0, value);
}

/* Async rising edge detect enable */
void bcm2835_gpio_aren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPAREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, value, value);
}
void bcm2835_gpio_clr_aren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPAREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, 0, value);
}

/* Async falling edge detect enable */
void bcm2835_gpio_afen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPAFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, value, value);
}
void bcm2835_gpio_clr_afen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPAFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits(paddr, 0, value);
}

/* Set pullup/down */
void bcm2835_gpio_pud(uint8_t pud)
{
    if( pud_type_rpi4 )
    {
        pud_compat_setting = pud;
    }
    else {
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPPUD/4;
    bcm2835_peri_write(paddr, pud);
}
}

/* Pullup/down clock
// Clocks the value of pud into the GPIO pin
*/
void bcm2835_gpio_pudclk(uint8_t pin, uint8_t on)
{
    if( pud_type_rpi4 )
    {
        if( on )
            bcm2835_gpio_set_pud( pin, pud_compat_setting);
    }
    else
    {
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPPUDCLK0/4 + pin/32;
    uint8_t shift = pin % 32;
    bcm2835_peri_write(paddr, (on ? 1 : 0) << shift);
}
}

/* Read GPIO pad behaviour for groups of GPIOs */
uint32_t bcm2835_gpio_pad(uint8_t group)
{
  if (bcm2835_pads == MAP_FAILED)
    return 0;
  
    volatile uint32_t* paddr = bcm2835_pads + BCM2835_PADS_GPIO_0_27/4 + group;
    return bcm2835_peri_read(paddr);
}

/* Set GPIO pad behaviour for groups of GPIOs
// powerup value for all pads is
// BCM2835_PAD_SLEW_RATE_UNLIMITED | BCM2835_PAD_HYSTERESIS_ENABLED | BCM2835_PAD_DRIVE_8mA
*/
void bcm2835_gpio_set_pad(uint8_t group, uint32_t control)
{
  if (bcm2835_pads == MAP_FAILED)
    return;
  
    volatile uint32_t* paddr = bcm2835_pads + BCM2835_PADS_GPIO_0_27/4 + group;
    bcm2835_peri_write(paddr, control | BCM2835_PAD_PASSWRD);
}

/* Some convenient arduino-like functions
// milliseconds
*/
void bcm2835_delay(unsigned int millis)
{
    struct timespec sleeper;
    
    sleeper.tv_sec  = (time_t)(millis / 1000);
    sleeper.tv_nsec = (long)(millis % 1000) * 1000000;
    nanosleep(&sleeper, NULL);
}

/* microseconds */
void bcm2835_delayMicroseconds(uint64_t micros)
{
    struct timespec t1;
    uint64_t        start;
	
    if (debug)
    {
	/* Cant access sytem timers in debug mode */
	printf("bcm2835_delayMicroseconds %lld\n", (long long int) micros);
	return;
    }

    /* Calling nanosleep() takes at least 100-200 us, so use it for
    // long waits and use a busy wait on the System Timer for the rest.
    */
    start =  bcm2835_st_read();
   
    /* Not allowed to access timer registers (result is not as precise)*/
    if (start==0)
    {
	t1.tv_sec = 0;
	t1.tv_nsec = 1000 * (long)(micros);
	nanosleep(&t1, NULL);
	return;
    }

    if (micros > 450)
    {
	t1.tv_sec = 0;
	t1.tv_nsec = 1000 * (long)(micros - 200);
	nanosleep(&t1, NULL);
    }    
  
    bcm2835_st_delay(start, micros);
}

/*
// Higher level convenience functions
*/

/* Set the state of an output */
void bcm2835_gpio_write(uint8_t pin, uint8_t on)
{
    if (on)
	bcm2835_gpio_set(pin);
    else
	bcm2835_gpio_clr(pin);
}

/* Set the state of a all 32 outputs in the mask to on or off */
void bcm2835_gpio_write_multi(uint32_t mask, uint8_t on)
{
    if (on)
	bcm2835_gpio_set_multi(mask);
    else
	bcm2835_gpio_clr_multi(mask);
}

/* Set the state of a all 32 outputs in the mask to the values in value */
void bcm2835_gpio_write_mask(uint32_t value, uint32_t mask)
{
    bcm2835_gpio_set_multi(value & mask);
    bcm2835_gpio_clr_multi((~value) & mask);
}

/* Set the pullup/down resistor for a pin
//
// The GPIO Pull-up/down Clock Registers control the actuation of internal pull-downs on
// the respective GPIO pins. These registers must be used in conjunction with the GPPUD
// register to effect GPIO Pull-up/down changes. The following sequence of events is
// required:
// 1. Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
// to remove the current Pull-up/down)
// 2. Wait 150 cycles ? this provides the required set-up time for the control signal
// 3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
// modify ? NOTE only the pads which receive a clock will be modified, all others will
// retain their previous state.
// 4. Wait 150 cycles ? this provides the required hold time for the control signal
// 5. Write to GPPUD to remove the control signal
// 6. Write to GPPUDCLK0/1 to remove the clock
//
// RPi has P1-03 and P1-05 with 1k8 pullup resistor
//
// RPI 4 uses a different PUD method - no clock

*/
void bcm2835_gpio_set_pud(uint8_t pin, uint8_t pud)
{
    if( pud_type_rpi4 )
    {
        int shiftbits = (pin & 0xf) << 1;
        uint32_t bits;
        uint32_t pull;
        
        switch (pud)
        {
           case BCM2835_GPIO_PUD_OFF:  pull = 0; break;
           case BCM2835_GPIO_PUD_UP:   pull = 1; break;
           case BCM2835_GPIO_PUD_DOWN: pull = 2; break;
           default: return;
        }
                
        volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPPUPPDN0/4 + (pin >> 4);
        
        bits = bcm2835_peri_read_nb( paddr );
        bits &= ~(3 << shiftbits);
        bits |= (pull << shiftbits);
        
        bcm2835_peri_write_nb( paddr, bits );
        
    } else
    {
    bcm2835_gpio_pud(pud);
    delayMicroseconds(10);
    bcm2835_gpio_pudclk(pin, 1);
    delayMicroseconds(10);
    bcm2835_gpio_pud(BCM2835_GPIO_PUD_OFF);
    bcm2835_gpio_pudclk(pin, 0);
}

}


uint8_t bcm2835_gpio_get_pud(uint8_t pin)
{
    uint8_t ret = BCM2835_GPIO_PUD_ERROR;
    
    if( pud_type_rpi4 )
    {
        uint32_t bits;
        volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPPUPPDN0/4 + (pin >> 4);
        bits = (bcm2835_peri_read_nb( paddr ) >> ((pin & 0xf)<<1)) & 0x3;
        
        switch (bits)
        {
            case 0: ret = BCM2835_GPIO_PUD_OFF; break;
            case 1: ret = BCM2835_GPIO_PUD_UP; break;
            case 2: ret = BCM2835_GPIO_PUD_DOWN; break;
            default: ret = BCM2835_GPIO_PUD_ERROR;
        }   
    }
    
    return ret;
}

static void bcm2835_aux_spi_reset(void)
 {
     volatile uint32_t* cntl0 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL0/4;
     volatile uint32_t* cntl1 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL1/4;
 
     bcm2835_peri_write(cntl1, 0);
     bcm2835_peri_write(cntl0, BCM2835_AUX_SPI_CNTL0_CLEARFIFO);
}

int bcm2835_spi_begin(void)
{
    volatile uint32_t* paddr;

    if (bcm2835_spi0 == MAP_FAILED)
      return 0; /* bcm2835_init() failed, or not root */
    
    /* Set the SPI0 pins to the Alt 0 function to enable SPI0 access on them */
    bcm2835_gpio_fsel(RPI_GPIO_P1_26, BCM2835_GPIO_FSEL_ALT0); /* CE1 */
    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_ALT0); /* CE0 */
    bcm2835_gpio_fsel(RPI_GPIO_P1_21, BCM2835_GPIO_FSEL_ALT0); /* MISO */
    bcm2835_gpio_fsel(RPI_GPIO_P1_19, BCM2835_GPIO_FSEL_ALT0); /* MOSI */
    bcm2835_gpio_fsel(RPI_GPIO_P1_23, BCM2835_GPIO_FSEL_ALT0); /* CLK */
    
    /* Set the SPI CS register to the some sensible defaults */
    paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    bcm2835_peri_write(paddr, 0); /* All 0s */
    
    /* Clear TX and RX fifos */
    bcm2835_peri_write_nb(paddr, BCM2835_SPI0_CS_CLEAR);

    return 1; // OK
}

void bcm2835_spi_end(void)
{  
    /* Set all the SPI0 pins back to input */
    bcm2835_gpio_fsel(RPI_GPIO_P1_26, BCM2835_GPIO_FSEL_INPT); /* CE1 */
    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_INPT); /* CE0 */
    bcm2835_gpio_fsel(RPI_GPIO_P1_21, BCM2835_GPIO_FSEL_INPT); /* MISO */
    bcm2835_gpio_fsel(RPI_GPIO_P1_19, BCM2835_GPIO_FSEL_INPT); /* MOSI */
    bcm2835_gpio_fsel(RPI_GPIO_P1_23, BCM2835_GPIO_FSEL_INPT); /* CLK */
}

void bcm2835_spi_setBitOrder(uint8_t order)
{
    bcm2835_spi_bit_order = order;
}

/* defaults to 0, which means a divider of 65536.
// The divisor must be a power of 2. Odd numbers
// rounded down. The maximum SPI clock rate is
// of the APB clock
*/
void bcm2835_spi_setClockDivider(uint16_t divider)
{
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CLK/4;
    bcm2835_peri_write(paddr, divider);
}

void bcm2835_spi_set_speed_hz(uint32_t speed_hz)
{
	uint16_t divider = (uint16_t) ((uint32_t) BCM2835_CORE_CLK_HZ / speed_hz);
	divider &= 0xFFFE;
	bcm2835_spi_setClockDivider(divider);
}

void bcm2835_spi_setDataMode(uint8_t mode)
{
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    /* Mask in the CPO and CPHA bits of CS */
    bcm2835_peri_set_bits(paddr, mode << 2, BCM2835_SPI0_CS_CPOL | BCM2835_SPI0_CS_CPHA);
}

/* Writes (and reads) a single byte to SPI */
uint8_t bcm2835_spi_transfer(uint8_t value)
{
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;
    uint32_t ret;

    /* This is Polled transfer as per section 10.6.1
    // BUG ALERT: what happens if we get interupted in this section, and someone else
    // accesses a different peripheral? 
    // Clear TX and RX fifos
    */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);

    /* Set TA = 1 */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);

    /* Maybe wait for TXD */
    while (!(bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))
	;

    /* Write to FIFO, no barrier */
    bcm2835_peri_write_nb(fifo, bcm2835_correct_order(value));

    /* Wait for DONE to be set */
    while (!(bcm2835_peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE))
	;

    /* Read any byte that was sent back by the slave while we sere sending to it */
    ret = bcm2835_correct_order(bcm2835_peri_read_nb(fifo));

    /* Set TA = 0, and also set the barrier */
    bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);

    return ret;
}

/* Writes (and reads) an number of bytes to SPI */
void bcm2835_spi_transfernb(char* tbuf, char* rbuf, uint32_t len)
{
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;
    uint32_t TXCnt=0;
    uint32_t RXCnt=0;

    /* This is Polled transfer as per section 10.6.1
    // BUG ALERT: what happens if we get interupted in this section, and someone else
    // accesses a different peripheral? 
    */

    /* Clear TX and RX fifos */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);

    /* Set TA = 1 */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);

    /* Use the FIFO's to reduce the interbyte times */
    while((TXCnt < len)||(RXCnt < len))
    {
        /* TX fifo not full, so add some more bytes */
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))&&(TXCnt < len ))
        {
	    bcm2835_peri_write_nb(fifo, bcm2835_correct_order(tbuf[TXCnt]));
	    TXCnt++;
        }
        /* Rx fifo not empty, so get the next received bytes */
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD))&&( RXCnt < len ))
        {
	    rbuf[RXCnt] = bcm2835_correct_order(bcm2835_peri_read_nb(fifo));
	    RXCnt++;
        }
    }
    /* Wait for DONE to be set */
    while (!(bcm2835_peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE))
	;

    /* Set TA = 0, and also set the barrier */
    bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);
}

/* Writes an number of bytes to SPI */
void bcm2835_spi_writenb(const char* tbuf, uint32_t len)
{
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;
    uint32_t i;

    /* This is Polled transfer as per section 10.6.1
    // BUG ALERT: what happens if we get interupted in this section, and someone else
    // accesses a different peripheral?
    // Answer: an ISR is required to issue the required memory barriers.
    */

    /* Clear TX and RX fifos */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);

    /* Set TA = 1 */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);

    for (i = 0; i < len; i++)
    {
	/* Maybe wait for TXD */
	while (!(bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))
	    ;
	
	/* Write to FIFO, no barrier */
	bcm2835_peri_write_nb(fifo, bcm2835_correct_order(tbuf[i]));
	
	/* Read from FIFO to prevent stalling */
	while (bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD)
	    (void) bcm2835_peri_read_nb(fifo);
    }
    
    /* Wait for DONE to be set */
    while (!(bcm2835_peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE)) {
	while (bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD)
		(void) bcm2835_peri_read_nb(fifo);
    };

    /* Set TA = 0, and also set the barrier */
    bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);
}

/* Writes (and reads) an number of bytes to SPI
// Read bytes are copied over onto the transmit buffer
*/
void bcm2835_spi_transfern(char* buf, uint32_t len)
{
    bcm2835_spi_transfernb(buf, buf, len);
}

void bcm2835_spi_chipSelect(uint8_t cs)
{
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    /* Mask in the CS bits of CS */
    bcm2835_peri_set_bits(paddr, cs, BCM2835_SPI0_CS_CS);
}

void bcm2835_spi_setChipSelectPolarity(uint8_t cs, uint8_t active)
{
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    uint8_t shift = 21 + cs;
    /* Mask in the appropriate CSPOLn bit */
    bcm2835_peri_set_bits(paddr, active << shift, 1 << shift);
}

void bcm2835_spi_write(uint16_t data)
{
#if 0
	char buf[2];

	buf[0] = data >> 8;
	buf[1] = data & 0xFF;

	bcm2835_spi_transfern(buf, 2);
#else
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;

    /* Clear TX and RX fifos */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);

    /* Set TA = 1 */
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);

	/* Maybe wait for TXD */
	while (!(bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))
	    ;

	/* Write to FIFO */
	bcm2835_peri_write_nb(fifo,  (uint32_t) data >> 8);
	bcm2835_peri_write_nb(fifo,  data & 0xFF);


    /* Wait for DONE to be set */
    while (!(bcm2835_peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE))
	;

    /* Set TA = 0, and also set the barrier */
    bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);
#endif
}

int bcm2835_aux_spi_begin(void)
{
    volatile uint32_t* enable = bcm2835_aux + BCM2835_AUX_ENABLE/4;
    volatile uint32_t* cntl0 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL0/4;
    volatile uint32_t* cntl1 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL1/4;

    if (bcm2835_spi1 == MAP_FAILED)
	return 0; /* bcm2835_init() failed, or not root */

    /* Set the SPI pins to the Alt 4 function to enable SPI1 access on them */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_36, BCM2835_GPIO_FSEL_ALT4);	/* SPI1_CE2_N */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_35, BCM2835_GPIO_FSEL_ALT4);	/* SPI1_MISO */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_38, BCM2835_GPIO_FSEL_ALT4);	/* SPI1_MOSI */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_40, BCM2835_GPIO_FSEL_ALT4);	/* SPI1_SCLK */

    bcm2835_aux_spi_setClockDivider(bcm2835_aux_spi_CalcClockDivider(1000000));	// Default 1MHz SPI

    bcm2835_peri_write(enable, BCM2835_AUX_ENABLE_SPI0);
    bcm2835_peri_write(cntl1, 0);
    bcm2835_peri_write(cntl0, BCM2835_AUX_SPI_CNTL0_CLEARFIFO);

    return 1; /* OK */
}

void bcm2835_aux_spi_end(void)
{
    /* Set all the SPI1 pins back to input */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_36, BCM2835_GPIO_FSEL_INPT);	/* SPI1_CE2_N */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_35, BCM2835_GPIO_FSEL_INPT);	/* SPI1_MISO */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_38, BCM2835_GPIO_FSEL_INPT);	/* SPI1_MOSI */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_40, BCM2835_GPIO_FSEL_INPT);	/* SPI1_SCLK */
}

#define DIV_ROUND_UP(n,d)	(((n) + (d) - 1) / (d))

uint16_t bcm2835_aux_spi_CalcClockDivider(uint32_t speed_hz)
{
    uint16_t divider;

    if (speed_hz < (uint32_t) BCM2835_AUX_SPI_CLOCK_MIN) {
	speed_hz = (uint32_t) BCM2835_AUX_SPI_CLOCK_MIN;
    } else if (speed_hz > (uint32_t) BCM2835_AUX_SPI_CLOCK_MAX) {
	speed_hz = (uint32_t) BCM2835_AUX_SPI_CLOCK_MAX;
    }

    divider = (uint16_t) DIV_ROUND_UP(BCM2835_CORE_CLK_HZ, 2 * speed_hz) - 1;

    if (divider > (uint16_t) BCM2835_AUX_SPI_CNTL0_SPEED_MAX) {
	return (uint16_t) BCM2835_AUX_SPI_CNTL0_SPEED_MAX;
    }

    return divider;
}

static uint32_t spi1_speed;

void bcm2835_aux_spi_setClockDivider(uint16_t divider)
{
    spi1_speed = (uint32_t) divider;
}

void bcm2835_aux_spi_write(uint16_t data)
{
    volatile uint32_t* cntl0 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL0/4;
    volatile uint32_t* cntl1 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL1/4;
    volatile uint32_t* stat = bcm2835_spi1 + BCM2835_AUX_SPI_STAT/4;
    volatile uint32_t* io = bcm2835_spi1 + BCM2835_AUX_SPI_IO/4;

    uint32_t _cntl0 = (spi1_speed << BCM2835_AUX_SPI_CNTL0_SPEED_SHIFT);
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_CS2_N;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_ENABLE;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_MSBF_OUT;
    _cntl0 |= 16; // Shift length

    bcm2835_peri_write(cntl0, _cntl0);
    bcm2835_peri_write(cntl1, BCM2835_AUX_SPI_CNTL1_MSBF_IN);

    while (bcm2835_peri_read(stat) & BCM2835_AUX_SPI_STAT_TX_FULL)
	;

    bcm2835_peri_write(io, (uint32_t) data << 16);
}

void bcm2835_aux_spi_writenb(const char *tbuf, uint32_t len) {
    volatile uint32_t* cntl0 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL0/4;
    volatile uint32_t* cntl1 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL1/4;
    volatile uint32_t* stat = bcm2835_spi1 + BCM2835_AUX_SPI_STAT/4;
    volatile uint32_t* txhold = bcm2835_spi1 + BCM2835_AUX_SPI_TXHOLD/4;
    volatile uint32_t* io = bcm2835_spi1 + BCM2835_AUX_SPI_IO/4;

    char *tx = (char *) tbuf;
    uint32_t tx_len = len;
    uint32_t count;
    uint32_t data;
    uint32_t i;
    uint8_t byte;

    uint32_t _cntl0 = (spi1_speed << BCM2835_AUX_SPI_CNTL0_SPEED_SHIFT);
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_CS2_N;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_ENABLE;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_MSBF_OUT;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_VAR_WIDTH;

    bcm2835_peri_write(cntl0, _cntl0);
    bcm2835_peri_write(cntl1, BCM2835_AUX_SPI_CNTL1_MSBF_IN);

    while (tx_len > 0) {

	while (bcm2835_peri_read(stat) & BCM2835_AUX_SPI_STAT_TX_FULL)
	    ;

	count = MIN(tx_len, 3);
	data = 0;

	for (i = 0; i < count; i++) {
	    byte = (tx != NULL) ? (uint8_t) *tx++ : (uint8_t) 0;
	    data |= byte << (8 * (2 - i));
	}

	data |= (count * 8) << 24;
	tx_len -= count;

	if (tx_len != 0) {
	    bcm2835_peri_write(txhold, data);
	} else {
	    bcm2835_peri_write(io, data);
	}

	while (bcm2835_peri_read(stat) & BCM2835_AUX_SPI_STAT_BUSY)
	    ;

	(void) bcm2835_peri_read(io);
    }
}

void bcm2835_aux_spi_transfernb(const char *tbuf, char *rbuf, uint32_t len) {
    volatile uint32_t* cntl0 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL0/4;
    volatile uint32_t* cntl1 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL1/4;
    volatile uint32_t* stat = bcm2835_spi1 + BCM2835_AUX_SPI_STAT/4;
    volatile uint32_t* txhold = bcm2835_spi1 + BCM2835_AUX_SPI_TXHOLD/4;
    volatile uint32_t* io = bcm2835_spi1 + BCM2835_AUX_SPI_IO/4;

	char *tx = (char *)tbuf;
	char *rx = (char *)rbuf;
	uint32_t tx_len = len;
	uint32_t rx_len = len;
	uint32_t count;
	uint32_t data;
	uint32_t i;
	uint8_t byte;

	uint32_t _cntl0 = (spi1_speed << BCM2835_AUX_SPI_CNTL0_SPEED_SHIFT);
	_cntl0 |= BCM2835_AUX_SPI_CNTL0_CS2_N;
	_cntl0 |= BCM2835_AUX_SPI_CNTL0_ENABLE;
	_cntl0 |= BCM2835_AUX_SPI_CNTL0_MSBF_OUT;
	_cntl0 |= BCM2835_AUX_SPI_CNTL0_VAR_WIDTH;

	bcm2835_peri_write(cntl0, _cntl0);
	bcm2835_peri_write(cntl1, BCM2835_AUX_SPI_CNTL1_MSBF_IN);

	while ((tx_len > 0) || (rx_len > 0)) {

		while (!(bcm2835_peri_read(stat) & BCM2835_AUX_SPI_STAT_TX_FULL) && (tx_len > 0)) {
			count = MIN(tx_len, 3);
			data = 0;

			for (i = 0; i < count; i++) {
				byte = (tx != NULL) ? (uint8_t) *tx++ : (uint8_t) 0;
				data |= byte << (8 * (2 - i));
			}

			data |= (count * 8) << 24;
			tx_len -= count;

			if (tx_len != 0) {
				bcm2835_peri_write(txhold, data);
			} else {
				bcm2835_peri_write(io, data);
			}

		}

		while (!(bcm2835_peri_read(stat) & BCM2835_AUX_SPI_STAT_RX_EMPTY) && (rx_len > 0)) {
			count = MIN(rx_len, 3);
			data = bcm2835_peri_read(io);

			if (rbuf != NULL) {
				switch (count) {
				case 3:
					*rx++ = (char)((data >> 16) & 0xFF);
					/*@fallthrough@*/
					/* no break */
				case 2:
					*rx++ = (char)((data >> 8) & 0xFF);
					/*@fallthrough@*/
					/* no break */
				case 1:
					*rx++ = (char)((data >> 0) & 0xFF);
				}
			}

			rx_len -= count;
		}

		while (!(bcm2835_peri_read(stat) & BCM2835_AUX_SPI_STAT_BUSY) && (rx_len > 0)) {
			count = MIN(rx_len, 3);
			data = bcm2835_peri_read(io);

			if (rbuf != NULL) {
				switch (count) {
				case 3:
					*rx++ = (char)((data >> 16) & 0xFF);
					/*@fallthrough@*/
					/* no break */
				case 2:
					*rx++ = (char)((data >> 8) & 0xFF);
					/*@fallthrough@*/
					/* no break */
				case 1:
					*rx++ = (char)((data >> 0) & 0xFF);
				}
			}

			rx_len -= count;
		}
	}
}

void bcm2835_aux_spi_transfern(char *buf, uint32_t len) {
	bcm2835_aux_spi_transfernb(buf, buf, len);
}

/* Writes (and reads) a single byte to AUX SPI */
uint8_t bcm2835_aux_spi_transfer(uint8_t value)
{
    volatile uint32_t* cntl0 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL0/4;
    volatile uint32_t* cntl1 = bcm2835_spi1 + BCM2835_AUX_SPI_CNTL1/4;
    volatile uint32_t* stat = bcm2835_spi1 + BCM2835_AUX_SPI_STAT/4;
    volatile uint32_t* io = bcm2835_spi1 + BCM2835_AUX_SPI_IO/4;

    uint32_t data;

    uint32_t _cntl0 = (spi1_speed << BCM2835_AUX_SPI_CNTL0_SPEED_SHIFT);
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_CS2_N;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_ENABLE;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_MSBF_OUT;
    _cntl0 |= BCM2835_AUX_SPI_CNTL0_CPHA_IN;
    _cntl0 |= 8; // Shift length.

    uint32_t _cntl1 = BCM2835_AUX_SPI_CNTL1_MSBF_IN;

    bcm2835_peri_write(cntl1, _cntl1);
    bcm2835_peri_write(cntl0, _cntl0);

    bcm2835_peri_write(io, (uint32_t) bcm2835_correct_order(value) << 24);

    while (bcm2835_peri_read(stat) & BCM2835_AUX_SPI_STAT_BUSY)
        ;

    data = bcm2835_correct_order(bcm2835_peri_read(io) & 0xff);

    bcm2835_aux_spi_reset();

    return data;
}


int bcm2835_i2c_begin(void)
{
    uint16_t cdiv;

    if (   bcm2835_bsc0 == MAP_FAILED
	|| bcm2835_bsc1 == MAP_FAILED)
      return 0; /* bcm2835_init() failed, or not root */

#ifdef I2C_V1
    volatile uint32_t* paddr = bcm2835_bsc0 + BCM2835_BSC_DIV/4;
    /* Set the I2C/BSC0 pins to the Alt 0 function to enable I2C access on them */
    bcm2835_gpio_fsel(RPI_GPIO_P1_03, BCM2835_GPIO_FSEL_ALT0); /* SDA */
    bcm2835_gpio_fsel(RPI_GPIO_P1_05, BCM2835_GPIO_FSEL_ALT0); /* SCL */
#else
    volatile uint32_t* paddr = bcm2835_bsc1 + BCM2835_BSC_DIV/4;
    /* Set the I2C/BSC1 pins to the Alt 0 function to enable I2C access on them */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_03, BCM2835_GPIO_FSEL_ALT0); /* SDA */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_05, BCM2835_GPIO_FSEL_ALT0); /* SCL */
#endif    

    /* Read the clock divider register */
    cdiv = bcm2835_peri_read(paddr);
    /* Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    */
    i2c_byte_wait_us = ((float)cdiv / BCM2835_CORE_CLK_HZ) * 1000000 * 9;

    return 1;
}

void bcm2835_i2c_end(void)
{
#ifdef I2C_V1
    /* Set all the I2C/BSC0 pins back to input */
    bcm2835_gpio_fsel(RPI_GPIO_P1_03, BCM2835_GPIO_FSEL_INPT); /* SDA */
    bcm2835_gpio_fsel(RPI_GPIO_P1_05, BCM2835_GPIO_FSEL_INPT); /* SCL */
#else
    /* Set all the I2C/BSC1 pins back to input */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_03, BCM2835_GPIO_FSEL_INPT); /* SDA */
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_05, BCM2835_GPIO_FSEL_INPT); /* SCL */
#endif
}

void bcm2835_i2c_setSlaveAddress(uint8_t addr)
{
    /* Set I2C Device Address */
#ifdef I2C_V1
    volatile uint32_t* paddr = bcm2835_bsc0 + BCM2835_BSC_A/4;
#else	
    volatile uint32_t* paddr = bcm2835_bsc1 + BCM2835_BSC_A/4;
#endif
    bcm2835_peri_write(paddr, addr);
}

/* defaults to 0x5dc, should result in a 166.666 kHz I2C clock frequency.
// The divisor must be a power of 2. Odd numbers
// rounded down.
*/
void bcm2835_i2c_setClockDivider(uint16_t divider)
{
#ifdef I2C_V1
    volatile uint32_t* paddr = bcm2835_bsc0 + BCM2835_BSC_DIV/4;
#else
    volatile uint32_t* paddr = bcm2835_bsc1 + BCM2835_BSC_DIV/4;
#endif    
    bcm2835_peri_write(paddr, divider);
    /* Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    */
    i2c_byte_wait_us = ((float)divider / BCM2835_CORE_CLK_HZ) * 1000000 * 9;
}

/* set I2C clock divider by means of a baudrate number */
void bcm2835_i2c_set_baudrate(uint32_t baudrate)
{
	uint32_t divider;
	/* use 0xFFFE mask to limit a max value and round down any odd number */
	divider = (BCM2835_CORE_CLK_HZ / baudrate) & 0xFFFE;
	bcm2835_i2c_setClockDivider( (uint16_t)divider );
}

/* Writes an number of bytes to I2C */
uint8_t bcm2835_i2c_write(const char * buf, uint32_t len)
{
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    

    uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = BCM2835_I2C_REASON_OK;

    /* Clear FIFO */
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    /* Clear Status */
    bcm2835_peri_write(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
    /* Set Data Length */
    bcm2835_peri_write(dlen, len);
    /* pre populate FIFO with max buffer */
    while( remaining && ( i < BCM2835_BSC_FIFO_SIZE ) )
    {
        bcm2835_peri_write_nb(fifo, buf[i]);
        i++;
        remaining--;
    }
    
    /* Enable device and start transfer */
    bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    /* Transfer is over when BCM2835_BSC_S_DONE */
    while(!(bcm2835_peri_read(status) & BCM2835_BSC_S_DONE ))
    {
        while ( remaining && (bcm2835_peri_read(status) & BCM2835_BSC_S_TXD ))
    	{
	    /* Write to FIFO */
	    bcm2835_peri_write(fifo, buf[i]);
	    i++;
	    remaining--;
    	}
    }

    /* Received a NACK */
    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR)
    {
	reason = BCM2835_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT)
    {
	reason = BCM2835_I2C_REASON_ERROR_CLKT;
    }

    /* Not all data is sent */
    else if (remaining)
    {
	reason = BCM2835_I2C_REASON_ERROR_DATA;
    }

    bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

/* Read an number of bytes from I2C */
uint8_t bcm2835_i2c_read(char* buf, uint32_t len)
{
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    

    uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = BCM2835_I2C_REASON_OK;

    /* Clear FIFO */
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    /* Clear Status */
    bcm2835_peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
    /* Set Data Length */
    bcm2835_peri_write_nb(dlen, len);
    /* Start read */
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST | BCM2835_BSC_C_READ);
    
    /* wait for transfer to complete */
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (remaining && bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD)
    	{
	    /* Read from FIFO, no barrier */
	    buf[i] = bcm2835_peri_read_nb(fifo);
	    i++;
	    remaining--;
    	}
    }
    
    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD))
    {
        /* Read from FIFO, no barrier */
        buf[i] = bcm2835_peri_read_nb(fifo);
        i++;
        remaining--;
    }
    
    /* Received a NACK */
    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR)
    {
	reason = BCM2835_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT)
    {
	reason = BCM2835_I2C_REASON_ERROR_CLKT;
    }

    /* Not all data is received */
    else if (remaining)
    {
	reason = BCM2835_I2C_REASON_ERROR_DATA;
    }

    bcm2835_peri_set_bits(status, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

/* Read an number of bytes from I2C sending a repeated start after writing
// the required register. Only works if your device supports this mode
*/
uint8_t bcm2835_i2c_read_register_rs(char* regaddr, char* buf, uint32_t len)
{   
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    
	uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = BCM2835_I2C_REASON_OK;
    
    /* Clear FIFO */
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    /* Clear Status */
    bcm2835_peri_write(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
    /* Set Data Length */
    bcm2835_peri_write(dlen, 1);
    /* Enable device and start transfer */
    bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN);
    bcm2835_peri_write(fifo, regaddr[0]);
    bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    /* poll for transfer has started */
    while ( !( bcm2835_peri_read(status) & BCM2835_BSC_S_TA ) )
    {
        /* Linux may cause us to miss entire transfer stage */
        if(bcm2835_peri_read(status) & BCM2835_BSC_S_DONE)
            break;
    }
    
    /* Send a repeated start with read bit set in address */
    bcm2835_peri_write(dlen, len);
    bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST  | BCM2835_BSC_C_READ );
    
    /* Wait for write to complete and first byte back. */
    bcm2835_delayMicroseconds(i2c_byte_wait_us * 3);
    
    /* wait for transfer to complete */
    while (!(bcm2835_peri_read(status) & BCM2835_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (remaining && bcm2835_peri_read(status) & BCM2835_BSC_S_RXD)
    	{
	    /* Read from FIFO */
	    buf[i] = bcm2835_peri_read(fifo);
	    i++;
	    remaining--;
    	}
    }
    
    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (bcm2835_peri_read(status) & BCM2835_BSC_S_RXD))
    {
        /* Read from FIFO */
        buf[i] = bcm2835_peri_read(fifo);
        i++;
        remaining--;
    }
    
    /* Received a NACK */
    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR)
    {
		reason = BCM2835_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT)
    {
	reason = BCM2835_I2C_REASON_ERROR_CLKT;
    }

    /* Not all data is sent */
    else if (remaining)
    {
	reason = BCM2835_I2C_REASON_ERROR_DATA;
    }

    bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

/* Sending an arbitrary number of bytes before issuing a repeated start 
// (with no prior stop) and reading a response. Some devices require this behavior.
*/
uint8_t bcm2835_i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len)
{   
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    

    uint32_t remaining = cmds_len;
    uint32_t i = 0;
    uint8_t reason = BCM2835_I2C_REASON_OK;
    
    /* Clear FIFO */
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );

    /* Clear Status */
    bcm2835_peri_write(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);

    /* Set Data Length */
    bcm2835_peri_write(dlen, cmds_len);
 
    /* pre populate FIFO with max buffer */
    while( remaining && ( i < BCM2835_BSC_FIFO_SIZE ) )
    {
        bcm2835_peri_write_nb(fifo, cmds[i]);
        i++;
        remaining--;
    }

    /* Enable device and start transfer */
    bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    /* poll for transfer has started (way to do repeated start, from BCM2835 datasheet) */
    while ( !( bcm2835_peri_read(status) & BCM2835_BSC_S_TA ) )
    {
        /* Linux may cause us to miss entire transfer stage */
        if(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE)
            break;
    }
    
    remaining = buf_len;
    i = 0;

    /* Send a repeated start with read bit set in address */
    bcm2835_peri_write(dlen, buf_len);
    bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST  | BCM2835_BSC_C_READ );
    
    /* Wait for write to complete and first byte back. */
    bcm2835_delayMicroseconds(i2c_byte_wait_us * (cmds_len + 1));
    
    /* wait for transfer to complete */
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (remaining && bcm2835_peri_read(status) & BCM2835_BSC_S_RXD)
    	{
	    /* Read from FIFO, no barrier */
	    buf[i] = bcm2835_peri_read_nb(fifo);
	    i++;
	    remaining--;
    	}
    }
    
    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (bcm2835_peri_read(status) & BCM2835_BSC_S_RXD))
    {
        /* Read from FIFO */
        buf[i] = bcm2835_peri_read(fifo);
        i++;
        remaining--;
    }
    
    /* Received a NACK */
    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR)
    {
	reason = BCM2835_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT)
    {
	reason = BCM2835_I2C_REASON_ERROR_CLKT;
    }

    /* Not all data is sent */
    else if (remaining)
    {
	reason = BCM2835_I2C_REASON_ERROR_DATA;
    }

    bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

/* SMI support courtesy Benoit Bouchez */
int bcm2835_smi_begin(void)
{
    volatile uint32_t* paddr;
    uint32_t defConfig;

    if (bcm2835_smi == MAP_FAILED)
      return 0; /* bcm2835_smi_init() failed, or not root */

    /* Set the SMI pins to the Alt 1 function to enable SMI access on them */
    bcm2835_gpio_fsel(2, BCM2835_GPIO_FSEL_ALT1); /* SA3 */
    bcm2835_gpio_fsel(3, BCM2835_GPIO_FSEL_ALT1); /* SA2 */
    bcm2835_gpio_fsel(4, BCM2835_GPIO_FSEL_ALT1); /* SA1 */
    bcm2835_gpio_fsel(5, BCM2835_GPIO_FSEL_ALT1); /* SA0 */
    bcm2835_gpio_fsel(6, BCM2835_GPIO_FSEL_ALT1); /* SOE_N / SE */
    bcm2835_gpio_fsel(7, BCM2835_GPIO_FSEL_ALT1); /* SWE_N / SRW_N */
    bcm2835_gpio_fsel(8, BCM2835_GPIO_FSEL_ALT1); /* SD0 */
    bcm2835_gpio_fsel(9, BCM2835_GPIO_FSEL_ALT1); /* SD1 */
    bcm2835_gpio_fsel(10, BCM2835_GPIO_FSEL_ALT1); /* SD2 */
    bcm2835_gpio_fsel(11, BCM2835_GPIO_FSEL_ALT1); /* SD3 */
    bcm2835_gpio_fsel(12, BCM2835_GPIO_FSEL_ALT1); /* SD4 */
    bcm2835_gpio_fsel(13, BCM2835_GPIO_FSEL_ALT1); /* SD5 */
    bcm2835_gpio_fsel(14, BCM2835_GPIO_FSEL_ALT1); /* SD6 */
    bcm2835_gpio_fsel(15, BCM2835_GPIO_FSEL_ALT1); /* SD7 */

    /* Set the SMI clock to 125MHz -> one clock period lasts 8 ns */
    paddr = bcm2835_clk + SMICLK_CNTL;
    bcm2835_peri_write (paddr, 0x5A000000);     /* Disable SMI clock */
    paddr = bcm2835_clk + SMICLK_DIV;
    bcm2835_peri_write (paddr, 0x5A004000);     /* Set clock divider to 4 */
    paddr = bcm2835_clk + SMICLK_CNTL;
    bcm2835_peri_write (paddr, 0x5A000016);     /* Enable SMI clock with PLLD */

    /* Set a default useable configuration for the four SMI config slots */
    /* 8 bits, Intel mode, always pace, 10 clocks for R/W setup = 80 ns
     20 clocks for R/W strobe = 160 ns, 20 clocks for R/W hold = 160 ns,
     1 clock for pace control (not used but a value is needed) */
    defConfig = BCM2835_SMI_RW_WID8 | BCM2835_SMI_RW_MODE80  |
             BCM2835_SMI_RW_PACEALL | (10<<BCM2835_SMI_RW_SETUP_LS) |
             (20<<BCM2835_SMI_RW_STROBE_LS) | (20<<BCM2835_SMI_RW_HOLD_LS) |
             (1<<BCM2835_SMI_RW_PACE_LS);

    paddr = bcm2835_smi + BCM2835_SMI_READ0;
    bcm2835_peri_write (paddr, defConfig);
    paddr = bcm2835_smi + BCM2835_SMI_WRITE0;
    bcm2835_peri_write (paddr, defConfig);
    paddr = bcm2835_smi + BCM2835_SMI_READ1;
    bcm2835_peri_write (paddr, defConfig);
    paddr = bcm2835_smi + BCM2835_SMI_WRITE1;
    bcm2835_peri_write (paddr, defConfig);
    paddr = bcm2835_smi + BCM2835_SMI_READ2;
    bcm2835_peri_write (paddr, defConfig);
    paddr = bcm2835_smi + BCM2835_SMI_WRITE2;
    bcm2835_peri_write (paddr, defConfig);
    paddr = bcm2835_smi + BCM2835_SMI_READ3;
    bcm2835_peri_write (paddr, defConfig);
    paddr = bcm2835_smi + BCM2835_SMI_WRITE3;
    bcm2835_peri_write (paddr, defConfig);

    return 1; // OK
}  /* bcm2835_smi_begin */
/* --------------------------------------------------- */

void bcm2835_smi_end(void)
{
    /* Set all the SMI pins back to input */
    bcm2835_gpio_fsel(2, BCM2835_GPIO_FSEL_INPT); /* SA3 */
    bcm2835_gpio_fsel(3, BCM2835_GPIO_FSEL_INPT); /* SA2 */
    bcm2835_gpio_fsel(4, BCM2835_GPIO_FSEL_INPT); /* SA1 */
    bcm2835_gpio_fsel(5, BCM2835_GPIO_FSEL_INPT); /* SA0 */
    bcm2835_gpio_fsel(6, BCM2835_GPIO_FSEL_INPT); /* SOE_N / SE */
    bcm2835_gpio_fsel(7, BCM2835_GPIO_FSEL_INPT); /* SWE_N / SRW_N */
    bcm2835_gpio_fsel(8, BCM2835_GPIO_FSEL_INPT); /* SD0 */
    bcm2835_gpio_fsel(9, BCM2835_GPIO_FSEL_INPT); /* SD1 */
    bcm2835_gpio_fsel(10, BCM2835_GPIO_FSEL_INPT); /* SD2 */
    bcm2835_gpio_fsel(11, BCM2835_GPIO_FSEL_INPT); /* SD3 */
    bcm2835_gpio_fsel(12, BCM2835_GPIO_FSEL_INPT); /* SD4 */
    bcm2835_gpio_fsel(13, BCM2835_GPIO_FSEL_INPT); /* SD5 */
    bcm2835_gpio_fsel(14, BCM2835_GPIO_FSEL_INPT); /* SD6 */
    bcm2835_gpio_fsel(15, BCM2835_GPIO_FSEL_INPT); /* SD7 */
}  /* bcm2835_smi_end */
/* --------------------------------------------------- */

void bcm2835_smi_set_timing (uint32_t smichannel, uint32_t readchannel,
                             uint32_t setupcycles, uint32_t strobecycles,
                             uint32_t holdcycles, uint32_t pacecycles)
{
    int RegOffset;
    volatile uint32_t* psmitiming;
    uint32_t RegValue;

    if (readchannel!=0)
    {
        switch (smichannel)
        {
            case 0 : RegOffset = BCM2835_SMI_READ0; break;
            case 1 : RegOffset = BCM2835_SMI_READ1; break;
            case 2 : RegOffset = BCM2835_SMI_READ2; break;
            case 3 : RegOffset = BCM2835_SMI_READ3; break;
            default : return;
        }
    }
    else
    {
        switch (smichannel)
        {
            case 0 : RegOffset = BCM2835_SMI_WRITE0; break;
            case 1 : RegOffset = BCM2835_SMI_WRITE1; break;
            case 2 : RegOffset = BCM2835_SMI_WRITE2; break;
            case 3 : RegOffset = BCM2835_SMI_WRITE3; break;
            default : return;
        }
    }

    /* Get current timing configuration of the slot */
    psmitiming = bcm2835_smi + RegOffset;
    RegValue = bcm2835_peri_read (psmitiming);
    /* Clear timing fields in register */
    RegValue &= ~(BCM2835_SMI_RW_SETUP_MSK|BCM2835_SMI_RW_HOLD_MSK|BCM2835_SMI_RW_PACE_MSK|BCM2835_SMI_RW_STROBE_MSK);
    /* Set timing values and write back to register */
    RegValue |= (setupcycles << BCM2835_SMI_RW_SETUP_LS)  |
                (strobecycles << BCM2835_SMI_RW_STROBE_LS) |
                (holdcycles << BCM2835_SMI_RW_HOLD_LS)   |
                (pacecycles << BCM2835_SMI_RW_PACE_LS);
    bcm2835_peri_write (psmitiming, RegValue);
}  /* bcm2835_set_smi_timing */
/* --------------------------------------------------- */

void bcm2835_smi_write(uint32_t smichannel, uint8_t data, uint32_t address)
{
    uint32_t status;
    volatile uint32_t* psmics = bcm2835_smi + BCM2835_SMI_DIRCS;
    volatile uint32_t* psmiaddr = bcm2835_smi + BCM2835_SMI_DIRADDR;
    volatile uint32_t* psmidata = bcm2835_smi + BCM2835_SMI_DIRDATA;

    /* Make sure we use a configuration that exists */
    if (smichannel>3) return;

    /* clear done bit if set */
    if (bcm2835_peri_read(psmics) & BCM2835_SMI_DIRCS_DONE)
    {
        bcm2835_peri_write (psmics, BCM2835_SMI_DIRCS_DONE);
    }

    /* Start write transfer */
    bcm2835_peri_write (psmiaddr, (smichannel<<BCM2835_SMI_DIRADRS_DEV_LS) | (address&BCM2835_SMI_DIRADRS_MSK));
    bcm2835_peri_write (psmidata, data);
    bcm2835_peri_write (psmics, BCM2835_SMI_DIRCS_WRITE|BCM2835_SMI_DIRCS_START|BCM2835_SMI_DIRCS_ENABLE);

    /* Wait until write cycle is finished */
    do {
        status = bcm2835_peri_read(psmics);
    } while ((status & BCM2835_SMI_DIRCS_DONE)==0);

    /* clear done bit */
    bcm2835_peri_write (psmics, BCM2835_SMI_DIRCS_DONE);
}  /* bcm2835_smi_write */
/* --------------------------------------------------- */

uint32_t bcm2835_smi_read (uint32_t smichannel, uint32_t address)
{
    uint32_t status;
    uint32_t data;
    volatile uint32_t* psmics = bcm2835_smi + BCM2835_SMI_DIRCS;
    volatile uint32_t* psmiaddr = bcm2835_smi + BCM2835_SMI_DIRADDR;
    volatile uint32_t* psmidata = bcm2835_smi + BCM2835_SMI_DIRDATA;

    /* Make sure we use a configuration that exists */
    if (smichannel>3) return 0;

    /* clear done bit if set */
    if (bcm2835_peri_read(psmics) & BCM2835_SMI_DIRCS_DONE)
    {
        bcm2835_peri_write (psmics, BCM2835_SMI_DIRCS_DONE);
    }

    /* Start read transfer */
    bcm2835_peri_write (psmiaddr, (smichannel<<BCM2835_SMI_DIRADRS_DEV_LS) | (address&BCM2835_SMI_DIRADRS_MSK));
    bcm2835_peri_write (psmics, BCM2835_SMI_DIRCS_START|BCM2835_SMI_DIRCS_ENABLE);

    /* Wait until read cycle is finished */
    do {
        status = bcm2835_peri_read(psmics);
    } while ((status & BCM2835_SMI_DIRCS_DONE)==0);

    /* Read data */
    data = bcm2835_peri_read (psmidata);

    /* clear done bit */
    bcm2835_peri_write (psmics, BCM2835_SMI_DIRCS_DONE);

    return data;
}  /* bcm2835_smi_read */
/* --------------------------------------------------- */

/* Read the System Timer Counter (64-bits) */
uint64_t bcm2835_st_read(void)
{
    volatile uint32_t* paddr;
    uint32_t hi, lo;
    uint64_t st;

    if (bcm2835_st==MAP_FAILED)
	return 0;

    paddr = bcm2835_st + BCM2835_ST_CHI/4;
    hi = bcm2835_peri_read(paddr);

    paddr = bcm2835_st + BCM2835_ST_CLO/4;
    lo = bcm2835_peri_read(paddr);
    
    paddr = bcm2835_st + BCM2835_ST_CHI/4;
    st = bcm2835_peri_read(paddr);
    
    /* Test for overflow */
    if (st == hi)
    {
        st <<= 32;
        st += lo;
    }
    else
    {
        st <<= 32;
        paddr = bcm2835_st + BCM2835_ST_CLO/4;
        st += bcm2835_peri_read(paddr);
    }
    return st;
}

/* Delays for the specified number of microseconds with offset */
void bcm2835_st_delay(uint64_t offset_micros, uint64_t micros)
{
    uint64_t compare = offset_micros + micros;

    while(bcm2835_st_read() < compare)
	;
}

/* PWM */

void bcm2835_pwm_set_clock(uint32_t divisor)
{
    if (   bcm2835_clk == MAP_FAILED
        || bcm2835_pwm == MAP_FAILED)
      return; /* bcm2835_init() failed or not root */
  
    /* From Gerts code */
    divisor &= 0xfff;
    /* Stop PWM clock */
    bcm2835_peri_write(bcm2835_clk + BCM2835_PWMCLK_CNTL, BCM2835_PWM_PASSWRD | 0x01);
    bcm2835_delay(110); /* Prevents clock going slow */
    /* Wait for the clock to be not busy */
    while ((bcm2835_peri_read(bcm2835_clk + BCM2835_PWMCLK_CNTL) & 0x80) != 0)
	bcm2835_delay(1); 
    /* set the clock divider and enable PWM clock */
    bcm2835_peri_write(bcm2835_clk + BCM2835_PWMCLK_DIV, BCM2835_PWM_PASSWRD | (divisor << 12));
    bcm2835_peri_write(bcm2835_clk + BCM2835_PWMCLK_CNTL, BCM2835_PWM_PASSWRD | 0x11); /* Source=osc and enable */
}

void bcm2835_pwm_set_mode(uint8_t channel, uint8_t markspace, uint8_t enabled)
{
  if (   bcm2835_clk == MAP_FAILED
       || bcm2835_pwm == MAP_FAILED)
    return; /* bcm2835_init() failed or not root */

  uint32_t control = bcm2835_peri_read(bcm2835_pwm + BCM2835_PWM_CONTROL);

  if (channel == 0)
    {
      if (markspace)
	control |= BCM2835_PWM0_MS_MODE;
      else
	control &= ~BCM2835_PWM0_MS_MODE;
      if (enabled)
	control |= BCM2835_PWM0_ENABLE;
      else
	control &= ~BCM2835_PWM0_ENABLE;
    }
  else if (channel == 1)
    {
      if (markspace)
	control |= BCM2835_PWM1_MS_MODE;
      else
	control &= ~BCM2835_PWM1_MS_MODE;
      if (enabled)
	control |= BCM2835_PWM1_ENABLE;
      else
	control &= ~BCM2835_PWM1_ENABLE;
    }

  /* If you use the barrier here, wierd things happen, and the commands dont work */
  bcm2835_peri_write_nb(bcm2835_pwm + BCM2835_PWM_CONTROL, control);
  /*  bcm2835_peri_write_nb(bcm2835_pwm + BCM2835_PWM_CONTROL, BCM2835_PWM0_ENABLE | BCM2835_PWM1_ENABLE | BCM2835_PWM0_MS_MODE | BCM2835_PWM1_MS_MODE); */

}

void bcm2835_pwm_set_range(uint8_t channel, uint32_t range)
{
  if (   bcm2835_clk == MAP_FAILED
       || bcm2835_pwm == MAP_FAILED)
    return; /* bcm2835_init() failed or not root */

  if (channel == 0)
      bcm2835_peri_write_nb(bcm2835_pwm + BCM2835_PWM0_RANGE, range);
  else if (channel == 1)
      bcm2835_peri_write_nb(bcm2835_pwm + BCM2835_PWM1_RANGE, range);
}

void bcm2835_pwm_set_data(uint8_t channel, uint32_t data)
{
  if (   bcm2835_clk == MAP_FAILED
       || bcm2835_pwm == MAP_FAILED)
    return; /* bcm2835_init() failed or not root */

  if (channel == 0)
      bcm2835_peri_write_nb(bcm2835_pwm + BCM2835_PWM0_DATA, data);
  else if (channel == 1)
      bcm2835_peri_write_nb(bcm2835_pwm + BCM2835_PWM1_DATA, data);
}

/* Allocate page-aligned memory. */
void *malloc_aligned(size_t size)
{
    void *mem;
    errno = posix_memalign(&mem, BCM2835_PAGE_SIZE, size);
    return (errno ? NULL : mem);
}

/* Map 'size' bytes starting at 'off' in file 'fd' to memory.
// Return mapped address on success, MAP_FAILED otherwise.
// On error print message.
*/
static void *mapmem(const char *msg, size_t size, int fd, off_t off)
{
    void *map = mmap(NULL, size, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, off);
    if (map == MAP_FAILED)
	fprintf(stderr, "bcm2835_init: %s mmap failed: %s\n", msg, strerror(errno));
    return map;
}

static void unmapmem(void **pmem, size_t size)
{
    if (*pmem == MAP_FAILED) return;
    munmap(*pmem, size);
    *pmem = MAP_FAILED;
}

/* Initialise this library. */
int bcm2835_init(void)
{
    int  memfd;
    int  ok;
    FILE *fp;

    if (debug) 
    {
        bcm2835_peripherals = (uint32_t*)BCM2835_PERI_BASE;

	bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
	bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
	bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
	bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
	bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
	bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4;
	bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4;
	bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
	bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
	bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;
        /* BEB */
	bcm2835_smi  = bcm2835_peripherals + BCM2835_SMI_BASE/4;


	return 1; /* Success */
    }

    /* Figure out the base and size of the peripheral address block
    // using the device-tree. Required for RPi2/3/4, optional for RPi 1
    */
    if ((fp = fopen(BMC2835_RPI2_DT_FILENAME , "rb")))
    {
        unsigned char buf[16];
        uint32_t base_address;
        uint32_t peri_size;
        if (fread(buf, 1, sizeof(buf), fp) >= 8)
        {
            base_address = (buf[4] << 24) |
              (buf[5] << 16) |
              (buf[6] << 8) |
              (buf[7] << 0);
            
            peri_size = (buf[8] << 24) |
              (buf[9] << 16) |
              (buf[10] << 8) |
              (buf[11] << 0);
            
            if (!base_address)
            {
                /* looks like RPI 4 */
                base_address = (buf[8] << 24) |
                      (buf[9] << 16) |
                      (buf[10] << 8) |
                      (buf[11] << 0);
                      
                peri_size = (buf[12] << 24) |
                (buf[13] << 16) |
                (buf[14] << 8) |
                (buf[15] << 0);
            }
            /* check for valid known range formats */
            if ((buf[0] == 0x7e) &&
                    (buf[1] == 0x00) &&
                    (buf[2] == 0x00) &&
                    (buf[3] == 0x00) &&
                    ((base_address == BCM2835_PERI_BASE) || (base_address == BCM2835_RPI2_PERI_BASE) || (base_address == BCM2835_RPI4_PERI_BASE)))
            {
                bcm2835_peripherals_base = (off_t)base_address;
                bcm2835_peripherals_size = (size_t)peri_size;
                if( base_address == BCM2835_RPI4_PERI_BASE )
                {
                    pud_type_rpi4 = 1;
                }
            }
        
        }
        
	fclose(fp);
    }
    /* else we are prob on RPi 1 with BCM2835, and use the hardwired defaults */

    /* Now get ready to map the peripherals block 
     * If we are not root, try for the new /dev/gpiomem interface and accept
     * the fact that we can only access GPIO
     * else try for the /dev/mem interface and get access to everything
     */
    memfd = -1;
    ok = 0;
    if (geteuid() == 0
#ifdef BCM2835_HAVE_LIBCAP
	|| bcm2835_has_capability(CAP_SYS_RAWIO)
#endif
	)
    {
      /* Open the master /dev/mem device */
      if ((memfd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) 
	{
	  fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n",
		  strerror(errno)) ;
	  goto exit;
	}
      
      /* Base of the peripherals block is mapped to VM */
      bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
      if (bcm2835_peripherals == MAP_FAILED) goto exit;
      
      /* Now compute the base addresses of various peripherals, 
      // which are at fixed offsets within the mapped peripherals block
      // Caution: bcm2835_peripherals is uint32_t*, so divide offsets by 4
      */
      bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
      bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
      bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
      bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
      bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
      bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4; /* I2C */
      bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4; /* I2C */
      bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
      bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
      bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;
      /* BEB */
      bcm2835_smi  = bcm2835_peripherals + BCM2835_SMI_BASE/4;


      ok = 1;
    }
    else
    {
      /* Not root, try /dev/gpiomem */
      /* Open the master /dev/mem device */
      if ((memfd = open("/dev/gpiomem", O_RDWR | O_SYNC) ) < 0) 
	{
	  fprintf(stderr, "bcm2835_init: Unable to open /dev/gpiomem: %s\n",
		  strerror(errno)) ;
	  goto exit;
	}
      
      /* Base of the peripherals block is mapped to VM */
      bcm2835_peripherals_base = 0;
      bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
      if (bcm2835_peripherals == MAP_FAILED) goto exit;
      bcm2835_gpio = bcm2835_peripherals;
      ok = 1;
    }

exit:
    if (memfd >= 0)
        close(memfd);

    if (!ok)
	bcm2835_close();

    return ok;
}

/* Close this library and deallocate everything */
int bcm2835_close(void)
{
    if (debug) return 1; /* Success */

    unmapmem((void**) &bcm2835_peripherals, bcm2835_peripherals_size);
    bcm2835_peripherals = MAP_FAILED;
    bcm2835_gpio = MAP_FAILED;
    bcm2835_pwm  = MAP_FAILED;
    bcm2835_clk  = MAP_FAILED;
    bcm2835_pads = MAP_FAILED;
    bcm2835_spi0 = MAP_FAILED;
    bcm2835_bsc0 = MAP_FAILED;
    bcm2835_bsc1 = MAP_FAILED;
    bcm2835_st   = MAP_FAILED;
    bcm2835_aux  = MAP_FAILED;
    bcm2835_spi1 = MAP_FAILED;
    /* BEB */
    bcm2835_smi = MAP_FAILED;

    return 1; /* Success */
}
// [END] Code region for BCM2835/RPi_RFID library

// [BEGIN] Code region for MRFC522/RPi_RFID library
void setSPIConfig() {
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);    // ~ 4 MHz
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
}

void PCD_Init() {
  if (bcm2835_gpio_lev(RSTPIN) == LOW) {	//The MFRC522 chip is in power down mode.
    bcm2835_gpio_write(RSTPIN, HIGH);		// Exit power down mode. This triggers a hard reset.
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
    delay(50);
  }
  else { // Perform a soft reset
    PCD_Reset();
  }

  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  PCD_WriteRegister2A(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  PCD_WriteRegister2A(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
  PCD_WriteRegister2A(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  PCD_WriteRegister2A(TReloadRegL, 0xE8);

  PCD_WriteRegister2A(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  PCD_WriteRegister2A(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}

int PICC_IsNewCardPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  byte result = PICC_RequestA(bufferATQA, &bufferSize);
  return (result == STATUS_OK || result == STATUS_COLLISION);
}

int PICC_ReadCardSerial() {
  byte result = PICC_Select(&uid, 0);
  return (result == STATUS_OK);
}

void PCD_Reset() {
  PCD_WriteRegister2A(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
  delay(50);
  // Wait for the PowerDown bit in CommandReg to be cleared
  while (PCD_ReadRegister1A(CommandReg) & (1<<4)) {
    // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
  }
}

void PCD_WriteRegister2A(byte reg, byte value) {
  char data[2];
  data[0] = reg & 0x7E;
  data[1] = value;
  bcm2835_spi_transfern(data, 2);
}

void PCD_WriteRegister3A(byte reg, byte count, byte *values) {
  for (byte index = 0; index < count; index++) {
  	PCD_WriteRegister2A(reg, values[index]);
	}
}

byte PCD_ReadRegister1A(byte reg) {
  char data[2];
  data[0] = 0x80 | ((reg) & 0x7E);
  bcm2835_spi_transfern(data,2);
  return (byte)data[1];
}

void PCD_ReadRegister4A(byte reg, byte count, byte *values, byte rxAlign) {
  if (count == 0) {
    return;
  }
  //Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
  byte address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index = 0;							// Index in values array.
  count--;								// One read is performed outside of the loop
  bcm2835_spi_transfer(address);
  while (index < count) {
    if (index == 0 && rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
      // Create bit mask for bit positions rxAlign..7
      byte mask = 0;
      for (byte i = rxAlign; i <= 7; i++) {
	mask |= (1 << i);
      }
      // Read value and tell that we want to read the same address again.
      byte value = bcm2835_spi_transfer(address);
      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    }
    else { // Normal case
      values[index] = bcm2835_spi_transfer(address);
    }
    index++;
  }
  values[index] = bcm2835_spi_transfer(0);			// Read the final byte. Send 0 to stop reading.
}

void PCD_AntennaOn() {
  byte value = PCD_ReadRegister1A(TxControlReg);
  if ((value & 0x03) != 0x03) {
    PCD_WriteRegister2A(TxControlReg, value | 0x03);
  }
}

byte PICC_RequestA(byte *bufferATQA, byte *bufferSize) {
  return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}

byte PICC_Select(Uid *uid, byte validBits) {
  int uidComplete;
  int selectDone;
  int useCascadeTag;
  byte cascadeLevel = 1;
  byte result;
  byte count;
  byte index;
  byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
  signed char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
  byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
  byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
  byte *responseBuffer;
  byte responseLength;

  // Description of buffer structure:
  //		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
  //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
  //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
  //		Byte 3: UID-data
  //		Byte 4: UID-data
  //		Byte 5: UID-data
  //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
  //		Byte 7: CRC_A
  //		Byte 8: CRC_A
  // The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
  //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
  //		========	=============	=====	=====	=====	=====
  //		 4 bytes		1			uid0	uid1	uid2	uid3
  //		 7 bytes		1			CT		uid0	uid1	uid2
  //						2			uid3	uid4	uid5	uid6
  //		10 bytes		1			CT		uid0	uid1	uid2
  //						2			CT		uid3	uid4	uid5
  //						3			uid6	uid7	uid8	uid9

  // Sanity checks
  if (validBits > 80) {
    return STATUS_INVALID;
  }

  // Prepare MFRC522
  PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = 0;
  while (!uidComplete) {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
    switch (cascadeLevel) {
    case 1:
      buffer[0] = PICC_CMD_SEL_CL1;
      uidIndex = 0;
      useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
      break;

    case 2:
      buffer[0] = PICC_CMD_SEL_CL2;
      uidIndex = 3;
      useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
      break;

    case 3:
      buffer[0] = PICC_CMD_SEL_CL3;
      uidIndex = 6;
      useCascadeTag = 0;						// Never used in CL3.
      break;

    default:
      return STATUS_INTERNAL_ERROR;
      break;
    }

    // How many UID bits are known in this Cascade Level?
    currentLevelKnownBits = validBits - (8 * uidIndex);
    if (currentLevelKnownBits < 0) {
      currentLevelKnownBits = 0;
    }
    // Copy the known bits from uid->uidByte[] to buffer[]
    index = 2; // destination index in buffer[]
    if (useCascadeTag) {
      buffer[index++] = PICC_CMD_CT;
    }
    byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytesToCopy) {
      byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
      if (bytesToCopy > maxBytes) {
	bytesToCopy = maxBytes;
      }
      for (count = 0; count < bytesToCopy; count++) {
	buffer[index++] = uid->uidByte[uidIndex + count];
      }
    }
    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
    if (useCascadeTag) {
      currentLevelKnownBits += 8;
    }

    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
    selectDone = 0;
    while (!selectDone) {
      // Find out how many bits and bytes to send and receive.
      if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
	//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
	buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
	// Calculate BCC - Block Check Character
	buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
	if (result != STATUS_OK) {
	  return result;
	}
	txLastBits		= 0; // 0 => All 8 bits are valid.
	bufferUsed		= 9;
	// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
	responseBuffer	= &buffer[6];
	responseLength	= 3;
      }
      else { // This is an ANTICOLLISION.
	//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
	txLastBits		= currentLevelKnownBits % 8;
	count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
	index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
	buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
	bufferUsed		= index + (txLastBits ? 1 : 0);
	// Store response in the unused part of buffer
	responseBuffer	= &buffer[index];
	responseLength	= sizeof(buffer) - index;
      }

      // Set bit adjustments
      rxAlign = txLastBits;											// Having a seperate variable is overkill. But it makes the next line easier to read.
      PCD_WriteRegister2A(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

      // Transmit the buffer and receive the response.
      result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
      if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
	result = PCD_ReadRegister1A(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
	if (result & 0x20) { // CollPosNotValid
	  return STATUS_COLLISION; // Without a valid collision position we cannot continue
	}
	byte collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
	if (collisionPos == 0) {
	  collisionPos = 32;
	}
	if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
	  return STATUS_INTERNAL_ERROR;
	}
	// Choose the PICC with the bit set.
	currentLevelKnownBits = collisionPos;
	count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
	index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
	buffer[index]	|= (1 << count);
      }
      else if (result != STATUS_OK) {
	return result;
      }
      else { // STATUS_OK
	if (currentLevelKnownBits >= 32) { // This was a SELECT.
	  selectDone = 1; // No more anticollision
	  // We continue below outside the while.
	}
	else { // This was an ANTICOLLISION.
	  // We now have all 32 bits of the UID in this Cascade Level
	  currentLevelKnownBits = 32;
	  // Run loop again to do the SELECT.
	}
      }
    } // End of while (!selectDone)

    // We do not check the CBB - it was constructed by us above.

    // Copy the found UID bytes from buffer[] to uid->uidByte[]
    index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
    bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
    for (count = 0; count < bytesToCopy; count++) {
      uid->uidByte[uidIndex + count] = buffer[index++];
    }

    // Check response SAK (Select Acknowledge)
    if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
      return STATUS_ERROR;
    }
    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
    if (result != STATUS_OK) {
      return result;
    }
    if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
      return STATUS_CRC_WRONG;
    }
    if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
      cascadeLevel++;
    }
    else {
      uidComplete = 1;
      uid->sak = responseBuffer[0];
    }
  } // End of while (!uidComplete)

  // Set correct uid->size
  uid->size = 3 * cascadeLevel + 1;

  return STATUS_OK;
}

byte PCD_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, byte *validBits, byte rxAlign, int checkCRC) {
  byte waitIRq = 0x30;		// RxIRq and IdleIRq
  return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

byte PCD_CalculateCRC(byte *data, byte length, byte *result) {
  PCD_WriteRegister2A(CommandReg, PCD_Idle);		// Stop any active command.
  PCD_WriteRegister2A(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
  PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
  PCD_WriteRegister3A(FIFODataReg, length, data);	// Write data to the FIFO
  PCD_WriteRegister2A(CommandReg, PCD_CalcCRC);		// Start the calculation

  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
  word i = 5000;
  byte n;
  while (1) {
    n = PCD_ReadRegister1A(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    if (n & 0x04) {						// CRCIRq bit set - calculation done
      break;
    }
    if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
      return STATUS_TIMEOUT;
    }
  }
  PCD_WriteRegister2A(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.

  // Transfer the result from the registers to the result buffer
  result[0] = PCD_ReadRegister1A(CRCResultRegL);
  result[1] = PCD_ReadRegister1A(CRCResultRegH);
  return STATUS_OK;
}

void PCD_SetRegisterBitMask(byte reg, byte mask) {
  byte tmp;
  tmp = PCD_ReadRegister1A(reg);
  PCD_WriteRegister2A(reg, tmp | mask);			// set bit mask
}

void PCD_ClearRegisterBitMask(byte reg, byte mask) {
  byte tmp;
  tmp = PCD_ReadRegister1A(reg);
  PCD_WriteRegister2A(reg, tmp & (~mask));		// clear bit mask
}

byte PCD_CommunicateWithPICC(byte command, byte waitIRq, byte *sendData, byte sendLen, byte *backData, byte *backLen, byte *validBits, byte rxAlign, int checkCRC) {
  byte n, _validBits;
  unsigned int i;

  // Prepare values for BitFramingReg
  byte txLastBits = validBits ? *validBits : 0;
  byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

  PCD_WriteRegister2A(CommandReg, PCD_Idle);			// Stop any active command.
  PCD_WriteRegister2A(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
  PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
  PCD_WriteRegister3A(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
  PCD_WriteRegister2A(BitFramingReg, bitFraming);		// Bit adjustments
  PCD_WriteRegister2A(CommandReg, command);				// Execute the command
  if (command == PCD_Transceive) {
    PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
  }

  // Wait for the command to complete.
  // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
  // Each iteration of the do-while-loop takes 17.86�s.
  i = 2000;
  while (1) {
    n = PCD_ReadRegister1A(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if (n & waitIRq) {					// One of the interrupts that signal success has been set.
      break;
    }
    if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
      return STATUS_TIMEOUT;
    }
    if (--i == 0) {						// The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
      return STATUS_TIMEOUT;
    }
  }

  // Stop now if any errors except collisions were detected.
  byte errorRegValue = PCD_ReadRegister1A(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
    return STATUS_ERROR;
  }

  // If the caller wants data back, get it from the MFRC522.
  if (backData && backLen) {
    n = PCD_ReadRegister1A(FIFOLevelReg);			// Number of bytes in the FIFO
    if (n > *backLen) {
      return STATUS_NO_ROOM;
    }
    *backLen = n;											// Number of bytes returned
    PCD_ReadRegister4A(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
    _validBits = PCD_ReadRegister1A(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if (validBits) {
      *validBits = _validBits;
    }
  }

  // Tell about collisions
  if (errorRegValue & 0x08) {		// CollErr
    return STATUS_COLLISION;
  }

  // Perform CRC_A validation if requested.
  if (backData && backLen && checkCRC) {
    // In this case a MIFARE Classic NAK is not OK.
    if (*backLen == 1 && _validBits == 4) {
      return STATUS_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if (*backLen < 2 || _validBits != 0) {
      return STATUS_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    byte controlBuffer[2];
    n = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
    if (n != STATUS_OK) {
      return n;
    }
    if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
      return STATUS_CRC_WRONG;
    }
  }

  return STATUS_OK;
}

byte PICC_REQA_or_WUPA(	byte command, byte *bufferATQA, byte *bufferSize){
  byte validBits;
  byte status;

  if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
    return STATUS_NO_ROOM;
  }
  PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
  validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, 0);
  if (status != STATUS_OK) {
    return status;
  }
  if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  }
  return STATUS_OK;
}
// [END] Code region for MRFC522/RPi_RFID library

// [BEGIN] Code region for Pluggable Authentication Module
/* expected hook */
PAM_EXTERN int pam_sm_setcred( pam_handle_t *pamh, int flags, int argc, const char **argv ) {
	return PAM_SUCCESS;
}

PAM_EXTERN int pam_sm_acct_mgmt(pam_handle_t *pamh, int flags, int argc, const char **argv) {
	printf("Acct mgmt\n");
	return PAM_SUCCESS;
}

/* expected hook, this is where custom stuff happens */
PAM_EXTERN int pam_sm_authenticate( pam_handle_t *pamh, int flags,int argc, const char **argv ) {

	//pretty much our main function
  	int cardfound = 0;
  	int checksum = 321;
  	int keysum = 0;
        int retval;
	int timeout = 0;

        const char* pUsername;
        retval = pam_get_user(pamh, &pUsername, "Username: ");

        printf("RFID Login from user: %s\n", pUsername);

  	if (!bcm2835_init()) {
    		printf("Failed to initialize. This tool needs root access, use sudo.\n");
  	}
  	bcm2835_gpio_fsel(RSTPIN, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_write(RSTPIN, LOW);
  	// Set SPI bus to work with MFRC522 chip.
  	setSPIConfig();

  	PCD_Init();

  	while(timeout < 11 && cardfound == 0){

    		// Look for a card
    		if(!PICC_IsNewCardPresent())
      			continue;
    		else
      			cardfound = 1;

    		if(!PICC_ReadCardSerial())
      			continue;

    		// Get UID
    		for (byte i = 0; i < uid.size; i++){
    			keysum += uid.uidByte[i];
    		}

    		delay(1000);
		timeout++;
	}


    	if(checksum == keysum)
      		return PAM_SUCCESS;

	return PAM_AUTH_ERR;
}
// [End] Code region for Pluggable Authentication Module

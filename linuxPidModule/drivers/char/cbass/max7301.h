#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <asm-arm/arch-at91/at91sam9260.h>
#include <asm-arm/arch-at91/at91_aic.h>
#include <asm/io.h>
#include <asm/signal.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <asm/mach/irq.h>

#include <linux/init.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91_spi.h>
#include <asm/arch/spi.h>
#include <linux/wait.h>

#include <asm/hardware.h>
#include <asm/arch/at91_pio.h>
#include <asm/arch/at91_tc.h>
#include <asm/arch/gpio.h>
#include <asm-arm/arch-at91/at91_adc.h>
#include <linux/time.h> 

#include <linux/dma-mapping.h> 
#include <linux/pci.h>
#include <linux/spi/spi.h>


#include "atmel_spi.h"

#define MAX7301_write 0x8000
#define MAX7301_PORT(a) a+0x24
#define MAX7301_GPIO_OUTPUT 0x1
#define MAX7301_GPIO_INPUT_NO_PULLUP 0x2
#define MAX7301_GPIO_INPUT_PULLUP	0x3
#define MAX7301_PIN(a,b,c,d,e)	((a<<8)&0xff00)|((b<<6)&(0x00c0))|((c<<4)&(0x0030))|((d<<2)&(0x000c))|((e)&(0x0003))

extern void __iomem *spi1_base;
extern spinlock_t spilock;

unsigned int max7301_parse(unsigned int command,unsigned int address);
unsigned int write_spi_max7301(unsigned int chip1,unsigned int chip2,unsigned int number);
unsigned int read_spi_max7301(unsigned int chip1,unsigned int chip2,unsigned int number,unsigned int *rxbuff);
unsigned int contactors(unsigned int command);
unsigned int clutchbrake(unsigned int command);

struct max7301_driver{
	unsigned int cs,modfdis;
	unsigned int ncpha,cpol,mstr,spien,spidis,ps,pcsdec,dlybcs,pcs,lastxfer,csaat,bits,scbr,dlybs,dlybct;
};

static const struct max7301_driver max7301 = { /* Operations for controling the device */
     
//define pin configuration for the ad667 driver!	

   	.cs		=AT91_PIN_PC6	,
	.ncpha 		=(1<<1),
	.cpol		=(0<<0), //change this! seems to work
	//.ncpha 		=(0<<1),
	//.cpol		=(1<<0), //change this! seems to work
	.mstr 		=AT91_SPI_MSTR,
	.spien		=AT91_SPI_SPIEN,
	.spidis		=AT91_SPI_SPIDIS,
	.ps		=AT91_SPI_PS_VARIABLE,
	//.ps		=AT91_SPI_PS_FIXED,
	.modfdis	=AT91_SPI_MODFDIS,
	.pcsdec		=(0<<2),
	.dlybcs		=(0<<24),
	.pcs		=0<<16,
	//.lastxfer	=AT91_SPI_LASTXFER,
	.lastxfer	=1<<24,
	.csaat		=(1<<3),
	.bits		=AT91_SPI_BITS_16,
	.scbr		=(10 <<  8),
	.dlybs		=(0 << 16),
	.dlybct		=(0 << 24),
	
	
};


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
#include "ad5362_def.h"


#define SCULL_DEBUG 0

#undef PDEBUG             /* undef it, just in case */ 
#ifdef SCULL_DEBUG 
	#define PDEBUG(fmt, args...) printk(fmt, ## args) 
// #ifdef _ _KERNEL_ _ 
//      /* This one if debugging is on, and kernel space */ 

// #define PDEBUG(fmt, args...) printk(fmt, ## args) 
// #else 
//      /* This one for user space */ 
// #define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args) 
//#endif 
#else 
#define PDEBUG(fmt, args...) /* not debugging: nothing */ 
#endif 




extern void __iomem *spi1_base;
extern void *txbuffer_coherent,*rxbuffer_coherent;
extern spinlock_t spilock;

int hello(void);
void init_ad5362(void);
int dac5362(char operation, unsigned int control_packet);
int dac5362_crc(char operation, unsigned int control_packet);
int spi_out_ad5362(unsigned int control_packet);
unsigned int spi_read_ad5362(unsigned int control_packet);
unsigned int spi_read_ad5362_crc(unsigned int control_packet);
int spi_out_ad5362_crc(unsigned int control_packet);

struct ad5362_driver{
	unsigned int clear,reset,busy,ldac,sdo,sdi,sck,sync,modfdis;
	unsigned int ncpha,cpol,mstr,spien,spidis,ps,pcsdec,dlybcs,pcs,lastxfer,csaat,bits,scbr,dlybs,dlybct;
	char TxBuffer[20],RxBuffer[20];
};





static const struct ad5362_driver ad5362 = { /* Operations for controling the device */
     
//define pin configuration for the ad667 driver!	

   	.clear       = AT91_PIN_PB8,
        .reset             =AT91_PIN_PB5 ,
        .busy            = AT91_PIN_PB4,
	.ldac		   =AT91_PIN_PB9,
	.sdo		   =AT91_PIN_PB0,	
	.sdi		= AT91_PIN_PB1	,
	.sck		=AT91_PIN_PB2	,
	.sync		=AT91_PIN_PC4	,
	.ncpha 		=(0<<1),
	.cpol		=(0<<0),
	.mstr 		=AT91_SPI_MSTR,
	.spien		=AT91_SPI_SPIEN,
	.spidis		=AT91_SPI_SPIDIS,
	.ps		=AT91_SPI_PS_VARIABLE,
	//.ps		=AT91_SPI_PS_FIXED,
	.modfdis	=AT91_SPI_MODFDIS,
	.pcsdec		=(0<<2),
	.dlybcs		=(100<<24),
	.pcs		=0<<16,
	//.lastxfer	=AT91_SPI_LASTXFER,
	.lastxfer	=1<<24,
	.csaat		=(0<<3),
	.bits		=AT91_SPI_BITS_8,
	.scbr		=(30 <<  8),
	.dlybs		=(40 << 16),
	.dlybct		=(0 << 24),
	.TxBuffer[0]=0xaa,
	.TxBuffer[1]=0x0f,
	.TxBuffer[2]=0x0f,
	.TxBuffer[3]=0x00,
	.TxBuffer[4]=0x00,
	.TxBuffer[5]=0x00,
	.RxBuffer[0]=0x01,
	.RxBuffer[1]=0x02,
	.RxBuffer[2]=0x00,
	.RxBuffer[3]=0x00,
	.RxBuffer[4]=0x00,
	.RxBuffer[5]=0x00,
	
};


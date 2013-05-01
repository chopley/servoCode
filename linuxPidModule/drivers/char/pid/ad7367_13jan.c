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
#include "ad5362.h"
#include "ad7367.h"

#include "pid.h"
#include "atmel_spi.h"


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

struct ad7367_driver{
	 unsigned int busy,addr,vdrive,cnvst,miso,cs,mosi,sck,modfdis;
	unsigned int ncpha,cpol,mstr,spien,spidis,ps,pcsdec,dlybcs,pcs,lastxfer,csaat,bits,scbr,dlybs,dlybct;
};

static const struct ad7367_driver ad7367 = { /* Operations for controling the device */
     
//define pin configuration for the ad667 driver!	

   	.busy      = AT91_PIN_PB10,
        .addr             =AT91_PIN_PB11 ,
        .vdrive            = AT91_PIN_PB16,
	.cnvst		   =AT91_PIN_PB17,
	.miso		=AT91_PIN_PB0,
	.mosi		=AT91_PIN_PB1,
	.sck		=AT91_PIN_PB2,
	.cs		=AT91_PIN_PB3,
	.ncpha 		=(0<<1),
	.cpol		=(1<<0),
	.mstr 		=AT91_SPI_MSTR,
	.spien		=AT91_SPI_SPIEN,
	.spidis		=AT91_SPI_SPIDIS,
	.modfdis	=AT91_SPI_MODFDIS,
	.ps		=AT91_SPI_PS_VARIABLE,
	//.ps		=AT91_SPI_PS_FIXED,
	.pcsdec		=(0<<2),
	.dlybcs		=(0<<24),
	.pcs		=0<<16,
	//.lastxfer	=AT91_SPI_LASTXFER,
	.lastxfer	=0<<24,
	.csaat		=(1<<3),
	.bits		=AT91_SPI_BITS_14,
	.scbr		=(20 <<  8),
	.dlybs		=(0 << 16),
	.dlybct		=(0 << 24),	
};

unsigned int spi_out_ad7367(unsigned short value,int *output){

	struct device *dev_ptr,dev;
	unsigned int *tx_buffer_coherent,*rx_buffer_coherent;
	unsigned char datacom[4];
	dma_addr_t dma_address1,dma_address2;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	unsigned int temp;
	unsigned int control_packet;
	unsigned int sign1,sign2;
	int txbuffer[100];
	int rxbuffer[100];
	signed short test[100];
	//map dma address so the PDC can work!
	at91_set_gpio_output(ad7367.cnvst,1);
	
	sign1=0;
	sign2=0;
	//printk("temp %d\n",temp);
	
	dev_ptr=&dev;
	 
// 	if (dma_set_mask(dev_ptr, 0x0ffffff))     
// 		printk ("DMA supported\n");
// 	else { 
//     	 /* We'll have to live without DMA */ 
//     		printk ("mydev: DMA not supported\n"); 
// 	}

	
	tx_buffer_coherent=txbuffer_coherent;
	rx_buffer_coherent=rxbuffer_coherent;



	at91_set_gpio_input(ad7367.busy,0);
 	at91_set_gpio_output(ad7367.addr,value);
	at91_set_gpio_output(ad7367.vdrive,1);
	at91_set_gpio_output(ad7367.cnvst,0);
	


	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	at91_set_gpio_output(ad7367.cnvst,1);
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad7367.lastxfer|ad7367.spien,spi1_base+AT91_SPI_CR);
	
	temp=0;
	
	temp = gpio_get_value(ad7367.busy);
	while(((temp))&&(count<500)) {
 		temp = gpio_get_value(ad7367.busy);
 		if(count>498){
 			PDEBUG("!BUSY %04x %u\n",temp,count);
 		//	udelay(1);
 			}
 		count++;
 		}
//	at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));
	
//	at91_set_gpio_output(ad7367.cs,0);
	

//	dac_out=0x0000ffff;
//	control_packet = 0x00c00000|(dac_out&(0x0000ffff));
	//control_packet = command;
	control_packet = 0x00800000;
	txbuffer[0]=0;
	txbuffer[1]=0;
	txbuffer[2]=0;
	txbuffer[3]=1;
	txbuffer[0]=ad7367.lastxfer|ad7367.pcs|0x0000;
	txbuffer[1]=ad7367.lastxfer|ad7367.pcs|0x0000;
	rxbuffer[0]=0;
	rxbuffer[1]=0;
	rxbuffer[2]=0;
	rxbuffer[3]=0;
	
	__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
	//__raw_writel((1<<9|1<<1),spi1_base+SPI_PTCR);
	__raw_writel(ad7367.lastxfer|ad7367.spien,spi1_base+AT91_SPI_CR);

 	dma_address1=dma_map_single(&dev, txbuffer, 100,DMA_TO_DEVICE);
 	dma_address2=dma_map_single(&dev, rxbuffer, 100,DMA_FROM_DEVICE);
	
	if(dma_address1==NULL||dma_address2==NULL){
		printk("DMA-ADDRESS ALLOCATION FAILED\n");
	}

//	printk("DMA address %08x %08x\n",dma_address1,dma_address2);
	
	//disable PDC controller
 	
	at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));	
	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
//	at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));	
	//at91_sys_write(AT91_PIOB+PIO_PER,(1<<0)|(1<<1)|(1<<2)|(1<<3));	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	__raw_writel(ad7367.lastxfer,spi1_base+AT91_SPI_CR);
	__raw_writel(ad7367.mstr|ad7367.ps|ad7367.pcsdec|ad7367.dlybcs|ad7367.modfdis,spi1_base+AT91_SPI_MR);
	__raw_writel(ad7367.cpol|ad7367.ncpha|ad7367.csaat|ad7367.bits|ad7367.scbr|ad7367.dlybs|ad7367.dlybct,spi1_base+AT91_SPI_CSR(0));

	__raw_writel(dma_address2,spi1_base+SPI_RPR);	
	__raw_writel(dma_address1,spi1_base+SPI_TPR);
   	
	
	
	//at91_set_gpio_output(ad7367.cs,0);
	//temp = gpio_get_value(ad7367.miso);
	at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));
	temp=0;
	at91_set_gpio_input(ad7367.miso,1);
	at91_set_gpio_output(ad7367.cs,0);
	sign1 = gpio_get_value(ad7367.miso);
	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
	__raw_writel(0,spi1_base+SPI_TNCR);
	__raw_writel(0,spi1_base+SPI_RNCR);	
	__raw_writel(1,spi1_base+SPI_RCR);
	__raw_writel(1,spi1_base+SPI_TCR);
	
	
	//printk("MISO %d\n",temp);
	

//	__raw_writel(txbuffer[0],spi1_base+AT91_SPI_TDR);

	status = 0x0000;
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	//PDEBUG("count SPI1_SR %08x\n",status);
	//NOTE wait for transmit to complete with an error message if it doesn't
	

	count = 0;
	while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		count++;
		if(count>498){
			printk("ERROR: SPI TRANSMIT TIME OUT1- NO TRANSFER\n");
			PDEBUG("count %lu SPI1_SR %08x\n",count,status);
			}
		if(count==499){
			//reset
			control.error++;
			__raw_writel(1<<7,spi1_base+AT91_SPI_CR);
			
		}
		if(control.error!=0){
			printk("AD7367 SPI1 ERROR %d\n",control.error);
			//mdelay(1000);
			control.error=0;
		}
		
//		ndelay(10);
	}
// 	at91_sys_write(AT91_PIOB+PIO_PER,(1<<0));
 	at91_set_gpio_input(ad7367.miso,1);
// 	//at91_set_gpio_output(ad7367.cs,0);
 	sign2 = gpio_get_value(ad7367.miso);


	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
	__raw_writel(0,spi1_base+SPI_TNCR);
	__raw_writel(0,spi1_base+SPI_RNCR);	
	__raw_writel(1,spi1_base+SPI_RCR);
	__raw_writel(1,spi1_base+SPI_TCR);
	
	status = 0x0000;
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	count = 0;
	while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		count++;
		if(count>498){
			printk("ERROR: SPI TRANSMIT TIME OUT2- NO TRANSFER\n");
			PDEBUG("count %lu SPI1_SR %08x\n",count,status);
			}
		if(count==499){
			//reset
			control.error++;
			__raw_writel(1<<7,spi1_base+AT91_SPI_CR);
			
		}
		if(control.error!=0){
			printk("AD7367 SPI2 ERROR %d\n",control.error);
			mdelay(1000);
			control.error=0;
		}
		
//		ndelay(10);
	}
//	udelay(10);
	
	
//	rxbuffer[0] = __raw_readl(spi1_base+AT91_SPI_RDR); 
	dma_unmap_single(&dev, dma_address1, 100,DMA_TO_DEVICE);
	dma_unmap_single(&dev, dma_address2, 100,DMA_FROM_DEVICE);
	at91_set_gpio_output(ad7367.cnvst,1);
	at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));
	at91_set_gpio_output(ad7367.cs,1);
	//udelay(30);
	
	test[0]=rxbuffer[0];
	test[1]=rxbuffer[1];
//	printk("%08x %08x \n",test[0],test[1]);
	//printk("%08x %08x %08x %08x %08x %08x %08x %08x\n",txbuffer[0],txbuffer[1],txbuffer[2],rxbuffer[0],rxbuffer[1],rxbuffer[2],status,test[0]);
//	printk("MISO %d\n",temp);

	
	test[0]=test[0]<<1|(sign1<<15);
	test[1]=test[1]<<1|(sign2<<15);
//	test[0]=test[0];
//	test[1]=test[1];
	*(output+0)=(int)test[0];
	*(output+1)=(int)test[1];
	//printk("sign1 %d sign2 %d rxbuffer[0] %d rxbuffer[1] %d\n",sign1,sign2,test[0],test[1]);
	return 0;
}

int spi_out_ad7367_no_dma(unsigned short value){

	struct device *dev_ptr,dev;
	unsigned int *tx_buffer_coherent,*rx_buffer_coherent;
	unsigned char datacom[4];
	dma_addr_t dma_address1,dma_address2;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	unsigned int temp;
	unsigned int control_packet;
	unsigned int txbuffer[100];
	unsigned int rxbuffer[100];
	//map dma address so the PDC can work!
	at91_set_gpio_output(ad7367.cnvst,1);


	
	

	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2)|(1<<3));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2)|(1<<3));	
	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	__raw_writel(ad7367.lastxfer,spi1_base+AT91_SPI_CR);
	__raw_writel(ad7367.mstr|ad7367.ps|ad7367.pcsdec|ad7367.dlybcs,spi1_base+AT91_SPI_MR);
	__raw_writel(ad7367.cpol|ad7367.ncpha|ad7367.csaat|ad7367.bits|ad7367.scbr|ad7367.dlybs|ad7367.dlybct,spi1_base+AT91_SPI_CSR(0));

	at91_set_gpio_input(ad7367.busy,0);
 	at91_set_gpio_output(ad7367.addr,value);
	at91_set_gpio_output(ad7367.vdrive,1);
	at91_set_gpio_output(ad7367.cnvst,0);



	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	at91_set_gpio_output(ad7367.cnvst,1);
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad7367.lastxfer|ad7367.spien,spi1_base+AT91_SPI_CR);
	
	temp=0;
	
	temp = gpio_get_value(ad7367.busy);
	while(((temp))&&(count<500)) {
 		temp = gpio_get_value(ad7367.busy);
 		if(count>498){
 			PDEBUG("!BUSY %04x %u\n",temp,count);
 			udelay(1);
 			}
 		count++;
 		}

	
//	dac_out=0x0000ffff;
//	control_packet = 0x00c00000|(dac_out&(0x0000ffff));
	//control_packet = command;
	control_packet = 0x00800000;
	txbuffer[0]=1;
	txbuffer[1]=1;
	txbuffer[2]=1;
	txbuffer[3]=1;
	txbuffer[0]=ad7367.lastxfer|ad7367.pcs|0x0000;
	txbuffer[1]=ad7367.lastxfer|ad7367.pcs|0x0000;
	rxbuffer[0]=1;
	rxbuffer[1]=1;
	rxbuffer[2]=1;
	rxbuffer[3]=1;
	
	//__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
	__raw_writel((1<<9|1<<1),spi1_base+SPI_PTCR);
	__raw_writel(ad7367.lastxfer|ad7367.spien,spi1_base+AT91_SPI_CR);

 	//dma_address1=dma_map_single(&dev, txbuffer, 100,DMA_TO_DEVICE);
 //	dma_address2=dma_map_single(&dev, rxbuffer, 100,DMA_FROM_DEVICE);
	
//	if(dma_address1==NULL||dma_address2==NULL){
//		printk("DMA-ADDRESS ALLOCATION FAILED\n");
//	}

//	printk("DMA address %08x %08x\n",dma_address1,dma_address2);
	
	//disable PDC controller
 	

//	__raw_writel(dma_address2,spi1_base+SPI_RPR);	
//	__raw_writel(dma_address1,spi1_base+SPI_TPR);
   	
	
//	__raw_writel(0,spi1_base+SPI_TNCR);
//	__raw_writel(0,spi1_base+SPI_RNCR);	
//	__raw_writel(1,spi1_base+SPI_RCR);
//	__raw_writel(1,spi1_base+SPI_TCR);
	
	

	__raw_writel(txbuffer[0],spi1_base+AT91_SPI_TDR);

	status = 0x0000;
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	//PDEBUG("count SPI1_SR %08x\n",status);
	//NOTE wait for transmit to complete with an error message if it doesn't
	

	count = 0;
	while( (!(status&AT91_SPI_RDRF)) && (count<500)   ) {
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		count++;
		if(count>498){
			printk("ERROR: SPI TRANSMIT TIME OUT- NO TRANSFER\n");
			PDEBUG("count %lu SPI1_SR %08x\n",count,status);
			}
		
		ndelay(10);
	}

	
//	udelay(10);
	
	
	rxbuffer[0] = __raw_readl(spi1_base+AT91_SPI_RDR); 
//	dma_unmap_single(&dev, dma_address1, 100,DMA_TO_DEVICE);
//	dma_unmap_single(&dev, dma_address2, 100,DMA_FROM_DEVICE);
	at91_set_gpio_output(ad7367.cnvst,1);
	//udelay(30);
//	printk("%08x %08x %08x %08x %08x %08x\n",txbuffer[0],txbuffer[1],txbuffer[2],rxbuffer[0],rxbuffer[1],rxbuffer[2]);
	
	
	return 0;
}

unsigned int spi_out_ad7367_bit_banging(unsigned short value,int *output){

	
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	unsigned int temp;
	int out1;
	int out2;
	
	
	int txbuffer[100];
	int rxbuffer[100];



	count =0;
	at91_set_gpio_output(ad7367.cnvst,1);

	at91_sys_write(AT91_PIOB+PIO_PER,(1<<10)|(1<<11)|(1<<16)|(1<<17));	

	
	at91_set_gpio_input(ad7367.busy,0);
 	at91_set_gpio_output(ad7367.addr,value);
	at91_set_gpio_output(ad7367.vdrive,1);
	at91_set_gpio_output(ad7367.cnvst,0);
	at91_set_gpio_output(ad7367.sck,1);
	
	temp=0;
	
	temp = gpio_get_value(ad7367.busy);
	while(((temp))&&(count<500)) {
 		temp = gpio_get_value(ad7367.busy);
 		if(count>498){
 			PDEBUG("!BUSY %04x %u\n",temp,count);
 		//	udelay(1);
 			}
 		count++;
 		}
	at91_set_gpio_output(ad7367.cnvst,1);

	at91_set_gpio_input(ad7367.miso,1);
	at91_set_gpio_output(ad7367.cs,0);
	temp = gpio_get_value(ad7367.miso);
	//printk("temp1 %08x\n",temp);
	out1=0;
	out1=temp&(0x00000001);
	out1=out1<<1;
	
	count=0;
	while(count<14){
		temp = (spi_bitbanging_ad7367());
		out1=out1|(temp);
		out1 = out1<<1;
		count++;
	}
	
	out2=0;
	//at91_set_gpio_output(ad7367.sck,0);
	temp = gpio_get_value(ad7367.miso);
	out2=temp&(0x00000001);
	out2=out2<<1;
	//at91_set_gpio_output(ad7367.sck,1);

	count=0;
	while(count<14){
		temp = (spi_bitbanging_ad7367());
		out2=out2|(temp);
		out2 = out2<<1;
		count++;
	}
	//out1=55;
	//printk("out1 %08x\n",out1);
	at91_set_gpio_output(ad7367.cs,1);
	*(output) = out1;
	*(output+1) = out2;
	
	return 0;
}

int spi_bitbanging_ad7367(void){
	unsigned int temp;
	temp=0;
	
	at91_set_gpio_output(ad7367.sck,0);
	temp = gpio_get_value(ad7367.miso);
	//udelay(1);
	at91_set_gpio_output(ad7367.sck,1);
	//printk("temp %08x\n",temp);
	return temp;


}
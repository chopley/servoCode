#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include "max7301.h"




unsigned int max7301_parse(unsigned int command,unsigned int address){

	return (command<<8)|address;


}


unsigned int write_spi_max7301(unsigned int chip1,unsigned int chip2,unsigned int number){

	
	struct device *dev_ptr,dev;
	dma_addr_t dma_address1,dma_address2;
	unsigned int datacom[5];
	unsigned int return_val;
	int i;
	unsigned int buffer1[100];
	unsigned int buffer2[100];
	unsigned long status=0x0000;
	unsigned long count = 0x0000;	
	unsigned long irq_state;
	
	printk("datacom in write 7301 %08x 2 %08x\n",chip1,chip2);
	datacom[0]=(chip2)&0x0000ffff;
	datacom[1]=(chip1)&0x0000ffff;
	printk("datacom in write 7301 %08x 2 %08x\n",datacom[0],datacom[1]);

	buffer1[0]=max7301.lastxfer|max7301.pcs|datacom[0];
	buffer1[1]=max7301.lastxfer|max7301.pcs|datacom[1];
	buffer1[2]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer1[3]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer1[4]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer1[5]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer1[6]=max7301.lastxfer|max7301.pcs|0x0000;
	//printk("buffer 1 %04x 2 %04x\n",buffer1[0],buffer1[1]);

	//control_packet = 0x00000000;
	buffer2[0]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[1]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[2]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[3]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[4]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[5]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[6]=max7301.lastxfer|max7301.pcs|0x0000;	


	
	//NOTE load a pointer to the data
	//__raw_writel(dma_address_rx,spi1_base+SPI_TNPR);
	dev_ptr=&dev;
	dma_address1=dma_map_single(dev_ptr, buffer1, 100,DMA_TO_DEVICE);
	dma_address2=dma_map_single(dev_ptr, buffer2, 100,DMA_FROM_DEVICE);
		

	if((void *)dma_address1==NULL||(void *)dma_address2==NULL){
		printk("MAX7301 DMA-ADDRESS ALLOCATION FAILED\n");
	}
	//lock the SPI registers so that only this process has access until released.
	spin_lock_irqsave(&spilock,irq_state);
	//	printk("Number %d \n",number);
		//disable gpio if enabled
		at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
		//enable spi function of the pins
		at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
		
		//enable clock for spi
		at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
		__raw_writel(AT91_SPI_SPIDIS|(1<<7)|max7301.lastxfer,spi1_base+AT91_SPI_CR);
		__raw_writel(AT91_SPI_SPIEN|max7301.lastxfer,spi1_base+AT91_SPI_CR);
		__raw_writel(max7301.mstr|max7301.ps|max7301.pcsdec|max7301.dlybcs|max7301.modfdis,spi1_base+AT91_SPI_MR);
		__raw_writel(max7301.cpol|max7301.ncpha|max7301.csaat|max7301.bits|max7301.scbr|max7301.dlybs|max7301.dlybct,spi1_base+AT91_SPI_CSR(0));
		__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
	
	
		
	
		
		__raw_writel(dma_address1,spi1_base+SPI_TPR);
		__raw_writel(dma_address2,spi1_base+SPI_RPR);	
		//__raw_writel(dma_address3,spi1_base+SPI_TNPR);
		//__raw_writel(dma_address4,spi1_base+SPI_RNPR);
		at91_set_gpio_output(max7301.cs,0);
	//	udelay(1000);
		__raw_writel(2,spi1_base+SPI_RCR);
		__raw_writel(2,spi1_base+SPI_TCR);
		__raw_writel(0,spi1_base+SPI_TNCR);	
		__raw_writel(0,spi1_base+SPI_RNCR);
		
		//__raw_writel(3,spi1_base+SPI_TNCR);
		//__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);
	
	
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		//PDEBUG("count SPI1_SR %08x\n",status);
		//NOTE wait for transmit to complete with an error message if it doesn't
		
	//	udelay(1);
		count = 0;
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: MAX7301 SPI TRANSMIT TIME OUT- NO TRANSFER\n");
				//PDEBUG("count %lu SPI1_SR %08lx\n",count,status);
				}
			
			//udelay(1);
		}
	//----------------------------------------------------
		//udelay(1000);
		at91_set_gpio_output(max7301.cs,1);
		
	//	usleep(1);
	//	at91_set_gpio_output(max7301.cs,1);
	spin_unlock_irqrestore(&spilock,irq_state);

	dma_unmap_single(dev_ptr, dma_address1, 100,DMA_TO_DEVICE);
	dma_unmap_single(dev_ptr, dma_address2, 100,DMA_FROM_DEVICE);
	
//	printk("WRITE MAX7301\n");



//	return_val=(((buffer4[3]<<16)&(0x00ff0000))|((buffer4[4]<<8)&(0x0000ff00))|(buffer4[5]&(0x000000ff)));
//	printk("%08x\n",return_val);
	printk("WRITE/READ MAX7301 %08x %08x %08x %08x %08x %08x\n",(buffer2[0]&0x00ff),(buffer2[1]&0x00ff),(buffer2[2]&0x00ff),(buffer2[3]&0x00ff),(buffer2[4]&0x00ff),(buffer2[5]&0x00ff));
	return_val=0;

	
	return return_val;

}

unsigned int read_spi_max7301(unsigned int chip1,unsigned int chip2,unsigned int number,unsigned int *rxbuff){


	
	struct device *dev_ptr,dev;
	dma_addr_t dma_address1,dma_address2;
	unsigned int datacom[5];
	unsigned int return_val;
	int i;
	unsigned int buffer1[100];
	unsigned int buffer2[100];
	unsigned long status=0x0000;
	unsigned long count = 0x0000;	
	unsigned long irq_state;
	
	number = 2;
//	chip2 = 0xd800;
//	chip1 = 0xd800;	
//	printk("Number %d \n",number);
	//printk("datacom in %08x 2 %08x\n",chip1,chip2);
	datacom[0]=(chip2)&0x0000ffff;
	datacom[1]=(chip1)&0x0000ffff;
	//printk("datacom 1 %04x 2 %04x\n",datacom[0],datacom[1]);
//	datacom[0]=(control_packet>>16)&0x000000ff;
//	datacom[0]=(control_packet>>24)&0x000000ff;
	
	buffer1[0]=max7301.lastxfer|max7301.pcs|datacom[1];
	buffer1[1]=max7301.lastxfer|max7301.pcs|datacom[0];
	buffer1[2]=max7301.lastxfer|max7301.pcs|datacom[1];
	buffer1[3]=max7301.lastxfer|max7301.pcs|datacom[0];
	buffer1[4]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer1[5]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer1[6]=max7301.lastxfer|max7301.pcs|0x0000;

	//control_packet = 0x00000000;
	buffer2[0]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[1]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[2]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[3]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[4]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[5]=max7301.lastxfer|max7301.pcs|0x0000;
	buffer2[6]=max7301.lastxfer|max7301.pcs|0x0000;	
	udelay(10);

 	
	//NOTE load a pointer to the data
	//__raw_writel(dma_address_rx,spi1_base+SPI_TNPR);
	dev_ptr=&dev;
	dma_address1=dma_map_single(dev_ptr, buffer1, 100,DMA_TO_DEVICE);
	dma_address2=dma_map_single(dev_ptr, buffer2, 100,DMA_FROM_DEVICE);
		

	if((void *)dma_address1==NULL||(void *)dma_address2==NULL){
		printk("MAX7301 DMA-ADDRESS ALLOCATION FAILED\n");
	}

	spin_lock_irqsave(&spilock,irq_state);
	
		//disable gpio if enabled
		at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));
		at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
		//enable spi function of the pins
		at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
		
		at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
		__raw_writel(AT91_SPI_SPIDIS|(1<<7)|max7301.lastxfer,spi1_base+AT91_SPI_CR);
		__raw_writel(AT91_SPI_SPIEN|max7301.lastxfer,spi1_base+AT91_SPI_CR);
		__raw_writel(max7301.mstr|max7301.ps|max7301.pcsdec|max7301.dlybcs|max7301.modfdis,spi1_base+AT91_SPI_MR);
		__raw_writel(max7301.cpol|max7301.ncpha|max7301.csaat|max7301.bits|max7301.scbr|max7301.dlybs|max7301.dlybct,spi1_base+AT91_SPI_CSR(0));
		__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);


		//enable clock for spi
		
		
	
		
		
		
		__raw_writel(dma_address1,spi1_base+SPI_TPR);
		__raw_writel(dma_address2,spi1_base+SPI_RPR);	
		//__raw_writel(dma_address3,spi1_base+SPI_TNPR);
		//__raw_writel(dma_address4,spi1_base+SPI_RNPR);
		at91_set_gpio_output(max7301.cs,0);
		udelay(1);
		__raw_writel(2,spi1_base+SPI_RCR);
		__raw_writel(2,spi1_base+SPI_TCR);
		__raw_writel(0,spi1_base+SPI_TNCR);	
		__raw_writel(0,spi1_base+SPI_RNCR);
		
		//__raw_writel(3,spi1_base+SPI_TNCR);
		//__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);
	
	
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		//PDEBUG("count SPI1_SR %08x\n",status);
		//NOTE wait for transmit to complete with an error message if it doesn't
		
		//udelay(1);
		count = 0;
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: MAX7301 SPI TRANSMIT TIME OUT- NO TRANSFER\n");
				//PDEBUG("count %lu SPI1_SR %08lx\n",count,status);
				}
			
			//udelay(1);
		}
	//----------------------------------------------------
		udelay(1);
		at91_set_gpio_output(max7301.cs,1);
		udelay(10);
		at91_set_gpio_output(max7301.cs,0);
		__raw_writel(2,spi1_base+SPI_RCR);
		__raw_writel(2,spi1_base+SPI_TCR);
		__raw_writel(0,spi1_base+SPI_TNCR);	
		__raw_writel(0,spi1_base+SPI_RNCR);
	
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		//PDEBUG("count SPI1_SR %08x\n",status);
		//NOTE wait for transmit to complete with an error message if it doesn't
		
		udelay(100);
		count = 0;
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: MAX7301 SPI TRANSMIT TIME OUT- NO TRANSFER\n");
				//PDEBUG("count %lu SPI1_SR %08lx\n",count,status);
				}
			
			//udelay(1);
		}
	
	
	
		at91_set_gpio_output(max7301.cs,1);
	spin_unlock_irqrestore(&spilock,irq_state);

	dma_unmap_single(dev_ptr, dma_address1, 100,DMA_TO_DEVICE);
	dma_unmap_single(dev_ptr, dma_address2, 100,DMA_FROM_DEVICE);




//	return_val=(((buffer4[3]<<16)&(0x00ff0000))|((buffer4[4]<<8)&(0x0000ff00))|(buffer4[5]&(0x000000ff)));
	printk("READ MAX7301 %08x %08x %08x %08x %08x %08x\n",(buffer2[0]&0x00ff),(buffer2[1]&0x00ff),(buffer2[2]&0x00ff),(buffer2[3]&0x00ff),(buffer2[4]&0x00ff),(buffer2[5]&0x00ff));
	*rxbuff = buffer2[3]&0x00ff;
	*(rxbuff+1) = buffer2[2]&0x00ff;

	return_val = 0;
	return return_val;

}

unsigned int contactors(unsigned int command){


}


unsigned int clutchbrake(unsigned int command){

}
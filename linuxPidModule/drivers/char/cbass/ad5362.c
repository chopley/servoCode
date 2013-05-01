#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include "ad5362.h"
//#include "crc_gen.h"
#define __NO_VERSION__   
/* Deal with CONFIG_MODVERSIONS */
// #if CONFIG_MODVERSIONS==1
// #define MODVERSIONS
// #include <linux/modversions.h>
// #endif      
extern spinlock_t spilock;

int hello(void){
		printk("hello in ad5362.c\n");
		return 1;
	
	}

void init_ad5362(void){
	unsigned short cbuffer[8];
	unsigned short mbuffer[8];
	unsigned int ret;
	unsigned int i;
	//reset the ad5362
	
	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOC+PIO_PER,(1<<4));	

	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
	__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs|ad5362.modfdis,spi1_base+AT91_SPI_MR);
	__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));

	__raw_writel((1<<7),spi1_base+AT91_SPI_CR);
 	
	at91_set_gpio_output(ad5362.reset,1);
 	at91_set_gpio_output(ad5362.clear,1);
	at91_set_gpio_output(ad5362.reset,1);
 	at91_set_gpio_output(ad5362.clear,1);
	udelay(1);
	//this allows the Busy pin to move up and down- i.e setting the reset and clear pins as inputs
	at91_set_gpio_output(ad5362.reset,0);
 	udelay(1);
	at91_set_gpio_output(ad5362.reset,1);
	udelay(500);

	for(i=0;i<=7;i++){
		cbuffer[i]=0x8000;
		mbuffer[i]=0xffff;
		
	}
	
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH1<<16)|((cbuffer[0])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH2<<16)|((cbuffer[1])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH3<<16)|((cbuffer[2])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH4<<16)|((cbuffer[3])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH5<<16)|((cbuffer[4])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH6<<16)|((cbuffer[5])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH7<<16)|((cbuffer[6])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(CREGISTER_WRITE<<20)|(CH8<<16)|((cbuffer[7])&(0x00ffff)));
	
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH1<<16)|((mbuffer[0])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH2<<16)|((mbuffer[1])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH3<<16)|((mbuffer[2])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH4<<16)|((mbuffer[3])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH5<<16)|((mbuffer[4])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH6<<16)|((mbuffer[5])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH7<<16)|((mbuffer[6])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH8<<16)|((mbuffer[7])&(0x00ffff)));
	printk("DAC UPDATED\n");

	//ret=dac5362_write(0,0);
// 	udelay(500);
// 	at91_set_gpio_output(ad5362.reset,1);
// 	at91_set_gpio_output(ad5362.clear,0);
// 	udelay(500);
// 	at91_set_gpio_output(ad5362.reset,0);
// 	at91_set_gpio_output(ad5362.clear,1);
// 	udelay(500);
// 	at91_set_gpio_output(ad5362.reset,1);
// 	at91_set_gpio_output(ad5362.clear,1);
// 	udelay(500);
}



int dac5362_crc(char operation, unsigned int control_packet){
	//adds CRC checking to the DAC data uploading
	unsigned long status=0x0000;
	unsigned long irq_state;
	unsigned int return_val;
	//unsigned long count = 0x0000;
	
	


//	PDEBUG("dac5362\n");
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	
	
	at91_set_gpio_output(ad5362.ldac,1);
	at91_set_gpio_output(ad5362.clear,1);


	//set to correct peripheral settings for spi
	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	
	

	//from at91_spi.h
// 	__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
// 	__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs|ad5362.modfdis,spi1_base+AT91_SPI_MR);
// 	__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));
// 	return_val = 0;
	if(operation==READ_AD5362){
		//return_val = spi_read_ad5362(control_packet);
		return_val = spi_read_ad5362_crc(control_packet);
	}
	else if (operation==WRITE_AD5362){
		return_val = spi_out_ad5362_crc(control_packet);
	}
	



//	spin_unlock_irqrestore(&spilock,irq_state);
	return return_val;
}



int spi_out_ad5362_crc(unsigned int control_packet){

	struct device *dev_ptr,dev;
	//crc_t crc;
	unsigned char datacom[4];
	dma_addr_t dma_address,dma_address_rx;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	unsigned int temp;
	//unsigned int control_packet;
	unsigned int buffer[100];
	unsigned int rxbuffer[100];
	unsigned long irq_state;


	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	at91_set_gpio_input(ad5362.busy,0);
	at91_set_gpio_output(ad5362.ldac,0);
	
//	dac_out=0x0000ffff;
//	control_packet = 0x00c00000|(dac_out&(0x0000ffff));
	//control_packet = command;
	//control_packet = 0x00800000;

	datacom[3]=(control_packet)&0x000000ff;
	datacom[2]=(control_packet>>8)&0x000000ff;
	datacom[1]=(control_packet>>16)&0x000000ff;
	datacom[0]=(control_packet>>24)&0x000000ff;

	
	//PDEBUG("control %08x \n",control_packet);
	buffer[0]=ad5362.lastxfer|ad5362.pcs|datacom[0];
	buffer[1]=ad5362.lastxfer|ad5362.pcs|datacom[1];
	buffer[2]=ad5362.lastxfer|ad5362.pcs|datacom[2];
	buffer[3]=ad5362.lastxfer|ad5362.pcs|datacom[3];
	buffer[4]=0;
	buffer[5]=0;

	control_packet = 0x00000000;
	rxbuffer[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|((0x000000ff)&(control_packet>>24));
	rxbuffer[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|((0x000000ff)&(control_packet>>16));
	rxbuffer[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|((0x000000ff)&(control_packet>>8));
	rxbuffer[3]=ad5362.lastxfer|(0<<18)|(0<<17)|(0<<16)|(0x00000000);
	rxbuffer[4]=ad5362.lastxfer|(0<<18)|(0<<17)|(0<<16)|(0x00000000);
	rxbuffer[5]=ad5362.lastxfer|(0<<18)|(0<<17)|(0<<16)|(0x00000000);


		
	//map dma address so the PDC can work!
	dev_ptr=&dev;
	dma_address=dma_map_single(dev_ptr, buffer, 100,DMA_TO_DEVICE);
	dma_address_rx=dma_map_single(dev_ptr, rxbuffer, 100,DMA_FROM_DEVICE);
	if((void *)dma_address==NULL||(void *)dma_address_rx==NULL){
		printk("AD5362 DMA-ADDRESS ALLOCATION FAILED\n");
	}	
	spin_lock_irqsave(&spilock,irq_state);
		__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
		__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs|ad5362.modfdis,spi1_base+AT91_SPI_MR);
		__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));
	
		at91_set_gpio_output(ad5362.sync,0);
		__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
		//NOTE load a pointer to the data
		__raw_writel(dma_address,spi1_base+SPI_TPR);
		__raw_writel(dma_address_rx,spi1_base+SPI_RPR);
		__raw_writel(4,spi1_base+SPI_TCR);
		__raw_writel(4,spi1_base+SPI_RCR);
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	
	
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		//PDEBUG("count SPI1_SR %08x\n",status);
		//NOTE wait for transmit to complete with an error message if it doesn't
		
	
		count =  0;
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: ad5362 SPI TRANSMIT TIME OUT- NO TRANSFER\n");
				PDEBUG("count %lu SPI1_SR %08lx\n",count,status);
				}
			if(count==499){
				//reset
				//control.error++;
				__raw_writel(1<<7,spi1_base+AT91_SPI_CR);
				
			}
			//if(control.error!=0){
			//	printk("DAC5362 SPI ERROR %d\n",control.error);
			//	mdelay(1000);
				//control.error=0;
			//}
		//	ndelay(10);
		}
		
		//udelay(10);
		at91_set_gpio_output(ad5362.sync,1);
	spin_unlock_irqrestore(&spilock,irq_state);
	temp=0;
	temp = gpio_get_value(ad5362.busy);
	count = 0;
	//wait for !busy to rise again- typical 1-5us depending on what is being updated
	while((!(temp))&&(count<500)) {
		temp = gpio_get_value(ad5362.busy);
		if(count>498){
			PDEBUG("ad5362 !BUSY %04x \n",temp);
		//	udelay(1);
			}
		count++;
		}
	//NOTE drop LDAC to signal that the outputs should be updated from the register
	
	at91_set_gpio_output(ad5362.ldac,0);
	
	
	//NOTE LDAC minimum pulse width low is 10ns- give it some extra time-
	//NOTE THIS COULD BE IMPROVED AS THE MINIMUM SLEEP TIME HERE IS OF THE ORDER OF 1uS-> I HAVE TRIED A NOOP LOOP BUT THIS DOESN'T APPEAR TO GIVE ENOUGH IMPROVEMENT TO WARRANT THROWING OUT THE GENERIC NATURE OF NDELAY()
	ndelay(11);
	at91_set_gpio_output(ad5362.ldac,1);


	dma_unmap_single(dev_ptr, dma_address_rx, 100,DMA_TO_DEVICE);
	dma_unmap_single(dev_ptr, dma_address, 100,DMA_FROM_DEVICE);

	return 0;
}



unsigned int spi_read_ad5362_crc(unsigned int control_packet){

	struct device *dev_ptr,dev;
	
	unsigned char datacom[4];
	dma_addr_t dma_address1,dma_address2;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	//unsigned int temp;
	unsigned int return_val;
	//unsigned int control_packet;
	int i;
	unsigned int buffer1[100];
	unsigned int buffer2[100];
	unsigned int buffer3[100];
	unsigned int buffer4[100];
	unsigned long irq_state;
	//unsigned int buffer5[100];
	//map dma address so the PDC can work!
	
	
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	
	at91_set_gpio_output(ad5362.ldac,1);
	at91_set_gpio_output(ad5362.clear,1);

	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	datacom[3]=(control_packet)&0x000000ff;
	datacom[2]=(control_packet>>8)&0x000000ff;
	datacom[1]=(control_packet>>16)&0x000000ff;
	datacom[0]=(control_packet>>24)&0x000000ff;

	buffer1[0]=ad5362.lastxfer|ad5362.pcs|datacom[0];
	buffer1[1]=ad5362.lastxfer|ad5362.pcs|datacom[1];
	buffer1[2]=ad5362.lastxfer|ad5362.pcs|datacom[2];
	buffer1[3]=ad5362.lastxfer|ad5362.pcs|datacom[3];
	buffer1[4]=ad5362.lastxfer|ad5362.pcs|0x0000;
	buffer1[5]=ad5362.lastxfer|ad5362.pcs|0x0000;




	buffer2[0]=0x0000;
	buffer2[1]=0x0000;
	buffer2[2]=0x0000;
	buffer2[3]=0x0000;
	buffer2[4]=0x0000;
	buffer2[5]=0x0000;
	spin_lock_irqsave(&spilock,irq_state);
		__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
		__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs|ad5362.modfdis,spi1_base+AT91_SPI_MR);
		__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));
	
	
		at91_set_gpio_input(ad5362.busy,0);
		
	//	datacom[0]=(control_packet>>24)&0x000000ff;
	
		
		//PDEBUG("control %08x \n",control_packet);
	//	buffer[0]=ad5362.lastxfer|ad5362.pcs|datacom[0];
	//	buffer[1]=ad5362.lastxfer|ad5362.pcs|datacom[1];
	//	buffer[2]=ad5362.lastxfer|ad5362.pcs|datacom[2];
	//	buffer[3]=ad5362.lastxfer|ad5362.pcs|datacom[3];
	//	buffer[4]=0;
	//	buffer[5]=0;
	
		//control_packet = 0x00054800;
		//value = 0x6800;
	//	datacom[0]=control_packet&(0x000000ff);
	//	datacom[1]=(control_packet>>8)&0x000000ff;
	//	datacom[2]=(control_packet>>16)&0x000000ff;
		//PDEBUG("control %08x \n",control_packet);
		
// 		buffer1[6]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[7]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[8]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[9]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[10]=ad5362.lastxfer|ad5362.pcs|0x0000;
	
		//control_packet = 0x00000000;
		
// 		buffer2[6]=0x0000;	
// 		buffer2[7]=0x0000;
// 		buffer2[8]=0x0000;
// 		buffer2[9]=0x0000;
// 		buffer2[10]=0x0000;
// 	
		__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
		//NOTE load a pointer to the data
		//__raw_writel(dma_address_rx,spi1_base+SPI_TNPR);
		dev_ptr=&dev;
		dma_address1=dma_map_single(dev_ptr, buffer1, 100,DMA_TO_DEVICE);
		dma_address2=dma_map_single(dev_ptr, buffer2, 100,DMA_FROM_DEVICE);
			
	
		if((void *)dma_address1==NULL||(void *)dma_address2==NULL){
			printk("AD5362 DMA-ADDRESS ALLOCATION FAILED\n");
		}
		
		__raw_writel(dma_address1,spi1_base+SPI_TPR);
		__raw_writel(dma_address2,spi1_base+SPI_RPR);	
		//__raw_writel(dma_address3,spi1_base+SPI_TNPR);
		//__raw_writel(dma_address4,spi1_base+SPI_RNPR);
		at91_set_gpio_output(ad5362.sync,0);
		
		__raw_writel(4,spi1_base+SPI_RCR);
		__raw_writel(4,spi1_base+SPI_TCR);
		__raw_writel(0,spi1_base+SPI_TNCR);	
		__raw_writel(0,spi1_base+SPI_RNCR);
		
		//__raw_writel(3,spi1_base+SPI_TNCR);
		//__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);
	
	
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		//PDEBUG("count SPI1_SR %08x\n",status);
		//NOTE wait for transmit to complete with an error message if it doesn't
		
		udelay(1);
		count = 0;
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: ad5362 SPI TRANSMIT TIME OUT- NO TRANSFER\n");
				PDEBUG("count %lu SPI1_SR %08lx\n",count,status);
				}
			
			//udelay(1);
		}
	//----------------------------------------------------
	//	udelay(10);
		at91_set_gpio_output(ad5362.sync,1);
		udelay(1);
		at91_set_gpio_output(ad5362.sync,0);
		__raw_writel(3,spi1_base+SPI_TCR);
		__raw_writel(3,spi1_base+SPI_RCR);
		__raw_writel(0,spi1_base+SPI_TNCR);	
		__raw_writel(0,spi1_base+SPI_RNCR);
	
	// 
	// 
		count = 0;
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500))  {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: ad5362 SPI receive TIME OUT- NO TRANSFER\n");
				PDEBUG("count %08lx SPI1_SR %08lx\n",count,status);
				}
			
			//udelay(1);
		}
	//	udelay(20);
		at91_set_gpio_output(ad5362.sync,1);
	spin_unlock_irqrestore(&spilock,irq_state);
	dma_unmap_single(dev_ptr, dma_address1, 100,DMA_TO_DEVICE);
	dma_unmap_single(dev_ptr, dma_address2, 100,DMA_FROM_DEVICE);


// 	for(i=0;i<10;i++){
// 		printk("i = %d %08x %08x \n",i,buffer1[i],buffer2[i]);
// 		buffer3[i]=0;
// 		buffer4[i]=0;
// 		buffer3[i]=buffer1[i];
// 		
// 		buffer4[i]=buffer2[i];
// 	}



	return_val=(((buffer2[3]<<16)&(0x00ff0000))|((buffer2[4]<<8)&(0x0000ff00))|(buffer2[5]&(0x000000ff)));
//	printk("%08x\n",return_val);


	
	return return_val;


}


unsigned int spi_read_ad5362(unsigned int control_packet){

	struct device *dev_ptr,dev;
	
	unsigned char datacom[4];
	dma_addr_t dma_address1,dma_address2;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	//unsigned int temp;
	unsigned int return_val;
	//unsigned int control_packet;
	int i;
	unsigned int buffer1[100];
	unsigned int buffer2[100];
	unsigned int buffer3[100];
	unsigned int buffer4[100];
	unsigned long irq_state;
	//unsigned int buffer5[100];
	//map dma address so the PDC can work!
	
	
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	
	at91_set_gpio_output(ad5362.ldac,1);
	at91_set_gpio_output(ad5362.clear,1);

	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	datacom[2]=(control_packet)&0x000000ff;
	datacom[1]=(control_packet>>8)&0x000000ff;
	datacom[0]=(control_packet>>16)&0x000000ff;

	buffer1[0]=ad5362.lastxfer|ad5362.pcs|datacom[0];
	buffer1[1]=ad5362.lastxfer|ad5362.pcs|datacom[1];
	buffer1[2]=ad5362.lastxfer|ad5362.pcs|datacom[2];
	buffer1[3]=ad5362.lastxfer|ad5362.pcs|0x0000;
	buffer1[4]=ad5362.lastxfer|ad5362.pcs|0x0000;
	buffer1[5]=ad5362.lastxfer|ad5362.pcs|0x0000;
	buffer2[0]=0x0000;
	buffer2[1]=0x0000;
	buffer2[2]=0x0000;
	buffer2[3]=0x0000;
	buffer2[4]=0x0000;
	buffer2[5]=0x0000;
	spin_lock_irqsave(&spilock,irq_state);
		__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
		__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs|ad5362.modfdis,spi1_base+AT91_SPI_MR);
		__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));
	
	
		at91_set_gpio_input(ad5362.busy,0);
		
	//	datacom[0]=(control_packet>>24)&0x000000ff;
	
		
		//PDEBUG("control %08x \n",control_packet);
	//	buffer[0]=ad5362.lastxfer|ad5362.pcs|datacom[0];
	//	buffer[1]=ad5362.lastxfer|ad5362.pcs|datacom[1];
	//	buffer[2]=ad5362.lastxfer|ad5362.pcs|datacom[2];
	//	buffer[3]=ad5362.lastxfer|ad5362.pcs|datacom[3];
	//	buffer[4]=0;
	//	buffer[5]=0;
	
		//control_packet = 0x00054800;
		//value = 0x6800;
	//	datacom[0]=control_packet&(0x000000ff);
	//	datacom[1]=(control_packet>>8)&0x000000ff;
	//	datacom[2]=(control_packet>>16)&0x000000ff;
		//PDEBUG("control %08x \n",control_packet);
		
// 		buffer1[6]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[7]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[8]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[9]=ad5362.lastxfer|ad5362.pcs|0x0000;
// 		buffer1[10]=ad5362.lastxfer|ad5362.pcs|0x0000;
	
		//control_packet = 0x00000000;
		
// 		buffer2[6]=0x0000;	
// 		buffer2[7]=0x0000;
// 		buffer2[8]=0x0000;
// 		buffer2[9]=0x0000;
// 		buffer2[10]=0x0000;
// 	
		__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
		//NOTE load a pointer to the data
		//__raw_writel(dma_address_rx,spi1_base+SPI_TNPR);
		dev_ptr=&dev;
		dma_address1=dma_map_single(dev_ptr, buffer1, 100,DMA_TO_DEVICE);
		dma_address2=dma_map_single(dev_ptr, buffer2, 100,DMA_FROM_DEVICE);
			
	
		if((void *)dma_address1==NULL||(void *)dma_address2==NULL){
			printk("AD5362 DMA-ADDRESS ALLOCATION FAILED\n");
		}
		
		__raw_writel(dma_address1,spi1_base+SPI_TPR);
		__raw_writel(dma_address2,spi1_base+SPI_RPR);	
		//__raw_writel(dma_address3,spi1_base+SPI_TNPR);
		//__raw_writel(dma_address4,spi1_base+SPI_RNPR);
		at91_set_gpio_output(ad5362.sync,0);
		
		__raw_writel(3,spi1_base+SPI_RCR);
		__raw_writel(3,spi1_base+SPI_TCR);
		__raw_writel(0,spi1_base+SPI_TNCR);	
		__raw_writel(0,spi1_base+SPI_RNCR);
		
		//__raw_writel(3,spi1_base+SPI_TNCR);
		//__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);
	
	
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		//PDEBUG("count SPI1_SR %08x\n",status);
		//NOTE wait for transmit to complete with an error message if it doesn't
		
		udelay(1);
		count = 0;
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: ad5362 SPI TRANSMIT TIME OUT- NO TRANSFER\n");
				PDEBUG("count %lu SPI1_SR %08lx\n",count,status);
				}
			
			//udelay(1);
		}
	//----------------------------------------------------
	//	udelay(10);
		at91_set_gpio_output(ad5362.sync,1);
		udelay(1);
		at91_set_gpio_output(ad5362.sync,0);
		__raw_writel(3,spi1_base+SPI_TCR);
		__raw_writel(3,spi1_base+SPI_RCR);
		__raw_writel(0,spi1_base+SPI_TNCR);	
		__raw_writel(0,spi1_base+SPI_RNCR);
	
	// 
	// 
		count = 0;
		status = 0x0000;
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		while( (!(status&AT91_SPI_RXBUFF)) && (count<500))  {
			status= __raw_readl(spi1_base+AT91_SPI_SR); 
			count++;
			if(count>498){
				printk("ERROR: ad5362 SPI receive TIME OUT- NO TRANSFER\n");
				PDEBUG("count %08lx SPI1_SR %08lx\n",count,status);
				}
			
			//udelay(1);
		}
	//	udelay(20);
		at91_set_gpio_output(ad5362.sync,1);
	spin_unlock_irqrestore(&spilock,irq_state);
	dma_unmap_single(dev_ptr, dma_address1, 100,DMA_TO_DEVICE);
	dma_unmap_single(dev_ptr, dma_address2, 100,DMA_FROM_DEVICE);


// 	for(i=0;i<10;i++){
// 		printk("i = %d %08x %08x \n",i,buffer1[i],buffer2[i]);
// 		buffer3[i]=0;
// 		buffer4[i]=0;
// 		buffer3[i]=buffer1[i];
// 		
// 		buffer4[i]=buffer2[i];
// 	}



	return_val=(((buffer2[3]<<16)&(0x00ff0000))|((buffer2[4]<<8)&(0x0000ff00))|(buffer2[5]&(0x000000ff)));
//	printk("%08x\n",return_val);


	
	return return_val;

}

int dac5362(char operation, unsigned int control_packet){

	unsigned long status=0x0000;
	//unsigned long count = 0x0000;
	
	


//	PDEBUG("dac5362\n");
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	
	
	at91_set_gpio_output(ad5362.ldac,1);
	at91_set_gpio_output(ad5362.clear,1);


	//set to correct peripheral settings for spi
	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2));	
	at91_sys_write(AT91_PIOB+PIO_PER,(1<<3));	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	
	

	//from at91_spi.h
	__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
	__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs|ad5362.modfdis,spi1_base+AT91_SPI_MR);
	__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));
	
	if(operation==READ_AD5362){
		spi_read_ad5362(control_packet);
	}
	else if (operation==WRITE_AD5362){
		spi_out_ad5362(control_packet);
	}
	



	
	return 1;
}

int spi_out_ad5362(unsigned int control_packet){

	struct device *dev_ptr,dev;
	//crc_t crc;
	unsigned char datacom[4];
	dma_addr_t dma_address,dma_address_rx;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	unsigned int temp;
	//unsigned int control_packet;
	unsigned int buffer[100];
	unsigned int rxbuffer[100];
	
	//map dma address so the PDC can work!


	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	at91_set_gpio_input(ad5362.busy,0);
	
	
//	dac_out=0x0000ffff;
//	control_packet = 0x00c00000|(dac_out&(0x0000ffff));
	//control_packet = command;
	//control_packet = 0x00800000;
	
	datacom[2]=control_packet&(0x000000ff);
	datacom[1]=(control_packet>>8)&0x000000ff;
	datacom[0]=(control_packet>>16)&0x000000ff;

	
	//PDEBUG("control %08x \n",control_packet);
	buffer[0]=ad5362.lastxfer|ad5362.pcs|datacom[0];
	buffer[1]=ad5362.lastxfer|ad5362.pcs|datacom[1];
	buffer[2]=ad5362.lastxfer|ad5362.pcs|datacom[2];
	buffer[3]=0;
	buffer[4]=0;
	buffer[5]=0;

	control_packet = 0x00000000;
	rxbuffer[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|((0x000000ff)&(control_packet>>16));
	rxbuffer[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|((0x000000ff)&(control_packet>>8));
	rxbuffer[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|((0x000000ff)&control_packet);
	rxbuffer[3]=ad5362.lastxfer|(0<<18)|(0<<17)|(0<<16)|(0x00000000);
	rxbuffer[4]=ad5362.lastxfer|(0<<18)|(0<<17)|(0<<16)|(0x00000000);
	rxbuffer[5]=ad5362.lastxfer|(0<<18)|(0<<17)|(0<<16)|(0x00000000);

	at91_set_gpio_output(ad5362.sync,0);
 	__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
	//NOTE load a pointer to the data


	dev_ptr=&dev;
	dma_address=dma_map_single(dev_ptr, buffer, 100,DMA_TO_DEVICE);
	dma_address_rx=dma_map_single(dev_ptr, rxbuffer, 100,DMA_FROM_DEVICE);
	if((void *)dma_address==NULL||(void *)dma_address_rx==NULL){
		printk("AD5362 DMA-ADDRESS ALLOCATION FAILED\n");
	}
   	__raw_writel(dma_address,spi1_base+SPI_TPR);
	__raw_writel(dma_address_rx,spi1_base+SPI_RPR);
	__raw_writel(3,spi1_base+SPI_TCR);
	__raw_writel(3,spi1_base+SPI_RCR);
	__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);


	status = 0x0000;
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	//PDEBUG("count SPI1_SR %08x\n",status);
	//NOTE wait for transmit to complete with an error message if it doesn't
	

	count =  0;
	while( (!(status&AT91_SPI_RXBUFF)) && (count<500)   ) {
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		count++;
		if(count>498){
			printk("ERROR: ad5362 SPI TRANSMIT TIME OUT- NO TRANSFER\n");
			PDEBUG("count %lu SPI1_SR %08lx\n",count,status);
			}
		if(count==499){
			//reset
			//control.error++;
			__raw_writel(1<<7,spi1_base+AT91_SPI_CR);
			
		}
		//if(control.error!=0){
		//	printk("DAC5362 SPI ERROR %d\n",control.error);
		//	mdelay(1000);
			//control.error=0;
		//}
	//	ndelay(10);
	}

	udelay(5);
	temp=0;
	temp = gpio_get_value(ad5362.busy);
	count = 0;
	//wait for !busy to rise again- typical 1-5us depending on what is being updated
	while((!(temp))&&(count<500)) {
		temp = gpio_get_value(ad5362.busy);
		if(count>498){
			PDEBUG("ad5362 !BUSY %04x %lu\n",temp,count);
		//	udelay(1);
			}
		count++;
		}
	//NOTE drop LDAC to signal that the outputs should be updated from the register
	
	at91_set_gpio_output(ad5362.ldac,0);
	at91_set_gpio_output(ad5362.sync,1);
	
	//NOTE LDAC minimum pulse width low is 10ns- give it some extra time-
	//NOTE THIS COULD BE IMPROVED AS THE MINIMUM SLEEP TIME HERE IS OF THE ORDER OF 1uS-> I HAVE TRIED A NOOP LOOP BUT THIS DOESN'T APPEAR TO GIVE ENOUGH IMPROVEMENT TO WARRANT THROWING OUT THE GENERIC NATURE OF NDELAY()
	ndelay(11);
	at91_set_gpio_output(ad5362.ldac,1);


	dma_unmap_single(dev_ptr, dma_address_rx, 100,DMA_TO_DEVICE);
	dma_unmap_single(dev_ptr, dma_address, 100,DMA_FROM_DEVICE);

	return 0;
}


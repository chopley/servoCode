
//make ARCH=arm CROSS_COMPILE=arm-linux- INSTALL_MOD_PATH=/media/embedded modules

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
//#include <sys/ioctl.h>

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

#define SLOW_INT 63934


static unsigned int irq;
static unsigned int irq_PC15_1PPS;

static int Device_Open = 0;


/* Deal with CONFIG_MODVERSIONS */

#if CONFIG_MODVERSIONS==1
#define MODVERSIONS
#include <linux/modversions.h>
#endif 

static DECLARE_WAIT_QUEUE_HEAD(wq);
//#define PDEBUG(fmt, args...) printk(fmt, ## args) 

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

//This defines whether the kernel module will even attempt to write to the hardware
#define hardware_connected 1
//This turns off the interrupt- note that using the read() function to communicate from Userspace requires the interrupt as implemented here, so you can only use ioctl() to communicate if the interrupt is turned off. Also note that with the interrupt turned on there are very rapid, periodic sampling of the encoder and tachometer values, which is why this option is provided (if these appear to be causing an issue during SPI testing they can be turned off relatively easily)
#define interrupt_enable 1


#define SUCCESS 0






void __iomem *timer0_base;
void __iomem *adc_base;
void __iomem *spi1_base;
void __iomem *dmabuf;


static irqreturn_t pps_irq(int irq, void *_cf, struct pt_regs *r)
{	

		struct timeval time_struct;
//		at91_sys_read(AT91_AIC_IVR);//need to read this at the start	
		do_gettimeofday(&(time_struct));
		printk(KERN_ALERT "1PPS  interrupt %ld %ld \n",time_struct.tv_sec,time_struct.tv_usec);
//		at91_sys_write(AT91_AIC_EOICR,0xffffffff);//indicates the interrupt has been handled
	return IRQ_HANDLED;
}

/*this interrupt is the heart beat of the control loop- basically updates values in the pid control structure.*/
static irqreturn_t pid_irq(int irq, void *_cf, struct pt_regs *r)
{
				
				
//		at91_sys_read(AT91_AIC_IVR);//need to read this at the start	
		printk(KERN_ALERT "handling interrupt\n");
		at91_sys_write(AT91_AIC_IECR,1<<AT91SAM9260_ID_TC0|at91_sys_read(AT91_AIC_IMR));
		__raw_writel(AT91_TC_TIMER_CLOCK4|AT91_TC_CPCTRG|AT91_TC_WAVESEL_UP_AUTO|AT91_TC_WAVE,timer0_base+AT91_TC_CMR);
		//this should try keep the tick interval accurate- not quite ticking at a round number of uSeconds correctly but not bad.
		__raw_writel(1900,timer0_base+AT91_TC_RC);
		__raw_writel(AT91_TC_CPCS,timer0_base+AT91_TC_IER);
		__raw_writel(AT91_TC_CLKEN,timer0_base+AT91_TC_CCR);	
		__raw_writel(AT91_TC_SYNC,timer0_base+AT91_TC_BCR);	
		at91_sys_write(AT91_AIC_EOICR,0xffffffff);	
		

		__raw_writel(AT91_TC_TIMER_CLOCK4|AT91_TC_CPCTRG|AT91_TC_WAVESEL_UP_AUTO|AT91_TC_WAVE,timer0_base+AT91_TC_CMR);	
		__raw_writel(SLOW_INT,timer0_base+AT91_TC_RC);
		__raw_writel(AT91_TC_CPCS,timer0_base+AT91_TC_IER);
		__raw_writel(AT91_TC_CLKEN,timer0_base+AT91_TC_CCR);	
		__raw_writel(AT91_TC_SYNC,timer0_base+AT91_TC_BCR);
		at91_sys_write(AT91_AIC_EOICR,0xffffffff);	
				
				
				
		
		__raw_readl(timer0_base+AT91_TC_SR);
		at91_sys_write(AT91_AIC_EOICR,0xffff);

	
	
	
	__raw_readl(timer0_base+AT91_TC_SR);
	at91_sys_write(AT91_AIC_EOICR,0xffff);
	
	return IRQ_HANDLED;
}


/*READ FUNCTION*/
ssize_t user_pid_read(struct file *filp, char __user *buffer,size_t length, loff_t *offset)
{
	DEFINE_WAIT(wait);
//	struct pid_structure update;
	
//	bytes_read = copy_to_user(buffer,&update,length);
//	returnval = len-bytes_read;
//	return returnval;
} 

/*this function handles writing the value to the module structure*/
ssize_t user_pid_write(struct file * file,char *buf, size_t count, loff_t *loft) 
{
//	struct pid_structure update;
//	bytes_written = copy_from_user(&update,buf,sizeof(update));
  //	return bytes_written;
};

/*IOCTL FUNCTION*/
int user_ioctl(struct inode *inode, struct file *file,unsigned int ioctl_num,unsigned long arg)
{	
//	struct pid_structure update;
	int bytes_written =0 ;
	int bytes_read = 0;
	int retval=-1;

	//this is called using a pointer to the userspace control structure as the argument//
//	switch(ioctl_num){
	/*write control structure*/
//	case DEV_IOCTL_WRITE_CONTROL_STRUCTURE:
//		bytes_written=copy_from_user(&update,(void *)arg,sizeof(update));
 //		spin_lock_irqsave(&mylock,irq_state1);
 //			control=update;
 //		spin_unlock_irqrestore(&mylock,irq_state1);
//		retval = bytes_written;
//		break;


	return retval;

}

/* Called when a process OPENS the device file*/
static ssize_t user_pid_open(struct file * file,char *buf, size_t count, loff_t *loft) 
{	unsigned int ret,res;
	unsigned int irq;
	unsigned long irq_state1;
	unsigned long irq_state2;



	//static int counter = 0;
	printk(KERN_ALERT "-----------------------------\n---------------------Opening a channel to C-BASS Device Driver---------------------\n");
	if (Device_Open)
		return -EBUSY;

	Device_Open++;
//-------------------------------	

	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_TC0|at91_sys_read(AT91_PMC_PCSR));
// 		//ENABLE THE INTERRUPT ON THE AIC FOR THE ADC PERIPHERAL
	at91_sys_write(AT91_AIC_IECR,1<<AT91SAM9260_ID_TC0|at91_sys_read(AT91_AIC_IMR));

	//__raw_writel(AT91_TC_TIMER_CLOCK1|AT91_TC_CPCTRG|AT91_TC_WAVESEL_UP_AUTO|AT91_TC_WAVE,timer0_base+AT91_TC_CMR);
	__raw_writel(AT91_TC_TIMER_CLOCK4|AT91_TC_CPCTRG|AT91_TC_WAVESEL_UP_AUTO|AT91_TC_WAVE,timer0_base+AT91_TC_CMR);
	//this should try keep the tick interval accurate- not quite ticking at a round number of uSeconds correctly but not bad.
	//__raw_writel(22500,timer0_base+AT91_TC_RC);
	__raw_writel(1900,timer0_base+AT91_TC_RC);
	__raw_writel(AT91_TC_CPCS,timer0_base+AT91_TC_IER);
	__raw_writel(AT91_TC_CLKEN,timer0_base+AT91_TC_CCR);	
	__raw_writel(AT91_TC_SYNC,timer0_base+AT91_TC_BCR);	
	at91_sys_write(AT91_AIC_EOICR,0xffffffff);	

//-----------------------------------------	
	try_module_get(THIS_MODULE);

	return SUCCESS;

};

/* Called when a process closes the device file*/
static int user_pid_release(struct inode *inode, struct file *file)
{	unsigned int ret,res;
	unsigned int irq;
	unsigned long irq_state1;
	unsigned long irq_state2;

	irq = AT91SAM9260_ID_TC0;
	

	Device_Open--;		/* We're now ready for our next caller */
	printk(KERN_ALERT "-----------------------------\n---------------------Closing a channel to C-BASS Device Driver---------------------\n");
	__raw_writel(AT91_TC_TIMER_CLOCK4|AT91_TC_CPCTRG|AT91_TC_WAVESEL_UP_AUTO|AT91_TC_WAVE,timer0_base+AT91_TC_CMR);	
	__raw_writel(SLOW_INT,timer0_base+AT91_TC_RC);
	__raw_writel(AT91_TC_CPCS,timer0_base+AT91_TC_IER);
	__raw_writel(AT91_TC_CLKEN,timer0_base+AT91_TC_CCR);	
	__raw_writel(AT91_TC_SYNC,timer0_base+AT91_TC_BCR);
	at91_sys_write(AT91_AIC_EOICR,0xffffffff);	
	module_put(THIS_MODULE);

	return 0;
}

 /* Operations for controling the device */
struct file_operations user_pid_file_fops = 
{
        .owner             = THIS_MODULE,
        .read              = user_pid_read,
        .write             = (void *)user_pid_write,
	.ioctl		    =user_ioctl,
	.open		   =(void *)user_pid_open,
	.release	   =user_pid_release,	
};

/*Creating a misc device called PID*/
static struct miscdevice pid = 
{ 
        MISC_DYNAMIC_MINOR,
        "pid",                   /* Name sets here,  will be shown in /dev */
        &user_pid_file_fops
};

/*Initialise the module*/
static int mod_init(void)
{
	extern unsigned int irq;
	extern unsigned int irq_PC15_1PPS;
	unsigned int ret,res;

	//dev_pid_ptr=&dev_pid;

	printk(KERN_ALERT "Loading the PPS Interrupt detection\n");
	//map the timer memory
	timer0_base = ioremap(AT91SAM9260_BASE_TC0,SZ_16K);
	adc_base = ioremap(AT91SAM9260_BASE_ADC,SZ_16K);
	spi1_base = ioremap(AT91SAM9260_BASE_SPI1,SZ_16K);	
	





	

	irq = AT91SAM9260_ID_TC0;
	irq_PC15_1PPS = AT91SAM9260_ID_IRQ1;
	printk("starting IRQ\n");
//------------------------------------------------------
	
 	

	res = request_irq(irq,(void *)pid_irq, IRQF_SHARED, "Timer0_IRQ",&irq);
	if(res==0){
		at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_TC0|at91_sys_read(AT91_PMC_PCSR));
		//ENABLE THE INTERRUPT ON THE AIC FOR THE ADC PERIPHERAL//SLOW INTERRUPT i.e flags.Interrupt_Mode=1;
		at91_sys_write(AT91_AIC_IECR,1<<AT91SAM9260_ID_TC0|at91_sys_read(AT91_AIC_IMR));

		__raw_writel(AT91_TC_TIMER_CLOCK4|AT91_TC_CPCTRG|AT91_TC_WAVESEL_UP_AUTO|AT91_TC_WAVE,timer0_base+AT91_TC_CMR);	
		__raw_writel(SLOW_INT,timer0_base+AT91_TC_RC);
		__raw_writel(AT91_TC_CPCS,timer0_base+AT91_TC_IER);
		__raw_writel(AT91_TC_CLKEN,timer0_base+AT91_TC_CCR);	
		__raw_writel(AT91_TC_SYNC,timer0_base+AT91_TC_BCR);
		at91_sys_write(AT91_AIC_EOICR,0xffffffff);//this is written to indicate the end of interrupt	
	
	}
	else{ 
		printk("Request for Interrupt Line Failed\n");
		
	}
		printk("Here\n");
//	res=request_irq(AT91_PIN_PC15, pps_irq, IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING, "PPS_IRQ", NULL);
/** Set pin as GPIO input, without internal pull up */
	if(at91_set_gpio_input(AT91_PIN_PC15, 0)) {
	    printk(KERN_DEBUG"Could not set pin %i for GPIO input.\n", AT91_PIN_PC15);
	}
	/** Set deglitch for pin */
	if(at91_set_deglitch(AT91_PIN_PC15, 1)) {
	    printk(KERN_DEBUG"Could not set pin %i for GPIO deglitch.\n", AT91_PIN_PC15);
	}
	at91_set_B_periph(AT91_PIN_PC15,0);


	res = request_irq(irq_PC15_1PPS,(void *)pps_irq, IRQF_TRIGGER_RISING, "PPS_IRQ",&irq_PC15_1PPS);
	if(res==0){
		printk("Request for Interrupt Line Suceeded\n");
	//	at91_sys_write(AT91_AIC_SMR(AT91SAM9260_ID_IRQ1),AT91_AIC_SRCTYPE_RISING);
	//	at91_sys_write(AT91_AIC_IECR,1<<AT91SAM9260_ID_IRQ1|at91_sys_read(AT91_AIC_IMR));//set the IRQ1 interrupt on and make sure you don't remove the rest of the interrupts

		at91_sys_write(AT91_AIC_EOICR,0xffffffff);	
	}
	else{ 
		printk("Request for Interrupt Line Failed\n");
		
	}
	
	at91_sys_write(AT91_AIC_EOICR,0xffffffff);

 	ret = misc_register(&pid);
	
	return 0;


}

/*Remove  the moodule*/
int mod_exit(void)
{
	int ret;
	extern unsigned int irq;
	extern unsigned int irq_PC15_1PPS;
	//unmap the memory region
//	dma_free_coherent(&dev_pid, 1000,&dma_address1_coherent,GFP_ATOMIC);
//	dma_free_coherent(&dev_pid, 1000,&dma_address2_coherent,GFP_ATOMIC);
	
	printk("Unloading the C-BASS driver\n");
	__raw_writel(AT91_TC_COVFS,timer0_base+AT91_TC_IDR);
	at91_sys_write(AT91_AIC_IDCR,1<<AT91SAM9260_ID_TC0);
	
 	ret = misc_deregister(&pid);
	free_irq(irq, &irq);
 	free_irq(irq_PC15_1PPS, &irq_PC15_1PPS);
 //	free_irq(AT91_PIN_PC15, 0);
	

	iounmap(adc_base);
	iounmap(spi1_base);
	iounmap(timer0_base);
	return ret;
	//if(ret !=0)
	//	printk("Module Deregistering Failed\n");
}


module_init(mod_init);
module_exit(mod_exit); 

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Charles Copley");
MODULE_DESCRIPTION("C-BASS CONTROLLER AT91SAM9260");
MODULE_ALIAS("platform:at91_spi");

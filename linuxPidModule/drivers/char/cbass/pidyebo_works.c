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
#include "telescope_constants.h"
//#define adc_base (0xfffe0000)

//this defines the structure used to copy data between kernel and userspace- check that this file is the same in both ALWAYS!!



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





#define SUCCESS 0




MODULE_AUTHOR("Charles Copley.");
MODULE_LICENSE("GPL");
extern unsigned long loops_per_jiffy;
//for reading from the device;
static char msg[50];
static char *msg_Ptr;
static int Device_Open = 0;
static volatile int timer_counter=0;
volatile unsigned int dac_out=0;
void __iomem *timer0_base;
void __iomem *adc_base;
void __iomem *spi1_base;
void __iomem *dmabuf;
struct device *dev_pid_ptr,dev_pid;
dma_addr_t dma_address1_coherent,dma_address2_coherent;
void *txbuffer_coherent,*rxbuffer_coherent;

volatile unsigned short dacout=0;

static unsigned int daccounter=0;

static int flag = 0; 
static volatile int readflag = 0; 

int update(void);
int emergency_stop(void);
int az_emergency_stop(void);
int alt_emergency_stop(void);
int dac_write(int dac_number, unsigned short dacvalue);
unsigned short read_encoder(int encoder_number);
int soft_limits(int azimuth_zone,unsigned int azimuth_encoder, unsigned short altitude_encoder,unsigned int azimuth_command,unsigned short altitude_command,unsigned int *azimuth_command_out,unsigned short *altitude_command_out);


struct encoder_transfer{
	long alt_encoder[3],az_encoder[3];
}encoder;

struct encoder_driver{
	
	unsigned int cs,clock,data;
	
};







struct ad667_driver{

	unsigned int add0,add1,add2,add3,data0,data1,data2,data3,cs0,cs1;	

};







static const struct ad667_driver ad667 = { /* Operations for controling the device */
     
//define pin configuration for the ad667 driver!	

   	.add0           = AT91_PIN_PB1,
        .add1              =AT91_PIN_PB2 ,
        .add2             = AT91_PIN_PB3,
	.add3		   =AT91_PIN_PB4,
	.data0		   =AT91_PIN_PB5,	
	.data1		= AT91_PIN_PB8	,
	.data2		=AT91_PIN_PB9	,
	.data3		=AT91_PIN_PB10	,
	.cs0		=AT91_PIN_PB11	,
	.cs1		=AT91_PIN_PB16	,
};

static const struct encoder_driver angle = { /* Operations for controling the device */
     //define pin config for the angle encoders
	.cs =  AT91_PIN_PB20,
	.clock = AT91_PIN_PB18,
	.data = AT91_PIN_PB19,
};







static irqreturn_t pid_irq(int irq, void *_cf, struct pt_regs *r)
{
		//this interrupt is the heart beat of the control loop- basically updates values in the pid control structure.
	int soft_lim,*az_encoder_ptr;
	unsigned int temp;
	int azimuth_test;
	signed short test;
	int i;
	unsigned short alt_temp[5];
	unsigned short az_temp[5];
	int angle_alt,angle_az;
	int ret;
	int led_level;
	int *adc1,*adc2,output[100],output1[100];
	temp = at91_sys_read(AT91_AIC_IVR);
	//PDEBUG("IRQ handled\n");
	
	//temp = gpio_get_value(AT91_PIN_PB0);
	//led_level=0x00;
	//toggle the led using EOR
	//led_level = temp^0x01;
	//at91_set_gpio_output(AT91_PIN_PB0,led_level);

	//change the output level to the correct level
	//at91_set_gpio_output(AT91_PIN_PB0,led_level);

	timer_counter++;
	
	//set the timing of the actual readings taken- Check the period of this interrupt- nominally if the overflow is set at 45000 then this interrupt triggers every 1ms, if 22500 then it triggers every 0.5ms and so on. So if it triggers every 1ms and the timer_counter value is 100 then the pid loop is running at 0.1s refresh rate.
	//Timer Clock maximum speed set at overflow on 22500
	//19 - > 9980 us
	//20 -> 10430 us
	//10 > 10480
	//5 -> 7990
	//40 20475 uS
	//39 19775 uS
	//38 19475
	//37 18976
	//36  18478
	//35 17976
	//30 15480
	//20 10430
	//16 8488
	//12 TOO FAST WITH COORDINATE TRANSFORMATION!
	if(timer_counter >=control.interrupt_rate){
		
		daccounter++;
		timer_counter=0;
		control.counter++;
		if(control.counter == 10){
			ret = emergency_stop();
			//control.counter = 0;
		}
		if(control.azimuth_zone >2){
			printk("Azimuth Zone not correctly added by USERSPACE\n");
			ret = emergency_stop();
		}
	//PDEBUG("%lu\n",control.alt_adc);
		if(readflag==1 ){
			
			if(control.update==1){
				ret = update();
				
			}
			
			//flag gets set to zero by the read process
			    
			readflag =0;
			flag = 1; 
			if(control.azimuth_zone<=2){
			control.time=__raw_readl(timer0_base+AT91_TC_CV); 
			
// 			control.az_adc = 100;
// 			encoder.alt_encoder[1]=encoder.alt_encoder[2];	
// 			encoder.alt_encoder[2]=encoder.alt_encoder[3];
// 			
// 			encoder.alt_encoder[3]=control.alt_encoder;
// 			
			//simulated movement of the angle encoders for now- needs to be changed on the real thing
			//control.alt_encoder += (short)control.alt_simulation; 
			//control.az_encoder += (short)control.az_simulation; 
			//control.prev_alt_encoder = control.alt_encoder;
			//control.prev_az_encoder = control.az_encoder;
			
			alt_temp[0] =10;
			alt_temp[1] =0;
			az_temp[0]= 10;
			az_temp[1]=0;
			while(abs(alt_temp[0]-alt_temp[1])>2){
				alt_temp[0] = read_encoder(1);
				alt_temp[1] = read_encoder(1);
			}
			while(abs(az_temp[0]-az_temp[1])>2){
				az_temp[0] = read_encoder(0);
				az_temp[1] = read_encoder(0);
			}
			azimuth_test = (int)control.az_encoder - (int)az_temp[0];
			//check azimuth zone of operation
			if(azimuth_test < -33000 && (control.interrupt_number!=0)){
				control.azimuth_zone--;
				printk("AZ --\n");
			}
			if(azimuth_test >33000 && (control.interrupt_number!=0)){
				control.azimuth_zone++;
				printk("AZ ++\n");
			}
			if(control.azimuth_zone<0){
				control.azimuth_zone =0;
			}
			if(control.azimuth_zone>2){
				control.azimuth_zone =2;
			}
			control.alt_encoder =alt_temp[0];
			control.az_encoder=az_temp[0];
			control.az_encoder_large = control.az_encoder +65535*control.azimuth_zone;
			control.interrupt_number=1;
// 			if(abs(alt_temp[0]-alt_temp[1])<2){
// 				control.alt_encoder=alt_temp[0];
// 			}
// 			if(abs(az_temp[0]-az_temp[1])<2){
// 				control.az_encoder=az_temp[0];
// 			}
// 			
 			
			//udelay(30);
			//alt_temp[1] = read_encoder(0);
			
			//az_temp[1] = read_encoder(1);
// 			test=0;
// 			while((alt_temp[0]!=alt_temp[1])||(az_temp[0]!=az_temp[1])) {
// 				alt_temp[0] = read_encoder(0);
// 				alt_temp[1] = read_encoder(0);
// 				az_temp[0] = read_encoder(1);
// 			//udelay(30);
// 			
// 				az_temp[1] = read_encoder(1);
// //			udelay(30);
// 			test++;
// 			}
			
// 			test=0;

 			
// 			angle_alt = (int)control.alt_encoder - (int)control.prev_alt_encoder;
// 			angle_az = (int)control.az_encoder - (int)control.prev_az_encoder;
// 			if(fabs(angle_alt)>32767){
// 				printk("Past value\n");
// 			}
			//printk("encoders %u %u\n",control.alt_encoder,control.az_encoder);

		//	PDEBUG("angle_encoder %04x\n",ret);
		//	init_ad5362();
			

//  			ret = dac_write(0,control.alt_command);
//  			ret = dac_write(1,control.alt_command);
//  			ret = dac_write(2,control.alt_command);
//  			ret = dac_write(3,control.alt_command);
			//dacout+=50;
			//
			//ret=dac5362(READ,CREGISTER_READ);
			//printk("%08x\n",(XREGISTER_WRITE<<20)|(CH2<<16)|(dacout&(0x00ffff)));
			//ret=dac5362(WRITE_AD5362,(XREGISTER_WRITE<<20)|(CH5<<16)|((control.az1_dac+32767)&(0x00ffff)));
			soft_lim =0;
			soft_lim = soft_limits(control.azimuth_zone,control.az_encoder_large,control.alt_encoder,control.az_command_large,control.alt_command,&(control.az_command_large),&(control.alt_command));
			//soft_lim=0;
			if(soft_lim==0){
				ret=dac5362_crc(WRITE_AD5362,control.az1_dac_control);	
				ret=dac5362_crc(WRITE_AD5362,control.az2_dac_control);
				ret=dac5362_crc(WRITE_AD5362,control.alt1_dac_control);
				ret=dac5362_crc(WRITE_AD5362,control.alt2_dac_control);
	
				//make sure the DAC registers do not become corrupted- I'm sure there is a better way!
				for(i=0;i++;i<=7){
					ret=dac5362_crc(WRITE_AD5362,control.cbuffer_crc[i]);
					ret=dac5362_crc(WRITE_AD5362,control.mbuffer_crc[i]);
				}
				control.cbuffer_read_ret[0]=0;
				control.cbuffer_read_ret[0] = dac5362_crc(READ_AD5362,control.cbuffer_crc_read[0]);
			}
			//azimuth soft limit reached
// 			if(soft_lim==1){
// 					ret=dac5362_crc(WRITE_AD5362,0xd07fffbd);
// 					ret=dac5362_crc(WRITE_AD5362,0xd17fffd6);
// 					ret=dac5362_crc(WRITE_AD5362,0xc87fff4e);
// 					ret=dac5362_crc(WRITE_AD5362,0xc97fff25);
// 					control.az1_dac_control = 0xd07fffbd;
// 					control.az2_dac_control = 0xd17fffd6;
// 					ret=dac5362_crc(WRITE_AD5362,control.alt1_dac_control);
// 					ret=dac5362_crc(WRITE_AD5362,control.alt2_dac_control);
// 	
// 				//make sure the DAC registers do not become corrupted- I'm sure there is a better way!
// 				for(i=0;i++;i<=7){
// 					ret=dac5362_crc(WRITE_AD5362,control.cbuffer_crc[i]);
// 					ret=dac5362_crc(WRITE_AD5362,control.mbuffer_crc[i]);
// 				}
// 				control.cbuffer_read_ret[0]=0;
// 				control.cbuffer_read_ret[0] = dac5362_crc(READ_AD5362,control.cbuffer_crc_read[0]);
// 			}
// 			//elevation soft limit reached
// 			if(soft_lim==2){
// 
// 				
// 				ret=dac5362_crc(WRITE_AD5362,0xd27fff6b);
// 				ret=dac5362_crc(WRITE_AD5362,0xd37fff00);
// 				ret=dac5362_crc(WRITE_AD5362,0xca7fff98);
// 				ret=dac5362_crc(WRITE_AD5362,0xcb7ffff3);
// 				control.alt1_dac_control = 0xd27fff6b;
// 				control.alt2_dac_control = 0xd37fff00;	
// 				ret=dac5362_crc(WRITE_AD5362,control.az1_dac_control);	
// 				ret=dac5362_crc(WRITE_AD5362,control.az2_dac_control);
// 				
// 	
// 				//make sure the DAC registers do not become corrupted- I'm sure there is a better way!
// 				for(i=0;i++;i<=7){
// 					ret=dac5362_crc(WRITE_AD5362,control.cbuffer_crc[i]);
// 					ret=dac5362_crc(WRITE_AD5362,control.mbuffer_crc[i]);
// 				}
// 				control.cbuffer_read_ret[0]=0;
// 				control.cbuffer_read_ret[0] = dac5362_crc(READ_AD5362,control.cbuffer_crc_read[0]);
// 			}
		//	ret=dac5362(WRITE_AD5362,(XREGISTER_WRITE<<20)|(CH6<<16)|((control.az2_dac+32767)&(0x00ffff)));
		//	ret=dac5362(WRITE_AD5362,(XREGISTER_WRITE<<20)|(CH7<<16)|((control.alt1_dac+32767)&(0x00ffff)));
		//	ret=dac5362(WRITE_AD5362,(XREGISTER_WRITE<<20)|(CH8<<16)|((control.alt2_dac+32767)&(0x00ffff)));

			//
			//ret=dac5362(WRITE_AD5362,(XREGISTER_WRITE<<20)|(ALL<<16)|((control.alt1_dac+32767)&(0x00ffff)));
			
			//control.alt_adc = __raw_readl(adc_base+AT91_ADC_CHR(0));
			adc2 = spi_out_ad7367(1,output1);
			adc1 = spi_out_ad7367(0,output);

			//adc1 = spi_out_ad7367_bit_banging(0,output);
			//adc2 = spi_out_ad7367_bit_banging(1,output1);
// 			
 			control.tacho1 = ((*(output+0))&(0x0000ffff));
			control.tacho3 = ((*(output+1))&(0x0000ffff));
 			control.tacho4 = ((*(output1+0))&(0x0000ffff));
 			control.tacho2 = ((*(output1+1))&(0x0000ffff));
			
			//ret = spi_out_ad7367(1);
			//printk("Hello\n");
			//ret = spi_read_ad5362(dacout);
			//ret = dac_write(0,control.alt_encoder);
			//ret = dac_write(1,control.az_encoder);
			//ret = dac_write(2,control.alt_encoder);
			//ret = dac_write(3,control.az_encoder);

			//PDEBUG("encoder %u alt_dac %d \n",control.alt_encoder,control.alt_dac);
			}
			control.counter = timer_counter;
			do_gettimeofday(&(control.time_struct));
			wake_up_interruptible(&wq);
			//PDEBUG("Process Awakened\n");
			}
	}
//	get conversion data from last time around.
	
	//start next conversion
	__raw_writel(AT91_ADC_START,adc_base+AT91_ADC_CR);
	__raw_readl(timer0_base+AT91_TC_SR);
	
	 
	at91_sys_write(AT91_AIC_EOICR,0xffff);
	return IRQ_HANDLED;
}


static ssize_t user_pid_read(struct file *filp, char __user *buffer,
                       size_t length, loff_t *offset)
{
/*
	 * Number of bytes actually written to the buffer 
	 */
	DEFINE_WAIT(wait);
	int bytes_read = 0;
//	int result;
	int len=0;
	int ret;

	          
	
	//PDEBUG("\"%s\" writing: going to sleep\n",current->comm); 
	//prepare_to_wait(&wq, &wait, TASK_INTERRUPTIBLE); 
	//schedule(  ); 
		


 	msg_Ptr = msg;
// 	flag = 0;
 	readflag=1;
 	//PDEBUG("Processed put to sleep\n");
 	//flag=1;
 	wait_event_interruptible(wq, flag != 0); 

			


	len = sizeof(control);
//	PDEBUG("Wait event awake again %d %d\n",len,bytes_read);	
 //	PDEBUG("Wait event awake again %d\n",result);	
	flag = 0;
//set flag=0 so that we know the read function has been called.
	
	bytes_read = copy_to_user(buffer,&control,length);
//	PDEBUG("Wait event  %d %d\n",len,bytes_read);

	return len-bytes_read;

} 



static ssize_t user_pid_write(struct file * file,char *buf, size_t count, loff_t *loft) {
 	//this function handles writing the value to a specific DAC [AD667]-needs to be trimmed a lot but for now is ok for testing- Charles Copley 12 August 2008
	
	//enable the conversion!
	//char string[100];
	int bytes_written =0 ;
	
	bytes_written = copy_from_user(&control,buf,sizeof(control)); /* Taking data from user and parsing to kernel */
	//PDEBUG("control.alt_command %u\n",control.alt_command); 
	
	
	
  	return bytes_written; /* We don't want overflow and kernel panic */

	
	
	
  	
 
};

int user_ioctl(struct inode *inode,	/* see include/linux/fs.h */
		 struct file *file,	/* ditto */
		 unsigned int ioctl_num,	/* number and param for ioctl */
		 unsigned long arg){

	unsigned short alt_temp[5];
	unsigned short az_temp[5];
	int *adc1,*adc2,output[100],output1[100];

	


	//this is called using a pointer to the userspace control structure as the argument
	switch(ioctl_num){

	case DEV_IOCTL_WRITE_CONTROL_STRUCTURE:
		copy_from_user(&control,arg,sizeof(control));
		break;
	case DEV_IOCTL_READ_CONTROL_STRUCTURE:
		control.ioctl_num++;
		alt_temp[0] =10;
		alt_temp[1] =0;
		az_temp[0]= 10;
		az_temp[1]=0;
		while(abs(alt_temp[0]-alt_temp[1])>2){
			alt_temp[0] = read_encoder(1);
			alt_temp[1] = read_encoder(1);
		}
		while(abs(az_temp[0]-az_temp[1])>2){
			az_temp[0] = read_encoder(0);
			az_temp[1] = read_encoder(0);
		}
		control.alt_encoder =alt_temp[0];
		control.az_encoder=az_temp[0];
		control.az_encoder_large = control.az_encoder +65535*control.azimuth_zone;
	
		adc2 = spi_out_ad7367(1,output1);
		adc1 = spi_out_ad7367(0,output);
				
		control.tacho1 = ((*(output+0))&(0x0000ffff));
		control.tacho3 = ((*(output+1))&(0x0000ffff));
		control.tacho4 = ((*(output1+0))&(0x0000ffff));
		control.tacho2 = ((*(output1+1))&(0x0000ffff));

		copy_to_user(arg,&control,sizeof(control));
		break;
	}

	

	return SUCCESS;

}



static ssize_t user_pid_open(struct file * file,char *buf, size_t count, loff_t *loft) {
	
	static int counter = 0;

	if (Device_Open)
		return -EBUSY;

	Device_Open++;
	
	try_module_get(THIS_MODULE);

	return SUCCESS;

};

// * Called when a process closes the device file.
// */
static int user_pid_release(struct inode *inode, struct file *file)
{
	Device_Open--;		/* We're now ready for our next caller */

	/* 
	 * Decrement the usage count, or else once you opened the file, you'll
	 * never get get rid of the module. 
	 */
	module_put(THIS_MODULE);

	return 0;
}









static const struct file_operations user_pid_file_fops = { /* Operations for controling the device */
        .owner             = THIS_MODULE,
        .read              = user_pid_read,
        .write             = user_pid_write,
	.ioctl			=user_ioctl,
	.open		   =user_pid_open,
	.release	   =user_pid_release,	
};

static struct miscdevice pid = { /*Creating a misc device called ad667*/
        MISC_DYNAMIC_MINOR,
        "pid",                   /* Name sets here,  will be shown in /dev */
        &user_pid_file_fops
};



static int mod_init(void){
	unsigned int ret,res;
	unsigned int irq;
	//flag=1;
	struct spi_master	*master;
	struct CONTROLLER	*c;

	dev_pid_ptr=&dev_pid;
	
	msg_Ptr=msg;
	//map the timer memory
	timer0_base = ioremap(AT91SAM9260_BASE_TC0,SZ_16K);
	adc_base = ioremap(AT91SAM9260_BASE_ADC,SZ_16K);
	spi1_base = ioremap(AT91SAM9260_BASE_SPI1,SZ_16K);	
	//dmabuf = ioremap (0x0f00000 /*14M*/, SZ_16K /* 1M */);
	//spi_access_bus(1);
	//at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
//	at91_set_gpio_output(AT91_PIN_PC14,0);
	at91_set_gpio_output(AT91_PIN_PB20,0);
	at91_set_gpio_output(AT91_PIN_PB21,0);
	at91_set_gpio_output(AT91_PIN_PB22,0);
	at91_set_gpio_output(AT91_PIN_PB23,0);
	printk("Testing\n");
	//txbuffer_coherent=dma_alloc_coherent(&dev_pid, 1000,&dma_address1_coherent,GFP_ATOMIC );
	//rxbuffer_coherent = dma_alloc_coherent(&dev_pid, 1000,&dma_address2_coherent,GFP_ATOMIC);
	//reset the SPI interface
	init_ad5362();
	control.azimuth_zone = 100;
	control.interrupt_number =0 ;
	control.ioctl_num =0;
	control.update=0;
	control.error=0;
	control.alt_adc = 0;
	control.az_adc=0;
	control.alt_command=0;
	control.az_command_large=0;
	control.alt1_dac=0;
	control.alt2_dac=0;
	control.az1_dac=0;
	control.az2_dac=0;
	control.alt_err=0;
	control.az_err=0;
	control.alt_p=0;
	control.az_p=0;
	control.alt_i=0;
	control.az_i=0;
	control.alt_d=0;
	control.az_d=0;
	control.alt_err_prev=0;
	control.az_err_prev=0;
	control.alt_pid=0;
	control.az_pid=0;
	control.interrupt_rate=20;
	control.encoder_wavelength = 10;
	control.alt_encoder = read_encoder(1);
	control.az_encoder_large = read_encoder(0);
	hello();
	
	
	//define the peripheral ID for IRQ
	irq = AT91SAM9260_ID_TC0;
	printk(KERN_ALERT "Loading the PID driver module\n");	
	PDEBUG("DEBUG MODE ON\n");
	//turn on SLEEP and scale clock by maximum prescale value I think my CPU clock is ~180MHZ. this prescales by  (45) so get ADC clock of ~4MHz. Apparently the maximum is ~5MHz (page 749 6221G–ATARM–31-Jan-08 DATASHEET) for the ADC clock at 10bit mode- 
	//PDEBUG(fmt, args...)
	PDEBUG("TC_CMR %08x\n",__raw_readl(timer0_base+AT91_TC_CMR));
 	PDEBUG("TC_CV %08x\n",__raw_readl(timer0_base+AT91_TC_CV));
	PDEBUG("TC_RA %08x\n",__raw_readl(timer0_base+AT91_TC_RA));
	PDEBUG("TC_RB %08x\n",__raw_readl(timer0_base+AT91_TC_RB));
	PDEBUG("TC_RC %08x\n",__raw_readl(timer0_base+AT91_TC_RC));
	PDEBUG("TC_SR %08x\n",__raw_readl(timer0_base+AT91_TC_SR));
	PDEBUG("TC_IMR %08x\n",__raw_readl(timer0_base+AT91_TC_IMR));
	PDEBUG("TC_BMR %08x\n",__raw_readl(timer0_base+AT91_TC_BMR));

	
//------------------------------------------------------
 	res = request_irq(irq,pid_irq, IRQF_SHARED, "Timer0_IRQ",(void*)4);
 	if(res==0){
		
	//	__raw_writel();
		at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_TC0|at91_sys_read(AT91_PMC_PCSR));
		//ENABLE THE INTERRUPT ON THE AIC FOR THE ADC PERIPHERAL
		at91_sys_write(AT91_AIC_IECR,1<<AT91SAM9260_ID_TC0|at91_sys_read(AT91_AIC_IMR));
	
		__raw_writel(AT91_TC_TIMER_CLOCK1|AT91_TC_CPCTRG|AT91_TC_WAVESEL_UP_AUTO|AT91_TC_WAVE,timer0_base+AT91_TC_CMR);
		//this should try keep the tick interval accurate- not quite ticking at a round number of uSeconds correctly but not bad.
		__raw_writel(22500,timer0_base+AT91_TC_RC);
		__raw_writel(AT91_TC_CPCS,timer0_base+AT91_TC_IER);
		__raw_writel(AT91_TC_CLKEN,timer0_base+AT91_TC_CCR);	
		__raw_writel(AT91_TC_SYNC,timer0_base+AT91_TC_BCR);	



	
	
 		at91_sys_write(AT91_AIC_EOICR,0xffffffff);
 	}
// 	
	else 
	//interrupt request failed from linux 
		printk("Request for Interrupt Line Failed\n");
// 	
// 
//	irq = AT91SAM9260_ID_ADC;
	
//	res = request_irq(irq,adc_irq, IRQF_SHARED, "Timer0_IRQ",(void*)4);
 //	if(res==0){
		
	//	__raw_writel();
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_ADC|at91_sys_read(AT91_PMC_PCSR));
		//ENABLE THE INTERRUPT ON THE AIC FOR THE ADC PERIPHERAL
 	at91_sys_write(AT91_AIC_IECR,1<<AT91SAM9260_ID_ADC|at91_sys_read(AT91_AIC_IMR));
// 		
 		__raw_writel(AT91_ADC_SLEEP|(45<<8),adc_base+AT91_ADC_MR);	
 		PDEBUG("ADC_MR %08x\n",__raw_readl(adc_base+AT91_ADC_MR));	
		//enable the ADC0 input
		__raw_writel(AT91_ADC_CH(0),adc_base+AT91_ADC_CHER);
 		PDEBUG("ADC_CHSR %08x\n",__raw_readl(adc_base+AT91_ADC_CHSR));	
		__raw_writel(AT91_ADC_START,adc_base+AT91_ADC_CR);
// 	
 		at91_sys_write(AT91_AIC_EOICR,0xffffffff);


//-----------------------------------------------------------
 //	}
// 	
	//else 
	//interrupt request failed from linux 
		//printk("Request for Interrupt Line Failed\n");

// 	REGISTER A ADC DEVICE
	PDEBUG("TC_CMR %08x\n",__raw_readl(timer0_base+AT91_TC_CMR));
 	PDEBUG("TC_CV %08x\n",__raw_readl(timer0_base+AT91_TC_CV));
	PDEBUG("TC_RA %08x\n",__raw_readl(timer0_base+AT91_TC_RA));
	PDEBUG("TC_RB %08x\n",__raw_readl(timer0_base+AT91_TC_RB));
	PDEBUG("TC_RC %08x\n",__raw_readl(timer0_base+AT91_TC_RC));
	PDEBUG("TC_SR %08x\n",__raw_readl(timer0_base+AT91_TC_SR));
	PDEBUG("TC_IMR %08x\n",__raw_readl(timer0_base+AT91_TC_IMR));
	PDEBUG("TC_BMR %08x\n",__raw_readl(timer0_base+AT91_TC_BMR));
 	ret = misc_register(&pid);
	
	return 0;


}

static int mod_exit(void){
	int ret;
	unsigned int irq;
	//unmap the memory region
//	dma_free_coherent(&dev_pid, 1000,&dma_address1_coherent,GFP_ATOMIC);
//	dma_free_coherent(&dev_pid, 1000,&dma_address2_coherent,GFP_ATOMIC);
	
	irq = AT91SAM9260_ID_TC0;
	printk("Unloading the ADC driver\n");
	__raw_writel(AT91_TC_COVFS,timer0_base+AT91_TC_IDR);
	at91_sys_write(AT91_AIC_IDCR,1<<AT91SAM9260_ID_TC0);
 	ret = misc_deregister(&pid);
 	free_irq(irq, (void*)4);
	

	iounmap(adc_base);
	iounmap(spi1_base);
	iounmap(timer0_base);
	return 0;
	//if(ret !=0)
	//	printk("Module Deregistering Failed\n");
}


unsigned short read_encoder(int encoder_number){

	int clock,data,cs;
	int counter =0;
	int temp;
	int mask=1;
	unsigned int encoder=0;
	unsigned int encoder2=0;
	unsigned short angle_encoder=0;
	
	int state_time;
	char value[20];
	char value2[20];
	int wavelength;

	wavelength = control.encoder_wavelength;
	wavelength = 10;
	state_time = wavelength>>1;
	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<20)|(1<<19)|(1<<18));
		
	cs =  AT91_PIN_PB20;
	clock = AT91_PIN_PB18;
	data = AT91_PIN_PB19;

	control.encoder_error=0;
	//at91_set_gpio_output(angle.cs,1);
	at91_set_gpio_output(clock,1);
	at91_set_gpio_output(cs,encoder_number);
	udelay(20);
	at91_set_gpio_input(data, 1);
	
	at91_set_gpio_output(clock,0);
	udelay(state_time);
	
//	at91_set_gpio_output(clock,1);


	for(counter=0;counter<=15;counter++){
	value[counter]=0;
	value2[counter]=0;
	at91_set_gpio_output(clock,1);
	udelay(state_time);
	value[counter] = at91_get_gpio_value(data);
	at91_set_gpio_output(clock,0);
	udelay(state_time);
	value2[counter]=at91_get_gpio_value(data);
	//udelay(3);
	//temp=value[counter];
//	PDEBUG("Angle Encoder %04x\n",temp);
	//angle_encoder = angle_encoder | ((temp<<(15-counter))&(mask));
	//mask=0x0001<<(15-counter);
	//temp=0;
	//angle_encoder =angle_encoder<<1;
	}
	//printk("\n",temp);
	at91_set_gpio_output(clock,1);
	udelay(20);
	
	temp=0;
	//printk("one ");
	//for(counter=0;counter<=15;counter++){
	//	printk("%i",value[counter]);
	//}
	//printk("two ");
	//for(counter=0;counter<=15;counter++){
	//	printk("%i",value2[counter]);
	//}
	//printk("\n");
	//PDEBUG("end %i\n",angle_encoder);
	at91_set_gpio_output(clock,1);
	for(counter=0;counter<=15;counter++){
		encoder = encoder | (value[counter]<<(15-counter));
		encoder2 = encoder2 | (value2[counter]<<(15-counter));
	}
	
	if(encoder !=encoder2){
		control.encoder_error=1;
	}
	angle_encoder = encoder & (0x0000ffff);
	//printk("encoder %d\n",angle_encoder);
	return angle_encoder;
}


int dac_write(int dac_number, unsigned short dacvalue){

	unsigned short mask;
	unsigned short temp;
	unsigned int add0,add1,add2,add3,data0,data1,data2,data3,cs0,cs1;
	mask = 1;
	cs0 = AT91_PIN_PB11;
	cs1 = AT91_PIN_PB16;
	
	//PDEBUG("%ld\n",dacvalue);

	add0 = AT91_PIN_PB1;
	add1 = AT91_PIN_PB2;
	add2 = AT91_PIN_PB3;
	add3 = AT91_PIN_PB4;

	data0 = AT91_PIN_PB5;
	data1 = AT91_PIN_PB8;
	data2 = AT91_PIN_PB9;
	data3 = AT91_PIN_PB10;	

	dacvalue = dacvalue>>4;
	
	at91_set_gpio_output(ad667.add0,1);
	at91_set_gpio_output(add1,1);
	at91_set_gpio_output(add2,1);
	at91_set_gpio_output(add3,1);
	at91_set_gpio_output(data0,1);
	at91_set_gpio_output(data1,1);
	at91_set_gpio_output(data2,1);
	at91_set_gpio_output(data3,1);
	at91_set_gpio_output(cs0,1);
	at91_set_gpio_output(cs1,1);

	at91_set_gpio_output(cs0,dac_number&mask);
	at91_set_gpio_output(cs1,dac_number&(mask<<1));

	at91_set_gpio_output(add0,0);
	at91_set_gpio_output(add1,1);
	at91_set_gpio_output(add2,1);
	at91_set_gpio_output(add3,1);

	at91_set_gpio_output(data0,(dacvalue&mask));
	
	at91_set_gpio_output(data1,(dacvalue>>1&(mask)));
	at91_set_gpio_output(data2,(dacvalue>>2&(mask)));
	at91_set_gpio_output(data3,(dacvalue>>3&(mask)));


	//PDEBUG("%ld %ld %ld %ld\n",(dacvalue&mask));
//	udelay(1000);
	at91_set_gpio_output(add0,1);
	at91_set_gpio_output(add1,0);
	at91_set_gpio_output(add2,1);
	at91_set_gpio_output(add3,1);



	at91_set_gpio_output(data0,(dacvalue>>4&mask));
	at91_set_gpio_output(data1,(dacvalue>>5&(mask)));
	at91_set_gpio_output(data2,(dacvalue>>6&(mask)));
	at91_set_gpio_output(data3,(dacvalue>>7&(mask)));
	//udelay(1000);

	at91_set_gpio_output(add0,1);
	at91_set_gpio_output(add1,1);
	at91_set_gpio_output(add2,0);
	at91_set_gpio_output(add3,1);

	at91_set_gpio_output(data0,(dacvalue>>8&mask));
	at91_set_gpio_output(data1,(dacvalue>>9&(mask)));
	at91_set_gpio_output(data2,(dacvalue>>10&(mask)));
	at91_set_gpio_output(data3,(dacvalue>>11&(mask)));
	//udelay(1000);
	
	at91_set_gpio_output(add0,1);
	at91_set_gpio_output(add1,1);
	at91_set_gpio_output(add2,1);
	at91_set_gpio_output(add3,0);
	//udelay(1000);

	return 1;

}


int update(void){

	int ret;
	
	init_ad5362();

	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH1<<16)|((control.cbuffer[0])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH2<<16)|((control.cbuffer[1])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH3<<16)|((control.cbuffer[2])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH4<<16)|((control.cbuffer[3])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH5<<16)|((control.cbuffer[4])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH6<<16)|((control.cbuffer[5])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH7<<16)|((control.cbuffer[6])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH8<<16)|((control.cbuffer[7])&(0x00ffff)));
	
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH1<<16)|((control.mbuffer[0])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH2<<16)|((control.mbuffer[1])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH3<<16)|((control.mbuffer[2])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH4<<16)|((control.mbuffer[3])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH5<<16)|((control.mbuffer[4])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH6<<16)|((control.mbuffer[5])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH7<<16)|((control.mbuffer[6])&(0x00ffff)));
	ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH8<<16)|((control.mbuffer[7])&(0x00ffff)));
	printk("DAC UPDATED\n");
	control.update=0;

	
	return 1;
}

int soft_limits(int azimuth_zone,unsigned int azimuth_encoder, unsigned short altitude_encoder,unsigned int azimuth_command,unsigned short altitude_command,unsigned int *az_command_out, unsigned short *alt_command_out){
	int azimuth_check, altitude_check;
	int output;
	int az_limit_lo,az_limit_hi, alt_limit_lo, alt_limit_hi;
	//from telescope_constants.h
	az_limit_lo = AZIMUTH_LIMIT_LO;
	az_limit_hi = AZIMUTH_LIMIT_HI;
	alt_limit_hi = ELEVATION_LIMIT_HI;
	alt_limit_lo = ELEVATION_LIMIT_LO;
	control.az_encoder_offset  =az_limit_lo;
	output =0;
	//azimuth_check = azimuth_encoder+65535*azimuth_zone;
	azimuth_check = azimuth_encoder;
	//udelay(50);
	altitude_check =altitude_encoder;
	if(azimuth_check <= az_limit_lo || azimuth_command<=az_limit_lo){
//		*(az_command_out) = az_limit_lo;
//		printk("AZIMUTH LIMIT STOP LO\n");
		if(azimuth_command<azimuth_encoder){
		output = 1;
		
		az_emergency_stop();
		}
	}
	
	if(azimuth_check >= az_limit_hi || azimuth_command>=az_limit_hi){
//		printk("AZIMUTH LIMIT STOP HI\n");
//		*(az_command_out) = az_limit_hi;
		if(azimuth_command>azimuth_encoder){
		output = 1;
		
		az_emergency_stop();
		}
	}

	if(altitude_check <= alt_limit_lo || altitude_command <= alt_limit_lo){
//		printk("ELEVATION LIMIT STOP LO\n");
//		*(alt_command_out) = alt_limit_lo;
		if(altitude_command<altitude_encoder){
		output = 2;
		
		alt_emergency_stop();
		}
	}
	if(altitude_check >= alt_limit_hi|| altitude_command >= alt_limit_hi){
//		*(alt_command_out) = alt_limit_hi;
//		printk("ELEVATION LIMIT STOP HI\n");
		if(altitude_command>altitude_encoder){
		output = 2;
		
		alt_emergency_stop();
		}
	}
//	printk("Az check %i %i %i %i\n",azimuth_encoder,azimuth_zone,azimuth_check,output);
	return output;
	
}
int emergency_stop(void){
	//this should reset all the DACs to zero and stop motor movement
	int ret;
	control.az1_dac = 0;
	control.az2_dac =0;
	control.alt1_dac = 0;
	control.alt2_dac = 0;
	//zero DACS manually too
	ret=dac5362_crc(WRITE_AD5362,0xd07fffbd);
	ret=dac5362_crc(WRITE_AD5362,0xd17fffd6);
	ret=dac5362_crc(WRITE_AD5362,0xd27fff6b);
	ret=dac5362_crc(WRITE_AD5362,0xd37fff00);
	ret=dac5362_crc(WRITE_AD5362,0xc87fff4e);
	ret=dac5362_crc(WRITE_AD5362,0xc97fff25);
	ret=dac5362_crc(WRITE_AD5362,0xca7fff98);
	ret=dac5362_crc(WRITE_AD5362,0xcb7ffff3);	
	printk("PID MODULE\nEMERGENCY STOP\nNO COMMUNICATION FROM PC CONTROL\nLOAD PC CONTROLLER\n");
	}
int az_emergency_stop(void){
	//this should reset all the DACs to zero and stop motor movement
	int ret;
	control.az1_dac = 0;
	control.az2_dac =0;
	control.alt1_dac = 0;
	control.alt2_dac = 0;
	//zero DACS manually too
	ret=dac5362_crc(WRITE_AD5362,0xd07fffbd);
	ret=dac5362_crc(WRITE_AD5362,0xd17fffd6);
	ret=dac5362_crc(WRITE_AD5362,0xd27fff6b);
	ret=dac5362_crc(WRITE_AD5362,0xd37fff00);
	ret=dac5362_crc(WRITE_AD5362,0xc87fff4e);
	ret=dac5362_crc(WRITE_AD5362,0xc97fff25);
	ret=dac5362_crc(WRITE_AD5362,0xca7fff98);
	ret=dac5362_crc(WRITE_AD5362,0xcb7ffff3);	
	//printk("PID MODULE\nEMERGENCY STOP\nNO COMMUNICATION FROM PC CONTROL\nLOAD PC CONTROLLER\n");
	}

int alt_emergency_stop(void){
	//this should reset all the DACs to zero and stop motor movement
	int ret;
	control.az1_dac = 0;
	control.az2_dac =0;
	control.alt1_dac = 0;
	control.alt2_dac = 0;
	//zero DACS manually too
	ret=dac5362_crc(WRITE_AD5362,0xd07fffbd);
	ret=dac5362_crc(WRITE_AD5362,0xd17fffd6);
	ret=dac5362_crc(WRITE_AD5362,0xd27fff6b);
	ret=dac5362_crc(WRITE_AD5362,0xd37fff00);
	ret=dac5362_crc(WRITE_AD5362,0xc87fff4e);
	ret=dac5362_crc(WRITE_AD5362,0xc97fff25);
	ret=dac5362_crc(WRITE_AD5362,0xca7fff98);
	ret=dac5362_crc(WRITE_AD5362,0xcb7ffff3);	
	//printk("PID MODULE\nEMERGENCY STOP\nNO COMMUNICATION FROM PC CONTROL\nLOAD PC CONTROLLER\n");
	}




module_init(mod_init);
module_exit(mod_exit); 

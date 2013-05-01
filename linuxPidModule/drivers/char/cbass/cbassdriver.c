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
#include "ad5362.h"
#include "ad7367.h"
#include "max7301.h"

#include "pid.h"
#include "atmel_spi.h"
#include "telescope_constants_kernel.h"
#include "crc_gen.h"
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


#define hardware_connected 1
#define interrupt_enable 0


#define SUCCESS 0






extern unsigned long loops_per_jiffy;
//for reading from the device;
static char msg[50];
static char *msg_Ptr;
static int Device_Open = 0;
volatile int timer_counter=0;
volatile unsigned int dac_out=0;
void __iomem *timer0_base;
void __iomem *adc_base;
void __iomem *spi1_base;
void __iomem *dmabuf;


unsigned int ad5362_crc_pack(unsigned int command,unsigned int channel,unsigned int value);
unsigned int ad5362_crc_pack1(unsigned int command,unsigned int channel,unsigned int value);
unsigned int check_limits(unsigned int az_encoder,unsigned int el_encoder,unsigned int AZMIN_VAL,unsigned int AZMAX_VAL,unsigned int ALTMIN_VAL,unsigned int ALTMAX_VAL);


ssize_t user_pid_read(struct file *filp, char __user *buffer,
                       size_t length, loff_t *offset);

ssize_t user_pid_write(struct file * file,char *buf, size_t count, loff_t *loft);

dma_addr_t dma_address1_coherent,dma_address2_coherent;
void *txbuffer_coherent,*rxbuffer_coherent;

volatile unsigned short dacout=0;

//static unsigned int daccounter=0;

//static int flag = 0; 
volatile int readflag = 0; 
volatile int delayflag = 0; 
volatile int wflag = 0;
volatile int stop_counter=0;
char Dac_Output;
struct pid_structure control;
struct device *dev_pid_ptr,dev_pid;

int update(void);
int emergency_stop(struct pid_structure *pid);
int az_emergency_stop(struct pid_structure *pid);
int alt_emergency_stop(struct pid_structure *pid);
int dac_write(int dac_number, unsigned short dacvalue);

unsigned short read_encoder(int encoder_number);

int soft_limits(int azimuth_zone,unsigned int azimuth_encoder, unsigned int altitude_encoder,unsigned int azimuth_command,unsigned int altitude_command,unsigned int *az_command_out, unsigned int *alt_command_out);

int tacho_check(int tacho1,int tacho2,int tacho3,int tacho4,int max,int min);

int get_antenna_readings(unsigned char command,unsigned int *azimuth_angle_long,unsigned short *azimuth_angle_short,unsigned int *altitude_angle_long,unsigned short *altitude_angle_short,int *azimuth_zone, int *tacho1,int *tacho2,int *tacho3,int *tacho4);

spinlock_t mylock = SPIN_LOCK_UNLOCKED;
spinlock_t tcounter = SPIN_LOCK_UNLOCKED;
spinlock_t delay = SPIN_LOCK_UNLOCKED;
spinlock_t flag_struct=SPIN_LOCK_UNLOCKED;
spinlock_t spilock = SPIN_LOCK_UNLOCKED;
spinlock_t encoder_lock = SPIN_LOCK_UNLOCKED;

static struct semaphore control_lock;
static struct semaphore max7301_lock;

/*FLAGS USED WITHIN THE MODULE TO INDICATE THE STATE*/
struct globalflags
{
	volatile char readflag;
	volatile char delayflag;
	volatile char wflag;
	volatile int stop_counter;
	volatile int timer_counter;
	volatile int timer_counter2;
	char interrupt_rate;
	volatile char Dac_Output;
	volatile char limit_zone;
	volatile char check_limits[2];
	volatile char timeout;
	volatile int timeout_counter;
	volatile int Interrupt_Mode;
	
}flags;


struct encoder_transfer
{
	long alt_encoder[3],az_encoder[3];
}encoder;

/*ENCODER DRIVER DEFINITIONS*/
struct encoder_driver
{
	unsigned int cs,clock,data;
};

struct ad667_driver
{
	unsigned int add0,add1,add2,add3,data0,data1,data2,data3,cs0,cs1;	
};

/* Operations for controling the device */
static const struct ad667_driver ad667 = 
{
     
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

 /* ENCODER DRIVER PIN DEFS */
static const struct encoder_driver angle = 
{
     //define pin config for the angle encoders
	.cs =  AT91_PIN_PB20,
	.clock = AT91_PIN_PB18,
	.data = AT91_PIN_PB19,
};

ssize_t user_pid_read(struct file *filp, char __user *buffer,size_t length, loff_t *offset)
{
}
/*this function handles writing the value to the module structure*/
ssize_t user_pid_write(struct file * file,char *buf, size_t count, loff_t *loft) 
{
	//unsigned long irq_state;
	unsigned long irq_state1;
	unsigned long irq_state2;
	int bytes_written =0 ;
	int ret;
	struct pid_structure update;
	bytes_written = copy_from_user(&update,buf,sizeof(update));
	
					//printk("HARDWARE CONNECTED\n");
					ret=dac5362_crc(WRITE_AD5362,update.az1_dac_control);	
					ret=dac5362_crc(WRITE_AD5362,update.az2_dac_control);
					ret=dac5362_crc(WRITE_AD5362,update.alt1_dac_control);	
					ret=dac5362_crc(WRITE_AD5362,update.alt2_dac_control);
		
  	return bytes_written;
};

/*IOCTL FUNCTION*/
int user_ioctl(struct inode *inode, struct file *file,unsigned int ioctl_num,unsigned long arg)
{	
	struct pid_structure update;
	//unsigned short alt_temp[5];
	//unsigned short az_temp[5];
	//int *adc1,*adc2,output[100],output1[100];
	unsigned long irq_state1;
	unsigned long irq_state2;
	int bytes_written =0 ;
	int bytes_read = 0;
	int retval=-1;
	int len=0;
	int dflag;
	int i;
	unsigned int ret;
	unsigned int temp;
	unsigned int max7301[5];
	unsigned int max7301RX[5];
	unsigned int ad5362[10];
	int tacho[10];
	unsigned int adc1,adc2;
	signed short tac1,tac2,tac3,tac4;
	int tacho1,tacho2,tacho3,tacho4;
	len = sizeof(control);
	int output[100],output1[100];
	unsigned short alt_temp[5];
	unsigned short az_temp[5];

	//this is called using a pointer to the userspace control structure as the argument//
	switch(ioctl_num){
	/*write control structure*/
	case DEV_IOCTL_WRITE_CONTROL_STRUCTURE:
		bytes_written=copy_from_user(&update,(void *)arg,sizeof(update));
 		spin_lock_irqsave(&mylock,irq_state1);
 			control=update;
 		spin_unlock_irqrestore(&mylock,irq_state1);
		retval = bytes_written;
		break;

	/*read control structure*/
	case DEV_IOCTL_READ_CONTROL_STRUCTURE:
		spin_lock_irqsave(&mylock,irq_state1);
			control.ioctl_num++;
			get_antenna_readings(hardware_connected,&control.az_encoder_long,&control.az_encoder,&control.alt_encoder_long,&control.az_encoder,&control.azimuth_zone, &control.tacho1,&control.tacho2,&control.tacho3,&control.tacho4);
			do_gettimeofday(&(control.time_struct));
			update = control;
		spin_unlock_irqrestore(&mylock,irq_state1);
		bytes_read = copy_to_user((void *)arg,&update,sizeof(update));
		
		retval = len-bytes_read;
		break;

	case DEV_IOCTL_WRITE_AZIMUTH_ZONE:
		bytes_written=copy_from_user(&update.azimuth_zone,(void *)arg,sizeof(update.azimuth_zone));
		spin_lock_irqsave(&mylock,irq_state1);
			control.azimuth_zone = update.azimuth_zone;
			printk("Setting Azimuth Zone %d\n",control.azimuth_zone);
			//	control.azimuth_zone =update.azimuth_zone;
		spin_unlock_irqrestore(&mylock,irq_state1);
		retval = bytes_written;
		break;

	case DEV_IOCTL_READ_AZIMUTH_ZONE:
		spin_lock_irqsave(&mylock,irq_state1);
			printk("\n-------------------------Current Azimuth Zone is %d\n------------------------\n",control.azimuth_zone);	
			update.azimuth_zone = control.azimuth_zone;	
		spin_unlock_irqrestore(&mylock,irq_state1);
		bytes_read=copy_to_user((void *)arg,&update.azimuth_zone,sizeof(update.azimuth_zone));
		retval = len-bytes_read;
		break;

	case DEV_IOCTL_WRITE_PID:
		bytes_written=copy_from_user(&update,(void *)arg,sizeof(update));
		spin_lock_irqsave(&mylock,irq_state1);
			control.az1_dac_control = update.az1_dac_control;
			control.az2_dac_control = update.az2_dac_control;
			control.alt1_dac_control = update.alt1_dac_control;
			control.alt2_dac_control = update.alt2_dac_control;
			control.az_command_long = update.az_command_long;
			control.alt_command_long = update.alt_command_long;
		spin_unlock_irqrestore(&mylock,irq_state1);
		retval = bytes_written;
		break;

	case DEV_IOCTL_WAIT:
		
		spin_lock_irqsave(&flag_struct,irq_state1);
				flags.readflag=1;
		spin_unlock_irqrestore(&flag_struct,irq_state1);
		wait_event_interruptible_timeout(wq, flags.wflag!=0,20000);
		spin_lock_irqsave(&flag_struct,irq_state1);
			flags.wflag=0;
		spin_unlock_irqrestore(&flag_struct,irq_state1);
 		
		spin_lock_irqsave(&mylock,irq_state1); 
			update=control;
 		spin_unlock_irqrestore(&mylock,irq_state1);
		bytes_read = copy_to_user((void *)arg,&update,sizeof(update));
		retval = len-bytes_read;
		break;

	case DEV_IOCTL_ENABLE_DAC:
		printk("ENABLE DAC\n");
		spin_lock_irqsave(&flag_struct,irq_state1);
			flags.Dac_Output=25;
			printk("ENABLE DAC %d\n",flags.Dac_Output);
		spin_unlock_irqrestore(&flag_struct,irq_state1);
		mdelay(10);
		
		
	break;

	case DEV_IOCTL_DISABLE_DAC:
		printk("DISABLE DAC\n");
		
			//printk("DISABLE DAC %d\n",flags.Dac_Output);
			spin_lock_irqsave(&mylock,irq_state1);
				spin_lock_irqsave(&flag_struct,irq_state1);
					flags.Dac_Output=0;
				spin_unlock_irqrestore(&flag_struct,irq_state1);
				temp = ad5362_crc_pack(XREGISTER_WRITE,ALL,0);
				temp = ad5362_crc_pack(XREGISTER_WRITE,ALL,0);
				temp = ad5362_crc_pack(XREGISTER_WRITE,ALL,0);
				temp = ad5362_crc_pack(XREGISTER_WRITE,ALL,0);
				ret=dac5362_crc(WRITE_AD5362,temp);	
				ret=dac5362_crc(WRITE_AD5362,temp);	
				ret=dac5362_crc(WRITE_AD5362,temp);
				ret=dac5362_crc(WRITE_AD5362,temp);		
				
			spin_unlock_irqrestore(&mylock,irq_state1);
			mdelay(10);
			
			
		
	break;

	case DEV_IOCTL_SET_DAC_REGISTERS:
		bytes_written=copy_from_user(&update,(void *)arg,sizeof(update));
		spin_lock_irqsave(&mylock,irq_state1);
			for(i=0;i<=7;i++){
				control.cbuffer_crc[i] = update.cbuffer_crc[i];
				control.mbuffer_crc[i] = update.mbuffer_crc[i];
				//ret=dac5362(WRITE_AD5362,(MREGISTER_WRITE<<20)|(CH1<<16)|((control.cbuffer[0])&(0x00ffff)));
 				//ret=dac5362(WRITE_AD5362,control.cbuffer[i]);
 				//ret=dac5362(WRITE_AD5362,control.mbuffer[i]);
				ret=dac5362_crc(WRITE_AD5362,control.cbuffer_crc[i]);
				ret=dac5362_crc(WRITE_AD5362,control.mbuffer_crc[i]);
 			}
		//	printk("DAC Return %08x\n",temp);
// 			temp=0;
// 			temp = spi_read_ad5362(0x00050800);
// 			printk("DAC Return %08x\n",temp);
// 			temp=0;
// 			temp = spi_read_ad5362(0x00054800);
// 			printk("DAC Return %08x\n",temp);
// 			temp=0;
// 			temp = spi_read_ad5362(0x00056800);
// 			printk("DAC Return %08x\n",temp);
// 			temp=0;
// 			temp = spi_read_ad5362(0x00052800);
// 			printk("DAC Return %08x\n",temp);
// 			printk("\n");
		spin_unlock_irqrestore(&mylock,irq_state1);
	break;

	case DEV_IOCTL_WRITE_AD5362:
		//if(down_interruptible(&max7301_lock))
			
 			//init_ad5362();
			//printk("WRITE ad5362_write\n");
			bytes_written=copy_from_user(&ad5362,(void *)arg,sizeof(ad5362));
			ret=dac5362_crc(WRITE_AD5362,ad5362[0]);
			//ret=dac5362(WRITE_AD5362,(XREGISTER_WRITE<<20)|(ALL<<16)|((-32000+32767)&(0x00ffff)));	
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[1]);
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[2]);
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[3]);
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[4]);
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[5]);
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[6]);
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[7]);
// 			ret=dac5362_crc(WRITE_AD5362,ad5362[8]);
			
			//printk("WRITE MAX7301aa %04x %04x %04x\n",max7301[0],max7301[1],max7301[2]);
		//up(&max7301_lock);
	break;

	case DEV_IOCTL_READ_AD5362:
			//ret = spi_read_ad5362(0x054400);
			bytes_written=copy_from_user(&ad5362,(void *)arg,sizeof(ad5362));
				ret = dac5362_crc(READ_AD5362,ad5362[1]);
			bytes_read = copy_to_user((void *)arg,&ret,sizeof(ret));
			retval = len-bytes_read;
			//printk("READ ad5362 Ret %08x \n",ret);
	break;

	case DEV_IOCTL_WRITE_MAX7301:
		if(down_interruptible(&max7301_lock))
			printk("Received Interrupt signal on MAX7301 Semaphore\n");

			//printk("WRITE MAX7301\n");
			bytes_written=copy_from_user(&max7301,(void *)arg,sizeof(max7301));
			write_spi_max7301(max7301[0],max7301[1],max7301[2]);
			
			//printk("WRITE MAX7301aa %04x %04x %04x\n",max7301[0],max7301[1],max7301[2]);
		up(&max7301_lock);
	break;

	case DEV_IOCTL_READ_MAX7301:
		if(down_interruptible(&max7301_lock))
			printk("Received Interrupt signal on DEV_IOCTL_READ_MAX7301 Semaphore\n");

			//printk("READ MAX7301\n");
			
			bytes_written=copy_from_user(&max7301,(void *)arg,sizeof(max7301));
		
			read_spi_max7301(max7301[0],max7301[1],max7301[2],max7301RX);
			//read_spi_max7301(max7301[0],max7301[1],max7301[2],max7301RX);
			//printk("READ MAX7301aa\n");
			bytes_read=copy_to_user((void *)arg,&max7301RX,sizeof(max7301RX));
		up(&max7301_lock);
		//retval = sizeof(max7301)-bytes_read;
	break;

	case DEV_IOCTL_READ_AD7367:
		//	printk("READ ad7367\n");
			adc1 = spi_out_ad7367(0,(int *)output);
			adc2 = spi_out_ad7367(1,(int *)output1);
			tac1 = ((*(output+0))&(0x0000ffff));
			tac3 = ((*(output+1))&(0x0000ffff));
			tac4 = ((*(output1+0))&(0x0000ffff));
			tac2= ((*(output1+1))&(0x0000ffff));
			tacho[0] = (int)tac1;
			tacho[1] = (int)tac2;
			tacho[2] = (int)tac3;
			tacho[3] = (int)tac4;
			bytes_read = copy_to_user((void *)arg,&tacho,sizeof(tacho));
			printk("Tacho %d %d %d %d\n",tacho[0],tacho[1],tacho[2],tacho[3]);
	break;

	case DEV_IOCTL_READ_ENCODER:
			alt_temp[0] = read_encoder(0);
			alt_temp[1] = read_encoder(1);
			alt_temp[2] = control.azimuth_zone;
			//printk("Angle Encoder %d %d\n",alt_temp[0],alt_temp[1]);
			bytes_read = copy_to_user((void *)arg,&alt_temp,sizeof(alt_temp));
	break;
	}
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
	init_ad5362();
	
		
	

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

	
	

	Device_Open--;		/* We're now ready for our next caller */
	printk(KERN_ALERT "-----------------------------\n---------------------Closing a channel to C-BASS Device Driver---------------------\n");
	
//	__raw_writel(AT91_TC_COVFS,timer0_base+AT91_TC_IDR);
//	at91_sys_write(AT91_AIC_IDCR,1<<AT91SAM9260_ID_TC0);
//	free_irq(irq, (void*)4);
		/* 
	 * Decrement the usage count, or else once you opened the file, you'll
	 * never get get rid of the module. 
	 */
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
	unsigned int ret,res;
	unsigned int irq;
	unsigned long irq_state1;
	unsigned long irq_state2;
	//flag=1;
//	struct spi_master	*master;
	//struct CONTROLLER	*c;

	dev_pid_ptr=&dev_pid;

	printk(KERN_ALERT "Loading the C-BASS driver module\n");
	printk("\n-------------------------Current Azimuth Zone is %d\n------------------------\n",control.azimuth_zone);
	msg_Ptr=msg;
	//map the timer memory
	timer0_base = ioremap(AT91SAM9260_BASE_TC0,SZ_16K);
	adc_base = ioremap(AT91SAM9260_BASE_ADC,SZ_16K);
	spi1_base = ioremap(AT91SAM9260_BASE_SPI1,SZ_16K);	

	spin_lock_init(&mylock);
	spin_lock_init(&tcounter);
	spin_lock_init(&delay);
	spin_lock_init(&flag_struct);
	spin_lock_init(&spilock);
	init_MUTEX(&max7301_lock);


	init_ad5362();

	
		get_antenna_readings(hardware_connected,&control.az_encoder_long,&control.az_encoder,&control.alt_encoder_long,&control.az_encoder,&control.azimuth_zone, &control.tacho1,&control.tacho2,&control.tacho3,&control.tacho4);
		printk("Starting Angle Encoder Values\nAZIMUTH %d\nALTITUDE %d\n",control.az_encoder,control.alt_encoder);
	
	
		
		
		
	
	


	irq = AT91SAM9260_ID_TC0;
	
//------------------------------------------------------
	
 	
	

 	ret = misc_register(&pid);
	
	return 0;


}

/*Remove  the moodule*/
int mod_exit(void)
{
	int ret;
	unsigned int irq;
	//unmap the memory region
//	dma_free_coherent(&dev_pid, 1000,&dma_address1_coherent,GFP_ATOMIC);
//	dma_free_coherent(&dev_pid, 1000,&dma_address2_coherent,GFP_ATOMIC);
	
	
	printk("Unloading the C-BASS driver\n");
	printk("\n-------------------------Current Azimuth Zone is %d\n------------------------\n",control.azimuth_zone);
	printk("\nPLEASE RELOAD THIS ON THE FOLLOWING MODULE LOADING USING ./azimuth zone %d \n",control.azimuth_zone);	
	
	
 	ret = misc_deregister(&pid);
 	
	

	iounmap(adc_base);
	iounmap(spi1_base);
	iounmap(timer0_base);
	return ret;
	//if(ret !=0)
	//	printk("Module Deregistering Failed\n");
}

unsigned short read_encoder(int encoder_number)
{

	int clock,data,cs;
	int counter =0;
	int temp;
//	int mask=1;
	unsigned int encoder=0;
	unsigned int encoder2=0;
	unsigned short angle_encoder=0;
	unsigned long irq_state;
	int state_time;
	char value[20];
	char value2[20];
	int wavelength;

	wavelength = control.encoder_wavelength;
	wavelength = 10;
	state_time = wavelength>>1;
	spin_lock_irqsave(&encoder_lock,irq_state);
	
		at91_sys_write(AT91_PIOB+PIO_PER,(1<<20)|(1<<24)|(1<<23));
			
		cs =  AT91_PIN_PB20;
		clock = AT91_PIN_PB24;
		data = AT91_PIN_PB23;
	
		control.encoder_error=0;
		//at91_set_gpio_output(angle.cs,1);
		at91_set_gpio_output(clock,1);
		at91_set_gpio_output(cs,encoder_number);
		udelay(10);
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
	//udelay(20);
	spin_unlock_irqrestore(&encoder_lock,irq_state);
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

int dac_write(int dac_number, unsigned short dacvalue)
{

	unsigned short mask;
//	unsigned short temp;
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

int update(void)
{

	int ret;
	unsigned int temp[8],cbuf[8],mbuf[8];
	unsigned int i;
	unsigned long irq_state1;
	unsigned long irq_state2;
	init_ad5362();
	
	//control.alt1_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH7,-5000);
	spin_lock_irqsave(&mylock,irq_state1);
	//temp[0] =ad5362_crc_pack(CREGISTER_WRITE,CH1,cbuf[0]);
	//temp[1] =ad5362_crc_pack(CREGISTER_WRITE,CH2,cbuf[1]);
	//temp[2] =ad5362_crc_pack(CREGISTER_WRITE,CH1,cbuf[2]);
	for(i=0;i<=7;i++){
		control.cbuffer[i]=0x8000;
		control.mbuffer[i]=0xffff;
		
	}
	
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
	spin_unlock_irqrestore(&mylock,irq_state1);
	
	return 1;
}

int soft_limits(int azimuth_zone,unsigned int azimuth_encoder, unsigned int altitude_encoder,unsigned int azimuth_command,unsigned int altitude_command,unsigned int *az_command_out, unsigned int *alt_command_out)
{
	long azimuth_check, altitude_check;
	int output;
	unsigned int az_limit_lo,az_limit_hi, alt_limit_lo, alt_limit_hi;
	//from telescope_constants.h
	az_limit_lo = AZIMUTH_LIMIT_LO;
	az_limit_hi = AZIMUTH_LIMIT_HI;
	alt_limit_hi = ELEVATION_LIMIT_HI;
	alt_limit_lo = ELEVATION_LIMIT_LO;
	control.az_encoder_offset  =az_limit_lo;
	output =0;
	//azimuth_check = azimuth_encoder+65535*azimuth_zone;
	azimuth_check = (long)azimuth_encoder;
	//udelay(50);
	altitude_check =(long)altitude_encoder;
	*az_command_out = azimuth_command;
	*alt_command_out = altitude_command;
	if(azimuth_check < az_limit_lo || azimuth_command<az_limit_lo){
//		*(az_command_out) = az_limit_lo;
//		printk("AZIMUTH LIMIT STOP LO\n");
		if(azimuth_command<azimuth_encoder){
		printk("Azimuth _error lo %ld %u %u\n",azimuth_check,azimuth_command,azimuth_encoder);
		output = 1;
		*az_command_out = az_limit_lo;
		
		//az_emergency_stop();
		}
	}
	
	if(azimuth_check > az_limit_hi || azimuth_command>az_limit_hi){
//		printk("AZIMUTH LIMIT STOP HI\n");
//		*(az_command_out) = az_limit_hi;
		if(azimuth_command>azimuth_encoder){
		printk("Azimuth _error hi %ld %u %u\n",azimuth_check,azimuth_command,azimuth_encoder);
		*az_command_out = az_limit_hi;
		output = 1;
		
		//az_emergency_stop();
		}
	}

	if(altitude_check < alt_limit_lo || altitude_command < alt_limit_lo){
//		printk("ELEVATION LIMIT STOP LO\n");
//		*(alt_command_out) = alt_limit_lo;
		if(altitude_command<altitude_encoder){
		printk("Altitude _error lo %ld %u %u\n",altitude_check,altitude_command,altitude_encoder);
		output = 2;
		*alt_command_out = alt_limit_lo;
		
		//alt_emergency_stop();
		}
	}
	if(altitude_check > alt_limit_hi|| altitude_command > alt_limit_hi){
//		*(alt_command_out) = alt_limit_hi;
//		printk("ELEVATION LIMIT STOP HI\n");
		if(altitude_command>altitude_encoder){
		printk("Altitude _errorhi  %ld %u %u\n",altitude_check,altitude_command,altitude_encoder);
		output = 2;
		*alt_command_out = alt_limit_hi;
		//alt_emergency_stop();
		}
	}
//	printk("Az check %i %i %i %i\n",azimuth_encoder,azimuth_zone,azimuth_check,output);
//	printk("Output  %d %d\n",*az_command_out, *alt_command_out);
	return output;
	
}

int emergency_stop(struct pid_structure *controli)
{
	//this should reset all the DACs to zero and stop motor movement
	int ret;
	struct pid_structure controlj;
	controlj = *controli;

	init_ad5362();
	controlj.az1_dac = 0;
	controlj.az2_dac =0;
	controlj.alt1_dac = 0;
	controlj.alt2_dac = 0;
 	controlj.alt1_dac_control = 0xc07fff00;
 	controlj.alt2_dac_control = 0xc07fff00;
 	controlj.az1_dac_control = 0xc07fff00;
 	controlj.az2_dac_control =  0xc07fff00;
//	controlj.DAC_Output = 0;
	*controli=controlj;
	
	//zero DACS manually too
	ret=dac5362_crc(WRITE_AD5362,0xd07fffbd);
	ret=dac5362_crc(WRITE_AD5362,0xd17fffd6);
	ret=dac5362_crc(WRITE_AD5362,0xd27fff6b);
	ret=dac5362_crc(WRITE_AD5362,0xd37fff00);
	ret=dac5362_crc(WRITE_AD5362,0xc87fff4e);
	ret=dac5362_crc(WRITE_AD5362,0xc97fff25);
	ret=dac5362_crc(WRITE_AD5362,0xca7fff98);
	ret=dac5362_crc(WRITE_AD5362,0xcb7ffff3);
	ret=dac5362_crc(WRITE_AD5362,DAC_ZERO_ALL);
	printk("PID MODULE\nEMERGENCY STOP\nNO COMMUNICATION FROM PC CONTROL\nLOAD PC CONTROLLER\n");
	return 0;
	}

int az_emergency_stop(struct pid_structure *controli)
{
	//this should reset all the DACs to zero and stop motor movement
	int ret;
	
	struct pid_structure controlj;
	controlj = *controli;

	controlj.az1_dac = 0;
	controlj.az2_dac =0;
	controlj.alt1_dac = 0;
	controlj.alt2_dac = 0;
	*controli=controlj;

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
	return 0;
	}

int alt_emergency_stop(struct pid_structure *controli)
{
	//this should reset all the DACs to zero and stop motor movement
	int ret;
	
	struct pid_structure controlj;
	controlj = *controli;

	controlj.az1_dac = 0;
	controlj.az2_dac =0;
	controlj.alt1_dac = 0;
	controlj.alt2_dac = 0;
	*controli=controlj;
	
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
	return 0;
	}

int get_antenna_readings(unsigned char command,unsigned int *azimuth_angle_long,unsigned short *azimuth_angle_short,unsigned int *altitude_angle_long,unsigned short *altitude_angle_short,int *azimuth_zone, int *tacho1,int *tacho2,int *tacho3,int *tacho4)
{
	
	int output[100],output1[100];
	
	unsigned int adc1,adc2;
	int retval=55;
	signed short tac1,tac2,tac3,tac4;
	unsigned short alt_temp[5];
	unsigned short az_temp[5];
	int azimuth_test;
	unsigned short previous_azimuth_angle;
	int az_zone;
//	spin_lock_irqsave(&mylock,irq_state1);
		previous_azimuth_angle = *azimuth_angle_short;
		alt_temp[0] =50;
		alt_temp[1] =0;
		az_temp[0]= 50;
		az_temp[1]=0;
		az_zone = *(azimuth_zone);
		//printk("Azimuth Zone %d\n",*azimuth_zone);
	
	
		while(abs(alt_temp[0]-alt_temp[1])>20){
			alt_temp[0] = read_encoder(1);
			alt_temp[1] = read_encoder(1);
			
		}
		while(abs(az_temp[0]-az_temp[1])>20){
			az_temp[0] = read_encoder(0);
			az_temp[1] = read_encoder(0);
		}
		*azimuth_angle_short = az_temp[0];
		azimuth_test = (long)(previous_azimuth_angle) - (long)az_temp[0];
		//check azimuth zone of operation
	
	// 	if(az_zone<0 && (az_zone<=90)){
	// 		printk("AZ Zone error 1 AZ ZOne %d\n",az_zone);
	// 		az_zone=0;
	// 	}
	// 	if(az_zone>2 && (az_zone<90)){
	// 		printk("AZ Zone error 2 AZ ZOne %d\n",az_zone);
	// 		az_zone=2;
	// 		
	// 	}
	
		if(azimuth_test < -33000 && az_zone<=2){
			az_zone--;
			printk("Azimuth_test Negative %d %d\n",azimuth_test,previous_azimuth_angle);
			printk("AZ --\n");
			printk("Azimuth Zone Changing From %d -> %d\n",*azimuth_zone,az_zone);
		}
		if(azimuth_test >33000 && az_zone<=2 ){
			az_zone++;
			printk("Azimuth_test Positive %d %d\n",azimuth_test,previous_azimuth_angle);
			printk("AZ ++\n");
			printk("Azimuth Zone Changing From %d -> %d\n",*azimuth_zone,az_zone);
		}
		
	//	*azimuth_angle=az_temp[0];
		if(az_zone<=2){
		*azimuth_angle_long = (unsigned int)az_temp[0] +65535*az_zone;
		*altitude_angle_long =(unsigned int)alt_temp[0];
		*azimuth_zone = az_zone;
		}
	//	printk("Encoder  %d %d %d %d\n",*azimuth_angle_long, *altitude_angle_long,*azimuth_zone,az_zone);
		if(command==1){
			adc1 = spi_out_ad7367(0,(int *)output);
			adc2 = spi_out_ad7367(1,(int *)output1);
		}
		else if(command==0){
			output[0] = 0xffffffff;
			output[1] = 0xffffffff;
			output1[0] = 0xffffffff;
			output1[1] = 0xffffffff;
		}
		//adc1 = spi_out_ad7367_bit_banging(0,output);
		//adc2 = spi_out_ad7367_bit_banging(1,output1);
		//printk("ADC1 %ld ADC2 %ld\n",*output,*output1);
		tac1 = ((*(output+0))&(0x0000ffff));
		tac3 = ((*(output+1))&(0x0000ffff));
		tac4 = ((*(output1+0))&(0x0000ffff));
		tac2= ((*(output1+1))&(0x0000ffff));
		*tacho1 = (int)tac1;
		*tacho2 = (int)tac2;
		*tacho3 = (int)tac3;
		*tacho4 = (int)tac4;
	
		if(az_zone>2){
		*azimuth_zone =100;
		retval = 1;
		}
		else if(az_zone<=2)
		retval = 0;

//	spin_unlock_irqrestore(&mylock,irq_state1);
	return retval;
} 

int tacho_check(int tacho1,int tacho2,int tacho3,int tacho4,int max,int min)
{
	int return_val=0;

	if(tacho1 > max){
		return_val= 1;
	}

	if(tacho2 > max){
		return_val= 2;
	}

	if(tacho3 > max){
		return_val= 3;
	}

	if(tacho1 > max){
		return_val= 4;
	}

	if(tacho1 < min){
		return_val =1;
	}

	if(tacho2 < min){
		return_val =2;
	}

	if(tacho3 < min){
		return_val =3;
	}

	if(tacho1 < min){
		return_val =4;
	}
	return return_val;
}

unsigned int ad5362_crc_pack(unsigned int command,unsigned int channel,unsigned int value)
{
	//this function takes inputs and prepares a suitable packet to be transmitted to the ad5362 DAC. Includeds a CRC for safety so will produce a four byte value stored in the unsigned int.
	unsigned char test_vec[10];
	unsigned int retval;
	unsigned int packet;
	 crc_t crc;
	packet = (command<<20)|(channel<<16)|(((unsigned short)value+32767)&(0x00ffff));
	test_vec[0]=(packet&0xff000000)>>24;
	test_vec[1]=(packet&0x00ff0000)>>16;
	test_vec[2]=(packet&0x0000ff00)>>8;
	test_vec[3]=(packet&0x000000ff);
	crc = crc_init();
   	crc = crc_update(crc, test_vec, 4);
	crc = crc_finalize(crc);
	retval = (packet<<8)|(crc&(0x000000ff));
	return retval;
}

unsigned int ad5362_crc_pack1(unsigned int command,unsigned int channel,unsigned int value)
{
	//this function takes inputs and prepares a suitable packet to be transmitted to the ad5362 DAC. Includeds a CRC for safety so will produce a four byte value stored in the unsigned int.
	unsigned char test_vec[10];
	unsigned int retval;
	unsigned int packet;
	 crc_t crc;
	packet = (command<<20)|(channel<<16)|(((unsigned short)value)&(0x00ffff));
	test_vec[0]=(packet&0xff000000)>>24;
	test_vec[1]=(packet&0x00ff0000)>>16;
	test_vec[2]=(packet&0x0000ff00)>>8;
	test_vec[3]=(packet&0x000000ff);
	crc = crc_init();
   	crc = crc_update(crc, test_vec, 4);
	crc = crc_finalize(crc);
	retval = (packet<<8)|(crc&(0x000000ff));
	return retval;
}	

unsigned int check_limits(unsigned int az_encoder,unsigned int alt_encoder,unsigned int AZMIN_VAL,unsigned int AZMAX_VAL,unsigned int ALTMIN_VAL,unsigned int ALTMAX_VAL)
{

	unsigned int retval=0;
	//printk("AZ ENCODER%d \n",az_encoder);
	//printk("ALT ENCODER%d \n",alt_encoder);
	if(az_encoder >=AZMAX_VAL){
		retval =2;
		printk("AZ LIMIT ENCODER MAXED %d \n",az_encoder);
	}
	else if (az_encoder<=AZMIN_VAL){
		retval =1;
		printk("AZ LIMIT ENCODER MIN %d \n",az_encoder);
	}
	
	if(alt_encoder >=ALTMAX_VAL){
		retval =4;
		printk("ALT LIMIT ENCODER MAX %d \n",alt_encoder);
	}
	else if (alt_encoder<=ALTMIN_VAL){
		retval =3;
		printk("ALT LIMIT ENCODER MIN %d \n",alt_encoder);
	}

	return retval;
}

module_init(mod_init);
module_exit(mod_exit); 

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Charles Copley");
MODULE_DESCRIPTION("C-BASS CONTROLLER AT91SAM9260");
MODULE_ALIAS("platform:at91_spi");

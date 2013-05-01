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



//#define adc_base (0xfffe0000)

//this defines the structure used to copy data between kernel and userspace- check that this file is the same in both ALWAYS!!

#include "pid.h"
#include "atmel_spi.h"

/* Deal with CONFIG_MODVERSIONS */
// #if CONFIG_MODVERSIONS==1
// #define MODVERSIONS
// #include <linux/modversions.h>
// #endif 


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
volatile unsigned short dacout=0;

static unsigned int daccounter=0;

static int flag = 0; 
static volatile int readflag = 0; 

void init_ad5362(void);
int dac_write(int dac_number, unsigned short dacvalue);
unsigned short read_encoder(int encoder_number);
int dac5362_write(int dac_number, unsigned short dacvalue);


int spi_out_ad5362(unsigned short value);
int spi_read_ad5362(unsigned short value);


struct encoder_transfer{
	long alt_encoder[3],az_encoder[3];
}encoder;

struct encoder_driver{
	
	unsigned int cs,clock,data;
	
};







struct ad667_driver{

	unsigned int add0,add1,add2,add3,data0,data1,data2,data3,cs0,cs1;	

};


struct ad5362_driver{
	unsigned int clear,reset,busy,ldac,sdo,sdi,sck,sync;
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
	.sync		=AT91_PIN_PB3	,
	.ncpha 		=(0<<1),
	.cpol		=(0<<0),
	.mstr 		=AT91_SPI_MSTR,
	.spien		=AT91_SPI_SPIEN,
	.spidis		=AT91_SPI_SPIDIS,
	.ps		=AT91_SPI_PS_VARIABLE,
	//.ps		=AT91_SPI_PS_FIXED,
	.pcsdec		=(0<<2),
	.dlybcs		=(30<<24),
	.pcs		=0<<16,
	//.lastxfer	=AT91_SPI_LASTXFER,
	.lastxfer	=1<<24,
	.csaat		=(0<<3),
	.bits		=AT91_SPI_BITS_8,
	.scbr		=(20 <<  8),
	.dlybs		=(20 << 16),
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
	unsigned int temp;
	int ret;
	int led_level;
	temp = at91_sys_read(AT91_AIC_IVR);
	//PDEBUG("IRQ handled\n");
	
	//temp = gpio_get_value(AT91_PIN_PB0);
	led_level=0x00;
	//toggle the led using EOR
	led_level = temp^0x01;
	//at91_set_gpio_value(AT91_PIN_PB0,led_level);

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
	if(timer_counter >=12){
		
		daccounter++;
		timer_counter=0;
		control.counter++;
	//PDEBUG("%lu\n",control.alt_adc);
			if(readflag==1){
			//flag gets set to zero by the read process
			readflag =0;
			flag = 1;     
			control.time=__raw_readl(timer0_base+AT91_TC_CV); 
			control.alt_adc = __raw_readl(adc_base+AT91_ADC_CHR(0));
			control.az_adc = 100;
			encoder.alt_encoder[1]=encoder.alt_encoder[2];	
			encoder.alt_encoder[2]=encoder.alt_encoder[3];
			encoder.alt_encoder[3]=control.alt_encoder;
			
			//simulated movement of the angle encoders for now- needs to be changed on the real thing
			control.alt_encoder += (short)control.alt_simulation; 
			control.az_encoder += (short)control.az_simulation; 
			
			control.alt_encoder = read_encoder(0);
			control.az_encoder = read_encoder(1);
			

		//	PDEBUG("angle_encoder %04x\n",ret);
		//	init_ad5362();
			

//  			ret = dac_write(0,control.alt_command);
//  			ret = dac_write(1,control.alt_command);
//  			ret = dac_write(2,control.alt_command);
//  			ret = dac_write(3,control.alt_command);
			dacout+=10;
			ret=dac5362_write(0,dacout);
			//ret = spi_read_ad5362(dacout);
			//ret = dac_write(0,control.alt_encoder);
			//ret = dac_write(1,control.az_encoder);
			//ret = dac_write(2,control.alt_encoder);
			//ret = dac_write(3,control.az_encoder);

			//PDEBUG("encoder %u alt_dac %d \n",control.alt_encoder,control.alt_dac);
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

	msg_Ptr=msg;
	//map the timer memory
	timer0_base = ioremap(AT91SAM9260_BASE_TC0,SZ_16K);
	adc_base = ioremap(AT91SAM9260_BASE_ADC,SZ_16K);
	spi1_base = ioremap(AT91SAM9260_BASE_SPI1,SZ_16K);	
	//dmabuf = ioremap (0x0f00000 /*14M*/, SZ_16K /* 1M */);
	//spi_access_bus(1);
	//at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));


	//reset the SPI interface
	init_ad5362();

	control.alt_adc = 0;
	control.az_adc=0;
	control.alt_command=0;
	control.az_command=0;
	control.alt_dac=0;
	control.az_dac=0;
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
	//unmap the memory region
	
	unsigned int irq;
	irq = AT91SAM9260_ID_TC0;
	printk("Unloading the ADC driver\n");
	__raw_writel(AT91_TC_COVFS,timer0_base+AT91_TC_IDR);
	at91_sys_write(AT91_AIC_IDCR,1<<AT91SAM9260_ID_TC0);
 	ret = misc_deregister(&pid);
 	free_irq(irq, (void*)4);
	
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
	unsigned short angle_encoder=0;
	cs =  AT91_PIN_PB20;
	clock = AT91_PIN_PB18;
	data = AT91_PIN_PB19;

	at91_set_gpio_output(angle.cs,1);
	at91_set_gpio_output(clock,1);
	at91_set_gpio_input(data, 1);
	at91_set_gpio_value(cs,encoder_number);
//	at91_set_gpio_value(clock,1);


	for(counter=0;counter<=16;counter++){
	at91_set_gpio_value(clock,0);
	udelay(5);
	at91_set_gpio_value(clock,1);
	udelay(5);
	temp = at91_get_gpio_value(data);
//	PDEBUG("Angle Encoder %04x\n",temp);
	angle_encoder = angle_encoder | (mask&temp);
	angle_encoder =angle_encoder<<1;
	}
	udelay(30);

//	PDEBUG("end %04x\n",angle_encoder);
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

	at91_set_gpio_value(cs0,dac_number&mask);
	at91_set_gpio_value(cs1,dac_number&(mask<<1));

	at91_set_gpio_value(add0,0);
	at91_set_gpio_value(add1,1);
	at91_set_gpio_value(add2,1);
	at91_set_gpio_value(add3,1);

	at91_set_gpio_value(data0,(dacvalue&mask));
	
	at91_set_gpio_value(data1,(dacvalue>>1&(mask)));
	at91_set_gpio_value(data2,(dacvalue>>2&(mask)));
	at91_set_gpio_value(data3,(dacvalue>>3&(mask)));


	//PDEBUG("%ld %ld %ld %ld\n",(dacvalue&mask));
//	udelay(1000);
	at91_set_gpio_value(add0,1);
	at91_set_gpio_value(add1,0);
	at91_set_gpio_value(add2,1);
	at91_set_gpio_value(add3,1);



	at91_set_gpio_value(data0,(dacvalue>>4&mask));
	at91_set_gpio_value(data1,(dacvalue>>5&(mask)));
	at91_set_gpio_value(data2,(dacvalue>>6&(mask)));
	at91_set_gpio_value(data3,(dacvalue>>7&(mask)));
	//udelay(1000);

	at91_set_gpio_value(add0,1);
	at91_set_gpio_value(add1,1);
	at91_set_gpio_value(add2,0);
	at91_set_gpio_value(add3,1);

	at91_set_gpio_value(data0,(dacvalue>>8&mask));
	at91_set_gpio_value(data1,(dacvalue>>9&(mask)));
	at91_set_gpio_value(data2,(dacvalue>>10&(mask)));
	at91_set_gpio_value(data3,(dacvalue>>11&(mask)));
	//udelay(1000);
	
	at91_set_gpio_value(add0,1);
	at91_set_gpio_value(add1,1);
	at91_set_gpio_value(add2,1);
	at91_set_gpio_value(add3,0);
	//udelay(1000);

	return 1;

}

int dac5362_write(int dac_number, unsigned short dacvalue){

	unsigned long status=0x0000;
	unsigned long count = 0x0000;
	
	


//	PDEBUG("dac5362\n");
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	
	at91_set_gpio_output(ad5362.ldac,1);
	at91_set_gpio_output(ad5362.clear,1);

//	at91_set_gpio_value(ad5362.ldac,1);
	//at91_set_gpio_value(ad5362.reset,1);
//	at91_set_gpio_value(ad5362.clear,1);
	
//	address = virt_to_phys(ad5362.TxBuffer);
//	AT91SAM9260_BASE_SPI1
	//from atmel_spi.h
	
	
	
	//PDEBUG("SPI1_TPR %u SPI1_TCR %u SPI1_PTSR %u\n",__raw_readl(spi1_base+SPI_TPR),__raw_readl(spi1_base+SPI_TCR),__raw_readl(spi1_base+SPI_PTSR));
	//set to correct peripheral settings for spi
	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2)|(1<<3));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2)|(1<<3));	
	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	
	

	//from at91_spi.h
	__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
	__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs,spi1_base+AT91_SPI_MR);
	__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));
	
	//spi_out_ad5362(dacvalue);
	spi_read_ad5362(dacvalue);

	//NOTE VERY IMPORTANT!! It appears that the PDC will vary the size of the trasnfer depending on whether you have selected Variable Chip select or Fixed chip select. This makes sense since the TDR requires an address bit and bit to tell whether to remain selected if it is using variable selection.
	//The long and short is that the PDC buffer should have not just the 8-bits that we want to transfer but the entire required format for TDR as explained in the manual/datasheet.
  	
	//control_packet = 0x00fafaaf;
	//dac_out+=1000;
	
// 	__raw_writel(dma_address,spi1_base+SPI_RPR);
// 	__raw_writel(dma_address_rx,spi1_base+SPI_TNPR);
// 	__raw_writel(dma_address_rx,spi1_base+SPI_RNPR);	
// 
//   	__raw_writel(3,spi1_base+SPI_TNCR);
//   	__raw_writel(0,spi1_base+SPI_RNCR);
// 
// 	
// 	
// 	__raw_writel(3,spi1_base+SPI_TNCR);
// 	__raw_writel(0,spi1_base+SPI_RCR);
 	

	//status= __raw_readl(spi1_base+AT91_SPI_RDR); 
	//status= __raw_readl(spi1_base+AT91_SPI_RDR); 
	//status= __raw_readl(spi1_base+AT91_SPI_RDR); 
	//PDEBUG("PIOB_PSR %08x PIOB_ABSR %08x SPI1_TPR %08x SPI1_TCR %08x SPI1_PTSR %08x SPI1_SR %08x \nSPI1_MR %08x SPI1_RDR %08x SPI1_CSR(0) %08x \n\n",at91_sys_read(AT91_PIOB+PIO_PSR),at91_sys_read(AT91_PIOB+PIO_ABSR),__raw_readl(spi1_base+SPI_TPR),__raw_readl(spi1_base+SPI_TCR),__raw_readl(spi1_base+SPI_PTSR),__raw_readl(spi1_base+AT91_SPI_SR),__raw_readl(spi1_base+AT91_SPI_MR),__raw_readl(spi1_base+AT91_SPI_RDR),__raw_readl(spi1_base+AT91_SPI_CSR(0)));	
	
	


// 	__raw_writel(dma_address_rx,spi1_base+SPI_TPR);
// 	__raw_writel(dma_address_rx,spi1_base+SPI_RPR);
// 	__raw_writel(3,spi1_base+SPI_TCR);
// 	__raw_writel(3,spi1_base+SPI_RCR);
// 	__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);


	
	//__raw_writel(ad5362.spidis,spi1_base+AT91_SPI_CR);
	
	
	//status= __raw_readl(spi1_base+AT91_SPI_SR); 

	
	//PDEBUG("count SPI1_SR %08x\n",status);
	//wait for transmit to complete with an error message if it doesn't
// 	while( (!(status& AT91_SPI_TXBUFE)) && (count<500)   ) {
// 		status= __raw_readl(spi1_base+AT91_SPI_SR); 
// 		count++;
// 		if(count>400)
// 			printk("ERROR: SPI RECEIVE TIME OUT- NO TRANSFER\n");
// 	//	PDEBUG("count %lu SPI1_SR %08x\n",count,status);
// 		//udelay(1000);
// 	}


// 	__raw_writel(dma_address_rx,spi1_base+SPI_TNPR);
// 	__raw_writel(dma_address_rx,spi1_base+SPI_RNPR);

	
	return 1;
}

int spi_out_ad5362(unsigned short value){

	struct device *dev_ptr,dev;
	
	unsigned char datacom[4];
	dma_addr_t *dma,dma_address,dma_address_rx;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	unsigned int temp;
	unsigned int control_packet;
	unsigned int buffer[100];
	unsigned int rxbuffer[100];
	//map dma address so the PDC can work!
	dev_ptr=&dev;
	dma_address=dma_map_single(dev_ptr, buffer, 100,DMA_BIDIRECTIONAL);
	dma_address_rx=dma_map_single(dev_ptr, rxbuffer, 100,DMA_BIDIRECTIONAL);
	
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	at91_set_gpio_input(ad5362.busy,0);
	
//	dac_out=0x0000ffff;
//	control_packet = 0x00c00000|(dac_out&(0x0000ffff));
	//control_packet = command;
	control_packet = 0x00800000;
	datacom[0]=value&(0x00ff);
	datacom[1]=(value>>8)&0x0ff;
	datacom[2]=0xc0;
	//PDEBUG("control %08x \n",control_packet);
	buffer[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|datacom[2];
	buffer[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|datacom[1];
	buffer[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|datacom[0];
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

	
 	__raw_writel((1<<8|1<<1),spi1_base+SPI_PTCR);
	//NOTE load a pointer to the data
   	__raw_writel(dma_address,spi1_base+SPI_TPR);
	__raw_writel(dma_address_rx,spi1_base+SPI_RPR);
	__raw_writel(3,spi1_base+SPI_TCR);
	__raw_writel(3,spi1_base+SPI_RCR);
	__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);


	status = 0x0000;
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	//PDEBUG("count SPI1_SR %08x\n",status);
	//NOTE wait for transmit to complete with an error message if it doesn't
	

	count = 0;
	while( (!(status&AT91_SPI_TXEMPTY)) && (count<500)   ) {
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		count++;
		if(count>400){
			printk("ERROR: SPI TRANSMIT TIME OUT- NO TRANSFER\n");
			PDEBUG("count %lu SPI1_SR %08x\n",count,status);
			}
		
		ndelay(10);
	}

	
	temp=0;
	temp = gpio_get_value(ad5362.busy);
	count = 0;
	//wait for !busy to rise again- typical 1-5us depending on what is being updated
	while((!(temp))&&(count<500)) {
		temp = gpio_get_value(ad5362.busy);
		if(count>400){
			PDEBUG("!BUSY %04x %u\n",temp,count);
			udelay(1);
			}
		count++;
		}
	//NOTE drop LDAC to signal that the outputs should be updated from the register
	at91_set_gpio_output(ad5362.ldac,0);

	
	//NOTE LDAC minimum pulse width low is 10ns- give it some extra time-
	//NOTE THIS COULD BE IMPROVED AS THE MINIMUM SLEEP TIME HERE IS OF THE ORDER OF 1uS-> I HAVE TRIED A NOOP LOOP BUT THIS DOESN'T APPEAR TO GIVE ENOUGH IMPROVEMENT TO WARRANT THROWING OUT THE GENERIC NATURE OF NDELAY()
	ndelay(11);
	at91_set_gpio_output(ad5362.ldac,1);


	dma_unmap_single(dev_ptr, dma_address_rx, 100,DMA_BIDIRECTIONAL);
	dma_unmap_single(dev_ptr, dma_address, 100,DMA_BIDIRECTIONAL);


}


int spi_read_ad5362(unsigned short value){

	struct device *dev_ptr,dev;
	
	unsigned char datacom[4];
	dma_addr_t *dma,dma_address1,dma_address2,dma_address3,dma_address4;
	unsigned long status=0x0000;
	unsigned long count = 0x0000;		
	unsigned int temp;
	
	unsigned int control_packet;
	int i;
	unsigned int buffer1[100];
	unsigned int buffer2[100];
	unsigned int buffer3[100];
	unsigned int buffer4[100];
	unsigned int buffer5[100];
	//map dma address so the PDC can work!
	dev_ptr=&dev;
	
	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	if(!(status&(1<<SPI_SPIENS_OFFSET)))
		__raw_writel(ad5362.lastxfer|ad5362.spien,spi1_base+AT91_SPI_CR);
	
	at91_set_gpio_output(ad5362.ldac,1);
	at91_set_gpio_output(ad5362.clear,1);

	at91_sys_write(AT91_PIOB+PIO_PDR,(1<<0)|(1<<1)|(1<<2)|(1<<3));	
	at91_sys_write(AT91_PIOB+PIO_ASR,(1<<0)|(1<<1)|(1<<2)|(1<<3));	
	
	//enable clock for spi
	at91_sys_write(AT91_PMC_PCER,1<<AT91SAM9260_ID_SPI1);
	__raw_writel(ad5362.lastxfer,spi1_base+AT91_SPI_CR);
	__raw_writel(ad5362.mstr|ad5362.ps|ad5362.pcsdec|ad5362.dlybcs,spi1_base+AT91_SPI_MR);
	__raw_writel(ad5362.cpol|ad5362.ncpha|ad5362.csaat|ad5362.bits|ad5362.scbr|ad5362.dlybs|ad5362.dlybct,spi1_base+AT91_SPI_CSR(0));




	dma_address1=dma_map_single(dev_ptr, buffer1, 100,DMA_BIDIRECTIONAL);
	dma_address2=dma_map_single(dev_ptr, buffer2, 100,DMA_BIDIRECTIONAL);
	dma_address3=dma_map_single(dev_ptr, buffer3, 100,DMA_BIDIRECTIONAL);
	dma_address4=dma_map_single(dev_ptr, buffer4, 100,DMA_BIDIRECTIONAL);
	at91_set_gpio_input(ad5362.busy,0);
	
//	at91_set_gpio_output(AT91_PIN_PB3,1);
//	dac_out=0x0000ffff;
//	control_packet = 0x00c00000|(dac_out&(0x0000ffff));
	//control_packet = command;
	control_packet = 0x00c00000;
	value = 0x0800;
	datacom[0]=value&(0x00ff);
	datacom[1]=(value>>8)&0x0ff;
	datacom[2]=0x05;
	//PDEBUG("control %08x \n",control_packet);
	buffer1[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|datacom[2];
	buffer1[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|datacom[1];
	buffer1[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|datacom[0];
	buffer1[3]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer1[4]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer1[5]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer1[6]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;

	control_packet = 0x00000000;
	buffer2[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer2[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer2[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer2[3]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer2[4]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer2[5]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer1[6]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;	


	buffer3[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer3[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer3[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer3[3]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer3[4]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer3[5]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer3[6]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer3[7]=0;
	buffer3[8]=0;
	buffer3[9]=0;
	buffer3[10]=0;
	buffer3[11]=0;	

	buffer4[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer4[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer4[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer4[3]=0;
	buffer4[4]=0;
	buffer4[5]=0;
	buffer4[6]=0;
	buffer4[7]=0;
	buffer4[8]=0;
	buffer4[9]=0;
	buffer4[10]=0;
	buffer4[11]=0;


	buffer5[0]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer5[1]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer5[2]=ad5362.lastxfer|(0<<19)|(0<<18)|(0<<17)|(0<<16)|0x0000;
	buffer5[3]=0;
	buffer5[4]=0;
	buffer5[5]=0;
	buffer5[6]=0;
	buffer5[7]=0;
	buffer5[8]=0;
	buffer5[9]=0;
	buffer5[10]=0;
	buffer5[11]=0;
	

//	at91_set_gpio_output(AT91_PIN_PB3,0);
 	__raw_writel((1<<8|1<<0),spi1_base+SPI_PTCR);
	//NOTE load a pointer to the data
	//__raw_writel(dma_address_rx,spi1_base+SPI_TNPR);
	__raw_writel(dma_address1,spi1_base+SPI_TPR);
   	__raw_writel(dma_address2,spi1_base+SPI_RPR);	
	__raw_writel(dma_address3,spi1_base+SPI_TNPR);
	__raw_writel(dma_address4,spi1_base+SPI_RNPR);
	

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
	while( (!(status&AT91_SPI_TXEMPTY)) && (count<500)   ) {
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		count++;
		if(count>400){
			printk("ERROR: SPI TRANSMIT TIME OUT- NO TRANSFER\n");
			PDEBUG("count %lu SPI1_SR %08x\n",count,status);
			}
		
		//udelay(1);
	}
//----------------------------------------------------
//	PDEBUG("OK count %lu SPI1_SR %08x\n",count,status);
//	at91_set_gpio_output(AT91_PIN_PB3,1);
//	udelay(1);
	//at91_set_gpio_output(AT91_PIN_PB3,0);
	//udelay(10);
// 	__raw_writel(dma_address1,spi1_base+SPI_RPR);
	
	__raw_writel(dma_address3,spi1_base+SPI_TPR);
	__raw_writel(dma_address4,spi1_base+SPI_RPR);	
	
	__raw_writel(3,spi1_base+SPI_TCR);
	__raw_writel(3,spi1_base+SPI_RCR);
	__raw_writel(0,spi1_base+SPI_TNCR);	
	__raw_writel(0,spi1_base+SPI_RNCR);
	//__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);
	
// 	__raw_writel(0,spi1_base+SPI_TNCR);	
// 	__raw_writel(0,spi1_base+SPI_RNCR);
// // 	__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);
// 
// 
// 	count = 0;
// 	status = 0x0000;
 	status= __raw_readl(spi1_base+AT91_SPI_SR); 
	while( (!(status&AT91_SPI_RXBUFF)) && (count<500))  {
		status= __raw_readl(spi1_base+AT91_SPI_SR); 
		count++;
		if(count>400){
			printk("ERROR: SPI receive TIME OUT- NO TRANSFER\n");
			PDEBUG("count %08x SPI1_SR %08x\n",count,status);
			}
		
		//udelay(1);
	}
//	buffer4[3]=__raw_readl(spi1_base+AT91_SPI_RDR);
// 
// 	count = 0;
// 	while((!(status&AT91_SPI_TXEMPTY)) && (count<500) ){
// 	status= __raw_readl(spi1_base+AT91_SPI_SR); 
// 	count++;
// 		if(count>400){
// 			printk("ERROR: ERROR ON RECEIVE TRANSMIT TIME OUT- NO TRANSFER\n");
// 			//PDEBUG("count %08x SPI1_SR %08x\n",count,status);
// 			}
// 	udelay(1);
// 	}
//---------------------------------------------------
// 	if((!(status&AT91_SPI_TXEMPTY))){
// 	__raw_writel(dma_address1,spi1_base+SPI_RPR);
// 	__raw_writel(6,spi1_base+SPI_RCR);
// 	__raw_writel(3,spi1_base+SPI_TCR);
// 	__raw_writel(0,spi1_base+SPI_TNCR);	
// 	__raw_writel(0,spi1_base+SPI_RNCR);
// 
// 	}
// 	
//	udelay(500);
	
//  	PDEBUG("HELLO\nbuffer1:%08x %08x %08x %08x %08x %08x %08x\n",buffer1[0],buffer1[1],buffer1[2],buffer1[3],buffer1[4],buffer1[5],buffer1[6]);
//  	
//  	PDEBUG("buffer2: %08x %08x %08x %08x %08x %08x %08x\n",buffer2[0],buffer2[1],buffer2[2],buffer2[3],buffer2[4],buffer2[5],buffer2[6]);
//  	
//  	PDEBUG("buffer3: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",buffer3[0],buffer3[1],buffer3[2],buffer3[3],buffer3[4],buffer3[5],buffer3[6],buffer3[7],buffer3[8],buffer3[9],buffer3[10],buffer3[11]);
 // 	PDEBUG("buffer4: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\nGOODBYE\n",buffer4[0],buffer4[1],buffer4[2],buffer4[3],buffer4[4],buffer4[5],buffer4[6],buffer4[7],buffer4[8],buffer4[9],buffer4[10],buffer4[11]);
	temp=0;
	//mdelay(1);
	//copy to a page memory location- not sure if this helps!
	for(i=0;i<=10;i++){
		buffer5[i]=buffer4[i];
	}
	temp =  ((buffer5[0]&0x000000ff)<<16) | ((buffer5[1]&0x000000ff)<<8) | ((buffer5[2]&0x000000ff));
	printk("count %lu SPI1_SR %08x TEMP %08x\n",count,status,temp);
	if((temp&0x00ffffff)!=0x0000ffff){
		//PDEBUG("buffer4: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\nGOODBYE\n",buffer4[0],buffer4[1],buffer4[2],buffer4[3],buffer4[4],buffer4[5],buffer4[6],buffer4[7],buffer4[8],buffer4[9],buffer4[10],buffer4[11]);
		//printk("%08x %08x %08x %08x\n",buffer5[0],buffer5[1],buffer5[2],buffer5[3]);
	}
	//mdelay(1);
	//PDEBUG("TEMP %08x\n",temp);
//	PDEBUG("OK2 count %lu SPI1_SR %08x\n",count,status);
	//__raw_writel(ad5362.spien,spi1_base+AT91_SPI_CR);
	//at91_set_gpio_output(AT91_PIN_PB3,1);
	//udelay(30);
// 	count = 0;
// 	while( (!(status&AT91_SPI_TXEMPTY)) && (count<500)   ) {
// 		status= __raw_readl(spi1_base+AT91_SPI_SR); 
// 		count++;
// 		if(count>400){
// 			printk("ERROR: SPI TRANSMIT TIME OUT- NO TRANSFER\n");
// 			PDEBUG("count %lu SPI1_SR %08x\n",count,status);
// 			}
// 		
// 		ndelay(10);
// 	}
	
// 	temp=0;
// 	temp = gpio_get_value(ad5362.busy);
// 	count = 0;
// 	//wait for !busy to rise again- typical 1-5us depending on what is being updated
// 	while((!(temp))&&(count<500)) {
// 		temp = gpio_get_value(ad5362.busy);
// 		if(count>400){
// 			PDEBUG("!BUSY %04x %u\n",temp,count);
// 			udelay(1);
// 			}
// 		count++;
// 		}
// 	//NOTE drop LDAC to signal that the outputs should be updated from the register
// 	at91_set_gpio_output(ad5362.ldac,0);
// 
// 	
// 	//NOTE LDAC minimum pulse width low is 10ns- give it some extra time-
// 	//NOTE THIS COULD BE IMPROVED AS THE MINIMUM SLEEP TIME HERE IS OF THE ORDER OF 1uS-> I HAVE TRIED A NOOP LOOP BUT THIS DOESN'T APPEAR TO GIVE ENOUGH IMPROVEMENT TO WARRANT THROWING OUT THE GENERIC NATURE OF NDELAY()
// 	ndelay(11);
// 	at91_set_gpio_output(ad5362.ldac,1);

	dma_unmap_single(dev_ptr, dma_address1, 100,DMA_BIDIRECTIONAL);
	dma_unmap_single(dev_ptr, dma_address2, 100,DMA_BIDIRECTIONAL);
	dma_unmap_single(dev_ptr, dma_address3, 100,DMA_BIDIRECTIONAL);
	dma_unmap_single(dev_ptr, dma_address4, 100,DMA_BIDIRECTIONAL);
	return 1;

}



void init_ad5362(void){
	
	unsigned int ret;
	//reset the ad5362
	__raw_writel((1<<7),spi1_base+AT91_SPI_CR);
 	
	at91_set_gpio_output(ad5362.reset,1);
 	at91_set_gpio_output(ad5362.clear,1);
	at91_set_gpio_value(ad5362.reset,1);
 	at91_set_gpio_value(ad5362.clear,1);
	udelay(1);
	//this allows the Busy pin to move up and down- i.e setting the reset and clear pins as inputs
	at91_set_gpio_value(ad5362.reset,0);
 	udelay(1);
	at91_set_gpio_value(ad5362.reset,1);
	udelay(500);
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


module_init(mod_init);
module_exit(mod_exit); 

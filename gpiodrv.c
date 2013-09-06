// ****************************************** //
// Title  : GPIO SWITCH
// Target : RK3066
//
// Author : kelvin
// ****************************************** //

#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/delay.h>
  
#include <linux/fs.h>            
#include <linux/mm.h>            
#include <linux/errno.h>         
#include <linux/types.h>         
#include <linux/fcntl.h>         
#include <linux/cdev.h>         
#include <linux/device.h>         
#include <linux/major.h>         
#include <linux/gpio.h>
#include <mach/iomux.h>

#include <asm/uaccess.h>  
#include <asm/io.h>  
#include <asm/mach-types.h>
  
#define DEV_NAME      "gpiodrv"   
#define DEV_MAJOR     0         
#define DEBUG_ON      1           
                                
#define gpiodrv_dbg(fmt,arg...)     if(DEBUG_ON) printk("== gpiodrv debug == " fmt, ##arg)

static int      gpiodrv_major = DEV_MAJOR;
static dev_t    dev;
static struct   cdev gpiodrv_cdev;
static struct   class *gpiodrv_class;

//--------------------------------------------------------------------
//0:0->RFID; 0:1->LDTONG; 1:0->1D/2D; 1:1->IC CARD
#define UART1_EN		RK30_PIN1_PA6
#define UART1_EN_IOMUX	GPIO1A6_UART1CTSN_SPI0RXD_NAME
#define UART1_SEL0	RK30_PIN1_PA7
#define UART1_SEL0_IOMUX	GPIO1A7_UART1RTSN_SPI0TXD_NAME
#define UART1_SEL1	RK30_PIN4_PB7
#define UART1_SEL1_IOMUX	GPIO4B7_SPI0CSN1_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
//0:0->finger; 0:1->custom display; 1:0->printer
#define UART3_EN		RK30_PIN0_PD4
#define UART3_EN_IOMUX	GPIO0D4_I2S22CHSDI_SMCADDR0_NAME
#define UART3_SEL0	RK30_PIN0_PD2
#define UART3_SEL0_IOMUX	GPIO0D2_I2S22CHLRCKRX_SMCOEN_NAME
#define UART3_SEL1	RK30_PIN0_PD0
#define UART3_SEL1_IOMUX	GPIO0D0_I2S22CHCLK_SMCCSN0_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
//custom display led control
#define LEDKX_PWR		RK30_PIN4_PD3
#define LEDKX_PWR_IOMUX	GPIO4D3_SMCDATA11_TRACEDATA11_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define FINGER_PWR	RK30_PIN2_PC2
#define FINGER_PWR_IOMUX	GPIO2C2_LCDC1DATA18_SMCBLSN1_HSADCDATA5_NAME
#define FINGER_INT	RK30_PIN4_PC1
#define FINGER_INT_IOMUX	GPIO4C1_SMCDATA1_TRACEDATA1_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define USBKEY_PWR			RK30_PIN4_PC3
#define USBKEY_PWR_IOMUX	GPIO4C3_SMCDATA3_TRACEDATA3_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define CARD_PWR		RK30_PIN4_PC4
#define CARD_PWR_IOMUX	GPIO4C4_SMCDATA4_TRACEDATA4_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define LDTONG_PWR	RK30_PIN4_PC5
#define LDTONG_PWR_IOMUX	GPIO4C5_SMCDATA5_TRACEDATA5_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define QX_PWR			RK30_PIN4_PC6
#define QX_PWR_IOMUX	GPIO4C6_SMCDATA6_TRACEDATA6_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define RFID_PWR		RK30_PIN0_PD3
#define RFID_PWR_IOMUX	GPIO0D3_I2S22CHLRCKTX_SMCADVN_NAME
#define RFID_INT		RK30_PIN0_PC7
#define RFID_INT_IOMUX	GPIO0C7_TRACECTL_SMCADDR3_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define SCAN_PWR			RK30_PIN4_PD6
#define SCAN_PWR_IOMUX	GPIO4D6_SMCDATA14_TRACEDATA14_NAME
#define SCAN_RST			RK30_PIN2_PC4
#define SCAN_RST_IOMUX	GPIO2C4_LCDC1DATA20_SPI1CSN0_HSADCDATA1_NAME
#define SCAN_PWDN			RK30_PIN2_PC6
#define SCAN_PWDN_IOMUX	GPIO2C6_LCDC1DATA22_SPI1RXD_HSADCDATA3_NAME
#define SCAN_TRIG			RK30_PIN2_PC5
#define SCAN_TRIG_IOMUX	GPIO2C5_LCDC1DATA21_SPI1TXD_HSADCDATA2_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
#define PRINT_PWR		RK30_PIN2_PC0
#define PRINT_PWR_IOMUX	GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME
//--------------------------------------------------------------------

//  ************************************************************ //
//  Device Open : 
//  ************************************************************ //
static int gpiodrv_open (struct inode *inode, struct file *filp)  
{
    return 0;  
}

//  ************************************************************ //
//  Device Release : 
//  ************************************************************ //
static int gpiodrv_release (struct inode *inode, struct file *filp)  
{  
    return 0;  
}  
  
//  ************************************************************ //
//  Device Release : 
//  IO Control
//  IOPWR  gpiodrv  On/Off 
//  ************************************************************ //
static long gpiodrv_ioctl (/*struct inode *inode, */struct file *filp,
                           unsigned int cmd, unsigned long arg)  
{
    printk("gpiodrv_ioctl,cmd=%d,arg=0x%08x\n",cmd,arg);
    switch( cmd )  
    {  
        case 1 :
            break;   
        case 0 :
            break;            
        default :
            break;
    };
    return 0;  
}  
  
//  ************************************************************ //
//  File Operation Struct :
//  ************************************************************ //
static struct file_operations gpiodrv_fops =  
{  
    .owner    = THIS_MODULE,  
    .unlocked_ioctl = gpiodrv_ioctl,  
    .open     = gpiodrv_open,       
    .release  = gpiodrv_release,    
};  

//  ************************************************************ //
//  Device Init :
//  ************************************************************ //
static int __init gpiodrv_init(void)  
{  
    int result;  
	if (0 == gpiodrv_major)
	{
		/* auto select a major */
		result = alloc_chrdev_region(&dev, 0, 1, DEV_NAME);
		gpiodrv_major = MAJOR(dev);
	}
	else
	{
		/* use load time defined major number */
		dev = MKDEV(gpiodrv_major, 0);
		result = register_chrdev_region(dev, 1, DEV_NAME);
	}

	memset(&gpiodrv_cdev, 0, sizeof(gpiodrv_cdev));

	/* initialize our char dev data */
	cdev_init(&gpiodrv_cdev, &gpiodrv_fops);

	/* register char dev with the kernel */
	result = cdev_add(&gpiodrv_cdev, dev, 1);
    
	if (0 != result)
	{
		unregister_chrdev_region(dev, 1);
		printk("Error registrating device object with the kernel\n");
	}

    gpiodrv_class = class_create(THIS_MODULE, DEV_NAME);
    device_create(gpiodrv_class, NULL, MKDEV(gpiodrv_major, MINOR(dev)), NULL,
                  DEV_NAME);

    if (result < 0)
        return result;  

#ifdef UART1_EN_IOMUX
		rk30_mux_api_set(UART1_EN_IOMUX,0);
#endif
		result = gpio_request(UART1_EN,NULL);
		if (result < 0 )
		{
			printk("gpio_request UART1_EN failed\n");
			return result;
		}
		gpio_direction_output(UART1_EN,0);

#ifdef UART1_SEL0_IOMUX
		rk30_mux_api_set(UART1_SEL0_IOMUX,0);
#endif
		result = gpio_request(UART1_SEL0,NULL);
		if (result < 0 )
		{
			printk("gpio_request UART1_SEL0 failed\n");
			return result;
		}
		gpio_direction_output(UART1_SEL0,0);

#ifdef UART1_SEL1_IOMUX
		rk30_mux_api_set(UART1_SEL1_IOMUX,0);
#endif
		result = gpio_request(UART1_SEL1,NULL);
		if (result < 0 )
		{
			printk("gpio_request UART1_SEL1 failed\n");
			return result;
		}
		gpio_direction_output(UART1_SEL1,0);

#ifdef UART3_EN_IOMUX
		rk30_mux_api_set(UART3_EN_IOMUX,0);
#endif
		result = gpio_request(UART3_EN,NULL);
		if (result < 0 )
		{
			printk("gpio_request UART3_EN failed\n");
			return result;
		}
		gpio_direction_output(UART3_EN,0);

#ifdef UART3_SEL0_IOMUX
		rk30_mux_api_set(UART3_SEL0_IOMUX,0);
#endif
		result = gpio_request(UART3_SEL0,NULL);
		if (result < 0 )
		{
			printk("gpio_request UART3_SEL0 failed\n");
			return result;
		}
		gpio_direction_output(UART3_SEL0,0);

#ifdef UART3_SEL1_IOMUX
		rk30_mux_api_set(UART3_SEL1_IOMUX,0);
#endif
		result = gpio_request(UART3_SEL1,NULL);
		if (result < 0 )
		{
			printk("gpio_request UART3_SEL1 failed\n");
			return result;
		}
		gpio_direction_output(UART3_SEL1,0);

#ifdef LEDKX_PWR_IOMUX
		rk30_mux_api_set(LEDKX_PWR_IOMUX,0);
#endif
		result = gpio_request(LEDKX_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request LEDKX_PWR failed\n");
			return result;
		}
		gpio_direction_output(LEDKX_PWR,0);

#ifdef FINGER_PWR_IOMUX
		rk30_mux_api_set(FINGER_PWR_IOMUX,0);
#endif
		result = gpio_request(FINGER_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request FINGER_PWR failed\n");
			return result;
		}
		gpio_direction_output(FINGER_PWR,0);

#ifdef FINGER_INT_IOMUX
		rk30_mux_api_set(FINGER_INT_IOMUX,0);
#endif
		result = gpio_request(FINGER_INT,NULL);
		if (result < 0 )
		{
			printk("gpio_request FINGER_INT failed\n");
			return result;
		}
		gpio_direction_input(FINGER_INT);

#ifdef USBKEY_PWR_IOMUX
		rk30_mux_api_set(USBKEY_PWR_IOMUX,0);
#endif
		result = gpio_request(USBKEY_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request USBKEY_PWR failed\n");
			return result;
		}
		gpio_direction_output(USBKEY_PWR,0);

#ifdef CARD_PWR_IOMUX
		rk30_mux_api_set(CARD_PWR_IOMUX,0);
#endif
		result = gpio_request(CARD_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request CARD_PWR failed\n");
			return result;
		}
		gpio_direction_output(CARD_PWR,0);

#ifdef LDTONG_PWR_IOMUX
		rk30_mux_api_set(LDTONG_PWR_IOMUX,0);
#endif
		result = gpio_request(LDTONG_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request LDTONG_PWR failed\n");
			return result;
		}
		gpio_direction_output(LDTONG_PWR,0);

#ifdef QX_PWR_IOMUX
		rk30_mux_api_set(QX_PWR_IOMUX,0);
#endif
		result = gpio_request(QX_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request QX_PWR failed\n");
			return result;
		}
		gpio_direction_output(QX_PWR,0);

#ifdef RFID_PWR_IOMUX
		rk30_mux_api_set(RFID_PWR_IOMUX,0);
#endif
		result = gpio_request(RFID_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request RFID_PWR failed\n");
			return result;
		}
		gpio_direction_output(RFID_PWR,0);

#ifdef RFID_INT_IOMUX
		rk30_mux_api_set(RFID_INT_IOMUX,0);
#endif
		result = gpio_request(RFID_INT,NULL);
		if (result < 0 )
		{
			printk("gpio_request RFID_PWR failed\n");
			return result;
		}
		gpio_direction_input(RFID_INT);

#ifdef SCAN_PWR_IOMUX
		rk30_mux_api_set(SCAN_PWR_IOMUX,0);
#endif
		result = gpio_request(SCAN_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request SCAN_PWR failed\n");
			return result;
		}
		gpio_direction_output(SCAN_PWR,0);

#ifdef SCAN_RST_IOMUX
		rk30_mux_api_set(SCAN_RST_IOMUX,0);
#endif
		result = gpio_request(SCAN_RST,NULL);
		if (result < 0 )
		{
			printk("gpio_request SCAN_RST failed\n");
			return result;
		}
		gpio_direction_output(SCAN_RST,0);

#ifdef SCAN_PWDN_IOMUX
		rk30_mux_api_set(SCAN_PWDN_IOMUX,0);
#endif
		result = gpio_request(SCAN_PWDN,NULL);
		if (result < 0 )
		{
			printk("gpio_request SCAN_PWDN failed\n");
			return result;
		}
		gpio_direction_output(SCAN_PWDN,0);

#ifdef SCAN_TRIG_IOMUX
		rk30_mux_api_set(SCAN_TRIG_IOMUX,0);
#endif
		result = gpio_request(SCAN_TRIG,NULL);
		if (result < 0 )
		{
			printk("gpio_request SCAN_TRIG failed\n");
			return result;
		}
		gpio_direction_output(SCAN_TRIG,0);

#ifdef PRINT_PWR_IOMUX
		rk30_mux_api_set(PRINT_PWR_IOMUX,0);
#endif
		result = gpio_request(PRINT_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request PRINT_PWR failed\n");
			return result;
		}
		gpio_direction_output(PRINT_PWR,0);


    gpiodrv_dbg("gpio driver loaded\n");

    return 0;  
}  

//  ************************************************************ //
//  Device Exit :
//  ************************************************************ //
static void __exit gpiodrv_exit(void)  
{  

    device_destroy(gpiodrv_class, MKDEV(gpiodrv_major, 0));
    class_destroy(gpiodrv_class);

    cdev_del(&gpiodrv_cdev);
    unregister_chrdev_region(dev, 1);
		gpio_free(RK30_PIN4_PD3);
		gpio_free(RK30_PIN4_PD4);
    gpiodrv_dbg("gpio driver unloaded");
}  

module_init(gpiodrv_init);  
module_exit(gpiodrv_exit);  

MODULE_AUTHOR("Kelvin <5404385@qq.com>");
MODULE_DESCRIPTION("gpio switch power driver");
MODULE_LICENSE("GPL");

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
  
#define DEV_NAME      "ctrl_gpio"   
#define DEV_MAJOR     0         
#define DEBUG_ON      1           
#define IO_GET_STATUS		0x1234                              
#define gpiodrv_dbg(fmt,arg...)     if(DEBUG_ON) printk("== gpiodrv == " fmt, ##arg)

static int      gpiodrv_major = DEV_MAJOR;
static dev_t    dev;
static struct   cdev gpiodrv_cdev;
static struct   class *gpiodrv_class;
struct device *gpio_dev; 

enum
{
	UART1_DISABLE		=0x00001100,UART_RFID,UART_SCAN,UART_LDTONG,UART_MAGCARD,
	UART3_DISABLE		=0x00000110,UART_FINGER,UART_LEDDISP,UART_PRINTER,
	DISP_PWR_LOW		=0x00000120,DISP_PWR_HIGH,
	FINGER_PWR_LOW	=0x00000130,FINGER_PWR_HIGH,
	FINGER_INT_VALUE=0x00000140,
	USBKEY_PWR_LOW	=0x00000150,USBKEY_PWR_HIGH,
	MAGCARD_PWR_LOW	=0x00000160,MAGCARD_PWR_HIGH,
	LDTONG_PWR_LOW	=0x00000170,LDTONG_PWR_HIGH,
	QX_PWR_LOW			=0x00000180,QX_PWR_HIGH,
	RFID_PWR_LOW		=0x00000190,RFID_PWR_HIGH,
	RFID_INT_VALUE	=0x000001A0,
	SCAN_PWR_LOW		=0x000001B0,SCAN_PWR_HIGH,
	SCAN_RST_LOW		=0x000001C0,SCAN_RST_HIGH,
	SCAN_PWDN_LOW		=0x000001D0,SCAN_PWDN_HIGH,
	SCAN_TRIG_LOW		=0x000001E0,SCAN_TRIG_HIGH,
	PRINT_PWR_LOW		=0x000001F0,PRINT_PWR_HIGH,
};

enum
{
	eDISP_PWR			= 1<<0,
	eFINGER_PWR		= 1<<1,
	eFINGER_INT		= 1<<2,
	eUSBKEY_PWR		= 1<<3,
	eMAGCARD_PWR	= 1<<4,
	eLDTONG_PWR		= 1<<5,
	eQX_PWR				= 1<<6,
	eRFID_PWR			= 1<<7,
	eRFID_INT			= 1<<8,
	eSCAN_PWR			= 1<<9,
	eSCAN_RST			= 1<<10,
	eSCAN_PWDN		= 1<<11,
	eSCAN_TRIG		= 1<<12,
	ePRINT_PWR		= 1<<13,
	eUART_RFID		= 1<<14,
	eUART_SCAN		= 1<<15,
	eUART_LDTONG	= 1<<16,
	eUART_MAGCARD	= 1<<17,
	eUART_FINGER	= 1<<18,
	eUART_LEDDISP	= 1<<19,
	eUART_PRINTER	= 1<<20,
};
//--------------------------------------------------------------------
//SEL1:SEL0; 0:0->RFID; 0:1->1D/2D; 1:0->LDTONG; 1:1->IC CARD
#define UART1_EN		RK30_PIN1_PA6
#define UART1_EN_IOMUX	GPIO1A6_UART1CTSN_SPI0RXD_NAME
#define UART1_SEL0	RK30_PIN1_PA7
#define UART1_SEL0_IOMUX	GPIO1A7_UART1RTSN_SPI0TXD_NAME
#define UART1_SEL1	RK30_PIN4_PB7
#define UART1_SEL1_IOMUX	GPIO4B7_SPI0CSN1_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
//SEL1:SEL0; 0:0->finger; 0:1->custom display; 1:0->printer
#define UART3_EN		RK30_PIN0_PD4
#define UART3_EN_IOMUX	GPIO0D4_I2S22CHSDI_SMCADDR0_NAME
#define UART3_SEL0	RK30_PIN0_PD2
#define UART3_SEL0_IOMUX	GPIO0D2_I2S22CHLRCKRX_SMCOEN_NAME
#define UART3_SEL1	RK30_PIN0_PD0
#define UART3_SEL1_IOMUX	GPIO0D0_I2S22CHCLK_SMCCSN0_NAME
//--------------------------------------------------------------------

//--------------------------------------------------------------------
//custom display led control
#define DISP_PWR		RK30_PIN4_PD3
#define DISP_PWR_IOMUX	GPIO4D3_SMCDATA11_TRACEDATA11_NAME
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
#define MAGCARD_PWR		RK30_PIN4_PC4
#define MAGCARD_PWR_IOMUX	GPIO4C4_SMCDATA4_TRACEDATA4_NAME
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
static unsigned int status=0;
//  ************************************************************ //
//  Device Open : 
//  ************************************************************ //
static int gpiodrv_open (struct inode *inode, struct file *filp)  
{
	gpiodrv_dbg("------%s------\n",__func__);
  return 0;  
}

//  ************************************************************ //
//  Device Release : 
//  ************************************************************ //
static int gpiodrv_release (struct inode *inode, struct file *filp)  
{  
	gpiodrv_dbg("------%s------\n",__func__);
  return 0;  
}  
  
static ssize_t gpiodrv_read (struct file *filp, char __user *buff, size_t length, loff_t *offset)
{
	gpiodrv_dbg("------%s------\n",__func__);
	return 0;
}

static ssize_t gpiodrv_write (struct file *filp, const char __user *buff, size_t length, loff_t *offset)
{
	gpiodrv_dbg("------%s------\n",__func__);
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
		unsigned long __user* argp = (unsigned long __user*)arg;
		unsigned long io_level=0;
    //printk("gpiodrv_ioctl,cmd=%d,arg=0x%08x\n",cmd,arg);
    switch( cmd )  
    {  
        case UART1_DISABLE://disable uart1
        	gpio_direction_output(UART1_EN,1);
        	gpiodrv_dbg("disable uart1 port\n");
        break;   
        case UART_RFID:
        case eUART_RFID:
        	gpio_direction_output(UART1_EN,0);
        	gpio_direction_output(UART1_SEL1,0);
        	gpio_direction_output(UART1_SEL0,0);       	
        	gpiodrv_dbg("switch uart1 to rfid\n");
        break;            
        case UART_SCAN:
        case eUART_SCAN:
        	gpio_direction_output(UART1_EN,0);
        	gpio_direction_output(UART1_SEL1,0);
        	gpio_direction_output(UART1_SEL0,1);       	
        	gpiodrv_dbg("switch uart1 to 1d/2d bar code\n");
        break;            
        case UART_LDTONG:
        case eUART_LDTONG:
        	gpio_direction_output(UART1_EN,0);
        	gpio_direction_output(UART1_SEL1,1);
        	gpio_direction_output(UART1_SEL0,0);       	
        	gpiodrv_dbg("switch uart1 to lai-dian-tong\n");
        break;            
        case UART_MAGCARD:
        case eUART_MAGCARD:
        	gpio_direction_output(UART1_EN,0);
        	gpio_direction_output(UART1_SEL1,1);
        	gpio_direction_output(UART1_SEL0,1);       	
        	gpiodrv_dbg("switch uart1 to magcard\n");
        break;            
        case UART3_DISABLE :
        	gpio_direction_output(UART3_EN,1);
        	gpiodrv_dbg("disable uart3 port\n");
        break;            
        case UART_FINGER:
        case eUART_FINGER:
        	gpio_direction_output(UART3_EN,0);
        	gpio_direction_output(UART3_SEL1,0);
        	gpio_direction_output(UART3_SEL0,0);       	
        	gpiodrv_dbg("switch uart1 to finger\n");
        break;            
        case UART_LEDDISP:
        case eUART_LEDDISP:
        	gpio_direction_output(UART3_EN,0);
        	gpio_direction_output(UART3_SEL1,0);
        	gpio_direction_output(UART3_SEL0,1);       	
        	gpiodrv_dbg("switch uart1 to led custom display\n");
        break;            
        case UART_PRINTER:
        case eUART_PRINTER:
        	gpio_direction_output(UART3_EN,0);
        	gpio_direction_output(UART3_SEL1,1);
        	gpio_direction_output(UART3_SEL0,0);       	
        	gpiodrv_dbg("switch uart1 to printer\n");
        break;            
        case eDISP_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(DISP_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set DISP_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eDISP_PWR;
        	if (io_level)
        	{
        		status |= eDISP_PWR;
        	}
        break;            
        case eFINGER_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(FINGER_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set FINGER_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eFINGER_PWR;
        	if (io_level)
        	{
        		status |= eFINGER_PWR;
        	}
        break;            
        case eFINGER_INT:
        	io_level = (unsigned long)gpio_get_value(FINGER_INT);
        	io_level = !!io_level;
        	if (io_level)
        	{
        		gpiodrv_dbg("get FINGER_INT_VALUE high\n");
	        	status |= eFINGER_INT;
        	}
        	else
        	{
        		gpiodrv_dbg("get FINGER_INT_VALUE low\n");
	        	status &= ~eFINGER_INT;
        	}
        break;            
        case eUSBKEY_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(USBKEY_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set USBKEY_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eUSBKEY_PWR;
        	if (io_level)
        	{
        		status |= eUSBKEY_PWR;
        	}
        break;            
        case eMAGCARD_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(MAGCARD_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set MAGCARD_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eMAGCARD_PWR;
        	if (io_level)
        	{
        		status |= eMAGCARD_PWR;
        	}
        break;            
        case eLDTONG_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(LDTONG_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set LDTONG_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eLDTONG_PWR;
        	if (io_level)
        	{
        		status |= eLDTONG_PWR;
        	}
        break;            
        case eQX_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(QX_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set QX_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eQX_PWR;
        	if (io_level)
        	{
        		status |= eQX_PWR;
        	}
        break;            
        case eRFID_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(RFID_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set RFID_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eRFID_PWR;
        	if (io_level)
        	{
        		status |= eRFID_PWR;
        	}
        break;            
        case eRFID_INT:
        	io_level = (unsigned long)gpio_get_value(RFID_INT);
        	io_level = !!io_level;
        	if (io_level)
        	{
        		gpiodrv_dbg("get RFID_INT_VALUE high\n");
        		status |= eRFID_INT;
        	}
        	else
        	{
        		gpiodrv_dbg("get RFID_INT_VALUE low\n");
        		status &= ~eRFID_INT;
        	}
        break;            
        case eSCAN_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(SCAN_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set SCAN_PWR %s\n",io_level ? "high" : "low");
        	status &= ~eSCAN_PWR;
        	if (io_level)
        	{
        		status |= eSCAN_PWR;
        	}
        break;            
        case eSCAN_RST:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(SCAN_RST,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set SCAN_RST %s\n",io_level ? "high" : "low");
        	status &= ~eSCAN_RST;
        	if (io_level)
        	{
        		status |= eSCAN_RST;
        	}
        break;            
        case eSCAN_PWDN :
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(SCAN_PWDN,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set SCAN_PWDN %s\n",io_level ? "high" : "low");
        	status &= ~eSCAN_PWDN;
        	if (io_level)
        	{
        		status |= eSCAN_PWDN;
        	}
        break;            
        case eSCAN_TRIG :
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(SCAN_TRIG,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set SCAN_TRIG %s\n",io_level ? "high" : "low");
        	status &= ~eSCAN_TRIG;
        	if (io_level)
        	{
        		status |= eSCAN_TRIG;
        	}
        break;            
        case ePRINT_PWR:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
					if (copy_from_user(&io_level, argp, sizeof(io_level)))
					{
						return -EFAULT;
					}
        	gpio_direction_output(PRINT_PWR,io_level ? 1 : 0);       	
        	gpiodrv_dbg("set PRINT_PWR %s\n",io_level ? "high" : "low");
        	status &= ~ePRINT_PWR;
        	if (io_level)
        	{
        		status |= ePRINT_PWR;
        	}
        break; 
        case IO_GET_STATUS:
        	if (argp == 0)
        	{
        		return -EFAULT;
        	}
        	io_level = (unsigned long)status;
        	if (copy_to_user(argp,&io_level,sizeof(io_level)))
        	{
        		return -EFAULT;
        	}
        break;           
        default:
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
    .read     = gpiodrv_read,
    .write    = gpiodrv_write,   
};  



static u32 StrtoInt(const char *str)
{
	u32 i,HexValue=0;
		
	//printk("strtoin=%s \n",str);
	if(!str)
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(i=0; *str; str++)
	{
		if((*str>='A' && *str<='F')||(*str>='a' && *str<='f')||(*str>='0' && *str<='9'))
		{
			 HexValue <<= 4;
			if(*str>='A' && *str<='F')
				HexValue += *str-'A'+10;
			else if(*str>='a' && *str<='f')
				HexValue += *str-'a'+10;
			else if(*str>='0' && *str<='9')
				HexValue += *str-'0';				
		}
		else
				return HexValue;
	}
	return HexValue;
}

static u32 io_num=0;

static ssize_t gpio_switch_show(struct device *dev,struct device_attribute *attr ,char *buf)
{
	return sprintf(buf,"%04x\n",io_num);
}

static ssize_t gpio_switch_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	u32 val=0,io_level=0;
	//printk("store buf=%s \n",buf);	
	val=(u32)StrtoInt(buf);
	gpiodrv_ioctl(0,val,(u32)&io_level);
	return count;
} 

static DEVICE_ATTR(gpio_switch, 0666, gpio_switch_show, gpio_switch_store);

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
  gpio_dev = device_create(gpiodrv_class, NULL, MKDEV(gpiodrv_major, MINOR(dev)), NULL,DEV_NAME);

	result = device_create_file(gpio_dev, &dev_attr_gpio_switch);
	if (result != 0) {
		printk("Failed to create gpio_switch sysfs files: %d\n", result);
		return result;
	}

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

#ifdef DISP_PWR_IOMUX
		rk30_mux_api_set(DISP_PWR_IOMUX,0);
#endif
		result = gpio_request(DISP_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request DISP_PWR failed\n");
			return result;
		}
		gpio_direction_output(DISP_PWR,0);

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

#ifdef MAGCARD_PWR_IOMUX
		rk30_mux_api_set(MAGCARD_PWR_IOMUX,0);
#endif
		result = gpio_request(MAGCARD_PWR,NULL);
		if (result < 0 )
		{
			printk("gpio_request MAGCARD_PWR failed\n");
			return result;
		}
		gpio_direction_output(MAGCARD_PWR,0);

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
		device_remove_file(gpio_dev,&dev_attr_gpio_switch);
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

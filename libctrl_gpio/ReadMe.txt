package com.ctrl.gpio;  

enum
{
	eDISP_PWR			= 1,
	eFINGER_PWR		= 2,
	eFINGER_INT		= 4,
	eUSBKEY_PWR		= 8,
	eMAGCARD_PWR	= 16,
	eLDTONG_PWR		= 32,
	eQX_PWR				= 64,
	eRFID_PWR			= 128,
	eRFID_INT			= 256,
	eSCAN_PWR			= 512,
	eSCAN_RST			= 1024,
	eSCAN_PWDN		= 2048,
	eSCAN_TRIG		= 4096,
	ePRINT_PWR		= 8192,
	//以上为IO操作枚举值，供activate和get_status函数调用
};
  
public class Ioctl {  
    public native static int convertRfid();
    public native static int convertScanner();
    public native static int convertLdtong();
    public native static int convertMagcard();
    public native static int convertFinger();
    public native static int convertLed();
    public native static int convertPrinter();
    public native static int activate(int type,int open);//参数：上电或下电IO口
    public native static int get_status(int type);//攻取IO状态(0为下电，1为上电，负值为不正常)

		static {
			try {
				System.loadLibrary("gtrl_gpio");
			} catch (UnsatisfiedLinkError ule) {
				System.err.println("WARNING: Could not load library!");
			}
		}
}  

package com.ctrl.gpio;  

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
	//以上为IO操作枚举值，供activate和get_status函数调用
	eUART_RFID		= 1<<14,
	eUART_SCAN		= 1<<15,
	eUART_LDTONG	= 1<<16,
	eUART_MAGCARD	= 1<<17,
	eUART_FINGER	= 1<<18,
	eUART_LEDDISP	= 1<<19,
	eUART_PRINTER	= 1<<20,
	//以上为端口切换输入参数
};
  
public class Demo {  
    static  
    {  
        System.loadLibrary("gtrl_gpio");  
    }  
    public native static int convertRfid();
    public native static int convertScanner();
    public native static int convertLdtong();
    public native static int convertMagcard();
    public native static int convertFinger();
    public native static int convertLed();
    public native static int convertPrinter();
    public native static int activate(int type,int open);//参数：上电或下电IO口
    public native static int get_status(int type);//攻取IO状态(0为下电，1为上电，负值为不正常)
}  

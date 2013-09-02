java类：GpioJNI

本地函数声明：
public class I2CTools {

//没有写参数的，都是以0表示无效，1表示有效；内部有电平转换，
//无论该管脚是低电平有效还是高电平有效，当输入1时，函数内部会转成相应的有效电平

//传入参数			0:切换到GPS端口，1:切换到BT端口，其它参数无效
	static public native void gpio_switch_gps_bluetooth(int flag);

//返回参数			0:GPS端口被选通，1:BT端口被选通
	static public native int gpio_get_gps_bluetooth();


	static public native void gpio_switch_gps_power(int flag);

	static public native int gpio_get_gps_power();

//传入参数			0:切换到485端口，1:切换到232端口，其它参数无效
	static public native void gpio_switch_rs485_rs323(int flag);

//返回参数			0:485端口被选通，1:232端口被选通
	static public native int gpio_get_rs485_rs323();

	static public native void gpio_switch_rs485_power(int flag);

	static public native int gpio_get_rs485_power();

	static public native void gpio_switch_rs232_power(int flag);

	static public native int gpio_get_rs232_power();
	

//传入参数		0:切换到SCAN端口，1:切换到RF端口，2:切换到IRED端口，其它参数无效
	static public native void gpio_switch_scan_rf_ired(int flag);
	

//返回参数			0:SCAN端口被选通，1:RF端口被选通，1:IRED端口被选通
	static public native int gpio_get_scan_rf_ired();

	static public native void gpio_switch_rf_power(int flag);

	static public native int gpio_get_rf_power();

	static public native void gpio_switch_scan_power(int flag);

	static public native int gpio_get_scan_power();

	static public native void gpio_switch_scan_powerdown(int flag);

	static public native int gpio_get_scan_powerdown();

	static public native void gpio_switch_scan_trig(int flag);

	static public native int gpio_get_scan_trig();

	static public native void gpio_switch_scan_reset(int flag);

	static public native int gpio_get_scan_reset();

	static public native void gpio_switch_rf_reset(int flag);

	static public native int gpio_get_rf_reset();

	static public native void gpio_switch_ired(int flag);

	static public native int gpio_get_ired();

}

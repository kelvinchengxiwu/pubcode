java�ࣺGpioJNI

���غ���������
public class I2CTools {

//û��д�����ģ�������0��ʾ��Ч��1��ʾ��Ч���ڲ��е�ƽת����
//���۸ùܽ��ǵ͵�ƽ��Ч���Ǹߵ�ƽ��Ч��������1ʱ�������ڲ���ת����Ӧ����Ч��ƽ

//�������			0:�л���GPS�˿ڣ�1:�л���BT�˿ڣ�����������Ч
	static public native void gpio_switch_gps_bluetooth(int flag);

//���ز���			0:GPS�˿ڱ�ѡͨ��1:BT�˿ڱ�ѡͨ
	static public native int gpio_get_gps_bluetooth();


	static public native void gpio_switch_gps_power(int flag);

	static public native int gpio_get_gps_power();

//�������			0:�л���485�˿ڣ�1:�л���232�˿ڣ�����������Ч
	static public native void gpio_switch_rs485_rs323(int flag);

//���ز���			0:485�˿ڱ�ѡͨ��1:232�˿ڱ�ѡͨ
	static public native int gpio_get_rs485_rs323();

	static public native void gpio_switch_rs485_power(int flag);

	static public native int gpio_get_rs485_power();

	static public native void gpio_switch_rs232_power(int flag);

	static public native int gpio_get_rs232_power();
	

//�������		0:�л���SCAN�˿ڣ�1:�л���RF�˿ڣ�2:�л���IRED�˿ڣ�����������Ч
	static public native void gpio_switch_scan_rf_ired(int flag);
	

//���ز���			0:SCAN�˿ڱ�ѡͨ��1:RF�˿ڱ�ѡͨ��1:IRED�˿ڱ�ѡͨ
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

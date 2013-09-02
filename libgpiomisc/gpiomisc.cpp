/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "gpio gpiomisc.c"
#include <utils/Log.h>

#include <fcntl.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include <linux/delay.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include "jni.h"

#define GPIO_ON				(1<<31)
#define GPIO_OFF			0
#define GPS_CTRL			(1<<0)
#define GPS_PWR				(1<<1)
#define BT_CTRL				(1<<2)
#define RS485_CTRL		(1<<3)
#define RS485_PWR			(1<<4)
#define RS232_CTRL		(1<<5)
#define RS232_PWR			(1<<6)
#define SER1_CS1			(1<<7)
#define SER1_CS2			(1<<8)
#define RF_PWR				(1<<9)
#define RF_RST				(1<<10)
#define SCAN_PWR			(1<<11)
#define SCAN_PWDN			(1<<12)
#define SCAN_TRIG			(1<<13)
#define SCAN_RST			(1<<14)
#define IRED_CTRL			(1<<15)

#define DEVICE_NAME "/dev/gpiomisc"
static int fd = 0;
//****************************************************
//JNI interface
//****************************************************
static void gpio_switch_gps_bluetooth(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){//turn on gps and turn off bt
		ioctl(fd, GPIO_OFF|BT_CTRL, 0);
		ioctl(fd, GPIO_ON|GPS_CTRL,0);
	}
	else if (flag == 1){//turn on bt and turn off gps
		ioctl(fd, GPIO_ON|BT_CTRL, 0);
		ioctl(fd, GPIO_OFF|GPS_CTRL,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_gps_bluetooth(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag,cs1,cs2;
	ioctl(fd,0,&flag);
	cs1 = !!(flag&GPS_CTRL);
	cs2 = !!(flag&BT_CTRL);
	if (cs1 == 1 && cs2 == 0){
		flag == 0;
	}
	else if (cs1 == 0 && cs2 == 1){
		flag = 1;
	}
	else{
		return -1;
	}
	LOGE("gpiomisc: %s --------",__func__);
	return flag;
}

static void gpio_switch_gps_power(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|GPS_PWR,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|GPS_PWR,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_gps_power(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&GPS_PWR);
}

static void gpio_switch_rs485_rs232(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){//turn on 485 and turn off 232
		ioctl(fd, GPIO_OFF|RS232_CTRL, 0);
		ioctl(fd, GPIO_ON|RS485_CTRL,0);
	}
	else if (flag == 1){//turn on 232 and turn off 485
		ioctl(fd, GPIO_ON|RS232_CTRL, 0);
		ioctl(fd, GPIO_OFF|RS485_CTRL,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_rs485_rs232(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag,cs1,cs2;
	ioctl(fd,0,&flag);
	cs1 = !!(flag&RS485_CTRL);
	cs2 = !!(flag&RS232_CTRL);
	if (cs1 == 1 && cs2 == 0){
		flag == 0;
	}
	else if (cs1 == 0 && cs2 == 1){
		flag = 1;
	}
	else{
		return -1;
	}
	LOGE("gpiomisc: %s --------",__func__);
	return flag;
}

static void gpio_switch_rs485_power(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|RS485_PWR,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|RS485_PWR,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_rs485_power(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&RS485_PWR);
}

static void gpio_switch_rs232_power(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|RS232_PWR,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|RS232_PWR,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_rs232_power(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&RS232_PWR);
}

static void gpio_switch_scan_rf_ired(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){//turn on scan and turn off rf,ired
		ioctl(fd, GPIO_OFF|SER1_CS1, 0);
		ioctl(fd, GPIO_ON|SER1_CS2,0);
	}
	else if (flag == 1){//turn on rf and turn off scan,ired
		ioctl(fd, GPIO_ON|SER1_CS1, 0);
		ioctl(fd, GPIO_OFF|SER1_CS2,0);
	}
	else if (flag == 2){//turn on ired and turn off scan,rf
		ioctl(fd, GPIO_ON|SER1_CS1|SER1_CS2, 0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_scan_rf_ired(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag,cs1,cs2;
	ioctl(fd,0,&flag);
	cs1 = !!(flag&SER1_CS1);
	cs2 = !!(flag&SER1_CS2);
	if (cs1 == 0 && cs2 == 1){
		flag == 0;
	}
	else if (cs1 == 1 && cs2 == 0){
		flag = 1;
	}
	else if (cs1 == 1 && cs2 == 1){
		flag = 2;
	}
	else{
		return -1;
	}
	LOGE("gpiomisc: %s --------",__func__);
	return flag;
}

static void gpio_switch_rf_power(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|RF_PWR,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|RF_PWR,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}


static int gpio_get_rf_power(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&RF_PWR);
}

static void gpio_switch_scan_power(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|SCAN_PWR,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|SCAN_PWR,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_scan_power(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&SCAN_PWR);
}

static void gpio_switch_scan_powerdown(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|SCAN_PWDN,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|SCAN_PWDN,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_scan_powerdown(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&SCAN_PWDN);
}

static void gpio_switch_scan_trig(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|SCAN_TRIG,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|SCAN_TRIG,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_scan_trig(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&SCAN_TRIG);
}

static void gpio_switch_scan_reset(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|SCAN_RST,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|SCAN_RST,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_scan_reset(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&SCAN_RST);
}

static void gpio_switch_rf_reset(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|RF_RST,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|RF_RST,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_rf_reset(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&RF_RST);
}

static void gpio_switch_ired(JNIEnv *env, jobject thiz,int flag)
{
	if (fd < 1) {
		return;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	if (flag == 0){
		ioctl(fd, GPIO_OFF|IRED_CTRL,0);
	}
	else if (flag == 1){
		ioctl(fd, GPIO_ON|IRED_CTRL,0);
	}
	else {
		return;
	}
	LOGE("gpiomisc: %s --------",__func__);
}

static int gpio_get_ired(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -1;
	}
	LOGE("gpiomisc: %s ++++++++",__func__);
	int flag;
	ioctl(fd,0,&flag);
	
	LOGE("gpiomisc: %s --------",__func__);
	return !!(flag&IRED_CTRL);
}

//=======================================================================

static const char *classPathName = "android/gpio/GpioJNI";

static JNINativeMethod methods[] = {
  {"gpio_switch_gps_bluetooth", "(I)V", (void*)gpio_switch_gps_bluetooth },
  {"gpio_get_gps_bluetooth", "()I", (void*)gpio_get_gps_bluetooth },
  {"gpio_switch_gps_power", "(I)V", (void*)gpio_switch_gps_power },
  {"gpio_get_gps_power", "()I", (void*)gpio_get_gps_power },
  {"gpio_switch_rs485_rs232", "(I)V", (void*)gpio_switch_rs485_rs232 },
  {"gpio_get_rs485_rs232", "()I", (void*)gpio_get_rs485_rs232 },
  {"gpio_switch_rs485_power", "(I)V", (void*)gpio_switch_rs485_power },
  {"gpio_get_rs485_power", "()I", (void*)gpio_get_rs485_power },
  {"gpio_switch_rs232_power", "(I)V", (void*)gpio_switch_rs232_power },
  {"gpio_get_rs232_power", "()I", (void*)gpio_get_rs232_power },
  {"gpio_switch_scan_rf_ired", "(I)V", (void*)gpio_switch_scan_rf_ired },
  {"gpio_get_scan_rf_ired", "()I", (void*)gpio_get_scan_rf_ired },
  {"gpio_switch_rf_power", "(I)V", (void*)gpio_switch_rf_power },
  {"gpio_get_rf_power", "()I", (void*)gpio_get_rf_power },
  {"gpio_switch_scan_power", "(I)V", (void*)gpio_switch_scan_power },
  {"gpio_get_scan_power", "()I", (void*)gpio_get_scan_power },
  {"gpio_switch_scan_powerdown", "(I)V", (void*)gpio_switch_scan_powerdown },
  {"gpio_get_scan_powerdown", "()I", (void*)gpio_get_scan_powerdown },
  {"gpio_switch_scan_trig", "(I)V", (void*)gpio_switch_scan_trig },
  {"gpio_get_scan_trig", "()I", (void*)gpio_get_scan_trig },
  {"gpio_switch_scan_reset", "(I)V", (void*)gpio_switch_scan_reset },
  {"gpio_get_scan_reset", "()I", (void*)gpio_get_scan_reset },
  {"gpio_switch_rf_reset", "(I)V", (void*)gpio_switch_rf_reset },
  {"gpio_get_rf_reset", "()I", (void*)gpio_get_rf_reset },
  {"gpio_switch_ired", "(I)V", (void*)gpio_switch_ired },
  {"gpio_get_ired", "()I", (void*)gpio_get_ired },
};


// ----------------------------------------------------------------------------

/*
 * This is called by the VM when the shared library is first loaded.
 */
 
jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
    jint result = -1;
    JNIEnv* env = NULL;
    jclass clazz;
    LOGI("JNI_OnLoad");

    if (vm->GetEnv((void**)&env, JNI_VERSION_1_4) != JNI_OK) {
        LOGE("ERROR: GetEnv failed");
        goto bail;
    }
    
		if((fd = open(DEVICE_NAME, O_RDWR)) == -1) 
		{
			LOGE("gpiomisc: failed to open /dev/gpiomisc -- %s.", strerror(errno));
			goto bail;
		}
    result = JNI_VERSION_1_4;
    LOGE("gpiomisc: find class ++++++++");
    clazz = env->FindClass(classPathName);
    LOGE("gpiomisc: find class --------");
    LOGE("gpiomisc: RegisterNatives +++++++++++");
    env->RegisterNatives(clazz,methods, sizeof(methods) / sizeof(methods[0]));
    LOGE("gpiomisc: RegisterNatives -----------");
bail:
    return result;
}

//onUnLoad方法，在JNI组件被释放时调用  
void JNI_OnUnload(JavaVM* vm, void* reserved){  
     LOGE("call JNI_OnUnload ~~!!");
     if (fd > 0){
     	close(fd);
    }  
}  

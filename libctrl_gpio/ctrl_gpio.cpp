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

#define LOG_TAG "ctrl_gpio"
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


#define IO_GET_STATUS		0x1234                              
#define DEVICE_NAME "/dev/ctrl_gpio"
static int fd = 0;

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

//****************************************************
//JNI interface
//****************************************************

static int native_convertRfid(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -EFAULT;
	}
	return ioctl(fd,eUART_RFID,0);
}

static int native_convertScanner(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -EFAULT;
	}
	return ioctl(fd,eUART_SCAN,0);
}

static int native_convertLdtong(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -EFAULT;
	}
	return ioctl(fd,eUART_LDTONG,0);
}

static int native_convertMagcard(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -EFAULT;
	}
	return ioctl(fd,eUART_MAGCARD,0);
}

static int native_convertFinger(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -EFAULT;
	}
	return ioctl(fd,eUART_FINGER,0);
}

static int native_convertLed(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -EFAULT;
	}
	return ioctl(fd,eUART_LEDDISP,0);
}

static int native_convertPrinter(JNIEnv *env, jobject thiz)
{
	if (fd < 1) {
		return -EFAULT;
	}
	return ioctl(fd,eUART_PRINTER,0);
}

static int native_activate(JNIEnv *env, jobject thiz,int type,int open)
{
	if (fd < 1) {
		return -EFAULT;
	}
	int ret = 0;
	open = !!open;
	switch (type)
	{
		case eDISP_PWR:
		case eFINGER_PWR:
		case eFINGER_INT:
		case eUSBKEY_PWR:
		case eMAGCARD_PWR:
		case eLDTONG_PWR:
		case eQX_PWR:
		case eRFID_PWR:
		case eRFID_INT:
		case eSCAN_PWR:
		case eSCAN_RST:
		case eSCAN_PWDN:
		case eSCAN_TRIG:
		case ePRINT_PWR:
			ret = ioctl(fd,type,&open);
		break;
		default:
			ret = -EFAULT;
		break;
	}
	return ret;
}

static int native_get_status(JNIEnv *env, jobject thiz,int type)
{
	if (fd < 1) {
		return  -EFAULT;
	}
	int ret = 0;
	int result;
	ret = ioctl(fd,IO_GET_STATUS,&result);
	if (ret != 0)
	{
		return  -EFAULT;
	}
	
	switch (type)
	{
		case eDISP_PWR:
		case eFINGER_PWR:
		case eFINGER_INT:
		case eUSBKEY_PWR:
		case eMAGCARD_PWR:
		case eLDTONG_PWR:
		case eQX_PWR:
		case eRFID_PWR:
		case eRFID_INT:
		case eSCAN_PWR:
		case eSCAN_RST:
		case eSCAN_PWDN:
		case eSCAN_TRIG:
		case ePRINT_PWR:
			ret = !!(result & type);
		break;
		default:
			ret = -EFAULT;
		break;
	}
	return ret;
}

//=======================================================================

static const char *classPathName = "com/ctrl/gpio/Ioctl";

static JNINativeMethod methods[] = {
  {"convertRfid", "()I", (void*)native_convertRfid },
  {"convertScanner", "()I", (void*)native_convertScanner },
  {"convertLdtong", "()I", (void*)native_convertLdtong },
  {"convertMagcard", "()I", (void*)native_convertMagcard },
  {"convertFinger", "()I", (void*)native_convertFinger },
  {"convertLed", "()I", (void*)native_convertLed },
  {"convertPrinter", "()I", (void*)native_convertPrinter },
  {"activate", "(II)I", (void*)native_activate },
  {"get_status", "(I)I", (void*)native_get_status },
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
    ALOGI("JNI_OnLoad");

    if (vm->GetEnv((void**)&env, JNI_VERSION_1_4) != JNI_OK) {
        ALOGE("ERROR: GetEnv failed");
        goto bail;
    }
    
		if((fd = open(DEVICE_NAME, O_RDWR)) == -1) 
		{
			ALOGE("failed to open /dev/ctrl_gpio -- %s.", strerror(errno));
			goto bail;
		}
    result = JNI_VERSION_1_4;
    clazz = env->FindClass(classPathName);
    env->RegisterNatives(clazz,methods, sizeof(methods) / sizeof(methods[0]));
bail:
    return result;
}

//onUnLoad方法，在JNI组件被释放时调用  
void JNI_OnUnload(JavaVM* vm, void* reserved){  
     ALOGE("call JNI_OnUnload ~~!!");
     if (fd > 0){
     	close(fd);
    }  
}  

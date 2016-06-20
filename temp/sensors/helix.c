/*
 *  helix.c - Linux kernel modules for sensortek Helix301x, Helix321x and Helix331x
 *  proximity/ambient light sensor
 *
 *  Copyright (c) 2013, The Linux Foundation. All Rights Reserved.
 *  Copyright (C) 2012 Lex Hsieh / sensortek <lex_hsieh@sitronix.com.tw> or
 *   <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Linux Foundation chooses to take subject only to the GPLv2 license
 *  terms, and distributes only under these terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/helix_1.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "helix.h"

#define DRIVER_VERSION  "3.4.4ts"

/* Driver Settings */
#define CONFIG_Helix_PS_ALS_USE_CHANGE_THRESHOLD
#ifdef CONFIG_Helix_PS_ALS_USE_CHANGE_THRESHOLD
#define Helix_ALS_CHANGE_THD	20	/* The threshold to trigger ALS interrupt, unit: lux */
#endif	/* #ifdef CONFIG_Helix_PS_ALS_USE_CHANGE_THRESHOLD */
#define Helix_INT_PS_MODE			1	/* 1, 2, or 3	*/
//#define Helix_POLL_PS
#define Helix_POLL_ALS		/* ALS interrupt is valid only when Helix_PS_INT_MODE = 1	or 4*/

/* Define Register Map */
#define Helix_STATE_REG 			0x00
#define Helix_PSCTRL_REG 			0x01
#define Helix_ALSCTRL_REG 		0x02
#define Helix_LEDCTRL_REG 		0x03
#define Helix_INT_REG 			0x04
#define Helix_WAIT_REG 			0x05
#define Helix_THDH1_PS_REG 		0x06
#define Helix_THDH2_PS_REG 		0x07
#define Helix_THDL1_PS_REG 		0x08
#define Helix_THDL2_PS_REG 		0x09
#define Helix_THDH1_ALS_REG 		0x0A
#define Helix_THDH2_ALS_REG 		0x0B
#define Helix_THDL1_ALS_REG 		0x0C
#define Helix_THDL2_ALS_REG 		0x0D
#define Helix_FLAG_REG 			0x10
#define Helix_DATA1_PS_REG	 	0x11
#define Helix_DATA2_PS_REG 		0x12
#define Helix_DATA1_ALS_REG 		0x13
#define Helix_DATA2_ALS_REG 		0x14
#define Helix_DATA1_OFFSET_REG 	0x15
#define Helix_DATA2_OFFSET_REG 	0x16
#define Helix_DATA1_IR_REG 		0x17
#define Helix_DATA2_IR_REG 		0x18
#define Helix_PDT_ID_REG 			0x3E
#define Helix_RSRVD_REG 			0x3F
#define Helix_SW_RESET_REG		0x80


/* Define state reg */
#define Helix_STATE_EN_IRS_SHIFT  	7
#define Helix_STATE_EN_AK_SHIFT  		6
#define Helix_STATE_EN_ASO_SHIFT  	5
#define Helix_STATE_EN_IRO_SHIFT  	4
#define Helix_STATE_EN_WAIT_SHIFT  	2
#define Helix_STATE_EN_ALS_SHIFT  	1
#define Helix_STATE_EN_PS_SHIFT  		0

#define Helix_STATE_EN_IRS_MASK	0x80
#define Helix_STATE_EN_AK_MASK	0x40
#define Helix_STATE_EN_ASO_MASK	0x20
#define Helix_STATE_EN_IRO_MASK	0x10
#define Helix_STATE_EN_WAIT_MASK	0x04
#define Helix_STATE_EN_ALS_MASK	0x02
#define Helix_STATE_EN_PS_MASK	0x01

/* Define PS ctrl reg */
#define Helix_PS_PRS_SHIFT  		6
#define Helix_PS_GAIN_SHIFT  		4
#define Helix_PS_IT_SHIFT  		0

#define Helix_PS_PRS_MASK			0xC0
#define Helix_PS_GAIN_MASK		0x30
#define Helix_PS_IT_MASK			0x0F

/* Define ALS ctrl reg */
#define Helix_ALS_PRS_SHIFT  		6
#define Helix_ALS_GAIN_SHIFT  	4
#define Helix_ALS_IT_SHIFT  		0

#define Helix_ALS_PRS_MASK		0xC0
#define Helix_ALS_GAIN_MASK		0x30
#define Helix_ALS_IT_MASK			0x0F

/* Define LED ctrl reg */
#define Helix_LED_IRDR_SHIFT  	6
#define Helix_LED_DT_SHIFT  		0

#define Helix_LED_IRDR_MASK		0xC0
#define Helix_LED_DT_MASK			0x3F

/* Define interrupt reg */
#define Helix_INT_CTRL_SHIFT  	7
#define Helix_INT_OUI_SHIFT  		4
#define Helix_INT_ALS_SHIFT  		3
#define Helix_INT_PS_SHIFT  		0

#define Helix_INT_CTRL_MASK		0x80
#define Helix_INT_OUI_MASK		0x10
#define Helix_INT_ALS_MASK		0x08
#define Helix_INT_PS_MASK			0x07

#define Helix_INT_ALS				0x08

/* Define flag reg */
#define Helix_FLG_ALSDR_SHIFT  		7
#define Helix_FLG_PSDR_SHIFT  		6
#define Helix_FLG_ALSINT_SHIFT  		5
#define Helix_FLG_PSINT_SHIFT  		4
#define Helix_FLG_OUI_SHIFT  			2
#define Helix_FLG_IR_RDY_SHIFT  		1
#define Helix_FLG_NF_SHIFT  			0

#define Helix_FLG_ALSDR_MASK		0x80
#define Helix_FLG_PSDR_MASK		0x40
#define Helix_FLG_ALSINT_MASK		0x20
#define Helix_FLG_PSINT_MASK		0x10
#define Helix_FLG_OUI_MASK		0x04
#define Helix_FLG_IR_RDY_MASK		0x02
#define Helix_FLG_NF_MASK			0x01

/* misc define */
#define MIN_ALS_POLL_DELAY_NS	110000000

#define DEVICE_NAME		"Helix_ps"
#define ALS_NAME		"light_sensor"
#define PS_NAME "proximity_sensor"

/* POWER SUPPLY VOLTAGE RANGE */
#define Helix3X1X_VDD_MIN_UV	2000000
#define Helix3X1X_VDD_MAX_UV	3300000
#define Helix3X1X_VIO_MIN_UV	1750000
#define Helix3X1X_VIO_MAX_UV	1950000

#define Helix_FIR_LEN 16
#define MAX_FIR_LEN 32

#define MAX_LUX 30000


#define HELIX_USE_PS_CALIBRATION
#ifdef	HELIX_USE_PS_CALIBRATION
#define DEFAULT_CROSS_TALK_MIN 2500
#define DEFAULT_CROSS_TALK_MAX 6500
#define DEFAULT_BLACK_HAIR 7000
#define CROSSTALK_FILE_PATCH "/persist/sensors/psensor/crosstalk" 
#endif

#define DEFAULT_BLACK_HAIR_STANDBY 	 2000
#define DEFAULT_CROSS_TALK 	 5000
#define NEAR_OFFSET 		 1800
#define FAR_OFFSET 	 		 800

#define NEAR_TO_FAR 1
#define FAR_TO_NEAR 0


struct pinctrl_config {
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*state[2];
	char			*name[2];
};

static struct sensors_classdev light_cdev = {
	.name = "light_sensor",
	.vendor = "Helix",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "65536",
	.resolution = "1.0",
	.sensor_power = "0.25",
	.min_delay = 50000,
	.max_delay = 2000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 2,
	.enabled = 0,
	.delay_msec = 50,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev proximity_cdev = {
	.name = "proximity_sensor",
	.vendor = "Helix",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "7",
	.resolution = "1.0",
	.sensor_power = "0.25",
	.min_delay = 10000,
	.max_delay = 2000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 3,
	.enabled = 0,
	.delay_msec = 50,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


static struct pinctrl_config pin_config = {
	.name = { "default", "sleep" },
};

struct data_filter {
	u16 raw[MAX_FIR_LEN];
	int sum;
	int number;
	int idx;
};

struct helix_data {
	struct i2c_client *client;
	struct helix_platform_data *pdata;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
#if (!defined(Helix_POLL_PS) || !defined(Helix_POLL_ALS))
    int32_t irq;
    struct work_struct Helix_work;
	struct workqueue_struct *Helix_wq;
#endif
	int		int_pin;
	uint8_t wait_reg;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend Helix_early_suspend;
#endif
	struct mutex io_lock;
	struct mutex update_lock;
	struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	struct wake_lock ps_wakelock;
	struct work_struct Helix_ps_work;
	struct workqueue_struct *Helix_ps_wq;
#ifdef Helix_POLL_PS
	struct wake_lock ps_nosuspend_wl;
#endif
	struct input_dev *als_input_dev;
	int32_t als_lux_last;
	uint32_t als_transmittance;
	bool als_enabled;
	bool als_suspend_enabled;
	bool ps_suspend_enabled;
	struct hrtimer als_timer;
	struct hrtimer ps_timer;
	ktime_t als_poll_delay;
	ktime_t ps_poll_delay;
#ifdef Helix_POLL_ALS
    struct work_struct Helix_als_work;
	struct workqueue_struct *Helix_als_wq;
#endif
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
	bool use_fir;
	struct data_filter      fir;
	atomic_t                firlength;
	bool ps_cal_is_support;
	struct delayed_work	delay_work_cal;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	unsigned int crosstalk;
	unsigned int black_hair;
#ifdef HELIX_USE_PS_CALIBRATION
	unsigned int avg_crosstalk;
	unsigned int ps_cal_result;
#endif
	
};

#if( !defined(CONFIG_Helix_PS_ALS_USE_CHANGE_THRESHOLD))
static uint32_t lux_threshold_table[] =
{
	3,
	10,
	40,
	65,
	145,
	300,
	550,
	930,
	1250,
	1700,
};

#define LUX_THD_TABLE_SIZE (sizeof(lux_threshold_table)/sizeof(uint32_t)+1)
static uint16_t code_threshold_table[LUX_THD_TABLE_SIZE+1];
#endif

struct helix_data *gl_helix_data;

static int32_t helix_init_all_setting(struct i2c_client *client, struct helix_platform_data *plat_data);
static int32_t helix_enable_ps(struct i2c_client *client, uint8_t enable);
static int32_t helix_enable_als(struct helix_data *client, uint8_t enable);
static int32_t helix_set_ps_thd_l(struct i2c_client *client, uint16_t thd_l);
static int32_t helix_set_ps_thd_h(struct i2c_client *client, uint16_t thd_h);
//static int32_t helix_set_als_thd_l(struct i2c_client *client, uint16_t thd_l);
//static int32_t helix_set_als_thd_h(struct i2c_client *client, uint16_t thd_h);
static int helix_device_ctl(struct helix_data *client, bool enable);
static int helix_read_alsch0(struct i2c_client *client);
static int helix_read_alsch1(struct i2c_client *client);
//static int32_t helix_set_ps_aoffset(struct helix_data *client, uint16_t offset);

static int LuxCalculation(struct helix_data *client)
{
 int luxValue = 0;
 unsigned int ch0data, ch1data;
 
//  printk("[Helix] %s\n", __func__);
 
	ch0data = helix_read_alsch0(client->client);
 	ch1data = helix_read_alsch1(client->client);
 
 	if(ch0data >= ch1data) {
  		luxValue = ch0data  * 39 /100;     //ch1data  * 26 /100;
 
  		if(luxValue> 4000){
   			luxValue = luxValue  * 15 /10;   //luxValue  * 123 /100;
   		}else{ 
   			luxValue = luxValue;
   		}
 	}else {
  		luxValue = ch1data  * 88 /100;     //ch1data  * 53 /100;
 
  		if(luxValue> 4000){
   			luxValue = luxValue  * 126 /100;  //luxValue  * 124 /100;
   		}else{ 
   			luxValue = luxValue;
  		}
  	}
 
  	if(luxValue > MAX_LUX)  {
   		luxValue = MAX_LUX;
  	}
 
  	if(luxValue < 0) {
   		luxValue = 0;
 	}
 
 	return luxValue;
}

static uint32_t helix_read_ps(struct i2c_client *client)
{
	struct helix_data *data = i2c_get_clientdata(client);
	unsigned int readValue;
	unsigned char upper, lower;
	int status1 = 0;//,status2=0;

// 	printk("[Helix] %s\n", __func__);

	mutex_lock(&data->update_lock);
	i2c_smbus_write_byte_data(client, 0x17, 0x01);
	msleep(6);
	upper = i2c_smbus_read_byte_data(client, 0x41);
	lower = i2c_smbus_read_byte_data(client, 0x42);
	status1 = i2c_smbus_read_byte_data(client, 0x40);
	mutex_unlock(&data->update_lock);
	readValue = (upper<<8)|lower;

	//status2 = i2c_smbus_read_byte_data(client, 0x40);

//	printk("%s,readValue=%x,status1 =%x,status2=%x\n",__func__,readValue,status1,status2);
	return readValue;
}

static int helix_read_alsch0(struct i2c_client *client)
{
	struct helix_data *data = i2c_get_clientdata(client);
	unsigned int readValue;
	unsigned char upper, lower;

// 	printk("[Helix] %s\n", __func__);

	mutex_lock(&data->update_lock);
	upper = i2c_smbus_read_byte_data(client, 0x43);
	lower = i2c_smbus_read_byte_data(client, 0x44);
	mutex_unlock(&data->update_lock);

	readValue = (upper<<8)|lower;

	return readValue;
}

static int helix_read_alsch1(struct i2c_client *client)
{
	struct helix_data *data = i2c_get_clientdata(client);
	unsigned int readValue;
	unsigned char upper, lower;

// 	printk("[Helix] %s\n", __func__);

	mutex_lock(&data->update_lock);
	upper = i2c_smbus_read_byte_data(client, 0x45);
	lower = i2c_smbus_read_byte_data(client, 0x46);
	mutex_unlock(&data->update_lock);

	readValue = (upper<<8)|lower;

	return readValue;
}

static unsigned int helix_check_clear_interrupt(struct helix_data *client)
{
	unsigned int ret;
	struct helix_data *data = client;

//	printk("[Helix] %s\n", __func__);

	ret = i2c_smbus_read_byte_data(data->client, 0x40);

	return ret;
}

void helix_swap(int *x, int *y)
{
 	int temp = *x;
 	*x = *y;
 	*y = temp;
}

#ifdef HELIX_USE_PS_CALIBRATION
static int read_file(char *filename)
{
	struct file *fp = NULL;
	char bufs[100];
	int ret;
	int data_val = 0;
	mm_segment_t fs;
	loff_t pos = 0;

	fp = filp_open(filename,O_RDONLY,0);
	if(IS_ERR(fp)){
		pr_err("%s:[%d]  Open %s fail \n", __func__,__LINE__,filename);
		return -1;
	}else{
		fs = get_fs();
		set_fs(KERNEL_DS);
		pos = fp->f_pos;
		ret = vfs_read(fp,bufs,sizeof(bufs),&pos);
		sscanf(bufs,"%d",&data_val);
		pr_err("%s:[%d] read val : %d \n", __func__,__LINE__,data_val);
		
		filp_close(fp,NULL);
		set_fs(fs);
		return data_val;	
	}
}
/*
static unsigned int bound(unsigned int val,unsigned int min,unsigned int max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;

	return val;
}
*/
static unsigned int helix_run_crosstalk_calibration(struct i2c_client *client)
{
 	struct helix_data *data = i2c_get_clientdata(client);
 	unsigned long sum_of_pdata = 0;
 	unsigned int temp_pdata[20];
 	unsigned int ArySize = 20;
 //	int crosstalk = 0;
 	int i, j;

 	for (i = 0; i < 20; i++ ) {
		//msleep(10);
 		temp_pdata[i] = helix_read_ps(client);
 	}
 	/* pdata sorting */
 	for (i = 0; i < ArySize - 1; i++  )
 		for (j = i +1; j < ArySize; j++  )
 			if (temp_pdata[i] > temp_pdata[j])
 				helix_swap(temp_pdata +  i, temp_pdata +  j);
 	/* calculate the cross-talk using central 10 data */
 	for (i = 5; i < 15; i++  ) {
// 		pr_info("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
 		sum_of_pdata = sum_of_pdata +  temp_pdata[i];
 	}
 	data->crosstalk = (unsigned int)sum_of_pdata/10;

// 	if(data->crosstalk > DEFAULT_BLACK_HAIR ){
//		crosstalk = read_file(CROSSTALK_FILE_PATCH); 
//		data->crosstalk = (crosstalk < 0) ? DEFAULT_CROSS_TALK : bound( crosstalk , DEFAULT_CROSS_TALK_MIN , DEFAULT_CROSS_TALK_MAX);
//	}
				
 	data->ps_thd_h = NEAR_OFFSET +  data->crosstalk;
 	data->ps_thd_l = FAR_OFFSET +  data->crosstalk;
	if(data->crosstalk > 5000)
		data->black_hair = DEFAULT_BLACK_HAIR_STANDBY + data->crosstalk;
	else
		data->black_hair = DEFAULT_BLACK_HAIR;

	data->ps_cal_is_support = true;
 	data->ps_cal_result = 1;
 	pr_err("%s(%d):  crosstalk = %d\n",	__func__,__LINE__, data->crosstalk);
 
 	return data->crosstalk;
 }
static unsigned int helix_run_crosstalk_calibration_delay(struct i2c_client *client)
{	
 	struct helix_data *data = i2c_get_clientdata(client);
	int crosstalk = 0;
	int crosstalk_file = 0;
        		
              disable_irq(data->irq);
	crosstalk = helix_run_crosstalk_calibration(client);              
              enable_irq(data->irq);
	
	if(crosstalk > DEFAULT_BLACK_HAIR){	
		crosstalk_file = read_file(CROSSTALK_FILE_PATCH); 
		if((crosstalk_file < 0) || (crosstalk < (crosstalk_file + DEFAULT_BLACK_HAIR_STANDBY)))
			data->crosstalk = crosstalk;
		else
			data->crosstalk = crosstalk_file;

		data->ps_thd_h = NEAR_OFFSET +  data->crosstalk;
 		data->ps_thd_l = FAR_OFFSET +  data->crosstalk;
		data->black_hair = DEFAULT_BLACK_HAIR_STANDBY + data->crosstalk; 
	}

	pr_err("%s(%d):  crosstalk = %d black_hair=%d \n",	__func__,__LINE__, data->crosstalk,data->black_hair);
 	return data->crosstalk;
}
#endif
unsigned int helix_read_ps_val(struct helix_data *data)
{
 	unsigned int temp_pdata[11];
 	unsigned int ArySize = 11;
 	int i, j;

 	for (i = 0; i < 11; i++ ) {
 		temp_pdata[i] = helix_read_ps(data->client);
 	}
 	/* pdata sorting */
 	for(i = 0; i < ArySize; i++  ){
 		for (j = i +1; j < ArySize; j++  )
 			if (temp_pdata[i] > temp_pdata[j])
 				helix_swap(temp_pdata +  i, temp_pdata +  j);
	}
	return temp_pdata[5];
}

bool helix_ps_is_near(void)
{
	struct helix_data *data = gl_helix_data;
	bool ps_is_near;
	unsigned int ps_val;
	unsigned char upper, lower;

/*
	if(data->ps_enabled){
		ps_val = helix_read_ps(data->client);
	}else{
		helix_enable_ps(data->client,1);
		ps_val = helix_read_ps(data->client);
		helix_enable_ps(data->client,0);
	}
	*/
	if(unlikely(data->ps_enabled)){
		upper = i2c_smbus_read_byte_data(data->client, 0x41);
		lower = i2c_smbus_read_byte_data(data->client, 0x42);
	}else{
		i2c_smbus_write_byte_data(data->client, 0x17, 0x02);
		msleep(5);
		upper = i2c_smbus_read_byte_data(data->client, 0x41);
		lower = i2c_smbus_read_byte_data(data->client, 0x42);
		//i2c_smbus_read_byte_data(data->client, 0x40);
		i2c_smbus_write_byte_data(data->client, 0x17, 0x0);
	}
	ps_val = (upper<<8)|lower;
	
 	if(ps_val > data->black_hair){
		ps_is_near = true;
		pr_err("ps_val=%d black_hair=%d \n",ps_val,data->black_hair);
	}else{
		ps_is_near = false;
	}

	return ps_is_near;
}
EXPORT_SYMBOL_GPL(helix_ps_is_near);

#if 0
inline uint32_t Helix_alscode2lux(struct helix_data *client, uint32_t alscode)
{
	alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));
    alscode<<=3;
    alscode/=client->als_transmittance;
	return alscode;
}

inline uint32_t Helix_lux2alscode(struct helix_data *client, uint32_t lux)
{
    lux*=client->als_transmittance;
    lux/=1100;
    if (unlikely(lux>=(1<<16)))
        lux = (1<<16) -1;
    return lux;
}

#ifndef CONFIG_Helix_PS_ALS_USE_CHANGE_THRESHOLD
static void Helix_init_code_threshold_table(struct helix_data *client)
{
    uint32_t i,j;
    uint32_t alscode;

    code_threshold_table[0] = 0;
#ifdef Helix_DEBUG_PRINTF
    printk(KERN_INFO "alscode[0]=%d\n",0);
#endif
    for (i=1,j=0;i<LUX_THD_TABLE_SIZE;i++,j++)
    {
        alscode = Helix_lux2alscode(client, lux_threshold_table[j]);
		dev_dbg(&client->client->dev, "alscode[%d]=%d\n", i, alscode);
        code_threshold_table[i] = (uint16_t)(alscode);
    }
    code_threshold_table[i] = 0xffff;
	dev_dbg(&client->client->dev, "alscode[%d]=%d\n", i, alscode);
}

static uint32_t Helix_get_lux_interval_index(uint16_t alscode)
{
    uint32_t i;
    for (i=1;i<=LUX_THD_TABLE_SIZE;i++)
    {
        if ((alscode>=code_threshold_table[i-1])&&(alscode<code_threshold_table[i]))
        {
            return i;
        }
    }
    return LUX_THD_TABLE_SIZE;
}
#else
inline void Helix_als_set_new_thd(struct helix_data *client, uint16_t alscode)
{
    int32_t high_thd,low_thd;
    high_thd = alscode + Helix_lux2alscode(client, Helix_ALS_CHANGE_THD);
    low_thd = alscode - Helix_lux2alscode(client, Helix_ALS_CHANGE_THD);
    if (high_thd >= (1<<16))
        high_thd = (1<<16) -1;
    if (low_thd <0)
        low_thd = 0;
    helix_set_als_thd_h(client, (uint16_t)high_thd);
    helix_set_als_thd_l(client, (uint16_t)low_thd);
}
#endif // CONFIG_Helix_PS_ALS_USE_CHANGE_THRESHOLD
#endif


static int32_t helix_init_all_reg(struct i2c_client *client, struct helix_platform_data *plat_data)
{
#if 0

	int32_t ret;
	uint8_t w_reg;

	w_reg = plat_data->state_reg;
    ret = i2c_smbus_write_byte_data(client->client, Helix_STATE_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

	client->ps_thd_h = plat_data->ps_thd_h;
	client->ps_thd_l = plat_data->ps_thd_l;

	w_reg = plat_data->psctrl_reg;
    ret = i2c_smbus_write_byte_data(client->client, Helix_PSCTRL_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	w_reg = plat_data->alsctrl_reg;
    ret = i2c_smbus_write_byte_data(client->client, Helix_ALSCTRL_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	w_reg = plat_data->ledctrl_reg;
    ret = i2c_smbus_write_byte_data(client->client, Helix_LEDCTRL_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	client->wait_reg = plat_data->wait_reg;

	if(client->wait_reg < 2)
	{
		printk(KERN_WARNING "%s: wait_reg should be larger than 2, force to write 2\n", __func__);
		client->wait_reg = 2;
	}
	else if (client->wait_reg > 0xFF)
	{
		printk(KERN_WARNING "%s: wait_reg should be less than 0xFF, force to write 0xFF\n", __func__);
		client->wait_reg = 0xFF;
	}
	w_reg = plat_data->wait_reg;
    ret = i2c_smbus_write_byte_data(client->client, Helix_WAIT_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	helix_set_ps_thd_h(client, client->ps_thd_h);
	helix_set_ps_thd_l(client, client->ps_thd_l);

	w_reg = 0;
#ifndef Helix_POLL_PS
	w_reg |= Helix_INT_PS_MODE;
#else
	w_reg |= 0x01;
#endif

#if (!defined(Helix_POLL_ALS) && (Helix_INT_PS_MODE != 0x02) && (Helix_INT_PS_MODE != 0x03))
	w_reg |= Helix_INT_ALS;
#endif
    ret = i2c_smbus_write_byte_data(client->client, Helix_INT_REG, w_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client->client, 0x87, 0x60);
	if (ret < 0) {
		dev_err(&client->client->dev,
			"%s: write i2c error\n", __func__);
		return ret;
	}
#else
	int err = 0;
	struct helix_data *data = i2c_get_clientdata(client);

// 	printk("[Helix] %s\n", __func__);

	mutex_lock(&data->update_lock); 
	err = i2c_smbus_write_byte_data(client, 0x02, 0x00);		// Interrupt Config
	if( err < 0 ) return err;
	err = i2c_smbus_write_byte_data(client, 0x0D, 0x17);		// LED frequency
	if( err < 0 ) return err;
	err = i2c_smbus_write_byte_data(client, 0x0E, 0x03); 		// sample delay
	if( err < 0 ) return err;
	err = i2c_smbus_write_byte_data(client, 0x0F, 0x17);		// LED current
	if( err < 0 ) return err;
	mutex_unlock(&data->update_lock);

#if 0
	helix_set_pilt(client, 1000);
	if( err < 0 ) return err;
	helix_set_piht(client, 3000);
	if( err < 0 ) return err;
#endif

	mutex_lock(&data->update_lock);
	err = i2c_smbus_write_byte_data(client, 0x14, 0x11);		// persistence		
	if( err < 0 ) return err;
	err = i2c_smbus_write_byte_data(client, 0x15, 0x0F);		// PS config
	if( err < 0 ) return err;
	err = i2c_smbus_write_byte_data(client, 0x16, 0x04);		// PS Interval		????
	if( err < 0 ) return err;
	err = i2c_smbus_write_byte_data(client, 0x25, 0xC5);		// ALS_Config	100ms integration time, 10cnt per lux
	if( err < 0 ) return err;
	err = i2c_smbus_write_byte_data(client, 0x26, 0x42);		// ALS_Interval
	if( err < 0 ) return err;
	mutex_unlock(&data->update_lock);

#ifdef HELIX_USE_PS_CALIBRATION
//	helix_run_crosstalk_calibration(client);
#endif

#endif

	return 0;
}

static int32_t helix_check_pid(struct helix_data *client)
{
#if 0
	int32_t err1, err2;

	err1 = i2c_smbus_read_byte_data(client->client,Helix_PDT_ID_REG);
	if (err1 < 0)
	{
		printk(KERN_ERR "%s: read i2c error, err=%d\n", __func__, err1);
		return err1;
	}

    err2 = i2c_smbus_read_byte_data(client->client,Helix_RSRVD_REG);
    if (err2 < 0)
    {
        printk(KERN_ERR "%s: read i2c error, err=%d\n", __func__, err2);
        return -1;
    }
	if(err2 == 0xC0)
		printk(KERN_INFO "%s: RID=0xC0!!!!!!!!!!!!!\n", __func__);
#endif
	return 0;
}


static int32_t helix_software_reset(struct i2c_client *client)
{

#if 0
    int32_t r;
    uint8_t w_reg;

    w_reg = 0x7F;
    r = i2c_smbus_write_byte_data(client->client,Helix_WAIT_REG,w_reg);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: write i2c error, ret=%d\n", __func__, r);
        return r;
    }
    r = i2c_smbus_read_byte_data(client->client,Helix_WAIT_REG);
    if (w_reg != r)
    {
        printk(KERN_ERR "%s: software reset: read-back value is not the same\n", __func__);
        return -1;
    }

    r = i2c_smbus_write_byte_data(client->client,Helix_SW_RESET_REG,0);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
        return r;
    }
    msleep(1);
#else
	int err;
	struct helix_data *data = i2c_get_clientdata(client);

// 	printk("[Helix] %s\n", __func__);

	mutex_lock(&data->update_lock);
	err = i2c_smbus_write_byte_data(client, 0x01, 0x01);		// Reset
	mutex_unlock(&data->update_lock);
	if( err < 0 ) 
		return err;

	mdelay(2);
#endif
    return 0;
}

#if 0
static int32_t helix_set_als_thd_l(struct i2c_client *client, uint16_t thd_l)
{
#if 0
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_l;
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    return i2c_smbus_write_word_data(client->client,Helix_THDL1_ALS_REG,thd_l);
#endif
}
static int32_t helix_set_als_thd_h(struct i2c_client *client, uint16_t thd_h)
{
#if 0
	uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_h;
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    return i2c_smbus_write_word_data(client->client,Helix_THDH1_ALS_REG,thd_h);
#endif
}
#endif

static int32_t helix_set_ps_thd_l(struct i2c_client *client, uint16_t thdLow)
{
#if 0
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_l;

    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    client->ps_thd_l = thd_l;
	return i2c_smbus_write_word_data(client->client,Helix_THDL1_PS_REG,thd_l);
#else
	struct helix_data *data = i2c_get_clientdata(client);
	int ret;
	unsigned char upper, lower;

// 	printk("[Helix] %s\n", __func__);
	lower = 0x00FF & thdLow;
	upper = (0xFF00 & thdLow)>>8;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, 0x13, lower);
	ret = i2c_smbus_write_byte_data(client, 0x12, upper);
	mutex_unlock(&data->update_lock);

//	data->ps_thd_l = thdLow;

	return ret;
#endif
}

static int32_t helix_set_ps_thd_h(struct i2c_client *client, uint16_t thdHigh)
{
#if 0
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_h;

    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    client->ps_thd_h = thd_h;
	return i2c_smbus_write_word_data(client->client,Helix_THDH1_PS_REG,thd_h);
#else
	struct helix_data *data = i2c_get_clientdata(client);
	int ret;
	unsigned char upper, lower;

// 	printk("[Helix] %s\n", __func__);

	lower = 0x00FF & thdHigh;
	upper = (0xFF00 & thdHigh)>>8;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, 0x11, lower);
	ret = i2c_smbus_write_byte_data(client, 0x10, upper);
	mutex_unlock(&data->update_lock);

//	data->ps_thd_h = thdHigh;

	return ret;
#endif
}

static inline uint32_t helix_get_ps_reading(struct helix_data *client)
{
#if 0
	int32_t word_data, tmp_word_data;

	tmp_word_data = i2c_smbus_read_word_data(client->client,Helix_DATA1_PS_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	return word_data;
#else

	struct helix_data *data = i2c_get_clientdata(client->client);
	unsigned int readValue;
	unsigned char upper, lower;

	mutex_lock(&data->update_lock);
	upper = i2c_smbus_read_byte_data(client->client, 0x41);
	lower = i2c_smbus_read_byte_data(client->client, 0x42);
	mutex_unlock(&data->update_lock);
	readValue = (upper<<8)|lower;

	return readValue;

#endif
}

#if 0
static int32_t helix_set_flag(struct helix_data *client, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	w_flag = org_flag_reg | (Helix_FLG_ALSINT_MASK | Helix_FLG_PSINT_MASK | Helix_FLG_OUI_MASK | Helix_FLG_IR_RDY_MASK);
	w_flag &= (~clr);
	//printk(KERN_INFO "%s: org_flag_reg=0x%x, w_flag = 0x%x\n", __func__, org_flag_reg, w_flag);
    return i2c_smbus_write_byte_data(client->client,Helix_FLAG_REG, w_flag);
}

static int32_t helix_get_flag(struct helix_data *client)
{
    return i2c_smbus_read_byte_data(client->client,Helix_FLAG_REG);
}
#endif

static int32_t helix_enable_ps(struct i2c_client *client, uint8_t enable)
{
    int32_t ret;


	struct helix_data *data = i2c_get_clientdata(client);

// 	printk("[Helix] %s\n", __func__);
	if(data->ps_enabled == enable)
		return 0;
	else 
		data->ps_enabled = enable;

    ret = helix_device_ctl(data, enable);
	if (ret)
		return ret;


	if(enable)
	{

		/* Power on and initalize the device */
		//turn on p sensor

		mutex_lock(&data->update_lock); 
		i2c_smbus_write_byte_data(client, 0x0D, 23);		// LED frequency
		i2c_smbus_write_byte_data(client, 0x0E, 3); 		// sample delay
		i2c_smbus_write_byte_data(client, 0x0F, 0x17);		// LED current
		mutex_unlock(&data->update_lock);	
		
		helix_set_ps_thd_l(client, data->ps_thd_l);
		helix_set_ps_thd_h(client, data->ps_thd_h);
		
		mutex_lock(&data->update_lock); 
		i2c_smbus_write_byte_data(client, 0x14, 0x11);		// persistence		
		i2c_smbus_write_byte_data(client, 0x15, 0x0F);		// PS config
		i2c_smbus_write_byte_data(client, 0x16, 0x04);		// PS Interval		????
		i2c_smbus_write_byte_data(client, 0x17, 0x02);		// PS Repeat mode		????
		mutex_unlock(&data->update_lock);

		enable_irq(data->irq);
		msleep(1);

	}
	else
	{

		mutex_lock(&data->update_lock);
		i2c_smbus_write_byte_data(client, 0x17, 0x00);		// PS Repeat mode		????
		mutex_unlock(&data->update_lock);

		disable_irq(data->irq);
	}
	

	return ret;
}

static int32_t helix_enable_als(struct helix_data *client, uint8_t enable)
{
    int32_t ret;
#if 0
	uint8_t w_state_reg;
	uint8_t curr_als_enable = (client->als_enabled)?1:0;

	if(curr_als_enable == enable)
		return 0;

	if (enable) {
		ret = helix_device_ctl(client, enable);
		if (ret)
			return ret;
	}
 
    ret = i2c_smbus_read_byte_data(client->client, Helix_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }
	w_state_reg = (uint8_t)(ret & (~(Helix_STATE_EN_ALS_MASK | Helix_STATE_EN_WAIT_MASK)));
	if(enable)
		w_state_reg |= Helix_STATE_EN_ALS_MASK;
	else if (client->ps_enabled)
		w_state_reg |= Helix_STATE_EN_WAIT_MASK;

    ret = i2c_smbus_write_byte_data(client->client, Helix_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

    if (enable)
    {
		client->als_enabled = true;
#ifdef Helix_POLL_ALS
		hrtimer_start(&client->als_timer, client->als_poll_delay, HRTIMER_MODE_REL);
#else
#ifndef Helix_POLL_PS
		if(!(client->ps_enabled))
#endif
			enable_irq(client->irq);
#endif
    }
	else
	{
		client->als_enabled = false;
#ifdef Helix_POLL_ALS
		hrtimer_cancel(&client->als_timer);
#else
#ifndef Helix_POLL_PS
		if(!(client->ps_enabled))
#endif
			disable_irq(client->irq);
#endif
	}



#else
//	uint8_t curr_als_enable = (client->als_enabled)?1:0;

//	printk("[Helix] %s\n", __func__);

//	if(curr_als_enable == enable)
//		return 0;
#if 0
	if (enable) {
		ret = helix_device_ctl(client, enable);
		if (ret)
			return ret;
	}
#endif
	if(client->als_enabled == enable)
		return 0;
	else
    	client->als_enabled = enable;

    ret = helix_device_ctl(client, enable);
	if (ret)
		return ret;
        
    if (enable){	
		mutex_lock(&client->update_lock);
		i2c_smbus_write_byte_data(client->client, 0x25, 0xC5); 	 // ALS_Config	 100ms integration time, 10cnt per lux
		i2c_smbus_write_byte_data(client->client, 0x26, 0x42); 	 // ALS_Interval
		i2c_smbus_write_byte_data(client->client, 0x27, 0x02); 	 // ALS_Repeat mode
		mutex_unlock(&client->update_lock);
		hrtimer_start(&client->als_timer, client->als_poll_delay, HRTIMER_MODE_REL);
    }else{
		mutex_lock(&client->update_lock);
 		i2c_smbus_write_byte_data(client->client, 0x27, 0x00); 	 // ALS_Repeat mode
		mutex_unlock(&client->update_lock);
 		hrtimer_cancel(&client->als_timer);
	}

#endif


    return ret;
}

#if 0
static inline int32_t helix_filter_reading(struct helix_data *client,
			int32_t word_data)
{
	int index;
	int firlen = atomic_read(&client->firlength);

	if (client->fir.number < firlen) {
		client->fir.raw[client->fir.number] = word_data;
		client->fir.sum += word_data;
		client->fir.number++;
		client->fir.idx++;
	} else {
		index = client->fir.idx % firlen;
		client->fir.sum -= client->fir.raw[index];
		client->fir.raw[index] = word_data;
		client->fir.sum += word_data;
		client->fir.idx++;
		word_data = client->fir.sum/firlen;
	}
	return word_data;
}
#endif

static inline int32_t helix_get_als_reading(struct helix_data *client)
{
#if 0
    int32_t word_data, tmp_word_data;

	tmp_word_data = i2c_smbus_read_word_data(client->client, Helix_DATA1_ALS_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	if (client->use_fir)
		word_data = helix_filter_reading(client, word_data);

	return word_data;
#else

//	printk("[Helix] %s\n", __func__);

	return helix_read_alsch0(client->client);


#endif
}

#if 0
static int32_t helix_get_ir_reading(struct helix_data *client)
{
    int32_t word_data, tmp_word_data;
	int32_t ret;
	uint8_t w_reg, retry = 0;

	if(client->ps_enabled)
	{
		helix_enable_ps(client, 0);
		client->ps_enabled = true;
	}
    ret = i2c_smbus_read_byte_data(client->client, Helix_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }
	w_reg = (uint8_t)(ret & (~Helix_STATE_EN_IRS_MASK));
	w_reg |= Helix_STATE_EN_IRS_MASK;

    ret = i2c_smbus_write_byte_data(client->client, Helix_STATE_REG, w_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	msleep(100);

	do
	{
		msleep(50);
		ret = helix_get_flag(client);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}
		retry++;
	}while(retry < 5 && ((ret&Helix_FLG_IR_RDY_MASK) == 0));

	if(retry == 5)
	{
		printk(KERN_ERR "%s: ir data is not ready for 300ms\n", __func__);
		return -EINVAL;
	}

	ret = helix_get_flag(client);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }

	ret = helix_set_flag(client, ret, Helix_FLG_IR_RDY_MASK);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	tmp_word_data = i2c_smbus_read_word_data(client->client, Helix_DATA1_IR_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;

	if(client->ps_enabled)
		helix_enable_ps(client, 1);
	return word_data;
}
#endif


static ssize_t Helix_als_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
    int32_t reading;

    reading = helix_get_als_reading(client);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static int Helix_als_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct helix_data *als_data = container_of(sensors_cdev,
						struct helix_data, als_cdev);
	int err;

	err = helix_enable_als(als_data, enabled);

	if (err < 0)
		return err;
	return 0;
}

static ssize_t Helix_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
    int32_t enable, ret;

    mutex_lock(&client->io_lock);
	enable = (client->als_enabled)?1:0;
    mutex_unlock(&client->io_lock);
    ret = i2c_smbus_read_byte_data(client->client,0x27);
    ret = (ret & 0x02)?1:0;

	if(enable != ret)
		printk(KERN_ERR "%s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t Helix_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client = dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: Enable ALS : %d\n", __func__, en);
	
    helix_enable_als(client, en);
	
    return size;
}

static ssize_t Helix_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client = dev_get_drvdata(dev);
    int32_t als_reading;
	uint32_t als_lux;
    als_reading = helix_get_als_reading(client);

#if 0
	mutex_lock(&client->io_lock);
	als_lux = Helix_alscode2lux(client, als_reading);
	mutex_unlock(&client->io_lock);
#else
	als_lux = LuxCalculation(client);
#endif
    return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t Helix_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&client->io_lock);
    client->als_lux_last = value;
	input_report_abs(client->als_input_dev, ABS_MISC, value);
	input_sync(client->als_input_dev);
	mutex_unlock(&client->io_lock);
	dev_dbg(dev, "%s: als input event %ld lux\n", __func__, value);

    return size;
}


static ssize_t Helix_als_transmittance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
    int32_t transmittance;
    mutex_lock(&client->io_lock);
    transmittance = client->als_transmittance;
    mutex_unlock(&client->io_lock);
    return scnprintf(buf, PAGE_SIZE, "%d\n", transmittance);
}


static ssize_t Helix_als_transmittance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	mutex_lock(&client->io_lock);
    client->als_transmittance = value;
    mutex_unlock(&client->io_lock);
    return size;
}

static ssize_t Helix_als_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			(u32)ktime_to_ms(client->als_poll_delay));
}

static inline void Helix_als_delay_store_fir(struct helix_data *client)
{
	client->fir.number = 0;
	client->fir.idx = 0;
	client->fir.sum = 0;
}

static int Helix_als_poll_delay_set(struct sensors_classdev *sensors_cdev,
						unsigned int delay_msec)
{
	struct helix_data *als_data = container_of(sensors_cdev,
						struct helix_data, als_cdev);
	uint64_t value = 0;

//	printk("[Helix] %s\n", __func__);

	value = delay_msec * 1000000;

	if (value < MIN_ALS_POLL_DELAY_NS)
		value = MIN_ALS_POLL_DELAY_NS;

	mutex_lock(&als_data->io_lock);
	if (value != ktime_to_ns(als_data->als_poll_delay))
		als_data->als_poll_delay = ns_to_ktime(value);

	if (als_data->use_fir)
		Helix_als_delay_store_fir(als_data);

	mutex_unlock(&als_data->io_lock);

	return 0;
}

static ssize_t Helix_als_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint64_t value = 0;
	int ret;
	struct helix_data *als_data =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if(ret < 0)
	{
		dev_err(dev, "%s:kstrtoull failed, ret=0x%x\n",	__func__, ret);
		return ret;
	}
#ifdef Helix_DEBUG_PRINTF
	dev_dbg(dev, "%s: set als poll delay=%lld\n", __func__, value);
#endif
	ret = Helix_als_poll_delay_set(&als_data->als_cdev, value);
	if (ret < 0)
		return ret;
	return size;
}

#if 0
static ssize_t Helix_als_ir_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
    int32_t reading;
    reading = helix_get_ir_reading(client);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}
#endif

static ssize_t Helix_als_firlen_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	int len = atomic_read(&client->firlength);

	dev_dbg(dev, "%s: len = %2d, idx = %2d\n",
			__func__, len, client->fir.idx);
	dev_dbg(dev, "%s: sum = %5d, ave = %5d\n",
			__func__, client->fir.sum, client->fir.sum/len);

	return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}

static ssize_t Helix_als_firlen_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	uint64_t value = 0;
	int ret;
	struct helix_data *client =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s:strict_strtoull failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}

	if (value > MAX_FIR_LEN) {
		dev_err(dev, "%s: firlen exceed maximum filter length\n",
			__func__);
	} else if (value < 1) {
		atomic_set(&client->firlength, 1);
		memset(&client->fir, 0x00, sizeof(client->fir));
	} else {
		atomic_set(&client->firlength, value);
		memset(&client->fir, 0x00, sizeof(client->fir));
	}
	return size;
}

static ssize_t Helix_als_fir_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", client->use_fir);
}

static ssize_t Helix_als_fir_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	uint64_t value = 0;
	int ret;
	struct helix_data *client =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s:strict_strtoull failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}

	if (value) {
		client->use_fir = true;
		memset(&client->fir, 0x00, sizeof(client->fir));
	} else {
		client->use_fir = false;
	}
	return size;
}

static ssize_t Helix_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
    uint32_t reading;
    reading = helix_get_ps_reading(client);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static ssize_t Helix_ps_calculation_def_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *data =  dev_get_drvdata(dev);
    unsigned int reading;
#ifdef HELIX_USE_PS_CALIBRATION
	if(data->ps_enabled){
		reading = helix_run_crosstalk_calibration(data->client);
	}else{
		helix_enable_ps(data->client,1);
		reading = helix_run_crosstalk_calibration(data->client);
		helix_enable_ps(data->client,0);
	}
#endif
	if(!(data->ps_cal_is_support)){
		reading = DEFAULT_CROSS_TALK; 
		pr_err("%s(%d): ps is not support cal  def_cal=%d  \n",__func__,__LINE__,reading);
	}
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static int Helix_ps_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct helix_data *client = container_of(sensors_cdev,
						struct helix_data, ps_cdev);
	int err;

//	printk("[Helix]---cbw---   %s in  enabled=%d \n", __func__,enabled);

	mutex_lock(&client->io_lock);
	err = helix_enable_ps(client->client, enabled);
	mutex_unlock(&client->io_lock);

	if (err < 0)
		return err;
	return 0;
}

static ssize_t Helix_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t enable, ret;
	struct helix_data *client =  dev_get_drvdata(dev);

    mutex_lock(&client->io_lock);
	enable = (client->ps_enabled)?1:0;
    mutex_unlock(&client->io_lock);
    ret = i2c_smbus_read_byte_data(client->client,0x17);
    ret = (ret > 0)?1:0;

	if(enable != ret)
		printk(KERN_ERR "%s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t Helix_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_info(dev, "%s: Enable PS : %d\n", __func__, en);
    mutex_lock(&client->io_lock);
    helix_enable_ps(client->client, en);
    mutex_unlock(&client->io_lock);
    return size;
}

#if 0
static ssize_t Helix_ps_enable_aso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
	struct helix_data *client =  dev_get_drvdata(dev);

    ret = i2c_smbus_read_byte_data(client->client,Helix_STATE_REG);
    ret = (ret & Helix_STATE_EN_ASO_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t Helix_ps_enable_aso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	uint8_t en;
    int32_t ret;
	uint8_t w_state_reg;

	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: Enable PS ASO : %d\n", __func__, en);

    ret = i2c_smbus_read_byte_data(client->client, Helix_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }
	w_state_reg = (uint8_t)(ret & (~Helix_STATE_EN_ASO_MASK));
	if(en)
		w_state_reg |= Helix_STATE_EN_ASO_MASK;

    ret = i2c_smbus_write_byte_data(client->client, Helix_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t Helix_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct helix_data *client =  dev_get_drvdata(dev);
    int32_t word_data, tmp_word_data;

	tmp_word_data = i2c_smbus_read_word_data(client->client, Helix_DATA1_OFFSET_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
		word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}

static ssize_t Helix_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	uint16_t offset;

	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	if(value > 65535)
	{
		printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, value);
		return -EINVAL;
	}

	offset = (uint16_t) ((value&0x00FF) << 8) | ((value&0xFF00) >>8);
	ret = i2c_smbus_write_word_data(client->client,Helix_DATA1_OFFSET_REG,offset);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	return size;
}
#endif


static ssize_t Helix_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
	struct helix_data *client =  dev_get_drvdata(dev);
    int32_t dist=1, ret;

    mutex_lock(&client->io_lock);
    ret = helix_get_flag(client);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: helix_get_flag failed, ret=0x%x\n", __func__, ret);
		return ret;
	}
    dist = (ret & Helix_FLG_NF_MASK)?1:0;

    client->ps_distance_last = dist;
	input_report_abs(client->ps_input_dev, ABS_DISTANCE, dist);
	input_sync(client->ps_input_dev);
    mutex_unlock(&client->io_lock);
	wake_lock_timeout(&client->ps_wakelock, 3*HZ);
	dev_dbg(dev, "%s: ps input event %d cm\n", __func__, dist);
    return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
#else
	struct helix_data *client =	dev_get_drvdata(dev);

    mutex_lock(&client->io_lock);
	input_report_abs(client->ps_input_dev, ABS_DISTANCE, client->ps_distance_last);
	input_sync(client->ps_input_dev);
	mutex_unlock(&client->io_lock);
	wake_lock_timeout(&client->ps_wakelock, 3*HZ);
//	dev_dbg(dev, "%s: ps input event %d cm\n", __func__, dist);
	return scnprintf(buf, PAGE_SIZE, "%d\n", client->ps_distance_last);
#endif
}


static ssize_t Helix_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&client->io_lock);
    client->ps_distance_last = value;
	input_report_abs(client->ps_input_dev, ABS_DISTANCE, value);
	input_sync(client->ps_input_dev);
    mutex_unlock(&client->io_lock);
	wake_lock_timeout(&client->ps_wakelock, 3*HZ);
	dev_dbg(dev, "%s: ps input event %ld cm\n", __func__, value);
    return size;
}


static ssize_t Helix_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
    int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct helix_data *client =  dev_get_drvdata(dev);
    mutex_lock(&client->io_lock);
    ps_thd_l1_reg = i2c_smbus_read_byte_data(client->client,Helix_THDL1_PS_REG);
    if(ps_thd_l1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l1_reg);
		return -EINVAL;
	}
    ps_thd_l2_reg = i2c_smbus_read_byte_data(client->client,Helix_THDL2_PS_REG);
    if(ps_thd_l2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l2_reg);
		return -EINVAL;
	}
    mutex_unlock(&client->io_lock);
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
#else

	int32_t msb, lsb, lowerThd;
	struct helix_data *client =	dev_get_drvdata(dev);
	mutex_lock(&client->io_lock);
	msb = i2c_smbus_read_byte_data(client->client,0x12);
	if(msb < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, msb);
		return -EINVAL;
	}
	lsb = i2c_smbus_read_byte_data(client->client,0x13);
	if(lsb < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, lsb);
		return -EINVAL;
	}
	mutex_unlock(&client->io_lock);
	
	lowerThd = msb<<8 | lsb;
	return scnprintf(buf, PAGE_SIZE, "%d\n", lowerThd);

#endif
}


static ssize_t Helix_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&client->io_lock);
    helix_set_ps_thd_l(client->client, value);
    mutex_unlock(&client->io_lock);
    return size;
}

static ssize_t Helix_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
    int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct helix_data *client =  dev_get_drvdata(dev);
    mutex_lock(&client->io_lock);
    ps_thd_h1_reg = i2c_smbus_read_byte_data(client->client,Helix_THDH1_PS_REG);
    if(ps_thd_h1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h1_reg);
		return -EINVAL;
	}
    ps_thd_h2_reg = i2c_smbus_read_byte_data(client->client,Helix_THDH2_PS_REG);
    if(ps_thd_h2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h2_reg);
		return -EINVAL;
	}
    mutex_unlock(&client->io_lock);
	ps_thd_h1_reg = ps_thd_h1_reg<<8 | ps_thd_h2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h1_reg);
#else

	int32_t msb, lsb, upperThd;
	struct helix_data *client =	dev_get_drvdata(dev);
	mutex_lock(&client->io_lock);
	msb = i2c_smbus_read_byte_data(client->client,0x10);
	if(msb < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, msb);
		return -EINVAL;
	}
	lsb = i2c_smbus_read_byte_data(client->client,0x11);
	if(lsb < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, lsb);
		return -EINVAL;
	}
	mutex_unlock(&client->io_lock);

	upperThd = msb<<8 | lsb;
	return scnprintf(buf, PAGE_SIZE, "%d\n", upperThd);

#endif
}


static ssize_t Helix_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct helix_data *client =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&client->io_lock);
    helix_set_ps_thd_h(client->client, value);
    mutex_unlock(&client->io_lock);
    return size;
}

static ssize_t Helix_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
    int32_t ps_reg[27];
	uint8_t cnt;
	struct helix_data *client =  dev_get_drvdata(dev);
    mutex_lock(&client->io_lock);
	for(cnt=0;cnt<25;cnt++)
	{
		ps_reg[cnt] = i2c_smbus_read_byte_data(client->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			mutex_unlock(&client->io_lock);
			printk(KERN_ERR "Helix_all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
			return -EINVAL;
		}
		else
		{
			dev_dbg(dev, "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = i2c_smbus_read_byte_data(client->client, Helix_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		mutex_unlock(&client->io_lock);
		printk( KERN_ERR "all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	dev_dbg(dev, "reg[0x%x]=0x%2X\n", Helix_PDT_ID_REG, ps_reg[cnt]);
	cnt++;
	ps_reg[cnt] = i2c_smbus_read_byte_data(client->client, Helix_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		mutex_unlock(&client->io_lock);
		printk( KERN_ERR "all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	dev_dbg(dev, "reg[0x%x]=0x%2X\n", Helix_RSRVD_REG, ps_reg[cnt]);
    mutex_unlock(&client->io_lock);

    return scnprintf(buf, PAGE_SIZE, "%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X\n",
		ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8],
		ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17],
		ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);

#else
#if 0
	int32_t ps_reg[4];
	uint8_t cnt, i=0;
	struct helix_data *client =	dev_get_drvdata(dev);
	mutex_lock(&client->io_lock);
	ps_reg[0] = i2c_smbus_read_byte_data(client->client, 0x00);
	i++;
	for(cnt = 0x40; cnt < 0x43; cnt++) {
		ps_reg[i] = i2c_smbus_read_byte_data(client->client, (cnt));
		i++;
	}
	mutex_unlock(&client->io_lock);

	return scnprintf(buf, PAGE_SIZE, "0x%2X 0x%2X 0x%2X 0x%2X\n", ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3]);
#endif
#endif
	struct helix_data *client = dev_get_drvdata(dev);
	int i=0, count=0;

	uint8_t buffer[71];

	for(i=0; i<0x47; i++)
	{
		buffer[i] = i2c_smbus_read_byte_data(client->client, (i));

		count += snprintf(&buf[count], PAGE_SIZE, "0x%2x: 0x%2x\n",
			i, buffer[i]);
	}
	count +=snprintf(&buf[count],PAGE_SIZE,"crosstalk=%d,black_hair=%d,ps_thd_h=%d,ps_thd_l=%d\n",client->crosstalk,client->black_hair,client->ps_thd_h,client->ps_thd_l);
	pr_err("%s(%d):  crosstalk = %d,black_hair=%d,ps_thd_h=%d,ps_thd_l=%d\n",	__func__,__LINE__, client->crosstalk,client->black_hair,client->ps_thd_h,client->ps_thd_l);
  
	return count;

}

static ssize_t Helix_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t Helix_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
	int ret;
	int32_t recv_data;
	struct helix_data *client =  dev_get_drvdata(dev);

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	recv_data = i2c_smbus_read_byte_data(client->client,value);
	printk("%s: reg 0x%x=0x%x\n", __func__, (int)value, recv_data);
	return size;
}


static ssize_t Helix_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t Helix_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	u8 addr_u8, cmd_u8;
	int32_t ret, i;
	char *token[10];
	struct helix_data *client =  dev_get_drvdata(dev);

	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	ret = kstrtoul(token[0], 16, (unsigned long *)&(addr));
	if (ret < 0) {

		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	ret = kstrtoul(token[1], 16, (unsigned long *)&(cmd));
	if (ret < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	dev_dbg(dev, "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);
	addr_u8 = (u8) addr;
	cmd_u8 = (u8) cmd;
	//mutex_lock(&client->io_lock);
	ret = i2c_smbus_write_byte_data(client->client,addr_u8,cmd_u8);
	//mutex_unlock(&client->io_lock);
	if (0 != ret)
	{
		printk(KERN_ERR "%s: i2c_smbus_write_byte_data fail\n", __func__);
		return ret;
	}

	return size;
}

static struct device_attribute als_enable_attribute = __ATTR(enable,0664,Helix_als_enable_show,Helix_als_enable_store);
static struct device_attribute als_lux_attribute = __ATTR(lux,0664,Helix_als_lux_show,Helix_als_lux_store);
static struct device_attribute als_code_attribute = __ATTR(code, 0444, Helix_als_code_show, NULL);
static struct device_attribute als_transmittance_attribute = __ATTR(transmittance,0664,Helix_als_transmittance_show,Helix_als_transmittance_store);
static struct device_attribute als_poll_delay_attribute =__ATTR(poll_delay, 0664, Helix_als_delay_show, Helix_als_delay_store);
//static struct device_attribute als_ir_code_attribute = __ATTR(ircode,0444,Helix_als_ir_code_show,NULL);
static struct device_attribute als_firlen_attribute =__ATTR(firlen, 0664, Helix_als_firlen_show, Helix_als_firlen_store);
static struct device_attribute als_fir_enable_attribute =__ATTR(fir_enable, 0664, Helix_als_fir_enable_show,Helix_als_fir_enable_store);

static struct attribute *Helix_als_attrs [] =
{
	&als_enable_attribute.attr,
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
	&als_poll_delay_attribute.attr,
//	&als_ir_code_attribute.attr,
	&als_firlen_attribute.attr,
	&als_fir_enable_attribute.attr,
    NULL
};

static struct attribute_group Helix_als_attribute_group = {
	.attrs = Helix_als_attrs,
};


static struct device_attribute ps_enable_attribute = __ATTR(enable,0664,Helix_ps_enable_show,Helix_ps_enable_store);
//static struct device_attribute ps_enable_aso_attribute = __ATTR(enableaso,0664,Helix_ps_enable_aso_show,Helix_ps_enable_aso_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0664,Helix_ps_distance_show, Helix_ps_distance_store);
//static struct device_attribute ps_offset_attribute = __ATTR(offset,0664,Helix_ps_offset_show, Helix_ps_offset_store);
static struct device_attribute ps_calculation_def_attribute = __ATTR(calculation_def,0664,Helix_ps_calculation_def_show,NULL);
static struct device_attribute ps_code_attribute = __ATTR(code, 0444, Helix_ps_code_show, NULL);
static struct device_attribute ps_code_thd_l_attribute = __ATTR(codethdl,0664,Helix_ps_code_thd_l_show,Helix_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(codethdh,0664,Helix_ps_code_thd_h_show,Helix_ps_code_thd_h_store);
static struct device_attribute recv_attribute = __ATTR(recv,0664,Helix_recv_show,Helix_recv_store);
static struct device_attribute send_attribute = __ATTR(send,0664,Helix_send_show, Helix_send_store);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0444, Helix_all_reg_show, NULL);

static struct attribute *Helix_ps_attrs [] =
{
    &ps_enable_attribute.attr,
//    &ps_enable_aso_attribute.attr,
    &ps_distance_attribute.attr,
//	&ps_offset_attribute.attr,
	&ps_calculation_def_attribute.attr,
    &ps_code_attribute.attr,
	&ps_code_thd_l_attribute.attr,
	&ps_code_thd_h_attribute.attr,
	&recv_attribute.attr,
	&send_attribute.attr,
	&all_reg_attribute.attr,
    NULL
};

static struct attribute_group Helix_ps_attribute_group = {
	.attrs = Helix_ps_attrs,
};

#ifdef Helix_POLL_ALS
static enum hrtimer_restart Helix_als_timer_func(struct hrtimer *timer)
{
	struct helix_data *client = container_of(timer, struct helix_data, als_timer);

//	printk("[Helix] %s\n", __func__);
	
	queue_work(client->Helix_als_wq, &client->Helix_als_work);
	hrtimer_forward_now(&client->als_timer, client->als_poll_delay);
	return HRTIMER_RESTART;
}

static void Helix_als_work_func(struct work_struct *work)
{
	struct helix_data *client = container_of(work, struct helix_data, Helix_als_work);
	int32_t reading;

    mutex_lock(&client->io_lock);
	reading = helix_get_als_reading(client);

//	printk("[Helix] %s\n", __func__);
	
	if(reading < 0)
		return;
#if 0
	client->als_lux_last = Helix_alscode2lux(client, reading);
#else
	client->als_lux_last = LuxCalculation(client);
#endif
	input_report_abs(client->als_input_dev, ABS_MISC, client->als_lux_last);
	input_sync(client->als_input_dev);
	mutex_unlock(&client->io_lock);
}
#endif
#if 0
static enum hrtimer_restart Helix_ps_timer_func(struct hrtimer *timer)
{
	struct helix_data *client = container_of(timer, struct helix_data, ps_timer);

//	printk("[Helix] %s\n", __func__);
	
	queue_work(client->Helix_ps_wq, &client->Helix_ps_work);
#ifdef Helix_POLL_PS
	hrtimer_forward_now(&client->ps_timer, client->ps_poll_delay);
	return HRTIMER_RESTART;
#else
	hrtimer_cancel(&client->ps_timer);
	return HRTIMER_NORESTART;
#endif
}
#endif
#if 0
static void Helix_ps_work_func(struct work_struct *work)
{
#if 0
	struct helix_data *client = container_of(work, struct helix_data, Helix_ps_work);
	uint32_t reading;
	int32_t near_far_state;
    uint8_t org_flag_reg;
	int32_t ret;
    uint8_t disable_flag = 0;
    mutex_lock(&client->io_lock);

	org_flag_reg = helix_get_flag(client);
	if(org_flag_reg < 0)
	{
		printk(KERN_ERR "%s: get_status_reg fail, ret=%d", __func__, org_flag_reg);
		goto err_i2c_rw;
	}
	near_far_state = (org_flag_reg & Helix_FLG_NF_MASK)?1:0;
	reading = helix_get_ps_reading(client);
	if(client->ps_distance_last != near_far_state)
	{
		client->ps_distance_last = near_far_state;
		input_report_abs(client->ps_input_dev, ABS_DISTANCE, near_far_state);
		input_sync(client->ps_input_dev);
		wake_lock_timeout(&client->ps_wakelock, 3*HZ);
#ifdef Helix_DEBUG_PRINTF
		printk(KERN_INFO "%s: ps input event %d cm, ps code = %d\n",__func__, near_far_state, reading);
#endif
	}
	ret = helix_set_flag(client, org_flag_reg, disable_flag);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:helix_set_flag fail, ret=%d\n", __func__, ret);
		goto err_i2c_rw;
	}

	mutex_unlock(&client->io_lock);
	return;

err_i2c_rw:
	mutex_unlock(&client->io_lock);
	msleep(30);
	return;
#else
	struct helix_data *client = container_of(work, struct helix_data, Helix_ps_work);
	uint32_t status;
	uint32_t near_far_state;

//	printk("[Helix] %s\n", __func__);
	mutex_lock(&client->io_lock);

	status = helix_check_clear_interrupt(client);
	if(status < 0)
	{
		printk(KERN_ERR "%s: get_status_reg fail, ret=%d", __func__, status);
		goto err_i2c_rw;
	}
	
	if(status == 0x11) {
		near_far_state = NEAR_TO_FAR;
		if(client->ps_distance_last != near_far_state){
			client->ps_distance_last = near_far_state;
			input_report_abs(client->ps_input_dev, ABS_DISTANCE, NEAR_TO_FAR);
			input_sync(client->ps_input_dev);
			wake_lock_timeout(&client->ps_wakelock, 3*HZ);
		}
	} else if(status == 0x12) {
		near_far_state = FAR_TO_NEAR;
		if(client->ps_distance_last != near_far_state){
			client->ps_distance_last = near_far_state;
			input_report_abs(client->ps_input_dev, ABS_DISTANCE, FAR_TO_NEAR);
			input_sync(client->ps_input_dev);
			wake_lock_timeout(&client->ps_wakelock, 3*HZ);
		}
	}

	mutex_unlock(&client->io_lock);
	return;

err_i2c_rw:
	mutex_unlock(&client->io_lock);
	msleep(30);
	return;
#endif
}

#endif

#if (!defined(Helix_POLL_PS) || !defined(Helix_POLL_ALS))
static void Helix_work_func(struct work_struct *work)
{
	
#if 0

	uint32_t reading;
    int32_t ret;
    uint8_t disable_flag = 0;
    uint8_t org_flag_reg;


	struct helix_data *client = container_of(work, struct helix_data, Helix_work);
	int32_t near_far_state;

	mutex_lock(&client->io_lock);


#if (Helix_INT_PS_MODE	== 0x03)
	near_far_state = gpio_get_value(client->int_pin);
#elif	(Helix_INT_PS_MODE	== 0x02)
	near_far_state = !(gpio_get_value(client->int_pin));
#endif


	org_flag_reg = helix_get_flag(client);
	if(org_flag_reg < 0)
	{
		printk(KERN_ERR "%s: get_status_reg fail, org_flag_reg=%d", __func__, org_flag_reg);
		goto err_i2c_rw;
	}

    if (org_flag_reg & Helix_FLG_ALSINT_MASK)
    {
		disable_flag |= Helix_FLG_ALSINT_MASK;
        reading = helix_get_als_reading(client);
		if(reading < 0)
		{
			printk(KERN_ERR "%s: helix_get_als_reading fail, ret=%d", __func__, reading);
			goto err_i2c_rw;
		}


		client->als_lux_last = Helix_alscode2lux(client, reading);

		input_report_abs(client->als_input_dev, ABS_MISC, client->als_lux_last);
		input_sync(client->als_input_dev);
    }

    if (org_flag_reg & Helix_FLG_PSINT_MASK)
    {
		disable_flag |= Helix_FLG_PSINT_MASK;
		near_far_state = (org_flag_reg & Helix_FLG_NF_MASK)?1:0;

		client->ps_distance_last = near_far_state;
		input_report_abs(client->ps_input_dev, ABS_DISTANCE, near_far_state);
		input_sync(client->ps_input_dev);
		wake_lock_timeout(&client->ps_wakelock, 3*HZ);
        reading = helix_get_ps_reading(client);

    }

    ret = helix_set_flag(client, org_flag_reg, disable_flag);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:reset_int_flag fail, ret=%d\n", __func__, ret);
		goto err_i2c_rw;
	}

	msleep(1);
    enable_irq(client->irq);
    mutex_unlock(&client->io_lock);
	return;

#else

	struct helix_data *client = container_of(work, struct helix_data, Helix_work);
	uint32_t status;
	uint32_t near_far_state;

	//printk("[Helix] %s\n", __func__);

	status = helix_check_clear_interrupt(client);
	if(status < 0)
	{
		printk(KERN_ERR "%s: get_status_reg fail, ret=%d", __func__, status);
		goto err_i2c_rw;
	}
//	printk("[Helix]---cbw---%d  %s status = %x(11 far ; 12 near )\n",__LINE__, __func__, status);

	if(status == 0x11) {
		near_far_state = NEAR_TO_FAR;
		if(client->ps_distance_last != near_far_state){
			client->ps_distance_last = near_far_state;
			input_report_abs(client->ps_input_dev, ABS_DISTANCE, NEAR_TO_FAR);
			input_sync(client->ps_input_dev);
			wake_lock_timeout(&client->ps_wakelock, 3*HZ);
		}
		helix_set_ps_thd_l(client->client, 0);
		helix_set_ps_thd_h(client->client, client->ps_thd_h);
	} else if(status == 0x12) {
		near_far_state = FAR_TO_NEAR;
		if(client->ps_distance_last != near_far_state){
			client->ps_distance_last = near_far_state;
			input_report_abs(client->ps_input_dev, ABS_DISTANCE, FAR_TO_NEAR);
			input_sync(client->ps_input_dev);
			wake_lock_timeout(&client->ps_wakelock, 3*HZ);
		}
		helix_set_ps_thd_l(client->client, client->ps_thd_l);
		helix_set_ps_thd_h(client->client, 65535);
	}

//	msleep(1);
//	printk("Helix enable irq\n");
	enable_irq(client->irq);

	return;
#endif


err_i2c_rw:
	msleep(30);
	enable_irq(client->irq);
	return;
}
#endif

#if (!defined(Helix_POLL_PS) || !defined(Helix_POLL_ALS))
static irqreturn_t Helix_oss_irq_handler(int irq, void *data)
{
	struct helix_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(pData->Helix_wq,&pData->Helix_work);
	return IRQ_HANDLED;
}
#endif	/*	#if (!defined(Helix_POLL_PS) || !defined(Helix_POLL_ALS))	*/

static inline void helix_init_fir(struct helix_data *client)
{
	memset(&client->fir, 0x00, sizeof(client->fir));
	atomic_set(&client->firlength, Helix_FIR_LEN);
}

static int32_t helix_init_all_setting(struct i2c_client *client, struct helix_platform_data *plat_data)
{
	int32_t ret;
	struct helix_data *data = i2c_get_clientdata(client);

//	printk("[Helix] %s\n", __func__);
	ret = helix_software_reset(client);
	if(ret < 0)
		return ret;

	helix_check_pid(data);
	if(ret < 0)
		return ret;

	ret = helix_init_all_reg(client, plat_data);
	if(ret < 0)
		return ret;
#ifndef CONFIG_Helix_PS_ALS_USE_CHANGE_THRESHOLD
	Helix_init_code_threshold_table(client);
#endif

	if (plat_data->use_fir)
		helix_init_fir(data);

    return 0;
}

#if (!defined(Helix_POLL_PS) || !defined(Helix_POLL_ALS))
static int helix_setup_irq(struct i2c_client *client)
{
	int irq, err = -EIO;
	struct helix_data *data = i2c_get_clientdata(client);

//	printk("[Helix] %s\n", __func__);

	if(gpio_is_valid(data->int_pin)) {
		err = gpio_request(data->int_pin,"Helix-int");
		if(err < 0){
			printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
			goto err_irq_gpio_req;
		}
		err = gpio_direction_input(data->int_pin);
		if(err < 0){
			printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
			goto err_request_any_context_irq;
		}
	}else{
		printk(KERN_INFO "%s: gpio_is_valid err \n",__func__);
	}
	irq = gpio_to_irq(data->int_pin);
//#ifdef Helix_DEBUG_PRINTF
	printk(KERN_INFO "%s: int pin #=%d, irq=%d\n",__func__, data->int_pin, irq);
//#endif
	if (irq <= 0){
		printk(KERN_ERR "irq number is not specified, irq # = %d, int pin=%d\n",irq, data->int_pin);
		goto err_request_any_context_irq;
	}
	data->irq = irq;
	err = request_any_context_irq(irq, Helix_oss_irq_handler, IRQF_TRIGGER_FALLING| IRQF_ONESHOT, DEVICE_NAME, data);

	if (err < 0){
		printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);
		goto err_request_any_context_irq;
	}
	disable_irq(irq);

	return 0;
err_request_any_context_irq:
	if(gpio_is_valid(data->int_pin))
		gpio_free(data->int_pin);
err_irq_gpio_req:
	return err;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void helix_early_suspend(struct early_suspend *h)
{
	struct helix_data *client = container_of(h, struct helix_data, Helix_early_suspend);
#ifndef Helix_POLL_PS
	int err;
#endif

    mutex_lock(&client->io_lock);
	if(client->als_enabled)
	{
		helix_enable_als(client, 0);
		client->als_enabled = true;
	}
	if(client->ps_enabled)
	{
#ifdef Helix_POLL_PS
		wake_lock(&client->ps_nosuspend_wl);
#else
		err = enable_irq_wake(client->irq);
		if (err)
			printk(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, client->irq, err);
#endif
	}
	mutex_unlock(&client->io_lock);
	return;
}

static void helix_late_resume(struct early_suspend *h)
{
	struct helix_data *client = container_of(h, struct helix_data, Helix_early_suspend);
#ifndef Helix_POLL_PS
	int err;
#endif

    mutex_lock(&client->io_lock);
	if(client->als_enabled)
		helix_enable_als(client, 1);

	if(client->ps_enabled)
	{
#ifdef Helix_POLL_PS
		wake_lock(&client->ps_nosuspend_wl);
#else
		err = disable_irq_wake(client->irq);
		if (err)
			printk(KERN_WARNING "%s: disable_irq_wake(%d) failed, err=(%d)\n", __func__, client->irq, err);
#endif
	}
	mutex_unlock(&client->io_lock);
	return;
}
#endif	//#ifdef CONFIG_HAS_EARLYSUSPEND

static int helix_power_ctl(struct helix_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			ret = regulator_enable(data->vdd);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator vdd enable failed ret=%d\n",
					ret);
			}
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "helix_power_ctl on=%d\n",on);
	} else if (on && !data->power_enabled) {

		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "helix_power_ctl on=%d\n",on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int helix_power_init(struct helix_data *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, Helix3X1X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, Helix3X1X_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					Helix3X1X_VDD_MIN_UV,
					Helix3X1X_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					Helix3X1X_VIO_MIN_UV,
					Helix3X1X_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, Helix3X1X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int helix_device_ctl(struct helix_data *client, bool enable)
{
	int ret;
	struct device *dev = &client->client->dev;

	if (enable && !client->power_enabled) {
		ret = helix_power_ctl(client, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		ret = helix_init_all_setting(client->client, client->pdata);
		if (ret < 0) {
			helix_power_ctl(client, false);
			dev_err(dev, "Failed to re-init device setting\n");
			goto err_exit;
		}
	} else if (!enable && client->power_enabled && !client->als_enabled && !client->ps_enabled ) {
		if (!client->als_enabled && !client->ps_enabled) {
			ret = helix_power_ctl(client, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
				client->als_enabled, client->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
			enable, client->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#ifdef CONFIG_OF
static int helix_parse_dt(struct device *dev,
			struct helix_platform_data *pdata)
{
	//int rc;
	struct device_node *np = dev->of_node;
	//u32 temp_val;

	pdata->int_pin = of_get_named_gpio_flags(np, "Helix,irq-gpio",
				0, &pdata->int_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}
#if 0
	rc = of_property_read_u32(np, "Helix,transmittance", &temp_val);
	if (!rc)
		pdata->transmittance = temp_val;
	else {
		dev_err(dev, "Unable to read transmittance\n");
		return rc;
	}

	rc = of_property_read_u32(np, "Helix,state-reg", &temp_val);
	if (!rc)
		pdata->state_reg = temp_val;
	else {
		dev_err(dev, "Unable to read state-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "Helix,psctrl-reg", &temp_val);
	if (!rc)
		pdata->psctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read psctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "Helix,alsctrl-reg", &temp_val);
	if (!rc)
		pdata->alsctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read alsctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "Helix,ledctrl-reg", &temp_val);
	if (!rc)
		pdata->ledctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read ledctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "Helix,wait-reg", &temp_val);
	if (!rc)
		pdata->wait_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read wait-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "Helix,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_thd_h = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "Helix,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_thd_l = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}
#endif
	pdata->use_fir = of_property_read_bool(np, "Helix,use-fir");

	return 0;
}
#else
static int helix_parse_dt(struct device *dev,
			struct helix_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

static int sensor_pinctrl_init(struct device *dev,
		struct pinctrl_config *config)
{
	config->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(config->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl\n");
		return PTR_ERR(config->pinctrl);
	}

	config->state[0] =
		pinctrl_lookup_state(config->pinctrl, config->name[0]);
	if (IS_ERR_OR_NULL(config->state[0])) {
		dev_err(dev, "Failed to look up %s\n", config->name[0]);
		return PTR_ERR(config->state[0]);
	}

	config->state[1] =
		pinctrl_lookup_state(config->pinctrl, config->name[1]);
	if (IS_ERR_OR_NULL(config->state[1])) {
		dev_err(dev, "Failed to look up %s\n", config->name[1]);
		return PTR_ERR(config->state[1]);
	}

	return 0;
}

static void helix_delay_work_func(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct helix_data *data;
	
	delay_work = to_delayed_work(work);
	data = container_of(delay_work, struct helix_data,delay_work_cal);
#ifdef HELIX_USE_PS_CALIBRATION
	if(data->ps_enabled){
		helix_run_crosstalk_calibration_delay(data->client);
	}else{
		helix_enable_ps(data->client,1);
		helix_run_crosstalk_calibration_delay(data->client);
		helix_enable_ps(data->client,0);
	}	
#endif
	if(!(data->ps_cal_is_support)){
		data->crosstalk = DEFAULT_CROSS_TALK; 
 		data->ps_thd_h = NEAR_OFFSET +  data->crosstalk;
 		data->ps_thd_l = FAR_OFFSET +  data->crosstalk;
		pr_err("%s(%d): ps is not support cal  def_cross=%d; l=%d; h=%d \n",__func__,__LINE__,data->crosstalk,data->ps_thd_l,data->ps_thd_h);
	}
}

static int helix_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int err = -ENODEV;
    struct helix_data *data;
	struct helix_platform_data *plat_data;
    printk(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_BYTE_DATA\n", __func__);
        return -ENODEV;
    }
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_WORD_DATA\n", __func__);
        return -ENODEV;
    }

	data = kzalloc(sizeof(struct helix_data),GFP_KERNEL);
	if(!data)
	{
		printk(KERN_ERR "%s: failed to allocate helix_data\n", __func__);
		return -ENOMEM;
	}

	data->client = client;

	gl_helix_data = data;
	i2c_set_clientdata(client,data);
	mutex_init(&data->io_lock);
	mutex_init(&data->update_lock);
	wake_lock_init(&data->ps_wakelock,WAKE_LOCK_SUSPEND, "Helix_input_wakelock");

#ifdef Helix_POLL_PS
	wake_lock_init(&data->ps_nosuspend_wl,WAKE_LOCK_SUSPEND, "Helix_nosuspend_wakelock");
#endif
	if (client->dev.of_node) {
		plat_data = devm_kzalloc(&client->dev,
			sizeof(struct helix_platform_data), GFP_KERNEL);
		if (!plat_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = helix_parse_dt(&client->dev, plat_data);
		dev_err(&client->dev,
			"%s: helix_parse_dt ret=%d\n", __func__, err);
		if (err)
			return err;
	} else
		plat_data = client->dev.platform_data;

	if (!plat_data) {
		dev_err(&client->dev,
			"%s: no helix platform data!\n", __func__);
		goto err_als_input_allocate;
	}
	data->als_transmittance = plat_data->transmittance;
	data->int_pin = plat_data->int_pin;
	data->use_fir = plat_data->use_fir;
	data->pdata = plat_data;
#if 0
	if (data->als_transmittance == 0) {
		dev_err(&client->dev,
			"%s: Please set als_transmittance\n", __func__);
		goto err_als_input_allocate;
	}
#endif
	err = sensor_pinctrl_init(&client->dev, &pin_config);
	if (err) {
		dev_err(&client->dev, "init pinctrl failed.\n");
		goto err_als_input_allocate;
	}
	pinctrl_select_state(pin_config.pinctrl, pin_config.state[0]);

	data->als_input_dev = devm_input_allocate_device(&client->dev);
	if (data->als_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate als device\n", __func__);
		err = -ENOMEM;
		goto err_als_input_allocate;
	}
	data->ps_input_dev = devm_input_allocate_device(&client->dev);
	if (data->ps_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);
		err = -ENOMEM;
		goto err_als_input_allocate;
	}
	data->als_input_dev->name = ALS_NAME;
	data->ps_input_dev->name = PS_NAME;
	set_bit(EV_ABS, data->als_input_dev->evbit);
	set_bit(EV_ABS, data->ps_input_dev->evbit);

#if 0
	input_set_abs_params(client->als_input_dev, ABS_MISC, 0, Helix_alscode2lux(client, (1<<16)-1), 0, 0);
#else
	input_set_abs_params(data->als_input_dev, ABS_MISC, 0, LuxCalculation(data), 0, 0);
#endif

	input_set_abs_params(data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	err = input_register_device(data->als_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register als input device\n", __func__);
		goto err_als_input_allocate;
	}
	err = input_register_device(data->ps_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps input device\n", __func__);
		goto err_als_input_allocate;
	}

	err = sysfs_create_group(&data->als_input_dev->dev.kobj, &Helix_als_attribute_group);
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
		goto err_als_input_allocate;
	}
	err = sysfs_create_group(&data->ps_input_dev->dev.kobj, &Helix_ps_attribute_group);
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		goto err_ps_sysfs_create_group;
	}
	err = sysfs_create_link(data->ps_input_dev->dev.kobj.parent,&data->ps_input_dev->dev.kobj,"psensor");
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group  link for ps\n", __func__);
		goto err_ps_sysfs_create_group;
	}
	input_set_drvdata(data->als_input_dev, data);
	input_set_drvdata(data->ps_input_dev, data);

#ifdef Helix_POLL_ALS
	data->Helix_als_wq = create_singlethread_workqueue("Helix_als_wq");
	INIT_WORK(&data->Helix_als_work, Helix_als_work_func);
	hrtimer_init(&data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	data->als_timer.function = Helix_als_timer_func;
#endif

	//data->Helix_ps_wq = create_singlethread_workqueue("Helix_ps_wq");
	//INIT_WORK(&data->Helix_ps_work, Helix_ps_work_func);
	//hrtimer_init(&data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	//data->ps_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	//data->ps_timer.function = Helix_ps_timer_func;
#if (!defined(Helix_POLL_ALS) || !defined(Helix_POLL_PS))
	data->Helix_wq = create_singlethread_workqueue("Helix_wq");
	INIT_WORK(&data->Helix_work, Helix_work_func);
	err = helix_setup_irq(client);
	if(err < 0)
		goto err_helix_setup_irq;
#endif

	err = helix_power_init(data, true);
	if (err)
		goto err_power_init;

	err = helix_power_ctl(data, true);
	if (err)
		goto err_power_on;

	err = i2c_smbus_read_byte_data(client,0x00);
	if(err < 0)
	{
		dev_err(&client->dev,"read chip id fail, err = %d",err);
		goto err_power_on;
	}
	printk("chip id = 0x%x\n",err);

	helix_init_all_setting(data->client, data->pdata);
	data->als_enabled = false;
	data->ps_enabled = false;
	data->ps_cal_is_support = false;
#ifdef CONFIG_HAS_EARLYSUSPEND
	data->Helix_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->Helix_early_suspend.suspend = helix_early_suspend;
	data->Helix_early_suspend.resume = helix_late_resume;
	register_early_suspend(&data->Helix_early_suspend);
#endif

	/* make sure everything is ok before registering the class device */
	data->als_cdev = light_cdev;
	data->als_cdev.sensors_enable = Helix_als_enable_set;
	data->als_cdev.sensors_poll_delay = Helix_als_poll_delay_set;
	err = sensors_classdev_register(&data->als_input_dev->dev, &data->als_cdev);
	if (err) {
		dev_err(&client->dev, "sensors class register failed.\n");
		goto err_power_on;
	}

	data->ps_cdev = proximity_cdev;
	data->ps_cdev.sensors_enable = Helix_ps_enable_set;
	err = sensors_classdev_register(&data->ps_input_dev->dev, &data->ps_cdev);
	if (err) {
		dev_err(&client->dev, "sensors class register failed.\n");
		goto err_class_sysfs;
	}
	
	INIT_DELAYED_WORK(&data->delay_work_cal,helix_delay_work_func);
	schedule_delayed_work(&data->delay_work_cal,msecs_to_jiffies(20*1000));

	/* enable device power only when it is enabled */
	err = helix_power_ctl(data, false);
	if (err)
		goto err_init_all_setting;

	dev_info(&client->dev, "%s: probe successfully", __func__);
	return 0;

err_init_all_setting:
	helix_power_ctl(data, false);
	sensors_classdev_unregister(&data->ps_cdev);
err_class_sysfs:
	sensors_classdev_unregister(&data->als_cdev);
err_power_on:
	helix_power_init(data, false);
err_power_init:
#ifndef Helix_POLL_PS
	free_irq(data->irq, client);
	gpio_free(plat_data->int_pin);
#endif
#if (!defined(Helix_POLL_ALS) || !defined(Helix_POLL_PS))
err_helix_setup_irq:
#endif
#ifdef Helix_POLL_ALS
	hrtimer_try_to_cancel(&data->als_timer);
	destroy_workqueue(data->Helix_als_wq);
#endif
	destroy_workqueue(data->Helix_ps_wq);
#if (!defined(Helix_POLL_ALS) || !defined(Helix_POLL_PS))
	destroy_workqueue(data->Helix_wq);
#endif
	sysfs_remove_group(&data->ps_input_dev->dev.kobj, &Helix_ps_attribute_group);
err_ps_sysfs_create_group:
	sysfs_remove_group(&data->als_input_dev->dev.kobj, &Helix_als_attribute_group);
err_als_input_allocate:
#ifdef Helix_POLL_PS
    wake_lock_destroy(&data->ps_nosuspend_wl);
#endif
    wake_lock_destroy(&data->ps_wakelock);
    mutex_destroy(&data->io_lock);
	kfree(data);
    return err;
}


static int helix_remove(struct i2c_client *client)
{
	struct helix_data *data = i2c_get_clientdata(client);
#ifndef Helix_POLL_PS
	free_irq(data->irq, client);
	gpio_free(data->int_pin);
#endif
#ifdef Helix_POLL_ALS
	hrtimer_try_to_cancel(&data->als_timer);
	destroy_workqueue(data->Helix_als_wq);
#endif
	destroy_workqueue(data->Helix_ps_wq);
#if (!defined(Helix_POLL_ALS) || !defined(Helix_POLL_PS))
	destroy_workqueue(data->Helix_wq);
#endif
	sysfs_remove_group(&data->ps_input_dev->dev.kobj, &Helix_ps_attribute_group);
	sysfs_remove_group(&data->als_input_dev->dev.kobj, &Helix_als_attribute_group);
#ifdef Helix_POLL_PS
	wake_lock_destroy(&data->ps_nosuspend_wl);
#endif
	wake_lock_destroy(&data->ps_wakelock);
    mutex_destroy(&data->io_lock);
	kfree(data);

    return 0;
}
static int helix_suspend(struct device *dev)
{
	int ret = 0;
	struct helix_data *data = dev_get_drvdata(dev);
	
	dev_dbg(dev,"suspending helix \n");

	if(data->als_enabled){
		helix_enable_als(data,0);
		data->als_suspend_enabled = true;
	}else{
		data->als_suspend_enabled = false;
	}

	if(data->ps_enabled){
		helix_enable_ps(data->client,0);
		data->ps_suspend_enabled = true;
	}else{
		data->ps_suspend_enabled = false;
	}	
	
/*
	if(data->ps_enabled){
		ret = enable_irq_wake(data->irq);
		if(ret){
			dev_err(dev,"%s: enable_irq_wake(%d) failed , err=%d \n",__func__,data->irq,ret);
			return ret;
		}
	}else{
		disable_irq(data->irq);
	//	ret = helix_power_ctl(data, false);// suspend   ps not use   power off
		pinctrl_select_state(pin_config.pinctrl,pin_config.state[1]);
	
	}
	*/
	return ret;
}
static int helix_resume(struct device *dev)
{
	int ret = 0;
	struct helix_data *data = dev_get_drvdata(dev);

	dev_dbg(dev,"resuming  helix \n");

	if(data->als_suspend_enabled){
		helix_enable_als(data,1);   // before use als    power on 
	}

	if(data->ps_suspend_enabled){
		helix_enable_ps(data->client,1);   // before use ps    power on 
	}

/*
	if(data->ps_enabled){
		ret = disable_irq_wake(data->irq);
		if(ret){	
			dev_err(dev,"%s: disable_irq_wake(%d) failed , err=%d \n",__func__,data->irq,ret);
			return ret;
		}
	}else{	
		pinctrl_select_state(pin_config.pinctrl,pin_config.state[0]);
		enable_irq(data->irq);	
	}
*/
	return ret;
}

static const struct i2c_device_id Helix_ps_id[] =
{
    { "Helix_ps", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, Helix_ps_id);

static struct of_device_id Helix_match_table[] = {
	{ .compatible = "Helix,helix", },
	{ },
};

static const struct  dev_pm_ops helix_pm_ops = {
	.suspend = helix_suspend,
	.resume  = helix_resume,
};

static struct i2c_driver Helix_ps_driver =
{
    .driver = {
        .name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = Helix_match_table,
		.pm = &helix_pm_ops,
    },
    .probe = helix_probe,
    .remove = helix_remove,
    .id_table = Helix_ps_id,
};


static int __init helix_init(void)
{
	int ret;
    ret = i2c_add_driver(&Helix_ps_driver);
    if (ret)
        return ret;

    return 0;
}

static void __exit helix_exit(void)
{
    i2c_del_driver(&Helix_ps_driver);
}

module_init(helix_init);
module_exit(helix_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sitronix.com.tw>");
MODULE_DESCRIPTION("Sensortek helix Proximity Sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

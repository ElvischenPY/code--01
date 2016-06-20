#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/i2c/aw2013_led.h>


#define AW2013_LED_VTG_MIN_UV		2600000
#define AW2013_LED_VTG_MAX_UV		3300000


#define AW2013_LED_RESET_DELAY  	8
#define AW_LED_CDEV_NAME   "breath_led"

/* register address */
#define AW_REG_CHIP_ID				0x00
#define AW_REG_RESET				0x00
#define AW_REG_GLOBAL_CONTROL		0x01
#define AW_REG_LED_STATUS			0x02
#define AW_REG_LED_ENABLE			0x30
#define AW_REG_LED_CONFIG_BASE		0x31
#define AW_REG_LED_BRIGHTNESS_BASE	0x34
#define AW_REG_TIMESET0_BASE		0x37
#define AW_REG_TIMESET1_BASE		0x38
#define AW_REG_DELAY_PERIOD_BASE	0x39

/* register bits */
#define AW_CHIP_ID					0x33
#define AW_LED_MOUDLE_ENABLE_MASK	0x01
#define AW_LED_FADE_OFF_MASK		0x40
#define AW_LED_FADE_ON_MASK			0x20
#define AW_LED_BREATHE_MODE_MASK	0x10
#define AW_LED_RESET_MASK			0x55


#define MAX_RISE_TIME_MS			7
#define MAX_HOLD_TIME_MS			5
#define MAX_FALL_TIME_MS			7
#define MAX_OFF_TIME_MS				7
#define MAX_PERIOD_DELAY_TIME_MS	8

struct aw2013_platform_data {
	int max_current;
	int rise_time_ms;
	int hold_time_ms;
	int fall_time_ms;
	int off_time_ms;
	int period_delay_time_ms;
};
struct aw2013_led_data {
	struct i2c_client *i2c;
	struct led_classdev cdev;
	struct aw2013_platform_data *pdata;
	struct work_struct brightness_work;
	struct mutex 	ops_lock;
	struct mutex lock;
	struct regulator *vdd;
	struct regulator *vcc;
	int id;
	bool poweron;
	int led_status;
	struct delayed_work aw2013_blink_time;
	
};

static DEFINE_MUTEX(i2c_rw_access);


/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct aw2013_led_data *gl_data;




/*******************************************************************************
*  Name: 	aw2013_led_i2c_read
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int aw2013_led_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;
	
	mutex_lock(&i2c_rw_access);
	if(likely(writelen > 0)) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter,msgs,2);
		if (ret < 0)
			dev_err(&client->dev, "%s:[%d] i2c read error.\n", __func__,__LINE__);
	}else{
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,	
			}
		};
		ret = i2c_transfer(client->adapter,msgs,1);
		if (ret < 0)
			dev_err(&client->dev, "%s:[%d] i2c read error.\n", __func__,__LINE__);
	}
	mutex_unlock(&i2c_rw_access);
	return ret;
}

/*******************************************************************************
*  Name:   aw2013_led_i2c_write
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int aw2013_led_i2c_write(struct i2c_client *client,char *writebuf,int writelen)
{
	int ret;
	struct i2c_msg msgs[] ={
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};
	mutex_lock(&i2c_rw_access);
	ret = i2c_transfer(client->adapter,msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s:[%d] i2c write error.\n", __func__,__LINE__);
	mutex_unlock(&i2c_rw_access);
	return ret;
}

/*******************************************************************************
*  Name:   aw2013_led_write_reg
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int aw2013_led_write_reg(struct i2c_client *client,u8 addr,const u8 val)
{
	u8 buf[2] = {0};
	
	buf[0] = addr;
	buf[1] = val;
	
	return aw2013_led_i2c_write(client,buf,sizeof(buf));
}

/*******************************************************************************
*  Name:   aw2013_led_read_reg
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int aw2013_led_read_reg(struct i2c_client *client,u8 addr ,u8 *val)
{
	return aw2013_led_i2c_read(client,&addr,1,val,1);
}

/*******************************************************************************
*  Name:   aw2013_led_power_init
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_power_init(struct aw2013_led_data *data,bool on)
{
	int ret;
	
	if (!on){
		if(regulator_count_voltages(data->vdd)>0)
			regulator_set_voltage(data->vdd,0,AW2013_LED_VTG_MAX_UV);
		regulator_put(data->vdd);
	}else{
		data->vdd = regulator_get(&data->i2c->dev,"vdd");
		if(IS_ERR(data->vdd)){
			ret = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev,"Regulator get vdd failed  ret=%d\n",ret);
			return ret;
		}
		if(regulator_count_voltages(data->vdd)>0){
			ret=regulator_set_voltage(data->vdd,AW2013_LED_VTG_MIN_UV,AW2013_LED_VTG_MAX_UV);
			if(ret){
				dev_err(&data->i2c->dev,"Regulator set vdd failed  ret=%d\n",ret);
				goto err_set_vdd_put;
			}
			
		}
	}
	
	return 0;
	
err_set_vdd_put:	
	regulator_put(data->vdd);
	return ret;
}

/*******************************************************************************
*  Name:   aw2013_led_power_on
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_power_on(struct aw2013_led_data *data, bool on)
{
	int ret = 0;
	if(!on){
		ret = regulator_disable(data->vdd);
		if(ret){
			dev_err(&data->i2c->dev,"Regulator vdd disable failed ret=%d\n",ret);
			return ret;
		}
		data->poweron = false;
	}else{
		ret = regulator_enable(data->vdd);
		if(ret){
			dev_err(&data->i2c->dev,"Regulator vdd enable failed ret=%d\n",ret);
			return ret;
		}
		data->poweron = true;
	}
	pr_info("%s:[%d] regulator power  on=%d  \n",__func__,__LINE__,on);
	return 0;
}

/*******************************************************************************
*  Name:   aw2013_led_reset
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_reset(struct aw2013_led_data *data)
{
	int ret = 0;
	
	ret = aw2013_led_write_reg(data->i2c,0x00,0x55);
	usleep(AW2013_LED_RESET_DELAY);
	if(ret<0){
		dev_err(&data->i2c->dev," %s:[%d] reset IC failed ret=%d\n",__func__,__LINE__,ret);
		return ret;
	}
	return 0;
}

/*******************************************************************************
*  Name:   aw2013_led_read_id
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_read_id(struct aw2013_led_data *data)
{
	u8 chip_id=0;
	int ret;
	
	ret = aw2013_led_read_reg(data->i2c,AW_REG_CHIP_ID,&chip_id);
	if(ret<=0){
		dev_err(&data->i2c->dev,"read reg %#x faild.  ret=%d\n",AW_REG_CHIP_ID,ret);
		return ret;
	}
	if(chip_id != AW_CHIP_ID){
		dev_err(&data->i2c->dev,"read chip id faild.\n");
		return -ENODEV;
	}
	return (int)chip_id;
}

static unsigned int aw2013_led_blink_time(void)
{
	struct aw2013_led_data *data = gl_data;
	unsigned int blink_time;
	
	blink_time = ((1<<data->pdata->rise_time_ms) + (1<<data->pdata->hold_time_ms) + 
					(1<<data->pdata->fall_time_ms) + (1<<data->pdata->off_time_ms) ) * 130 ;
	if(data->pdata->period_delay_time_ms > 0)
		blink_time = blink_time + (1 << (data->pdata->period_delay_time_ms - 1)) * 130;

	return blink_time;
}
static void aw2013_blink_time_work(struct work_struct *work)
{
	struct aw2013_led_data *data = gl_data;
	if(data->led_status == 1){
		data->led_status = 0;
	}
}

/*******************************************************************************
*  Name:   aw2013_led_brightness_work
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void aw2013_led_brightness_work(struct work_struct *work)
{
	struct aw2013_led_data *data = gl_data;//container_of(work,struct aw2013_led_data,brightness_work);
	u8 val;
		
	mutex_lock(&data->lock);
	
	if(!data->poweron){
		if(aw2013_led_power_on(data,true)){
			dev_err(&data->i2c->dev,"power on faild");
			mutex_unlock(&data->lock);
			return;
		}
	}
	
	if(data->cdev.brightness>0){
		data->led_status = 3;
		if(data->cdev.brightness > data->cdev.max_brightness)
			data->cdev.brightness = data->cdev.max_brightness;
		aw2013_led_write_reg(data->i2c,AW_REG_GLOBAL_CONTROL,AW_LED_MOUDLE_ENABLE_MASK);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_CONFIG_BASE + data->id ,data->pdata->max_current);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_BRIGHTNESS_BASE + data->id ,data->cdev.brightness);
		aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_ENABLE,val|(1<<data->id));
	}else{
		data->led_status = 0;
		aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_ENABLE,val&(~( 1 << data->id )));
	}
	aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
	if(val == 0){
		if(aw2013_led_power_on(data,false)){
			dev_err(&data->i2c->dev,"power off faild");
			mutex_unlock(&data->lock);
			return ;
		}
	}
	mutex_unlock(&data->lock);
}

/*******************************************************************************
*  Name:   aw2013_led_set_brightness
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void aw2013_led_set_brightness(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct aw2013_led_data *data = gl_data;//container_of(cdev,struct aw2013_led_data,cdev);
	
	data->cdev.brightness = brightness;
	schedule_work(&data->brightness_work);	
}
/*
static void aw2013_led_blink_set(struct aw2013_led_data *led, const int brightness, const int num)
{
	u8 val;
	//////mark
	
	return;	
}
*/
/*******************************************************************************
*  Name:   aw2013_led_breath_blink_store
*  Brief:  Provide a control interface for the user space to control the breath light  brightness and breath count 
*  Input:  brightness:  0~255  ; 
*		   num:  0-15(num<=0  off led ;  num >15 the light will always breath) num is blink number
*  Output: 
*  Return: 
*******************************************************************************/
static ssize_t aw2013_led_breath_blink_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t len)
{
	//unsigned long blinking;
	int num,brightness;
	u8 val;
	struct aw2013_led_data *data = gl_data;
		
	mutex_lock(&data->lock);
	
	if(!data->poweron){
		if(aw2013_led_power_on(data,true)){
			dev_err(&data->i2c->dev,"power on faild");
			mutex_unlock(&data->lock);
			return len;
		}
	}
	
	sscanf(buf," %d %d",&brightness,&num);
	//aw2013_led_blink_set(data,brightness,num);
	if(num > 0x0F)
		data->led_status = 2;
	else if (num < 0 || num == 0)
		data->led_status = 0;
	else {
		data->led_status = 1;
		schedule_delayed_work(&data->aw2013_blink_time,msecs_to_jiffies(num * aw2013_led_blink_time()));
	}
	if((brightness>0) && (num>0)){
		if(brightness > data->cdev.max_brightness)
			data->cdev.brightness = data->cdev.max_brightness;
		else
			data->cdev.brightness = brightness;
		aw2013_led_write_reg(data->i2c,AW_REG_GLOBAL_CONTROL,AW_LED_MOUDLE_ENABLE_MASK);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_CONFIG_BASE+data->id,AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |AW_LED_BREATHE_MODE_MASK | data->pdata->max_current);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_BRIGHTNESS_BASE + data->id,data->cdev.brightness);
		aw2013_led_write_reg(data->i2c, AW_REG_TIMESET0_BASE + data->id * 3,data->pdata->rise_time_ms << 4 |data->pdata->hold_time_ms);
		aw2013_led_write_reg(data->i2c, AW_REG_TIMESET1_BASE + data->id * 3,data->pdata->fall_time_ms << 4 |data->pdata->off_time_ms);
		//aw2013_led_write_reg(data->i2c,AW_REG_DELAY_PERIOD_BASE,&val);
		num = (num>0x0F)?0:num;
		aw2013_led_write_reg(data->i2c,AW_REG_DELAY_PERIOD_BASE + data->id * 3,data->pdata->period_delay_time_ms << 4 | num);
		aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_ENABLE,val | (1 << data->id));	
	}else{
		aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_ENABLE,val &(~(1 << data->id)));	
	}
	
	aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);

	/*
	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	if(val ==0 ){
		mutex_unlock(&data->lock);
		if(aw2013_led_power_on(data,false)){
			dev_err(&data->i2c->dev,"power off faild");
			return len;
		}
	}
	
	mutex_unlock(&data->lock);	
	pr_info("%s:[%d] set breath blink ok  \n", __func__,__LINE__);
	
	return len;
}
int  aw2013_led_breath_blink(int num,int brightness)
{
	u8 val;
	struct aw2013_led_data *data = gl_data;
	int rise_time_ms=0;
	int hold_time_ms=1;
	int fall_time_ms=0;
	int off_time_ms=0;
	int period_delay_time_ms=0;
	unsigned int blink_time;
	
	blink_time = ((1<<rise_time_ms) + (1<<hold_time_ms) + 
					(1<<fall_time_ms) + (1<<off_time_ms) ) * 130 ;
	if(period_delay_time_ms > 0)
		blink_time = blink_time + (1 << (period_delay_time_ms - 1)) * 130;

		if(data->led_status == 2 || data->led_status == 3)
			return 0;
		
	
	if(!data->poweron){
		if(aw2013_led_power_on(data,true)){
			dev_err(&data->i2c->dev,"power on faild");
			return -1;
		}
	}
	
//	pr_err("%s(%d): num=%d  brightness=%d \n",__func__,__LINE__,num,brightness);
	if(num > 0x0F)
		data->led_status = 2;
	else if (num < 0 || num == 0)
		data->led_status = 0;
	else {
		data->led_status = 1;
		schedule_delayed_work(&data->aw2013_blink_time,msecs_to_jiffies(num * blink_time /* aw2013_led_blink_time()*/));
	}

	if((brightness>0) && (num>0)){
		if(brightness > data->cdev.max_brightness)
			data->cdev.brightness = data->cdev.max_brightness;
		else
			data->cdev.brightness = brightness;
		aw2013_led_write_reg(data->i2c,AW_REG_GLOBAL_CONTROL,AW_LED_MOUDLE_ENABLE_MASK);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_CONFIG_BASE+data->id,AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |AW_LED_BREATHE_MODE_MASK | data->pdata->max_current);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_BRIGHTNESS_BASE + data->id,data->cdev.brightness);
		aw2013_led_write_reg(data->i2c, AW_REG_TIMESET0_BASE + data->id * 3,rise_time_ms << 4 |hold_time_ms);
		aw2013_led_write_reg(data->i2c, AW_REG_TIMESET1_BASE + data->id * 3,fall_time_ms << 4 |off_time_ms);
		num = (num>0x0F)?0:num;
		aw2013_led_write_reg(data->i2c,AW_REG_DELAY_PERIOD_BASE + data->id * 3,period_delay_time_ms << 4 | num);
		aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_ENABLE,val | (1 << data->id));	
	}else{
		aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
		aw2013_led_write_reg(data->i2c,AW_REG_LED_ENABLE,val &(~(1 << data->id)));	
	}
	

	aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
	/*
	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	if(val ==0 ){
		if(aw2013_led_power_on(data,false)){
			dev_err(&data->i2c->dev,"power off faild");
			return -1;
		}
	}
	
//	pr_info("%s:[%d] set breath blink ok  \n", __func__,__LINE__);
	
	return 0;
}
EXPORT_SYMBOL_GPL(aw2013_led_breath_blink);

/*******************************************************************************
*  Name:   aw2013_led_time_show
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static ssize_t aw2013_led_time_show(struct device *dev, struct device_attribute *attr, char * buf)
{
//	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led_data *data = gl_data;
	
	pr_info("led breath time(ms) rise=%d; on=%d; fall=%d; off=%d delay =%d \n",
				data->pdata->rise_time_ms, data->pdata->hold_time_ms,data->pdata->fall_time_ms,
				data->pdata->off_time_ms,data->pdata->period_delay_time_ms);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d \n",data->pdata->rise_time_ms,
					data->pdata->hold_time_ms,data->pdata->fall_time_ms, data->pdata->off_time_ms,
					data->pdata->period_delay_time_ms);
}

/*******************************************************************************
*  Name:   aw2013_led_time_store
*  Brief:  set the breath time of light 
*  Input:   rise time (0~7): 000:0.13s;  001:0.26s;  010:0.52s;  011:1.04s;  100:2.08s;  101:4.16s;  110:8.32s;  111:16.64s;
*			on time   (0~5): 000:0.13s;  001:0.26s;  010:0.52s;  011:1.04s;  100:2.08s;  101:4.16s;
*			fall time (0~7): 000:0.13s;  001:0.26s;  010:0.52s;  011:1.04s;  100:2.08s;  101:4.16s;  110:8.32s;  111:16.64s;
*			off time  (0~7): 000:0.13s;  001:0.26s;  010:0.52s;  011:1.04s;  100:2.08s;  101:4.16s;  110:8.32s;  111:16.64s;
*			delay time(0~8): 000:0s;     001:0.13s;  010:0.26s;  011:0.52s;  100:1.04s;  101:2.08s;  110:4.16s;  111;8.32s;   1000： 16.64s;
*  Output:
*  Return: 
*******************************************************************************/
static ssize_t aw2013_led_time_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int ret,rise_time_ms,hold_time_ms,fall_time_ms,off_time_ms,period_delay_time_ms;
	struct aw2013_led_data *data = gl_data;
	
	mutex_lock(&data->lock);
	ret = sscanf(buf,"%d %d %d %d %d ",&rise_time_ms,&hold_time_ms,&fall_time_ms,&off_time_ms,&period_delay_time_ms);
	data->pdata->rise_time_ms = (rise_time_ms > MAX_RISE_TIME_MS) ? data->pdata->rise_time_ms : rise_time_ms;
	data->pdata->hold_time_ms = (hold_time_ms > MAX_HOLD_TIME_MS) ? data->pdata->hold_time_ms : hold_time_ms;
	data->pdata->fall_time_ms = (fall_time_ms > MAX_FALL_TIME_MS) ? data->pdata->fall_time_ms : fall_time_ms;
	data->pdata->off_time_ms  = (off_time_ms > MAX_OFF_TIME_MS) ? data->pdata->off_time_ms : off_time_ms;
	data->pdata->period_delay_time_ms  = (period_delay_time_ms > MAX_PERIOD_DELAY_TIME_MS) ? data->pdata->period_delay_time_ms : period_delay_time_ms;
	
//	aw2013_led_blink_set(data,1);
	mutex_unlock(&data->lock);
	
	return len;
}

/*******************************************************************************
*  Name:   aw2013_led_status_show
*  Brief:  show the status of led :  0:off    1:blink(1-15)   2:always blink  3:always on
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static ssize_t aw2013_led_status_show(struct device *dev, struct device_attribute *attr, char * buf)
{
	struct aw2013_led_data *data = gl_data;
	u8 val;
	bool enabled = false;
	aw2013_led_read_reg(data->i2c,AW_REG_LED_ENABLE,&val);
	enabled = (val && (1 << data->id));
	if(!enabled)
			data->led_status = 0;
	return snprintf(buf, PAGE_SIZE, "%d \n",data->led_status);
}

static DEVICE_ATTR(status,0664,aw2013_led_status_show,NULL);
static DEVICE_ATTR(blink,0664,NULL,aw2013_led_breath_blink_store);
static DEVICE_ATTR(led_time,0664,aw2013_led_time_show,aw2013_led_time_store);
static struct attribute *aw2013_led_attributes[] = {
	&dev_attr_status.attr,
	&dev_attr_blink.attr,
	&dev_attr_led_time.attr,
	NULL,
};
static const struct attribute_group aw2013_led_attr_group = {
	.attrs = aw2013_led_attributes,
};

/*******************************************************************************
*  Name:   aw2013_led_parse_dt
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_parse_dt(struct device *dev,struct aw2013_led_data *data)
{
	struct device_node *np = dev->of_node;
	int ret;
	
	ret = of_property_read_u32(np,"aw2013,id",&data->id);
	if(ret<0){
		dev_err(dev,"Reading led num id faild  ret=%d \n",ret);
		return ret;
	}
	
	ret = of_property_read_u32(np,"aw2013,max-brightness",&data->cdev.max_brightness);
	if(ret<0){
		dev_err(dev,"Reading max-brightness faild  ret=%d \n",ret);
		return ret;
	}
	
	ret = of_property_read_u32(np,"aw2013,max-current",&data->pdata->max_current);
	if(ret<0){
		dev_err(dev,"Reading max-current faild  ret=%d \n",ret);
		return ret;
	}
	
	ret = of_property_read_u32(np,"aw2013,rise-time-ms",&data->pdata->rise_time_ms);
	if(ret<0){
		dev_err(dev,"Reading  rise-time-ms  faild  ret=%d \n",ret);
		return ret;
	}
	
	ret = of_property_read_u32(np,"aw2013,hold-time-ms",&data->pdata->hold_time_ms);
	if(ret<0){
		dev_err(dev,"Reading hold-time-ms  faild  ret=%d \n",ret);
		return ret;
	}
	
	ret = of_property_read_u32(np,"aw2013,fall-time-ms",&data->pdata->fall_time_ms);
	if(ret<0){
		dev_err(dev,"Reading fall-time-ms faild  ret=%d \n",ret);
		return ret;
	}
		
	ret = of_property_read_u32(np,"aw2013,off-time-ms",&data->pdata->off_time_ms);
	if(ret<0){
		dev_err(dev,"Reading off-time-ms faild  ret=%d \n",ret);
		return ret;
	}
	
	ret = of_property_read_u32(np,"aw2013,period-delay-time-ms",&data->pdata->period_delay_time_ms);
	if(ret<0){
		dev_err(dev,"Reading period_delay_time_ms faild  ret=%d \n",ret);
		return ret;
	}
	
	pr_info("%s:[%d]  DT ok  \n",__func__,__LINE__);
	return 0;
}
/*******************************************************************************
*  Name: aw2013_led_probe
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	static struct aw2013_led_data *data;
	
	pr_info("%s:[%d]  probe enter \n",__func__,__LINE__);
	
	if(client->dev.of_node){
		data = devm_kzalloc(&client->dev,sizeof(struct aw2013_led_data),GFP_KERNEL);
		if(!data){
			dev_err(&client->dev,"Failed to allocate aw2013_led_data memory\n");
			goto err_dev_kzalloc_data;
		}
		data->pdata = devm_kzalloc(&client->dev,sizeof(struct aw2013_platform_data),GFP_KERNEL);
		if(!data->pdata){
			dev_err(&client->dev,"Failed to allocate aw2013_platform_data memory\n");
			goto err_dev_kzalloc_pdata;
		}
		err = aw2013_led_parse_dt(&client->dev,data);
		
		if(err){
			dev_err(&client->dev,"DT parsing failed! \n");
			goto err_ret;
		}
	}
	if(!data){
		dev_err(&client->dev,"Invalid pdata\n");
		goto err_ret;
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		goto err_ret;
	}
	
	data->i2c = client;
	dev_set_drvdata(&client->dev,data);
	gl_data = data;
	//mutex_init(&data->ops_lock);
	mutex_init(&data->lock);
	
	err = aw2013_led_power_init(data,true);
	if(err<0){
		dev_err(&client->dev, "%s:[%d] power init failed.\n", __func__,__LINE__);
		goto err_power_init;
	}
	
	err = aw2013_led_power_on(data,true);
	if (err) {
		dev_err(&client->dev, "power up  failed.\n");
		goto err_power_on;
	}
	
	err = aw2013_led_reset(data);
	if(err<0){
		dev_err(&client->dev, "aw2013_led  reset  failed.\n");
		goto err_reset;
	}
		
	err = aw2013_led_read_id(data);
	if(err<0){
		dev_err(&client->dev,"read id failed. \n");
		goto err_check_device;
	}
	
	INIT_WORK(&data->brightness_work,aw2013_led_brightness_work);
	data->cdev.brightness_set = aw2013_led_set_brightness;
	data->cdev.name = AW_LED_CDEV_NAME;
	INIT_DELAYED_WORK(&data->aw2013_blink_time, aw2013_blink_time_work);
	
	err = led_classdev_register(&data->i2c->dev,&data->cdev);
	if(err<0){
		dev_err(&client->dev,"unable to register led brightness_set. \n");
		goto err_classdev_register;
	}
	
	err = sysfs_create_group(&data->cdev.dev->kobj,&aw2013_led_attr_group);
	if(err<0){
		dev_err(&client->dev,"sysfs_create_group: Error to create calibration attribute. \n");
		goto err_sysfs_create_group;
	}
	pr_info("%s:[%d]  probe end \n",__func__,__LINE__);

	return 0;
	
err_sysfs_create_group:	
	sysfs_remove_group(&data->cdev.dev->kobj,&aw2013_led_attr_group);
err_classdev_register:
	led_classdev_unregister(&data->cdev);
err_check_device:
err_reset:
	aw2013_led_power_on(data,false);
err_power_on:
	aw2013_led_power_init(data,false);
err_power_init:
err_ret:
err_dev_kzalloc_pdata:
	devm_kfree(&client->dev, data->pdata);
err_dev_kzalloc_data:
	devm_kfree(&client->dev, data);
	
	return err;
}
/*******************************************************************************
*  Name: aw2013_led_suspend
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_suspend(struct device *dev)
{
	return 0;
}
/*******************************************************************************
*  Name: aw2013_led_resume
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_resume(struct device *dev)
{
	return 0;
}
/*******************************************************************************
*  Name: aw2013_led_remove
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int aw2013_led_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id aw2013_led_id[] = {
	{"aw2013_led",0},
	{},
};
MODULE_DEVICE_TABLE(i2c, aw2013_led_id);
static struct of_device_id aw2013_led_match_table[] = {
	{ .compatible = "awinic,aw2013",},
	{ },
};
static struct dev_pm_ops aw2013_led_pm_ops = {
	.suspend = aw2013_led_suspend,
	.resume = aw2013_led_resume,
};
static struct i2c_driver aw2013_led_driver = {
	.probe = aw2013_led_probe,
	.remove = aw2013_led_remove,
	.id_table = aw2013_led_id,
	.driver = {
		.name = "aw2013_led",
		.owner = THIS_MODULE,
		.of_match_table = aw2013_led_match_table,
		.pm = &aw2013_led_pm_ops,
	},
};
/*******************************************************************************
*  Name: aw2013_led_init
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int __init aw2013_led_init(void)
{
	return i2c_add_driver(&aw2013_led_driver);
}

/*******************************************************************************
*  Name: aw2013_led_exit
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void __exit aw2013_led_exit(void)
{
	i2c_del_driver(&aw2013_led_driver);
}

module_init(aw2013_led_init);
module_exit(aw2013_led_exit);

MODULE_DESCRIPTION("aw2013 led driver");
MODULE_LICENSE("GPL v2");

/* 
 * drivers/input/touchscreen/msg2133_ts.c
 *
 * FocalTech msg2133 TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <mach/gpiomux.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/regulator/consumer.h>
//#include <mach/ldo.h>
//#include <linux/wingtech.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/msg2133a.h>
/*<begin xiongyunmei 20130415 add tp msg2133 firmware update*/
//#include <linux/msg2133a_update_i.h>
#include <linux/Soul_3.5_EVDO_MSG2133A_Each_V0x0301.0x0004_20140512.h>

/* xiongyunmei 20130415 add tp msg2133 firmware update end>*/
#include <linux/input/mt.h>//slot

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#define AUTO_SWAP_TWO_POINT	1

#define MSG2133_ESD 0
#define MSG2133_PROXIMITY_FILTER 0
#define MSG2133_CHECK_INCALL

#define VTG_MIN_UV		2600000
#define VTG_MAX_UV		3300000
#define I2C_VTG_MIN_UV	1800000
#define I2C_VTG_MAX_UV	1800000

#if TS_DEBUG_MSG
#define MSG2133_DBG(format, ...)	printk(""format" \n", ## __VA_ARGS__)//printk(KERN_INFO "MSG2133 " format "\n", ## __VA_ARGS__)
#else
#define MSG2133_DBG(format, ...)
#endif

#if TS_DEBUG_MSG_UPDATE
#define MSG2133_DBG_UPDATE(format, ...)	printk(""format" \n", ## __VA_ARGS__)//printk(KERN_INFO "MSG2133 " format "\n", ## __VA_ARGS__)
#else
#define MSG2133_DBG_UPDATE(format, ...)
#endif

#ifndef USE_OLD_REPORT
static struct mutex msg21xx_mutex;

#define REPORT_PACKET_LENGTH  8

#define MAX_TOUCH_FINGER 2

typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;


#endif
static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type );

static struct i2c_client *this_client;
static  struct i2c_driver msg2133_ts_driver;
static int tmp_prox;  //disable irq, when close prox func
spinlock_t prox_lock;

int init_irq,init_reset;
//static int tp_type;
/*<begin xiongyunmei 20130415 add tp msg2133 firmware update*/
extern const char GroupData[];
/* xiongyunmei 20130415 add tp msg2133 firmware update end>*/
#if MSG2133_ESD
//@hfs---for esd start
static int msg2133_close_flag =0;
static struct work_struct resume_work;
static struct workqueue_struct * msg2133_esd_resume_workqueue = NULL;

static void msg2133_esd_resume_expire(unsigned long data);
static void msg2133_esd_timer_start(void);
void msg2133_esd_resume_callback(struct work_struct *work);


static DEFINE_TIMER(msg2133_esd_resume_timer, msg2133_esd_resume_expire, 0, 0);

//@hfs---for esd end
#endif

#if MSG2133_PROXIMITY_FILTER
#define MSG2133_PROXIMITY_CLOSE_NUM 2
#define MSG2133_PROXIMITY_OFF_NUM 3

static int msg2133_close_num=0;
static int msg2133_off_num=0;

static int msg2133_proximity_report_lock =0;
static int msg2133_proximity_report_value=0;
static void msg2133_proximity_timer_start(void);
static void msg2133_proximity_timer_callback(void);
static DEFINE_TIMER(msg2133_proximity_timer, msg2133_proximity_timer_callback, 0, 0);

//static struct work_struct resume_work;
//static struct workqueue_struct * msg2133_esd_resume_workqueue = NULL;
//void msg2133_esd_resume_callback(struct work_struct *work);
#endif

#if 0//def MSG2133_CHECK_INCALL
static void msg2133_check_incall_timer_start(void);
static void msg2133_check_incall_expire(unsigned long data);

static DEFINE_TIMER(msg2133_chek_incall_timer, msg2133_check_incall_expire, 0, 0);
//static DEFINE_SPINLOCK(msg2133_prox_lock);

#endif
#if 0
static struct gpiomux_setting reset_gpio_config = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
    .dir = GPIOMUX_OUT_HIGH,
};
#endif
static struct gpiomux_setting mstar_int_act_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_8MA,
    .pull = GPIOMUX_PULL_UP,
};

/*
   static struct gpiomux_setting mstar_int_sus_cfg = {
   .func = GPIOMUX_FUNC_GPIO,
   .drv = GPIOMUX_DRV_2MA,
   .pull = GPIOMUX_PULL_DOWN,
   };
 */
static struct gpiomux_setting mstar_reset_act_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
};
/*
   static struct gpiomux_setting mstar_reset_sus_cfg = {
   .func = GPIOMUX_FUNC_GPIO,
   .drv = GPIOMUX_DRV_6MA,
   .pull = GPIOMUX_PULL_UP,
   };


   static struct msm_gpiomux_config msm_mstar_configs[] __initdata = {
   {
   .gpio      = 0,		
   .settings = {
   [GPIOMUX_ACTIVE] = &mstar_reset_act_cfg,
   [GPIOMUX_SUSPENDED] = &mstar_reset_sus_cfg,
   },
   },
   {
   .gpio      = 1,
   .settings = {
   [GPIOMUX_ACTIVE] = &mstar_int_act_cfg,
   [GPIOMUX_SUSPENDED] = &mstar_int_sus_cfg,
   },
   },
   };
 */
static int msg2133_ts_reset(void)
{
    unsigned long flags;
    int use_old;
    int status = 0;
    static int get_rst_gpio = 0;
    struct gpiomux_setting old_gpio_setting;
    spin_lock_irqsave(&prox_lock, flags);
    tmp_prox = 0;
    spin_unlock_irqrestore(&prox_lock, flags);
    use_old = 1;
    if(get_rst_gpio == 0){
        get_rst_gpio = 1;
        status = gpio_request(init_reset , "tp_reset_gpio");
        if(status < 0){
            pr_err("%s : gpio(%d) request failed.\n", __func__, init_reset);
            //return -1;
        }
    }
    if(use_old){
        //gpio_tlmm_config(GPIO_CFG(init_reset, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    }else{
        if (msm_gpiomux_write(init_reset, GPIOMUX_ACTIVE,
                    &mstar_reset_act_cfg, &old_gpio_setting)) {
            pr_err("%s : GPIO pins have no active setting\n", __func__);
        }
    }
    //#endif
    gpio_direction_output(init_reset, 1);
    gpio_set_value(init_reset, 1);
    mdelay(10);//msleep(10);
    gpio_set_value(init_reset , 0);
    mdelay(10);//msleep(10);	//sunny tell me fix the delay on 0726
    gpio_set_value(init_reset, 1);
    mdelay(10);//msleep(10);
    //gpio_free(init_reset);
    //pr_info("%s : reset\n", __func__);
    spin_lock_irqsave(&prox_lock, flags);
    tmp_prox = 1;
    spin_unlock_irqrestore(&prox_lock, flags);
    mdelay(50);//msleep(100);

    return 0;
}
/*<begin xiongyunmei 20130415 add tp msg2133 firmware update*/
#if MSG2133_UPDATE
#define FW_ADDR_MSG21XX          (0xC4 >> 1)
#define FW_ADDR_MSG21XX_TP       (0x4C >> 1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92 >> 1)
#ifdef  UPDATE_INFO
static unsigned char g_dwiic_info_data[1024];   // Buffer for info data
#endif

#if MSG2133_APK_UPDATE
static  char *fw_version_mstar;
#endif

#if MSG2133_APK_UPDATE || MSG2133_AUTO_UPDATE
static u8 temp[33][1024];//94
#endif

//static u8  Fmr_Loader[1024];
static u32 crc_tab[256];

static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;


static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size)
{
    //according to your platform.
    int rc;

    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = I2C_M_RD,
            .len = size,
            .buf = read_data,
        },
    };

    rc = i2c_transfer(this_client->adapter, msgs, 1);
    if( rc < 0 )
    {
        MSG2133_DBG("HalTscrCReadI2CSeq error %d\n", rc);
    }
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
    int rc;
    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = 0,
            .len = size,
            .buf = data,
        },
    };
    rc = i2c_transfer(this_client->adapter, msgs, 1);
    if( rc < 0 )
    {
        MSG2133_DBG("HalTscrCDevWriteI2CSeq error %d,addr = %d, this_client->addr=0x%x\n", rc,addr, this_client->addr);
    }
}
/*
   static void Get_Chip_Version(void)
   {
   MSG2133_DBG("[%s]: Enter!\n", __func__);
   unsigned char dbbus_tx_data[3];
   unsigned char dbbus_rx_data[2];

   dbbus_tx_data[0] = 0x10;
   dbbus_tx_data[1] = 0x1E;
   dbbus_tx_data[2] = 0xCE;
   HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
   HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
   if (dbbus_rx_data[1] == 0)
   {
// it is Catch2
MSG2133_DBG(MSG2133_DBG("*** Catch2 ***\n");)
//FwVersion  = 2;// 2 means Catch2
}
else
{
// it is catch1
MSG2133_DBG(MSG2133_DBG("*** Catch1 ***\n");)
//FwVersion  = 1;// 1 means Catch1
}

}
 */

static void dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

    // Delay some interval to guard the next transaction
    udelay ( 150);//200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
    MSG2133_DBG("\n******%s come in*******\n",__FUNCTION__);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);
    udelay ( 150 );//200 );        // delay about 0.1ms
}

static u8 drvISP_Read(u8 n, u8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0};
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay( 800 );//200);
    if (n == 1)
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
        *pDataToRead = dbbus_rx_data[0];
        //MSG2133_DBG("dbbus=%d,%d===drvISP_Read=====\n",dbbus_rx_data[0],dbbus_rx_data[1]);
    }
    else
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, pDataToRead, n);
    }

    return 0;
}

static void drvISP_WriteEnable(void)
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    udelay(150);//1.16
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1);
    udelay( 150 );//200);
}

static u8 drvISP_ReadStatus(void)
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(150);//200);
    drvISP_Read(1, &bReadData);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    return bReadData;
}

#if 0
static void drvISP_BlockErase(u32 addr)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;

    u32 timeOutCount=0;
    MSG2133_DBG("\n******%s come in*******\n",__FUNCTION__);
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 3);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(150);//200);
    timeOutCount=0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;//0xD8;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    //bWriteData[4] = (addr & 0xFF) ;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 5);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(150);//200);
    timeOutCount=0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}
#endif
static void drvISP_Program(u16 k, u8* pDataToWrite)
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u32 timeOutCount=0;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);
        for (i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        //msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(150);//200);

        timeOutCount=0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, TX_data, 133);   //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    }
}
#if MSG2133_APK_UPDATE
static ssize_t firmware_update_show ( struct device *dev,
        struct device_attribute *attr, char *buf )
{
    return sprintf ( buf, "%s\n", fw_version_mstar );
}
#endif
static void drvISP_Verify ( u16 k, u8* pDataToVerify )
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] ={ 0x10, 0x03, 0, 0, 0 };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index = 0;
    u32 timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( u8 ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( u8 ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( u8 ) ( addr + j * 128 );
        udelay ( 100 );        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 ); //write read flash addr
        udelay ( 100 );        // delay about 100us*****
        drvISP_Read ( 128, RX_data );
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //MSG2133_DBG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                // MSG2133_DBG ( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static void drvISP_ChipErase(void)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
    u32 timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 3 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}

/* update the firmware part, used by apk*/
/*show the fw version*/

/*static ssize_t firmware_update_c2 ( struct device *dev,
  struct device_attribute *attr, const char *buf, size_t size )*/

static ssize_t firmware_update_c2 (  size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    //   MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    msg2133_ts_reset();
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    //  MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    // MSG2133_DBG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    return size;
}

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
    ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}
static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}


static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////

    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

/*static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
  const char *buf, size_t size,  EMEM_TYPE_t emem_type )*/
static ssize_t firmware_update_c32 ( size_t size,  EMEM_TYPE_t emem_type )
{
    //u8  dbbus_tx_data[4];
    // u8  dbbus_rx_data[2] = {0};
    // Buffer for slave's firmware

    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u16 reg_data = 0;
    int mstar_count0 = 0;
    int mstar_count1 = 0;
    int mstar_count2 = 0;
    int mstar_count3 = 0;

    MSG2133_DBG(" firmware_update_c32\n");
    // return 0;
    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();

    mdelay ( 3000 ); // msleep ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );
    MSG2133_DBG(" firmware_update_c32:  1\n");
    //ResetSlave();
    msg2133_ts_reset();
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    MSG2133_DBG("**** prepare to program *****\n");
    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        mstar_count0++;
    }
    while ( (reg_data != 0x1C70));// && (mstar_count0--));

    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        mstar_count1++;
    }
    while ( (reg_data != 0x2F43) );//&& (mstar_count1--));

    MSG2133_DBG("***** calculate CRC 32 *****\n");
    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        MSG2133_DBG("******write i2c start %d\n", i);
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
            mstar_count2++;
        }
        while ( (reg_data != 0xD0BC));// && (mstar_count2--));

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        mstar_count3++;
    }
    while ( (reg_data != 0x9432));// && (mstar_count3--) );


    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    // MSG2133_DBG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
    //        crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        MSG2133_DBG ( "update FAILED\n" );
        msg2133_ts_reset();
        FwDataCnt = 0;
        //enable_irq(this_client->irq);		
        return ( 0 );
    }

    MSG2133_DBG ( "update OK\n" );
    msg2133_ts_reset();
    FwDataCnt = 0;
    //enable_irq(this_client->irq);

    return size;
#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    MSG2133_DBG(" drvTP_erase_emem_c33\n");
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}
#if 0
static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}
#endif

#ifdef  UPDATE_INFO
static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
    //u8  dwiic_rx_data[4];
    u16 reg_data=0;
    int deng=100;
    u8 u8i = 0;

    mdelay ( 300 );
    MSG2133_DBG(" drvTP_read_info_dwiic_c33\n");
    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    MSG2133_DBG(" drvTP_read_info_dwiic_c33...2\n");
    mdelay ( 50 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );
    MSG2133_DBG(" drvTP_read_info_dwiic_c33....3\n");
    mdelay ( 50 );
    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );
    MSG2133_DBG(" drvTP_read_info_dwiic_c33...4\n");
    mdelay ( 50 );
    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
    MSG2133_DBG(" drvTP_read_info_dwiic_c33....5\n");
    mdelay ( 50 );
    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
    MSG2133_DBG(" drvTP_read_info_dwiic_c33...6\n");
    mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );	
    MSG2133_DBG(" drvTP_read_info_dwiic_c33...7\n");
    mdelay ( 100 );

    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while (( reg_data != 0x5B58)&&(deng--) );



    MSG2133_DBG(" drvTP_read_info_dwiic_c33...8\n");
#if 1
    MSG2133_DBG("====for======== drvTP_read_info_dwiic_c33===========\n");
    for(u8i = 0; u8i<4;u8i++)
    {
        dwiic_tx_data[0] = 0x72;	
        dwiic_tx_data[1] = 0x80+u8i;
        dwiic_tx_data[2] = 0x00;
        dwiic_tx_data[3] = 0x01;
        dwiic_tx_data[4] = 0x00;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
        MSG2133_DBG(" drvTP_read_info_dwiic_c33...9\n");
        mdelay ( 50 );

        // recive info data
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[u8i*256], 256 );
    }
    MSG2133_DBG("====for======== drvTP_read_info_dwiic_c33===========end\n");
#else
    dwiic_tx_data[0] = 0x72;	
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
    MSG2133_DBG(" drvTP_read_info_dwiic_c33...9\n");
    mdelay ( 50 );

    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 1024 );
#endif

    MSG2133_DBG(" drvTP_read_info_dwiic_c33...10\n");
    mdelay ( 50 );
    return ( 1 );
}
#endif

#if 0
static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    return ( 1 );
}
#endif

/*static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
  const char *buf, size_t size, EMEM_TYPE_t emem_type )*/
static ssize_t firmware_update_c33 (size_t size, EMEM_TYPE_t emem_type )
{
    u32 i, j;
    u32 crc_main, crc_main_tp;
#ifdef  UPDATE_INFO	
    u32 crc_info, crc_info_tp;
#endif
    int yong=100;
    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
#ifdef  UPDATE_INFO	
    crc_info = 0xffffffff;
    MSG2133_DBG_UPDATE(" firmware_update_c33\n");

    drvTP_read_info_dwiic_c33();
    MSG2133_DBG_UPDATE(" firmware_update_c33-yrj--2\n");
    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

        g_dwiic_info_data[8]=temp[32][8];
        g_dwiic_info_data[9]=temp[32][9];
        g_dwiic_info_data[10]=temp[32][10];
        g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
        g_dwiic_info_data[12]=life_counter[0];
        g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
        drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
            MSG2133_DBG_UPDATE("polling 0x3CE4 is 0x2F43 : reg_data=0x%x\n", reg_data);
        }
        while ( (reg_data != 0x2F43)&&(yong--) );

        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[0], 1024 );

        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
            MSG2133_DBG_UPDATE("polling 0x3CE4 is 0xD0BC : reg_data=0x%x\n", reg_data);
        }
        while ( (reg_data != 0xD0BC )&&(de--));

    }
#endif

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    MSG2133_DBG_UPDATE("*********drvTP_erase_emem_c33 ok ***********\n");
    mdelay ( 500 );

    //ResetSlave();
    msg2133_ts_reset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );
    MSG2133_DBG_UPDATE("%s : ----------22111\n", __func__ );
    /////////////////////////
    // Program
    /////////////////////////
    yong=100;
    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            mdelay ( 10);
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
            MSG2133_DBG_UPDATE("polling 0x3CE4 is 0x1C70 : reg_data=0x%x\n", reg_data);
        }
        while ( (reg_data != 0x1C70)&&(yong--) );
    }
    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }
    MSG2133_DBG_UPDATE("%s : ----------222\n", __func__ );
    yong=100;
    // polling 0x3CE4 is 0x2F43
    do
    {
        mdelay ( 10 );
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        MSG2133_DBG_UPDATE("polling 0x3CE4 is 0x2F43 : reg_data=0x%x\n", reg_data);
    }
    while ( (reg_data != 0x2F43)&&(yong--) );

    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO )
            i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
        #ifdef  UPDATE_INFO
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
	#endif
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        for (j=0; j< 8;j++)
        {
            HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, &temp[i][j*128], 128 );
        }
        //HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );
        yong=100;
        // polling 0x3CE4 is 0xD0BC
        do
        {
            mdelay ( 10 );
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
            MSG2133_DBG_UPDATE("polling 0x3CE4 is 0xD0BC ===2=== : reg_data=0x%x\n", reg_data);
        }
        while ( (reg_data != 0xD0BC)&&(yong--) );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }
    mdelay ( 10 );
    MSG2133_DBG_UPDATE("%s : ----------333\n", __func__ );
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    yong=100;
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            mdelay ( 2 );
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
            MSG2133_DBG_UPDATE("polling 0x3CE4 is 0x9432 ===2=== : reg_data=0x%x\n", reg_data);
        }while ( (reg_data != 0x9432)&&(yong--) );
    }
    MSG2133_DBG_UPDATE("%s : ----------444\n", __func__ );
    crc_main = crc_main ^ 0xffffffff;
#ifdef  UPDATE_INFO	
    crc_info = crc_info ^ 0xffffffff;
#endif
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
#ifdef  UPDATE_INFO
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
#endif
    }
#ifdef  UPDATE_INFO
    pr_info( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
            crc_main, crc_info, crc_main_tp, crc_info_tp );
#else
    pr_info( "crc_main=0x%x,crc_main_tp=0x%x\n", crc_main, crc_main_tp );
#endif
    //drvDB_ExitDBBUS();

    update_pass = 1;
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;
#ifdef  UPDATE_INFO
        if ( crc_info_tp != crc_info )
            update_pass = 0;
#endif		
    }

    if ( !update_pass )
    {
        pr_err( "update FAILED\n" );
        msg2133_ts_reset();
        FwDataCnt = 0;
        enable_irq(this_client->irq);
        return ( 0 );
    }

    pr_info( "update OK\n" );
    msg2133_ts_reset();
    FwDataCnt = 0;
    enable_irq(this_client->irq);
    return size;
}


#if _FW_UPDATE_C3_
#if MSG2133_APK_UPDATE
static ssize_t firmware_update_store ( struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size )
#elif MSG2133_AUTO_UPDATE
static ssize_t firmware_update_store (  size_t size )
#endif
{
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    //msg2133_ts_reset();
    disable_irq(this_client->irq);
    MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store\n");
    msg2133_ts_reset();
    MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store: before Erase TP Flash\n");
    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );
    MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : after Erase TP Flash\n");
    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 1\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 2\n");
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 3\n");
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
    // c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 4\n");
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        MSG2133_DBG_UPDATE ( "dbbus_rx version[0]=0x%x\n", dbbus_rx_data[0] );
        MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 5\n");
        if (dbbus_rx_data[0] == 3 ){
            MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 6\n");
            return firmware_update_c33 ( size, EMEM_MAIN );
            //return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN);
        }else{
            //MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 7\n");
            //return firmware_update_c32 ( dev, attr, buf, size, EMEM_MAIN);
            return firmware_update_c32 ( size, EMEM_ALL );
        }
    }
    else
    {
        //MSG2133_DBG_UPDATE(" _FW_UPDATE_C3_ firmware_update_store : 8\n");
        return firmware_update_c2 ( size );
    } 
}
#else
/*static ssize_t firmware_update_store ( struct device *dev,
  struct device_attribute *attr, const char *buf, size_t size )*/

static ssize_t firmware_update_store (  size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    MSG2133_DBG(" firmware_update_store\n");
    msg2133_ts_reset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    msg2133_ts_reset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    //   MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    //   MSG2133_DBG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;

    return size;
}
#endif

#if MSG2133_APK_UPDATE
static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);
#endif

#if 0
/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    MSG2133_DBG(" +++++++ [%s] Enter!++++++\n", __func__);
    u16 k=0,i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = 0;
    u32 timeOutCount=0;
    for (k = 0; k < 94; i++)   // total  94 KB : 1 byte per R/W
    {
        addr = k * 1024;
        for (j = 0; j < 8; j++)   //128*8 cycle
        {
            bWriteData[2] = (u8)((addr + j * 128) >> 16);
            bWriteData[3] = (u8)((addr + j * 128) >> 8);
            bWriteData[4] = (u8)(addr + j * 128);
            //msctpc_LoopDelay ( 1 );        // delay about 100us*****
            udelay(150);//200);

            timeOutCount=0;
            while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
            {
                timeOutCount++;
                if ( timeOutCount >= 100000 ) 
                    break; /* around 1 sec timeout */
            }

            HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
            //msctpc_LoopDelay ( 1 );        // delay about 100us*****
            udelay(150);//200);
            drvISP_Read(128, RX_data);
            HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
            for (i = 0; i < 128; i++)   //log out if verify error
            {
                if (RX_data[i] != 0xFF)
                {
                    //MSG2133_DBG(MSG2133_DBG("k=%d,j=%d,i=%d===============erase not clean================",k,j,i);)
                    MSG2133_DBG("k=%d,j=%d,i=%d  erase not clean !!",k,j,i);
                }
            }
        }
    }
    MSG2133_DBG("read finish\n");
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{

    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    MSG2133_DBG(" +++++++ [%s] Enter!++++++\n", __func__);
    //msctpc_LoopDelay ( 100 ); 	   // delay about 100ms*****

    // Enable slave's ISP ECO mode

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;

    // Disable the Watchdog
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    MSG2133_DBG(MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
        dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);

    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    MSG2133_DBG(MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
        dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
    //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    MSG2133_DBG(MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
        dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 1,0
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
    MSG2133_DBG(MSG2133_DBG("chip erase+\n");)
        drvISP_BlockErase(0x00000);
    MSG2133_DBG(MSG2133_DBG("chip erase-\n");)
        drvISP_ExitIspMode();
    return size;
}
static DEVICE_ATTR(clear, 0777, firmware_clear_show, firmware_clear_store);
#endif //0
/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/
#if MSG2133_APK_UPDATE
static ssize_t firmware_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    //MSG2133_DBG("*** firmware_version_show fw_version_mstar = %s***\n", fw_version_mstar);
    return sprintf(buf, "%s\n", fw_version_mstar);
}

static ssize_t firmware_version_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;
    /*
       dbbusDWIICEnterSerialDebugMode();
       dbbusDWIICStopMCU();
       dbbusDWIICIICUseBus();
       dbbusDWIICIICReshape();

     */
    fw_version_mstar = kzalloc(sizeof(char), GFP_KERNEL);

    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;//if 2133A--->change to 2A
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    MSG2133_DBG("***major = %d ***\n", major);
    MSG2133_DBG("***minor = %d ***\n", minor);
    sprintf(fw_version_mstar,"%03d%03d", major, minor);
    //MSG2133_DBG("***fw_version_mstar = %s ***\n", fw_version_mstar);

    return size;
}

static DEVICE_ATTR(version, 0777, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
#elif MSG2133_AUTO_UPDATE
static ssize_t firmware_data_store( const char *buf, size_t size)
#endif
{
#if MSG2133_AUTO_UPDATE
    int i = 0;
    for (i = 0; i < 33; i++)
#endif		
    {
#if MSG2133_AUTO_UPDATE
        memcpy(temp[FwDataCnt], &buf[i*1024], 1024);
#elif MSG2133_APK_UPDATE
        memcpy(temp[FwDataCnt], buf, 1024);
#endif
        FwDataCnt++;
        MSG2133_DBG("***  firmware_data_store FwDataCnt = %d ***\n", FwDataCnt);
    }
    return size;
}

#if MSG2133_APK_UPDATE
static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);
#endif

#endif //endif MSG2133_UPDATE
/* xiongyunmei 20130415 add tp msg2133 firmware update end>*/
#if 0//def MSG2133_UPDATE
struct class *firmware_class;
struct device *firmware_cmd_dev;
static int update_switch = 0;
static int FwDataCnt;
static struct fw_version fw_v;
static unsigned char temp[94][1024];

static bool msg2133_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    MSG2133_DBG("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        MSG2133_DBG("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}

static bool msg2133_i2c_write(char *pbt_buf, int dw_lenth)
{
    int ret;
    MSG2133_DBG("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        MSG2133_DBG("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}

static void i2c_read_msg2133(unsigned char *pbt_buf, int dw_lenth)
{
    this_client->addr = MSG2133_FW_ADDR;
    i2c_master_recv(this_client, pbt_buf, dw_lenth);	//0xC5_8bit
    this_client->addr = MSG2133_TS_ADDR;
}

static void i2c_write_msg2133(unsigned char *pbt_buf, int dw_lenth)
{

    this_client->addr = MSG2133_FW_ADDR;
    i2c_master_send(this_client, pbt_buf, dw_lenth);		//0xC4_8bit
    this_client->addr = MSG2133_TS_ADDR;
}

static void i2c_read_update_msg2133(unsigned char *pbt_buf, int dw_lenth)
{	

    this_client->addr = MSG2133_FW_UPDATE_ADDR;
    i2c_master_recv(this_client, pbt_buf, dw_lenth);	//0x93_8bit
    this_client->addr = MSG2133_TS_ADDR;
}

static void i2c_write_update_msg2133(unsigned char *pbt_buf, int dw_lenth)
{	
    this_client->addr = MSG2133_FW_UPDATE_ADDR;
    i2c_master_send(this_client, pbt_buf, dw_lenth);	//0x92_8bit
    this_client->addr = MSG2133_TS_ADDR;
}

void dbbusDWIICEnterSerialDebugMode(void)//Check
{
    unsigned char data[5];
    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    i2c_write_msg2133(data, 5);
}

void dbbusDWIICStopMCU(void)//Check
{
    unsigned char data[1];
    // Stop the MCU
    data[0] = 0x37;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICUseBus(void)//Check
{
    unsigned char data[1];
    // IIC Use Bus
    data[0] = 0x35;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICReshape(void)//Check
{
    unsigned char data[1];
    // IIC Re-shape
    data[0] = 0x71;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICNotUseBus(void)//Check
{
    unsigned char data[1];
    // IIC Not Use Bus
    data[0] = 0x34;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICNotStopMCU(void)//Check
{
    unsigned char data[1];
    // Not Stop the MCU
    data[0] = 0x36;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICExitSerialDebugMode(void)//Check
{
    unsigned char data[1];
    // Exit the Serial Debug Mode
    data[0] = 0x45;
    i2c_write_msg2133(data, 1);
    // Delay some interval to guard the next transaction
}

void drvISP_EntryIspMode(void)//Check
{
    unsigned char bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
    i2c_write_update_msg2133(bWriteData, 5);
    msleep(10);           // delay about 10ms
}

void drvISP_WriteEnable(void)//Check
{
    unsigned char bWriteData[2] =
    {
        0x10, 0x06
    };
    unsigned char bWriteData1 = 0x12;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);
}

void drvISP_ExitIspMode(void)//Check
{
    unsigned char bWriteData = 0x24;
    i2c_write_update_msg2133(&bWriteData, 1);
}


#if 0//Old function
unsigned char drvISP_Read(unsigned char n, unsigned char *pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    unsigned char Read_cmd = 0x11;
    unsigned char i = 0;
    unsigned char dbbus_rx_data[16] = {0};
    i2c_write_update_msg2133(&Read_cmd, 1);
    //if (n == 1)
    {
        i2c_read_update_msg2133(&dbbus_rx_data[0], n + 1);

        for(i = 0; i < n; i++)
        {
            *(pDataToRead + i) = dbbus_rx_data[i + 1];
        }
    }
    //else
    {
        //     i2c_read_update_msg2133(pDataToRead, n);
    }
    return 0;
}
#endif
static unsigned char drvISP_Read(unsigned char n, unsigned char *pDataToRead) //First it needs send 0x11 to notify we want to get flash data back. //Check
{
    unsigned char Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0xFF, 0xFF};
    i2c_write_update_msg2133(&Read_cmd, 1);
    msleep(10);        // delay about 1000us*****
    if ( n == 1 )
    {
        i2c_read_update_msg2133( &dbbus_rx_data[0], 2 );

        // Ideally, the obtained dbbus_rx_data[0~1] stands for the following meaning:
        //  dbbus_rx_data[0]  |  dbbus_rx_data[1]  | status
        // -------------------+--------------------+--------
        //       0x00         |       0x00         |  0x00
        // -------------------+--------------------+--------
        //       0x??         |       0x00         |  0x??
        // -------------------+--------------------+--------
        //       0x00         |       0x??         |  0x??
        //
        // Therefore, we build this field patch to return the status to *pDataToRead.
        *pDataToRead = ( ( dbbus_rx_data[0] >= dbbus_rx_data[1] ) ? \
                dbbus_rx_data[0]  : dbbus_rx_data[1] );
    }
    else
    {
        i2c_read_update_msg2133 ( pDataToRead, n );
    }

    return 0;
}


unsigned char drvISP_ReadStatus(void)//Check
{
    unsigned char bReadData = 0;
    unsigned char bWriteData[2] =
    {
        0x10, 0x05
    };
    unsigned char bWriteData1 = 0x12;
    //    msleep(1);           // delay about 100us
    i2c_write_update_msg2133(bWriteData, 2);
    msleep(1);           // delay about 100us
    drvISP_Read(1, &bReadData);
    //    msleep(10);           // delay about 10ms
    i2c_write_update_msg2133(&bWriteData1, 1);
    return bReadData;
}


void drvISP_SectorErase(unsigned int addr)//This might remove
{
    unsigned char bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned char  bWriteData1 = 0x12;
    printk("The drvISP_ReadStatus0=%d\n", drvISP_ReadStatus());
    drvISP_WriteEnable();
    printk("The drvISP_ReadStatus1=%d\n", drvISP_ReadStatus());
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2133(&bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2133(bWriteData, 3);
    i2c_write_update_msg2133(&bWriteData1, 1);
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1); 
    while((drvISP_ReadStatus() & 0x01) == 0x01);

    drvISP_WriteEnable();
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x20; //Sector Erase
    bWriteData[2] = (( addr >> 16) & 0xFF);
    bWriteData[3] = (( addr >> 8 ) & 0xFF);
    bWriteData[4] = ( addr & 0xFF); 
    i2c_write_update_msg2133(&bWriteData, 5);
    i2c_write_update_msg2133(&bWriteData1, 1);
    while((drvISP_ReadStatus() & 0x01) == 0x01);
}

static void drvISP_BlockErase(unsigned int addr)//This might remove
{
    unsigned char bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned char bWriteData1 = 0x12;
    unsigned int timeOutCount=0;

    drvISP_WriteEnable();
    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);
    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2133(bWriteData, 3);
    i2c_write_update_msg2133(&bWriteData1, 1);
    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);

    timeOutCount=0;
    msleep(1);           // delay about 100us
    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        timeOutCount++;
        if ( timeOutCount > 10000 ) 
            break; /* around 1 sec timeout */
    }

    //pr_ch("The drvISP_ReadStatus3=%d\n", drvISP_ReadStatus());
    drvISP_WriteEnable();
    //pr_ch("The drvISP_ReadStatus4=%d\n", drvISP_ReadStatus());
    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    // bWriteData[4] = (addr & 0xFF) ;
    i2c_write_update_msg2133(bWriteData, 2);
    //i2c_write_update_msg2133( &bWriteData, 5);
    i2c_write_update_msg2133(&bWriteData1, 1);

    timeOutCount=0;
    msleep(1);           // delay about 100us
    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        timeOutCount++;
        if ( timeOutCount > 10000 ) 
            break; /* around 1 sec timeout */
    }
}

static void drvISP_ChipErase(void)//new, check
{
    unsigned char bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned char bWriteData1 = 0x12;
    unsigned int timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2133 ( bWriteData, 2 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2133 ( bWriteData, 3 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2133 ( bWriteData, 2 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );
    msleep(1);        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    i2c_write_update_msg2133 ( bWriteData, 2 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );
    msleep(1);        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}



void drvISP_Program(unsigned short k, unsigned char *pDataToWrite)//Check
{
    unsigned short i = 0;
    unsigned short j = 0;
    //U16 n = 0;
    unsigned char TX_data[133];
    unsigned char bWriteData1 = 0x12;
    unsigned int addr = k * 1024;
    unsigned int timeOutCount = 0; 

    for(j = 0; j < 8; j++)    //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);

        for(i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        msleep(1);        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        drvISP_WriteEnable();
        i2c_write_update_msg2133( TX_data, 133);   //write 133 byte per cycle
        i2c_write_update_msg2133(&bWriteData1, 1);
    }
}

static void auto_drvISP_Program(unsigned char *fw, int size)
{
    unsigned short i = 0;
    unsigned short j = 0;
    unsigned short time = size/128;
    unsigned char TX_data[133];
    unsigned char bWriteData1 = 0x12;
    unsigned int addr;
    for(j = 0; j < time; j++){    //128*8 cycle
        addr = j * 128;
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = addr >> 16;
        TX_data[3] = addr >> 8;
        TX_data[4] = addr;

        for(i = 0; i < 128; i++){
            TX_data[5 + i] = fw[addr + i];
        }

        while((drvISP_ReadStatus() & 0x01) == 0x01)
        {
            ;    //wait until not in write operation
        }

        drvISP_WriteEnable();
        i2c_write_update_msg2133( TX_data, 133);   //write 133 byte per cycle
        i2c_write_update_msg2133(&bWriteData1, 1);
    }
}

static void drvISP_Verify ( unsigned short k, unsigned char* pDataToVerify )//new, check
{
    unsigned short i = 0, j = 0;
    unsigned char bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    unsigned char RX_data[256];
    unsigned char bWriteData1 = 0x12;
    unsigned int addr = k * 1024;
    unsigned char index = 0;
    unsigned int timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( unsigned char ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( unsigned char ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( unsigned char ) ( addr + j * 128 );
        msleep(1);        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        i2c_write_update_msg2133 ( bWriteData, 5 ); //write read flash addr
        msleep(1);        // delay about 100us*****
        drvISP_Read ( 128, RX_data );
        i2c_write_update_msg2133 ( &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                printk( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static void _HalTscrHWReset ( void )//This function must implement by customer
{
#if 0
    gpio_direction_output ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 0 );
    mdelay ( 10 ); /* Note that the RST must be in LOW 10ms at least */
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    /* Enable the interrupt service thread/routine for INT after 50ms */
    mdelay ( 50 );
#endif


}


static ssize_t firmware_update_c2 ( struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size )//new, check
{
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();//this might need check
    _HalTscrHWReset();//this might need implement by customer
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }

    printk ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    return size;
}

static void drvDB_WriteReg ( unsigned char bank, unsigned char addr, unsigned short data )//New. Check
{
    unsigned char tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    i2c_write_msg2133 ( tx_data, 5 );
}

static void drvDB_WriteReg8Bit ( unsigned char bank, unsigned char addr, unsigned char data )//New. Check
{
    unsigned char tx_data[4] = {0x10, bank, addr, data};
    i2c_write_msg2133 ( tx_data, 4 );
}

static unsigned short drvDB_ReadReg ( unsigned char bank, unsigned char addr )//New. Check
{
    unsigned char tx_data[3] = {0x10, bank, addr};
    unsigned char rx_data[2] = {0};

    i2c_write_msg2133 ( tx_data, 3 );
    i2c_read_msg2133 ( &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static unsigned int Reflect ( unsigned int ref, char ch )////New. Check
{
    unsigned int value = 0;
    unsigned int i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

static void Init_CRC32_Table ( unsigned int *crc32_table )//New. Check
{
    unsigned int magicnumber = 0x04c11db7;
    unsigned int i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

unsigned int Get_CRC ( unsigned int text, unsigned int prevCRC, unsigned int *crc32_table )//New. Check
{
    unsigned int ulCRC = prevCRC;
    {
        ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    }
    return ulCRC ;
}


static int drvTP_erase_emem_c32 ( void )//New. Check
{
    /////////////////////////
    //Erase  all
    /////////////////////////

    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size,  EMEM_TYPE_t emem_type )//New, Check
{
    //unsigned char  dbbus_tx_data[4];
    //unsigned char  dbbus_rx_data[2] = {0};
    //unsigned char  Fmr_Loader[1024];   // Buffer for slave's firmware

    unsigned int i, j;
    unsigned int crc_main, crc_main_tp;
    unsigned int crc_info, crc_info_tp;
    unsigned int crc_tab[256];
    unsigned short reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();
    _HalTscrHWReset();//This might implement by customer
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x1C70 );


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        msg2133_i2c_write( temp[i], 1024 );//////////////////////

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    printk ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
            crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        printk ( "update FAILED\n" );
        FwDataCnt = 0;
        return ( 0 );
    }

    printk ( "update OK\n" );
    FwDataCnt = 0;
    return size;

#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )//New, Check
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, unsigned char *p, bool_t set_pce_high )//New, Check
{
    unsigned int i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}

static int drvTP_read_info_dwiic_c33 ( void )//New, Check
{
    unsigned char dwiic_tx_data[5];

    //drvDB_EnterDBBUS();
    _HalTscrHWReset();
    mdelay ( 300 );
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

    mdelay ( 50 );

    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    msg2133_i2c_write ( dwiic_tx_data, 5 );

    mdelay ( 50 );

    // recive info data
    msg2133_i2c_read ( &g_dwiic_info_data[0], 1024 );

    return ( 1 );
}

static int drvTP_info_updata_C33 ( unsigned short start_index, unsigned char *data, unsigned short size )//New, check
{
    // size != 0, start_index+size !> 1024
    unsigned short i;

    for ( i = 0;i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }

    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size, EMEM_TYPE_t emem_type )//New, check
{
    //unsigned char dbbus_tx_data[4];
    //unsigned char  dbbus_rx_data[2] = {0};
    unsigned char  life_counter[2];
    unsigned int i, j;
    unsigned int crc_main, crc_main_tp;
    unsigned int crc_info, crc_info_tp;
    unsigned int crc_tab[256];

    int update_pass = 1;
    unsigned short reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    drvTP_read_info_dwiic_c33();

    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        // drvTP_info_updata_C33 ( 8, temp[32][8], 4 );

        // updata life counter
        life_counter[0] = ( ( (g_dwiic_info_data[12] << 8 )+ g_dwiic_info_data[13] + 1 ) >> 8 ) & 0xFF;
        life_counter[1] = ( (g_dwiic_info_data[12] << 8 )+ g_dwiic_info_data[13] + 1 ) & 0xFF;
        //drvTP_info_updata_C33 ( 10, life_counter, 2 );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );

        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        msg2133_i2c_write ( g_dwiic_info_data, 1024 );

        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

    }


    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block


            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );


    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if( emem_type == EMEM_INFO ) i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        msg2133_i2c_write ( temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }

    printk ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
            crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );
        FwDataCnt = 0;
        return ( size );
    }

    printk ( "update OK\n" );
    FwDataCnt = 0;

    return size;
}

static ssize_t firmware_update_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    printk("tyd-tp: firmware_update_show\n");
    return sprintf(buf, "%03d%03d\n", fw_v.major, fw_v.minor);
}


#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size )//New, check
{
    //unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    _HalTscrHWReset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////

    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx id[0]=0x%x", dbbus_rx_data[0] );

    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        i2c_write_msg2133 ( dbbus_tx_data, 3 );
        i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
        printk ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 )
        {
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
        }
        else
        {
            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
        return firmware_update_c2 ( dev, attr, buf, size );
    }
    _HalTscrHWReset();
    mdelay ( 300 );
}

#else

static ssize_t firmware_update_store ( struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size )//New, check
{
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    _HalTscrHWReset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    prink ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();


    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;

    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    printk ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;

    return size;
}
#endif

static ssize_t firmware_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)//Check
{
    printk("tyd-tp: firmware_version_show\n");
    MSG2133_DBG("*** firmware_version_show fw_version = %03d.%03d.%02d***\n", fw_v.major, fw_v.minor,fw_v.VenderID);
    return sprintf(buf, "%03d%03d%02d\n", fw_v.major, fw_v.minor,fw_v.VenderID);
}

static ssize_t firmware_version_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[5] ;
    int time = 0;

    printk("%s\n", __func__);
    while(time < 10){
        dbbus_tx_data[0] = 0x53;
        dbbus_tx_data[1] = 0x00;
        dbbus_tx_data[2] = 0x74;
        msg2133_i2c_write(&dbbus_tx_data[0], 3);
        mdelay(50);
        msg2133_i2c_read(&dbbus_rx_data[0], 5);
        fw_v.major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
        fw_v.minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
        fw_v.VenderID = (dbbus_rx_data[4]);

        time++;
        if((fw_v.major & 0xff00) == 0)
            break;
        msleep(50);
    }

    return 0;
}


#if 0//Old firmware_update_store function
static ssize_t firmware_update_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    update_switch = 1;
    //drvISP_EntryIspMode();
    //drvISP_BlockErase(0x00000);
    //M by cheehwa _HalTscrHWReset();

    disable_irq_nosync(this_client->irq);

    msg2133_ts_reset();
    //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
    // Enable slave's ISP ECO mode
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    //pr_ch("dbbusDWIICIICReshape\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    // Disable the Watchdog
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //pr_ch("update\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133(dbbus_tx_data, 3);
    i2c_read_msg2133(&dbbus_rx_data[0], 2);
    //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    i2c_write_msg2133(dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133(&dbbus_rx_data[0], 2);
    //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    i2c_write_msg2133(dbbus_tx_data, 4);
    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
    MSG2133_DBG("entryisp\n");

    if(tp_type == TP_HUANGZE){
        drvISP_SectorErase(0x000000);
        drvISP_SectorErase(0x001000);
        drvISP_SectorErase(0x002000);
        drvISP_SectorErase(0x003000);
        drvISP_SectorErase(0x004000);
        drvISP_SectorErase(0x005000);
        drvISP_SectorErase(0x006000);
        drvISP_SectorErase(0x007000);
        drvISP_SectorErase(0x008000);
        drvISP_SectorErase(0x009000);
        drvISP_SectorErase(0x00a000);
        drvISP_SectorErase(0x00b000);
        drvISP_SectorErase(0x00c000);
        drvISP_SectorErase(0x00d000);
        drvISP_SectorErase(0x00e000);
        drvISP_SectorErase(0x00f000);

    }else{
        drvISP_BlockErase(0x00000);
    }  

    //msleep(1000);

    MSG2133_DBG("FwVersion=2");

    for(i = 0; i < 94; i++)     {   // total  94 KB : 1 byte per R/W
        //msleep(1);//delay_100us
        MSG2133_DBG("drvISP_Program\n");
        drvISP_Program(i, temp[i]);    // program to slave's flash
        //pr_ch("drvISP_Verify\n");
        //drvISP_Verify ( i, temp[i] ); //verify data
    }

    //MSG2133_DBG("update OK\n");
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    msg2133_ts_reset();
    MSG2133_DBG("update OK\n");
    update_switch = 0;
    enable_irq(this_client->irq);
    return size;
}
#endif

static ssize_t firmware_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)//Check
{
    printk("tyd-tp: firmware_data_show\n");
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)//Check
{
    int i;
    MSG2133_DBG("***FwDataCnt = %d ***\n", FwDataCnt);
    printk("tyd-tp: firmware_data_store\n");
    for(i = 0; i < 1024; i++){
        memcpy(temp[FwDataCnt], buf, 1024);
    }

    FwDataCnt++;
    return size;
}

static ssize_t firmware_clear_show(struct device *dev,
        struct device_attribute *attr, char *buf)//Check
{
    unsigned short k = 0, i = 0, j = 0;
    unsigned char bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    unsigned char RX_data[256];
    unsigned char bWriteData1 = 0x12;
    unsigned int addr = 0;
    MSG2133_DBG("\n");
    printk("tyd-tp: firmware_clear_show\n");
    for(k = 0; k < 94; i++)    // total  94 KB : 1 byte per R/W
    {
        addr = k * 1024;

        for(j = 0; j < 8; j++)    //128*8 cycle
        {
            bWriteData[2] = (unsigned char)((addr + j * 128) >> 16);
            bWriteData[3] = (unsigned char)((addr + j * 128) >> 8);
            bWriteData[4] = (unsigned char)(addr + j * 128);

            while((drvISP_ReadStatus() & 0x01) == 0x01)
            {
                ;    //wait until not in write operation
            }

            i2c_write_update_msg2133(bWriteData, 5);     //write read flash addr
            drvISP_Read(128, RX_data);
            i2c_write_update_msg2133(&bWriteData1, 1);    //cmd end

            for(i = 0; i < 128; i++)    //log out if verify error{
                if(RX_data[i] != 0xFF){
                    MSG2133_DBG("k=%d,j=%d,i=%d===============erase not clean================", k, j, i);
                }
        }
        }
        MSG2133_DBG("read finish\n");
        return sprintf(buf, "%03d.%03d\n", fw_v.major, fw_v.minor);
    }

    static ssize_t firmware_clear_store(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t size)
    {
        unsigned char dbbus_tx_data[4];
        unsigned char dbbus_rx_data[2] = {0};
        //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
        // Enable slave's ISP ECO mode

        dbbusDWIICEnterSerialDebugMode();
        dbbusDWIICStopMCU();
        dbbusDWIICIICUseBus();
        dbbusDWIICIICReshape();
        MSG2133_DBG("\n");
        printk("tyd-tp: firmware_clear_store\n");
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x08;
        dbbus_tx_data[2] = 0x0c;
        dbbus_tx_data[3] = 0x08;
        // Disable the Watchdog
        i2c_write_msg2133(dbbus_tx_data, 4);
        //Get_Chip_Version();
        //FwVersion  = 2;
        //if (FwVersion  == 2)
        {
            dbbus_tx_data[0] = 0x10;
            dbbus_tx_data[1] = 0x11;
            dbbus_tx_data[2] = 0xE2;
            dbbus_tx_data[3] = 0x00;
            i2c_write_msg2133(dbbus_tx_data, 4);
        }
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0x60;
        dbbus_tx_data[3] = 0x55;
        i2c_write_msg2133(dbbus_tx_data, 4);
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0x61;
        dbbus_tx_data[3] = 0xAA;
        i2c_write_msg2133(dbbus_tx_data, 4);
        //Stop MCU
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x0F;
        dbbus_tx_data[2] = 0xE6;
        dbbus_tx_data[3] = 0x01;
        i2c_write_msg2133(dbbus_tx_data, 4);
        //Enable SPI Pad
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x02;
        i2c_write_msg2133(dbbus_tx_data, 3);
        i2c_read_msg2133(&dbbus_rx_data[0], 2);
        MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
        dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
        i2c_write_msg2133(dbbus_tx_data, 4);
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x25;
        i2c_write_msg2133(dbbus_tx_data, 3);
        dbbus_rx_data[0] = 0;
        dbbus_rx_data[1] = 0;
        i2c_read_msg2133(&dbbus_rx_data[0], 2);
        MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
        dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
        i2c_write_msg2133(dbbus_tx_data, 4);
        //WP overwrite
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x0E;
        dbbus_tx_data[3] = 0x02;
        i2c_write_msg2133(dbbus_tx_data, 4);
        //set pin high
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x10;
        dbbus_tx_data[3] = 0x08;
        i2c_write_msg2133(dbbus_tx_data, 4);
        dbbusDWIICIICNotUseBus();
        dbbusDWIICNotStopMCU();
        dbbusDWIICExitSerialDebugMode();
        ///////////////////////////////////////
        // Start to load firmware
        ///////////////////////////////////////
        drvISP_EntryIspMode();
        MSG2133_DBG("chip erase+\n");

        if(tp_type == TP_HUANGZE){
            drvISP_SectorErase(0x000000);
            drvISP_SectorErase(0x001000);
            drvISP_SectorErase(0x002000);
            drvISP_SectorErase(0x003000);
            drvISP_SectorErase(0x004000);
            drvISP_SectorErase(0x005000);
            drvISP_SectorErase(0x006000);
            drvISP_SectorErase(0x007000);
            drvISP_SectorErase(0x008000);
            drvISP_SectorErase(0x009000);
            drvISP_SectorErase(0x00a000);
            drvISP_SectorErase(0x00b000);
            drvISP_SectorErase(0x00c000);
            drvISP_SectorErase(0x00d000);
            drvISP_SectorErase(0x00e000);
            drvISP_SectorErase(0x00f000);

        }else{
            drvISP_BlockErase(0x00000);
        }  


        MSG2133_DBG("chip erase-\n");
        drvISP_ExitIspMode();
        return size;
    }

    static DEVICE_ATTR(version, 0664, firmware_version_show, firmware_version_store);
    static DEVICE_ATTR(update, 0664, firmware_update_show, firmware_update_store);
    static DEVICE_ATTR(data, 0664, firmware_data_show, firmware_data_store);
    static DEVICE_ATTR(clear, 0664, firmware_clear_show, firmware_clear_store);

    void msg2133_init_fw_class(void)
    {
        firmware_class = class_create(THIS_MODULE,"mtk-tpd" );//client->name

        if(IS_ERR(firmware_class))
            pr_err("Failed to create class(firmware)!\n");

        firmware_cmd_dev = device_create(firmware_class,
                NULL, 0, NULL, "device");//device

        if(IS_ERR(firmware_cmd_dev))
            pr_err("Failed to create device(firmware_cmd_dev)!\n");

        // version /sys/class/mtk-tpd/device/version
        if(device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);

        // update /sys/class/mtk-tpd/device/update
        if(device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);

        // data /sys/class/mtk-tpd/device/data
        if(device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

        // clear /sys/class/mtk-tpd/device/clear
        if(device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);
    }

    static ssize_t msg2133_fw_update(unsigned char *fw, unsigned int size)
    {
        //unsigned char i;
        unsigned char dbbus_tx_data[4];
        unsigned char dbbus_rx_data[2] = {0};

        printk("%s, start update fireware\n", __func__);

        disable_irq_nosync(this_client->irq);

        msg2133_ts_reset();
        //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
        // Enable slave's ISP ECO mode
        dbbusDWIICEnterSerialDebugMode();
        dbbusDWIICStopMCU();
        dbbusDWIICIICUseBus();
        dbbusDWIICIICReshape();
        //pr_ch("dbbusDWIICIICReshape\n");
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x08;
        dbbus_tx_data[2] = 0x0c;
        dbbus_tx_data[3] = 0x08;
        // Disable the Watchdog
        i2c_write_msg2133(dbbus_tx_data, 4);
        //Get_Chip_Version();
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x11;
        dbbus_tx_data[2] = 0xE2;
        dbbus_tx_data[3] = 0x00;
        i2c_write_msg2133(dbbus_tx_data, 4);
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0x60;
        dbbus_tx_data[3] = 0x55;
        i2c_write_msg2133(dbbus_tx_data, 4);
        //pr_ch("update\n");
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0x61;
        dbbus_tx_data[3] = 0xAA;
        i2c_write_msg2133(dbbus_tx_data, 4);
        //Stop MCU
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x0F;
        dbbus_tx_data[2] = 0xE6;
        dbbus_tx_data[3] = 0x01;
        i2c_write_msg2133(dbbus_tx_data, 4);
        //Enable SPI Pad
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x02;
        i2c_write_msg2133(dbbus_tx_data, 3);
        i2c_read_msg2133(&dbbus_rx_data[0], 2);
        //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
        dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
        i2c_write_msg2133(dbbus_tx_data, 4);
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x25;
        i2c_write_msg2133(dbbus_tx_data, 3);
        dbbus_rx_data[0] = 0;
        dbbus_rx_data[1] = 0;
        i2c_read_msg2133(&dbbus_rx_data[0], 2);
        //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
        dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
        i2c_write_msg2133(dbbus_tx_data, 4);

        //WP overwrite
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x0E;
        dbbus_tx_data[3] = 0x02;
        i2c_write_msg2133(dbbus_tx_data, 4);
        //set pin high
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x1E;
        dbbus_tx_data[2] = 0x10;
        dbbus_tx_data[3] = 0x08;
        i2c_write_msg2133(dbbus_tx_data, 4);
        dbbusDWIICIICNotUseBus();
        dbbusDWIICNotStopMCU();
        dbbusDWIICExitSerialDebugMode();
        ///////////////////////////////////////
        // Start to load firmware
        ///////////////////////////////////////
        drvISP_EntryIspMode();
        MSG2133_DBG("entryisp\n");
        if(tp_type == TP_HUANGZE){
            drvISP_SectorErase(0x000000);
            drvISP_SectorErase(0x001000);
            drvISP_SectorErase(0x002000);
            drvISP_SectorErase(0x003000);
            drvISP_SectorErase(0x004000);
            drvISP_SectorErase(0x005000);
            drvISP_SectorErase(0x006000);
            drvISP_SectorErase(0x007000);
            drvISP_SectorErase(0x008000);
            drvISP_SectorErase(0x009000);
            drvISP_SectorErase(0x00a000);
            drvISP_SectorErase(0x00b000);
            drvISP_SectorErase(0x00c000);
            drvISP_SectorErase(0x00d000);
            drvISP_SectorErase(0x00e000);
            drvISP_SectorErase(0x00f000);

        }else{
            drvISP_BlockErase(0x00000);
        }  


        //msleep(1000);
        MSG2133_DBG("FwVersion=2");

        MSG2133_DBG("drvISP_Program\n");
        auto_drvISP_Program(fw, size);


        //MSG2133_DBG("update OK\n");
        drvISP_ExitIspMode();
        msg2133_ts_reset();
        printk("%s, update OK\n", __func__);
        enable_irq(this_client->irq);
        return size;
    }

    static void msg2133_get_version(struct fw_version *fw)
    {
        unsigned char dbbus_tx_data[3];
        unsigned char dbbus_rx_data[5] ;

        printk("%s\n", __func__);
        /*	
                dbbusDWIICEnterSerialDebugMode();
                dbbusDWIICStopMCU();
                dbbusDWIICIICUseBus();
                dbbusDWIICIICReshape();
         */
        MSG2133_DBG("\n");
        dbbus_tx_data[0] = 0x53;
        dbbus_tx_data[1] = 0x00;
        dbbus_tx_data[2] = 0x74;
        msg2133_i2c_write(&dbbus_tx_data[0], 3);
        mdelay(50);
        msg2133_i2c_read(&dbbus_rx_data[0], 5);
        fw->major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
        fw->minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
        fw->VenderID= (dbbus_rx_data[4]);
        printk("%s, major = 0x%x, minor = 0x%x\n,VenderID = 0x%x\n", __func__, fw->major, fw->minor,fw->VenderID);
    }



    static int msg2133_auto_update_fw(void)
    {
        int ret = 0, time=0;
        const struct firmware *fw;
        unsigned char *fw_buf;
        struct platform_device *pdev;
        struct fw_version fw_new, fw_old;


        printk("%s\n", __func__);

        pdev = platform_device_register_simple("msg2133_ts", 0, NULL, 0);
        if (IS_ERR(pdev)) {
            printk("%s: Failed to register firmware\n", __func__);
            return -1;
        }
        printk("%s, platform_device_register_simple\n", __func__);

        if(tp_type == TP_HUANGZE)
            ret = request_firmware(&fw, "huangze_fw.bin", &pdev->dev);
        else if(tp_type == TP_JUNDA)
            ret = request_firmware(&fw, "junda_fw.bin", &pdev->dev);
        else if(tp_type == TP_MUDOND)
            ret = request_firmware(&fw, "mudong_fw.bin", &pdev->dev);
        else
            return -1;

        if (ret) {
            printk("%s: request_firmware error\n",__func__);
            platform_device_unregister(pdev);
            return -1;
        }

        printk("%s, request_firmware\n", __func__);

        platform_device_unregister(pdev);
        printk("%s:fw size=%d\n", __func__,fw->size);

        fw_buf = kzalloc(fw->size, GFP_KERNEL | GFP_DMA);
        memcpy(fw_buf, fw->data, fw->size);

        while(time < 10){
            msg2133_get_version(&fw_old);
            time++;
            if((fw_old.major & 0xff00) == 0)
                break;
            msleep(50);
        }
        printk("%s: time = %d, fw_old.major = 0x%x, fw_old.minor = 0x%x\n", __func__, time, fw_old.major, fw_old.minor);

        fw_new.major = (fw_buf[0x8009] << 8) + fw_buf[0x8008];
        fw_new.minor = (fw_buf[0x800B] << 8) + fw_buf[0x800A];
        printk("%s: fw_new.major = 0x%x, fw_new.minor = 0x%x\n", __func__, fw_new.major, fw_new.minor);

        if((fw_old.major > fw_new.major) || ((fw_old.major == fw_new.major) && (fw_old.minor >= fw_new.minor)))
            return 0;

        printk("%s: update start\n", __func__);

        msg2133_fw_update(fw_buf, fw->size);


        printk("%s: update finish\n", __func__);
        release_firmware(fw);
        kfree(fw_buf);

        return 0;    

    }

#endif //endif MSG2133_UPDATE

#if 0//def MSG2133_PROXIMITY
    static struct input_dev *prox_input;
    static int is_suspend;
    static int last_statue;
    static int prox_enable;
    static int prox_enable_tmp;

    static int prox_i2c_write(char *pbt_buf, int dw_lenth)
    {
        int ret;
        printk("The msg_i2c_client->addr=0x%x\n",this_client->addr);
        ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

        if(ret <= 0)
            printk("msg_i2c_read_interface error\n");

        return ret;
    }
    static void prox_enable_prox(int enable)
    {
        unsigned char  dbbus_tx_data[4];
        //unsigned char dbbus_rx_data[4];
        int ret = -1;

        printk("######%s, enable = %d\n", __func__, enable);	

        if (enable){
            if(tp_type == TP_HUANGZE){
                dbbus_tx_data[0] = 0xFF;
                dbbus_tx_data[1] = 0x11;
                dbbus_tx_data[2] = 0xFF;
                dbbus_tx_data[3] = 0x01;
                disable_irq_nosync(this_client->irq);
                ret = prox_i2c_write(&dbbus_tx_data[0], 4);
                enable_irq(this_client->irq);
            }else if(tp_type == TP_JUNDA || tp_type == TP_MUDOND){
                dbbus_tx_data[0] = 0x52;
                dbbus_tx_data[1] = 0x01;
                dbbus_tx_data[2] = 0x24;
                dbbus_tx_data[3] = 0xA5;
                disable_irq_nosync(this_client->irq);
                ret = prox_i2c_write(&dbbus_tx_data[0], 4);
                enable_irq(this_client->irq);
            }
        }else if (enable == 0){
            if(tp_type == TP_HUANGZE){
                dbbus_tx_data[0] = 0xFF;
                dbbus_tx_data[1] = 0x11;
                dbbus_tx_data[2] = 0xFF;
                dbbus_tx_data[3] = 0x00;
                disable_irq_nosync(this_client->irq);
                ret = prox_i2c_write(&dbbus_tx_data[0], 4);
                enable_irq(this_client->irq);
            }else if(tp_type == TP_JUNDA  || tp_type == TP_MUDOND){
                dbbus_tx_data[0] = 0x52;
                dbbus_tx_data[1] = 0x01;
                dbbus_tx_data[2] = 0x24;
                dbbus_tx_data[3] = 0xA0;
                disable_irq_nosync(this_client->irq);
                ret = prox_i2c_write(&dbbus_tx_data[0], 4);
                enable_irq(this_client->irq);
            }
            //		mdelay(50);
            //		msg2133_ts_reset();

#if MSG2133_ESD

            msg2133_close_flag =0;

#endif
        }

        if(ret > 0)
            prox_enable_tmp = prox_enable = enable;
    }

    static ssize_t prox_enable_show(struct device *dev,
            struct device_attribute *attr, char *buf)
    {
        printk("%s\n", __func__);
        return sprintf(buf, "%d\n", prox_enable);
    }

    static ssize_t prox_enable_store(struct device *dev,
            struct device_attribute *attr,
            const char *buf, size_t count)
    {
        unsigned long enable;

        printk("%s, prox_enable = %d\n", __func__, prox_enable);

        enable = simple_strtoul(buf, NULL, 10);    
        enable = (enable > 0) ? 1 : 0;
        if(is_suspend){
            printk("%s, is_suspend = %ld, enable = %d\n", __func__, is_suspend, enable);
            prox_enable_tmp = enable;
            return count;
        }
        if(enable != prox_enable){  
            prox_enable_prox(enable);
        }
        last_statue = PROX_NORMAL;

        return count;
    }

    static DEVICE_ATTR(enable, S_IRUGO | S_IWUGO,
            prox_enable_show, prox_enable_store);

    static struct attribute *prox_attributes[] = {
        &dev_attr_enable.attr,
        NULL
    };

    static const struct attribute_group prox_attr_group = {
        .attrs = prox_attributes,
    };


    void msg2133_init_prox()
    {
        int ret;
        prox_enable = 0;
        prox_enable_tmp = 0;
        is_suspend = 0;
        tmp_prox = 1;

        prox_input = input_allocate_device();
        if (!prox_input) {
            printk("%s: failed to allocate prox input device\n", __func__);
            return -ENOMEM;
        }

        prox_input->name  = "prox";
        prox_input->id.bustype = BUS_I2C;

        __set_bit(EV_ABS, prox_input->evbit);    
        input_set_abs_params(prox_input, ABS_DISTANCE, 0, 1, 0, 0);

        ret = input_register_device(prox_input);
        if (ret < 0) {
            printk("%s, call input_register_device() error, ret = %d\n",__func__,  ret);
            input_free_device(prox_input);
            return ret;
        }

        ret = sysfs_create_group(&prox_input->dev.kobj, &prox_attr_group);
        if(ret < 0){
            printk("%s, call sysfs_create_group() error, ret = %d\n",__func__,  ret);
            input_unregister_device(prox_input);
            input_free_device(prox_input);
            return ret;
        }

        return 0;
    }

#endif //endif MSG2133_PROXIMITY

#ifdef VIRTUAL_KEYS
    static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)

    {
        return sprintf(buf,
                __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":80:900:64:60\n"
                "" __stringify(EV_KEY) ":" __stringify(/*KEY_HOME*/KEY_HOMEPAGE) ":240:900:64:60\n"
                "" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":420:900:64:60\n");

    }

    static struct kobj_attribute virtual_keys_attr = {
        .attr = {
            .name = "virtualkeys.msg2133a",
            .mode = S_IRUGO,
        },
        .show = &virtual_keys_show,
    };

    static struct attribute *properties_attrs[] = {
        &virtual_keys_attr.attr,
        NULL
    };

    static struct attribute_group properties_attr_group = {
        .attrs = properties_attrs,
    };

    static void virtual_keys_init(void)
    {
        int re=0;
        struct kobject *properties_kobj;

        MSG2133_DBG("%s\n",__func__);

        properties_kobj = kobject_create_and_add("board_properties", NULL);
        if (properties_kobj)
            re = sysfs_create_group(properties_kobj,
                    &properties_attr_group);
        if (!properties_kobj || re)
            pr_err("failed to create board_properties\n");    
    }

#endif

    static unsigned char msg2133_check_sum(unsigned char *pval)
    {
        int i, sum = 0;

        for(i = 0; i < 7; i++)
        {
            sum += pval[i];
        }

        return (unsigned char)((-sum) & 0xFF);
    }

#if AUTO_SWAP_TWO_POINT
    static u32 Distance(u16 X,u16 Y,u16 preX,u16 preY)
    { 
        u32 temp=0;
        temp=(((X-preX)*(X-preX))+((Y-preY)*(Y-preY)));
        return temp;
    }
#endif

#ifdef USE_OLD_REPORT
    static int msg2133_read_data(struct i2c_client *client)
    {

        int ret, keycode;
        unsigned char reg_val[8] = {0};
        int dst_x=0,dst_y=0;
        unsigned int temp_checksum;
        struct msg2133_ts_data *data= i2c_get_clientdata(this_client);
        struct ts_event *event = &data->event;
        static u8 preTouchStatus; //Previous Touch VA Status;

#if AUTO_SWAP_TWO_POINT
        u8 press[2] = {0,0};
        static u8 prepress[2] = {0,0};
        static u8 preTouchNum=0; 
        static u16 preX[2]={0xffff,0xffff},preY[2]={0xffff,0xffff};
        u16 XX[2]={0,0},YY[2]={0,0};
        u16 temp;
        u8 changepoints=0;
#endif

        event->touch_point = 0;
        ret = i2c_master_recv(client, reg_val, 8);
        MSG2133_DBG("%s: ret = %d\n", __func__, ret);
        if(ret <= 0)
            return ret;

        event->x1 =  ((reg_val[1] & 0xF0) << 4) | reg_val[2];	
        event->y1  = ((reg_val[1] & 0x0F) << 8) | reg_val[3];
        dst_x = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
        dst_y = ((reg_val[4] & 0x0F) << 8) | reg_val[6];

        temp_checksum = msg2133_check_sum(reg_val);

        MSG2133_DBG("%s: temp_checksum=0x%x;  reg_val[7]=0x%x; reg_val[0]=0x%x; reg_val[1]=0x%x; reg_val[5]=0x%x\n", 
                __func__, temp_checksum, reg_val[7], reg_val[0], reg_val[1], reg_val[5]);

        if((temp_checksum != reg_val[7]) || (reg_val[0] != 0x52) ){
            return 0;
        }else{
            if (reg_val[1] == 0xFF){
                if((reg_val[5]==0x0)||(reg_val[5]==0xFF)||(preTouchStatus==1)){
                    event->touch_point  = 0; //touch end
                }else{
                    keycode= reg_val[5];
                    if(keycode==1){
                        event->x1 = 80; // final X coordinate
                        event->y1 = 900; // final Y coordinate
                    }else if(keycode==2){
                        event->x1 = 240; // final X coordinate
                        event->y1 = 900; // final Y coordinate
                    }else if(keycode==4){
                        event->x1 = 420; // final X coordinate
                        event->y1 = 900; // final Y coordinate
                    }

                    event->touch_point  = 1;
                }
                preTouchStatus=0;
            }else{
                if ((dst_x == 0) && (dst_y == 0)){
                    event->touch_point  = 1; //one touch
#if AUTO_SWAP_TWO_POINT
                    press[0]=1;
                    press[1]=0;
#endif
                    event->x1 = (event->x1 * TS_WIDTH_MAX) / 2048;
                    event->y1 = (event->y1 * TS_HEIGHT_MAX) / 2048;
                }else{
                    event->touch_point  = 2; //two touch
                    if (dst_x > 2048) {    //transform the unsigh value to sign value
                        dst_x -= 4096;
                    }
                    if (dst_y > 2048){
                        dst_y -= 4096;
                    }
#if AUTO_SWAP_TWO_POINT
                    press[0]=1;
                    press[1]=1;
#endif
                    event->x2 = (event->x1 + dst_x);
                    event->y2 = (event->y1 + dst_y);

                    event->x1 = (event->x1 * TS_WIDTH_MAX) / 2048;
                    event->y1 = (event->y1 * TS_HEIGHT_MAX) / 2048;

                    event->x2 = (event->x2 * TS_WIDTH_MAX) / 2048;
                    event->y2 = (event->y2 * TS_HEIGHT_MAX) / 2048;
                }

#if AUTO_SWAP_TWO_POINT
                if(preTouchStatus==1)
                {

                    XX[0]=event->x1;
                    YY[0]=event->y1;
                    XX[1]=event->x2;
                    YY[1]=event->y2;

                    if(/*(touchData->nFingerNum==1)&&*/(preTouchNum==2))
                    {
                        if(Distance(XX[0],YY[0],preX[0],preY[0])>Distance(XX[0],YY[0],preX[1],preY[1]))
                            changepoints=1;
                    }
                    if((event->touch_point==2)&&(preTouchNum==1))
                    {
                        if(prepress[0]==1)
                        {
                            if(Distance(XX[0],YY[0],preX[0],preY[0])>Distance(XX[1],YY[1],preX[0],preY[0]))
                                changepoints=1;
                        }
                        else
                        {
                            if(Distance(XX[0],YY[0],preX[1],preY[1])<Distance(XX[1],YY[1],preX[1],preY[1]))
                                changepoints=1;
                        }
                    }
                    if((event->touch_point==1)&&(preTouchNum==1))
                    {
                        if(press[0]!=prepress[0])
                            changepoints=1;
                    }
                    if((event->touch_point==2)&&(preTouchNum==2))
                    {
                        //
                    }

                    if(changepoints==1)
                    {
                        temp=press[0];
                        press[0]=press[1];
                        press[1]=temp;

                        temp = event->x1;
                        event->x1= event->x2;
                        event->x2 =temp;

                        temp = event->y1;
                        event->y1= event->y2;
                        event->y2 =temp;
                    }
                }
                //save current status
                prepress[0]=press[0];
                prepress[1]=press[1];
                preX[0] = event->x1;
                preY[0] = event->y1;
                preX[1] = event->x2;
                preY[1] = event->y2;

                preTouchNum=event->touch_point;
                //end of save current status
#endif
                preTouchStatus=1;
            }

            //MSG2133_DBG("----x1=%d, y1=%d,x2=%d, y2=%d\n", event->x1, event->y1, event->x2, event->y2);

            MSG2133_DBG("%s, point = %d\n", __func__, event->touch_point);
            for(i=0;i<2;i++){
                if(event->touch_point & (0x01<<i)){
                    switch(event->touch_point) {
                        case 2:
                            MSG2133_DBG("%s,x2=%d, y2=%d\n", __func__, event->x2, event->y2);
                            input_mt_slot(data->input_dev, i-1);
                            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 2);
                            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 15);
                            input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
                            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
                            input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);

                        case 1:			
                            MSG2133_DBG("%s, x1=%d, y1=%d\n", __func__, event->x1, event->y1);
                            input_mt_slot(data->input_dev, i);
                            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 1);		  
                            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 15);
                            input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
                            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
                            input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
                            break;
                        default:
                            //MSG2133_DBG("==touch_point default =\n");
                            break;	
                    }
                }else{
                    input_mt_slot(data->input_dev, i);
                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                }
            }
            input_sync(data->input_dev);

            return 1;
        }
    }
    /*<begin xionyunmei 20130408 modify tp msg2133 two touchpoint*/
    static void msg2133_report_value(struct i2c_client *client)
    {
        struct msg2133_ts_data *data = i2c_get_clientdata(this_client);
        struct ts_event *event = &data->event;
        int i=0;
        MSG2133_DBG("%s, point = %d\n", __func__, event->touch_point);
        for(i=0;i<2;i++){
            if(event->touch_point & (0x01<<i)){
                switch(event->touch_point) {
                    case 2:
                        MSG2133_DBG("%s,x2=%d, y2=%d\n", __func__, event->x2, event->y2);
                        input_mt_slot(data->input_dev, i-1);
                        input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 2);
                        input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 15);
                        input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
                        input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
                        input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);

                    case 1:			
                        MSG2133_DBG("%s, x1=%d, y1=%d\n", __func__, event->x1, event->y1);
                        input_mt_slot(data->input_dev, i);
                        input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 1);		  
                        input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 15);
                        input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
                        input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
                        input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
                        break;
                    default:
                        //MSG2133_DBG("==touch_point default =\n");
                        break;	
                }
            }else{
                input_mt_slot(data->input_dev, i);
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
            }
        }
        input_sync(data->input_dev);
    }	/*end msg2133_report_value*/
#endif

#ifndef USE_OLD_REPORT
    static void msg21xx_do_work(struct work_struct *work)
    {
        u8 val[8] = {0};
        u8 Checksum = 0;
        u8 i;
        u32 delta_x = 0, delta_y = 0;
        u32 u32X = 0;
        u32 u32Y = 0;
        u8 touchkeycode = 0;
        struct msg2133_ts_data *data = i2c_get_clientdata(this_client);
        static TouchScreenInfo_t *touchData=NULL;
        static u32 preKeyStatus=0;
        u32	result = 0;
        static u8 preTouchStatus; //Previous Touch VA Status;
#if AUTO_SWAP_TWO_POINT
        u8 press[2] = {0,0};
        static u8 prepress[2] = {0,0};
        static u8 preTouchNum=0; 
        static u16 preX[2]={0xffff,0xffff},preY[2]={0xffff,0xffff};
        u16 XX[2]={0,0},YY[2]={0,0};
        u16 temp;
        u8 changepoints=0;
#endif

        if(touchData==NULL){
            touchData = kzalloc(sizeof(TouchScreenInfo_t), GFP_KERNEL);
            if(touchData==NULL){
                pr_err("%s :  touchData alloc failed.\n", __func__);
                return ;
            }
        }

        memset(touchData, 0, sizeof(TouchScreenInfo_t));
        mutex_lock(&msg21xx_mutex);
        result = i2c_master_recv(this_client,  &val[0],  8);
        if (result <0 ) {
            enable_irq(this_client->irq);
            mutex_unlock(&msg21xx_mutex);
            return;
        }
        Checksum = msg2133_check_sum(val); 

        if ((Checksum == val[7]) && (val[0] == 0x52))   //check the checksum  of packet
        {
            u32X = (((val[1] & 0xF0) << 4) | val[2]);         //parse the packet to coordinates
            u32Y = (((val[1] & 0x0F) << 8) | val[3]);
            delta_x = (((val[4] & 0xF0) << 4) | val[5]);
            delta_y = (((val[4] & 0x0F) << 8) | val[6]);

            if ((val[1] == 0xFF) && (val[2] == 0xFF) && (val[3] == 0xFF) && (val[4] == 0xFF) && (val[6] == 0xFF))
            {
                touchData->Point[0].X = 0; // final X coordinate
                touchData->Point[0].Y = 0; // final Y coordinate

                if((val[5]==0x0)||(val[5]==0xFF)||(preTouchStatus==1))// if((val[5]==0x0)||(val[5]==0xFF))
                {
                    touchData->nFingerNum = 0; //touch end
                    touchData->nTouchKeyCode = 0; //TouchKeyMode
                    touchData->nTouchKeyMode = 0; //TouchKeyMode
                }else{
                    touchData->nTouchKeyMode = 1; //TouchKeyMode
                    touchData->nTouchKeyCode = val[5]; //TouchKeyCode
                    touchData->nFingerNum = 1;
                }
                preTouchStatus=0;	  
            }else{
                touchData->nTouchKeyMode = 0; //Touch on screen...
                if ((delta_x == 0) && (delta_y == 0))
                {
                    touchData->nFingerNum = 1; //one touch
                    touchData->Point[0].X = (u32X * TS_WIDTH_MAX) / 2048;
                    touchData->Point[0].Y = (u32Y * TS_HEIGHT_MAX ) / 2048;//1781;

#if AUTO_SWAP_TWO_POINT
                    press[0]=1;
                    press[1]=0;
#endif
                }else{
                    u32 x2, y2;

                    touchData->nFingerNum = 2; //two touch

                    /* Finger 1 */
                    touchData->Point[0].X = (u32X * TS_WIDTH_MAX) / 2048;
                    touchData->Point[0].Y = (u32Y * TS_HEIGHT_MAX ) / 2048;//1781;

                    /* Finger 2 */
                    if (delta_x > 2048)     //transform the unsigh value to sign value
                    {
                        delta_x -= 4096;
                    }
                    if (delta_y > 2048)
                    {
                        delta_y -= 4096;
                    }

                    x2 = (u32)(u32X + delta_x);
                    y2 = (u32)(u32Y + delta_y);

                    touchData->Point[1].X = (x2 * TS_WIDTH_MAX) / 2048;
                    touchData->Point[1].Y = (y2 * TS_HEIGHT_MAX ) /2048;// 1781;
#if AUTO_SWAP_TWO_POINT
                    press[0]=1;
                    press[1]=1;
#endif
                }
#if AUTO_SWAP_TWO_POINT
                if(preTouchStatus==1)
                {
                    for(i=0;i<2;i++)
                    {
                        XX[i]=touchData->Point[i].X;
                        YY[i]=touchData->Point[i].Y;
                    }
                    if(/*(touchData->nFingerNum==1)&&*/(preTouchNum==2))
                    {
                        if(Distance(XX[0],YY[0],preX[0],preY[0])>Distance(XX[0],YY[0],preX[1],preY[1]))
                            changepoints=1;
                    }
                    if((touchData->nFingerNum==2)&&(preTouchNum==1))
                    {
                        if(prepress[0]==1)
                        {
                            if(Distance(XX[0],YY[0],preX[0],preY[0])>Distance(XX[1],YY[1],preX[0],preY[0]))
                                changepoints=1;
                        }
                        else
                        {
                            if(Distance(XX[0],YY[0],preX[1],preY[1])<Distance(XX[1],YY[1],preX[1],preY[1]))
                                changepoints=1;
                        }
                    }
                    if((touchData->nFingerNum==1)&&(preTouchNum==1))
                    {
                        if(press[0]!=prepress[0])
                            changepoints=1;
                    }
                    if((touchData->nFingerNum==2)&&(preTouchNum==2))
                    {}

                    if(changepoints==1)
                    {
                        temp=press[0];
                        press[0]=press[1];
                        press[1]=temp;

                        temp=touchData->Point[0].X;
                        touchData->Point[0].X=touchData->Point[1].X;
                        touchData->Point[1].X=temp;

                        temp=touchData->Point[0].Y;
                        touchData->Point[0].Y=touchData->Point[1].Y;
                        touchData->Point[1].Y=temp;
                    }
                }
                //save current status
                for(i=0;i<2;i++)
                {
                    prepress[i]=press[i];
                    preX[i]=touchData->Point[i].X;
                    preY[i]=touchData->Point[i].Y;
                }
                preTouchNum=touchData->nFingerNum;
                //end of save current status
#endif
                preTouchStatus=1;
            }

            if(touchData->nTouchKeyMode)
            {
                switch(touchData->nTouchKeyCode)
                {
                    case 1:
                        touchkeycode = KEY_MENU;
                        touchData->Point[0].X = 80;
                        touchData->Point[0].Y = 900;
                        break;
                    case 2:
                        touchkeycode = KEY_HOMEPAGE;
                        touchData->Point[0].X = 240;
                        touchData->Point[0].Y = 900;
                        break;				
                    case 4:
                        touchkeycode = KEY_BACK ;
                        touchData->Point[0].X = 420;
                        touchData->Point[0].Y = 900;
                        break;
                        //case 8:
                        //touchkeycode = KEY_SEARCH;
                        //break;
#if 0//CTP_PROXIMITY_FUN
                    case 0x40://leave.....  proximity 
                        //printk(KERN_INFO"------------------proximity leave-\n");
                        ps_state = 1;
                        enable_irq(this_client->irq);
                        mutex_unlock(&msg21xx_mutex);
                        return;
                    case 0x80://close.....
                        //printk(KERN_INFO"------------------proximity close-\n");
                        ps_state = 0;
                        enable_irq(this_client->irq);
                        mutex_unlock(&msg21xx_mutex);
                    default:
                        //printk(KERN_INFO"------------------proximity default-\n");
                        enable_irq(this_client->irq);
                        mutex_unlock(&msg21xx_mutex);
                        return;	

#endif 
                }
                if(preKeyStatus!=touchkeycode)
                {
                    preKeyStatus=touchkeycode;
                    //TP_DEBUG("&&&&&&&&useful key code report touch key code = %d,touchkeycode=%d\n",touchData->nTouchKeyCode,touchkeycode);
#if 0
					input_mt_slot(data->input_dev, 0);
                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 0);		  
                    //input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 15);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_X, touchData->Point[0].X);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_Y, touchData->Point[0].Y);
                    //input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);

                    //=====================
                    //input_report_key(input, touchkeycode, 1);
                    input_sync(data->input_dev);
#else
					//pr_info("%s : touch key code=%d\n", __func__, touchkeycode);
					input_report_key(data->input_dev, touchkeycode, 1);
                    input_sync(data->input_dev);
#endif
                    preTouchStatus=0;
                }
            }else{
                if((touchData->nFingerNum) == 0)   //touch end
                {
                    if(preKeyStatus!=0){
					#if 0	
                        input_mt_slot(data->input_dev, 0);
                        input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);//input_report_key(input, preKeyStatus, 0);
					#else
						//pr_info("%s : preKeyStatus=%d\n", __func__, preKeyStatus);
						input_report_key(data->input_dev, preKeyStatus, 0);
					#endif
                    }else{
                        for(i=0;i<2;i++)
                        {
                            input_mt_slot(data->input_dev, i);
                            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);	
                        }
                    }
                    input_sync(data->input_dev);

                }
                else //touch on screen
                {
                    if(touchData->nFingerNum)
                    {
                        for(i=0;i<2;i++)
                        {
                            input_mt_slot(data->input_dev, i);
                            if(press[i])
                            {
                                //printk(KERN_INFO"report:: u32X=%d,u32Y=%d,i=%d\n",touchData->Point[i].X,touchData->Point[i].Y,i);
                                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, i);	
                                input_report_abs(data->input_dev, ABS_MT_POSITION_X, touchData->Point[i].X);
                                input_report_abs(data->input_dev, ABS_MT_POSITION_Y, touchData->Point[i].Y);
                            }
                            else
                                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);	

                        }
                    }

                    input_sync(data->input_dev);
                }

                preKeyStatus=0; //clear key status..
            }
        }

        enable_irq(this_client->irq);
        mutex_unlock(&msg21xx_mutex);

    }
#endif

    /* xionyunmei 20130408 modify tp msg2133 two touchpoint end>*/
    static void msg2133_ts_pen_irq_work(struct work_struct *work)
    {
        MSG2133_DBG("%s\n", __func__);
#ifdef USE_OLD_REPORT
        if (msg2133_read_data(this_client)) {	
            msg2133_report_value(this_client);
        }
        enable_irq(this_client->irq);
#else
        msg21xx_do_work(work);
#endif

    }

    static irqreturn_t msg2133_ts_interrupt(int irq, void *dev_id)
    {
        unsigned long flags;
        struct msg2133_ts_data *msg2133_ts = (struct msg2133_ts_data *)dev_id;

        spin_lock_irqsave(&prox_lock, flags);
        if(tmp_prox == 0){
            spin_unlock_irqrestore(&prox_lock, flags);
            return IRQ_HANDLED;
        }
        spin_unlock_irqrestore(&prox_lock, flags);


        MSG2133_DBG("%s\n", __func__);
        disable_irq_nosync(this_client->irq);
        if (!work_pending(&msg2133_ts->pen_event_work)) {
            queue_work(msg2133_ts->ts_workqueue, &msg2133_ts->pen_event_work);
        }

        return IRQ_HANDLED;
    }

    static int msg2133a_power_init(struct msg2133_ts_data *data, bool on)
    {
        int rc;

        if (!on)
            goto pwr_deinit;

        if(data->client == NULL) return -1;
        data->vdd = regulator_get(&data->client->dev, "vdd");
        if (IS_ERR(data->vdd)) {
            rc = PTR_ERR(data->vdd);
            dev_err(&data->client->dev, "Regulator get failed vdd rc=%d\n", rc);
            return rc;
        }
        if (regulator_count_voltages(data->vdd) > 0) {
            rc = regulator_set_voltage(data->vdd, VTG_MIN_UV, VTG_MAX_UV);
            if (rc) {
                dev_err(&data->client->dev, "Regulator set_vtg failed vdd rc=%d\n", rc);
                goto reg_vdd_put;
            }
        }

        data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
        if (IS_ERR(data->vcc_i2c)) {
            rc = PTR_ERR(data->vcc_i2c);
            dev_err(&data->client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
            goto reg_vdd_set_vtg;
        }

        if (regulator_count_voltages(data->vcc_i2c) > 0) {
            rc = regulator_set_voltage(data->vcc_i2c, I2C_VTG_MIN_UV, I2C_VTG_MAX_UV);
            if (rc) {
                dev_err(&data->client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
                goto reg_vcc_i2c_put;
            }
        }
        return 0;

reg_vcc_i2c_put:
        regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
        if (regulator_count_voltages(data->vdd) > 0)
            regulator_set_voltage(data->vdd, 0, VTG_MAX_UV);
reg_vdd_put:
        regulator_put(data->vdd);
        return rc;

pwr_deinit:
        if (regulator_count_voltages(data->vdd) > 0)
            regulator_set_voltage(data->vdd, 0, VTG_MAX_UV);

        regulator_put(data->vdd);

        if (regulator_count_voltages(data->vcc_i2c) > 0)
            regulator_set_voltage(data->vcc_i2c, 0, I2C_VTG_MAX_UV);

        regulator_put(data->vcc_i2c);
        return 0;
    }

    static int msg2133a_power_on(struct msg2133_ts_data *data, bool on)
    {
        int rc;

        if (!on)
            goto power_off;
        rc = regulator_enable(data->vdd);
        if (rc) {
            dev_err(&data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
            return rc;
        }
        mdelay(10);
        rc = regulator_enable(data->vcc_i2c);
        if (rc) {
            dev_err(&data->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
            regulator_disable(data->vdd);
        }
        mdelay(10);

        return rc;

power_off:
        rc = regulator_disable(data->vdd);
        if (rc) {
            dev_err(&data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
            return rc;
        }

        rc = regulator_disable(data->vcc_i2c);
        if (rc) {
            dev_err(&data->client->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
            regulator_enable(data->vdd);
        }
        return rc;
    }

    static int msg2133_ts_suspend(struct device *dev)
    {
        struct msg2133_ts_data *data = dev_get_drvdata(dev);
        char  i;

        MSG2133_DBG("%s : enter\n", __func__);

        disable_irq(data->client->irq);

        /* release all touches */
        for (i = 0; i < data->event.touch_point; i++) {
            input_mt_slot(data->input_dev, i);
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
        }
        input_mt_report_pointer_emulation(data->input_dev, false);
        input_sync(data->input_dev);

		//input_report_key(data->input_dev, BTN_TOUCH, 0);
		//input_sync(data->input_dev);
		
        msg2133a_power_on(data, false);
        gpio_set_value(init_reset, 0);
        data->suspended = true;

        return 0;
    }

    static int msg2133_ts_resume(struct device *dev)
    {
        struct msg2133_ts_data *data = dev_get_drvdata(dev);

        MSG2133_DBG("%s : enter\n", __func__);

        if (!data->suspended) {
            dev_dbg(dev, "Already in awake state\n");
            return 0;
        }
        msg2133a_power_on(data, true);
        msg2133_ts_reset();
        enable_irq(data->client->irq);
        data->suspended = false;

        return 0;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    static void msg2133_ts_early_suspend(struct early_suspend *handler)
    {

        MSG2133_DBG("%s : prox_enable=%d\n", __func__, prox_enable);	
#if 1
        struct msg2133_ts_data *msg2133_ts = container_of(handler, struct msg2133_ts_data, early_suspend);
        msg2133_ts_suspend(&(msg2133_ts->client->dev));
#else
#if 0 //def MSG2133_PROXIMITY
        if(prox_enable){
            MSG2133_DBG(" suspend prox_enable ==1,return \n");
            return;
        }
#endif
        disable_irq_nosync(this_client->irq);
        msleep(3);
        gpio_set_value(init_reset, 0);
        //msg2133_device_power_off();

#if 0 //def MSG2133_PROXIMITY
        is_suspend = 1;
#endif
#endif
    }

    static void msg2133_ts_late_resume(struct early_suspend *handler)
    {	
        MSG2133_DBG("%s :  enter\n", __func__);
#if 1
        struct msg2133_ts_data *msg2133_ts = container_of(handler, struct msg2133_ts_data, early_suspend);
        msg2133_ts_resume(&(msg2133_ts->client->dev));
#else
#if 0 //def MSG2133_PROXIMITY
        if(prox_enable){
            MSG2133_DBG("msg2133_ts_resume: start check incall timer \n");
            msg2133_check_incall_timer_start();
            return;
        }
#endif

        msg2133_ts_reset();
        enable_irq(this_client->irq);

#if 0 //def MSG2133_PROXIMITY
        is_suspend = 0;
        if(prox_enable == 0 && prox_enable_tmp){
            printk("%s, prox_enable_tmp = %d\n", __func__, prox_enable_tmp);
            msleep(700);
            prox_enable_prox(prox_enable_tmp);
        }

#endif
#endif
    }
#endif

#ifdef CONFIG_PM
    static int fb_notifier_callback(struct notifier_block *self,
            unsigned long event, void *data)
    {
        int *blank;
        struct fb_event *evdata = data;	
        struct msg2133_ts_data *msg2133_data = container_of(self, struct msg2133_ts_data, fb_notif);

        MSG2133_DBG("%s : enter\n", __func__);

        if (evdata && evdata->data && event == FB_EVENT_BLANK &&
                msg2133_data && msg2133_data->client) {
            blank = evdata->data;
            if (*blank == FB_BLANK_UNBLANK)
                msg2133_ts_resume(&msg2133_data->client->dev);
            else if (*blank == FB_BLANK_POWERDOWN)
                msg2133_ts_suspend(&msg2133_data->client->dev);
        }

        return 0;
    }
#endif

    static int ver_msg2133_i2c_read(char *pbt_buf, int dw_lenth)
    {
        int ret;
        //MSG2133_DBG("The msg_i2c_client->addr=0x%x\n",this_client->addr);
        ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

        if(ret <= 0)
            MSG2133_DBG("msg_i2c_read_interface error\n");
        return ret;
    }

    static int ver_msg2133_i2c_write(char *pbt_buf, int dw_lenth)
    {
        int ret;
        //MSG2133_DBG("The msg_i2c_client->addr=0x%x\n",this_client->addr);
        ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

        if(ret <= 0)
            MSG2133_DBG("ver_msg2133_i2c_write : msg_i2c_read_interface error\n");

        return ret;
    }

    static int ver_msg2133_get_version(struct fw_version *fw)
    {
        int ret;
        unsigned char dbbus_tx_data[3];
        unsigned char dbbus_rx_data[11] = {0};

        dbbus_tx_data[0] = 0x53;
        dbbus_tx_data[1] = 0x00;
        dbbus_tx_data[2] = 0x2a;//0x74
        ret = ver_msg2133_i2c_write(&dbbus_tx_data[0], 3);
        if(ret <= 0){
            pr_err("%s : write error ret=%d\n", __func__, ret);
            return -1;
        }

        mdelay(50);
        ret = ver_msg2133_i2c_read(&dbbus_rx_data[0], 10);
        if(ret <= 0){
            fw->major = fw->minor = 0;
            pr_err("%s : read error ret=%d\n", __func__, ret);
            return -1;
        }
        fw->major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
        fw->minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
        fw->VenderID=dbbus_rx_data[4];
        MSG2133_DBG("%s, major = 0x%x, minor = 0x%x\n,VenderID = 0x%x\n", __func__, fw->major, fw->minor,fw->VenderID);
        //printk("dbbus_rx_data[5]=0x%x,dbbus_rx_data[6]=0x%x,dbbus_rx_data[7]=0x%x,dbbus_rx_data[8]=0x%x,dbbus_rx_data[9]=0x%x,dbbus_rx_data[10]=0x%x\n",dbbus_rx_data[5],dbbus_rx_data[6],dbbus_rx_data[7],dbbus_rx_data[8],dbbus_rx_data[9],dbbus_rx_data[10]);
        return 0;
    }
    /*<begin xiongyunmei 2013-04-07 add tpinfo with vendor id */

#define OFILM_MSG2133_ID		0xc
#define HUANGZE_MSG2133_ID	0x78
#define TP_INFO1		"Ofilm-msg2133"
#define TP_INFO2		"Huangze-msg2133"
    static int read_tpinfo(char *buf, char **start, off_t offset, int count,int *eof, void *data)
    {		
        int len=0,time=0;
        struct fw_version fw;
        while(time < 10){
            ver_msg2133_get_version(&fw);
            time++;
            if((fw.major & 0xff00) == 0)
                break;
            msleep(50);
        }
        MSG2133_DBG(" %s: vendor_id = 0x%x \n",__func__,fw.VenderID);
        switch (fw.VenderID){
            case OFILM_MSG2133_ID:
                len = sprintf(buf,"%s\n",TP_INFO1);
                break;

            case HUANGZE_MSG2133_ID:
                len = sprintf(buf,"%s\n",TP_INFO2);
                break;
            default:
                break;		
        }	

        return len;
    }

#ifdef CONFIG_OF
#if 0
    static int msg2133a_get_dt_coords(struct device *dev, char *name,
            struct msg2133_i2c_platform_data *pdata)
    {
        u32 coords[FT_COORDS_ARR_SIZE];
        struct property *prop;
        struct device_node *np = dev->of_node;
        int coords_size, rc;

        prop = of_find_property(np, name, NULL);
        if (!prop)
            return -EINVAL;
        if (!prop->value)
            return -ENODATA;

        coords_size = prop->length / sizeof(u32);
        if (coords_size != FT_COORDS_ARR_SIZE) {
            dev_err(dev, "invalid %s\n", name);
            return -EINVAL;
        }

        rc = of_property_read_u32_array(np, name, coords, coords_size);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read %s\n", name);
            return rc;
        }

        if (!strcmp(name, "mstar,panel-coords")) {
            pdata->panel_minx = coords[0];
            pdata->panel_miny = coords[1];
            pdata->panel_maxx = coords[2];
            pdata->panel_maxy = coords[3];
        } else if (!strcmp(name, "mstar,display-coords")) {
            pdata->x_min = coords[0];
            pdata->y_min = coords[1];
            pdata->x_max = coords[2];
            pdata->y_max = coords[3];
        } else {
            dev_err(dev, "unsupported property %s\n", name);
            return -EINVAL;
        }

        return 0;
    }
#endif
    static int msg2133a_parse_dt(struct device *dev, struct msg2133_i2c_platform_data *pdata)
    {
        //	int rc;
        struct device_node *np = dev->of_node;
#if 0	
        struct property *prop;
        u32 temp_val, num_buttons;
        u32 button_map[MAX_BUTTONS];

        pdata->name = "mstar";
        rc = of_property_read_string(np, "mstar,name", &pdata->name);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read name\n");
            return rc;
        }

        rc = ft5x06_get_dt_coords(dev, "mstar,panel-coords", pdata);
        if (rc && (rc != -EINVAL))
            return rc;

        rc = ft5x06_get_dt_coords(dev, "mstar,display-coords", pdata);
        if (rc)
            return rc;

        pdata->i2c_pull_up = of_property_read_bool(np, "mstar,i2c-pull-up");

        pdata->no_force_update = of_property_read_bool(np, "mstar,no-force-update");
#endif	
        /* reset, irq gpio info */
        pdata->reset_gpio = of_get_named_gpio_flags(np, "mstar,reset-gpio", 0, &pdata->reset_gpio_flags);
        if (pdata->reset_gpio < 0){
            return pdata->reset_gpio;
        }

        pdata->irq_gpio = of_get_named_gpio_flags(np, "mstar,irq-gpio", 0, &pdata->irq_gpio_flags);
        if (pdata->irq_gpio < 0){
            return pdata->irq_gpio;
        }

#if 0
        pdata->fw_name = "ft_fw.bin";
        rc = of_property_read_string(np, "mstar,fw-name", &pdata->fw_name);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read fw name\n");
            return rc;
        }

        rc = of_property_read_u32(np, "mstar,group-id", &temp_val);
        if (!rc){
            pdata->group_id = temp_val;
        }else{
            return rc;
        }

        rc = of_property_read_u32(np, "mstar,hard-reset-delay-ms", &temp_val);
        if (!rc)
            pdata->hard_rst_dly = temp_val;
        else
            return rc;

        rc = of_property_read_u32(np, "mstar,soft-reset-delay-ms", &temp_val);
        if (!rc)
            pdata->soft_rst_dly = temp_val;
        else
            return rc;

        rc = of_property_read_u32(np, "mstar,num-max-touches", &temp_val);
        if (!rc)
            pdata->num_max_touches = temp_val;
        else
            return rc;

        rc = of_property_read_u32(np, "mstar,fw-delay-aa-ms", &temp_val);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read fw delay aa\n");
            return rc;
        } else if (rc != -EINVAL)
            pdata->info.delay_aa =  temp_val;

        rc = of_property_read_u32(np, "mstar,fw-delay-55-ms", &temp_val);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read fw delay 55\n");
            return rc;
        } else if (rc != -EINVAL)
            pdata->info.delay_55 =  temp_val;

        rc = of_property_read_u32(np, "mstar,fw-upgrade-id1", &temp_val);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read fw upgrade id1\n");
            return rc;
        } else if (rc != -EINVAL)
            pdata->info.upgrade_id_1 =  temp_val;

        rc = of_property_read_u32(np, "mstar,fw-upgrade-id2", &temp_val);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read fw upgrade id2\n");
            return rc;
        } else if (rc != -EINVAL)
            pdata->info.upgrade_id_2 =  temp_val;

        rc = of_property_read_u32(np, "mstar,fw-delay-readid-ms",
                &temp_val);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read fw delay read id\n");
            return rc;
        } else if (rc != -EINVAL)
            pdata->info.delay_readid =  temp_val;

        rc = of_property_read_u32(np, "mstar,fw-delay-era-flsh-ms",
                &temp_val);
        if (rc && (rc != -EINVAL)) {
            dev_err(dev, "Unable to read fw delay erase flash\n");
            return rc;
        } else if (rc != -EINVAL)
            pdata->info.delay_erase_flash =  temp_val;

        pdata->info.auto_cal = of_property_read_bool(np, "mstar,fw-auto-cal");

        pdata->fw_vkey_support = of_property_read_bool(np, "mstar,fw-vkey-support");

        pdata->ignore_id_check = of_property_read_bool(np, "mstar,ignore-id-check");

        rc = of_property_read_u32(np, "mstar,family-id", &temp_val);
        if (!rc)
            pdata->family_id = temp_val;
        else
            return rc;

        prop = of_find_property(np, "mstar,button-map", NULL);
        if (prop) {
            num_buttons = prop->length / sizeof(temp_val);
            if (num_buttons > MAX_BUTTONS)
                return -EINVAL;

            rc = of_property_read_u32_array(np,
                    "mstar,button-map", button_map,
                    num_buttons);
            if (rc) {
                dev_err(dev, "Unable to read key codes\n");
                return rc;
            }
        }
#endif
        return 0;
    }
#else
    static int msg2133a_parse_dt(struct device *dev,
            struct msg2133_i2c_platform_data *pdata)
    {
        return -ENODEV;
    }
#endif

    static int msg2133_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
    {
        struct msg2133_ts_data *msg2133_ts;
        struct input_dev *input_dev;
        struct msg2133_i2c_platform_data *pdata;
        struct fw_version fw_old;

#if MSG2133_AUTO_UPDATE
        struct fw_version fw_new;
#endif

        struct gpiomux_setting old_gpio_setting;	
        int err = 0;
        int time = 0;

        printk("%s\n", __func__);

        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
            err = -ENODEV;
            dev_err(&client->dev, "i2c_check_functionality failed.\n");
            return err;
        }

        this_client = client;

        if (client->dev.of_node) {
            pdata = devm_kzalloc(&client->dev, sizeof(struct msg2133_i2c_platform_data), GFP_KERNEL);
            if (!pdata) {
                dev_err(&client->dev, "Failed to allocate memory\n");
                return -ENOMEM;
            }

            err = msg2133a_parse_dt(&client->dev, pdata);
            if (err) {
                dev_err(&client->dev, "DT parsing failed\n");
                return err;
            }
        } else{
            pdata = client->dev.platform_data;
        }
        if (pdata) {
            //msg2133_ts->power = pdata->power;
            init_irq = pdata->irq_gpio;
            init_reset=pdata->reset_gpio;
        }else{
            pr_err("%s : pdata is NULL.\n", __func__);
        }
        printk("%s : I2C addr=%x,client->irq=%d,init_irq=%d,init_reset=%d\n", 
                __func__, client->addr,client->irq,init_irq,init_reset);	

        msg2133_ts = kzalloc(sizeof(*msg2133_ts), GFP_KERNEL);
        if (!msg2133_ts)	{
            err = -ENOMEM;
            dev_err(&client->dev, "alloc for msg2133_ts failed.\n");
            return err;
        }
        i2c_set_clientdata(client, msg2133_ts); 
        msg2133_ts->client = client;
#ifndef USE_OLD_REPORT
        mutex_init(&msg21xx_mutex);
#endif
        INIT_WORK(&msg2133_ts->pen_event_work, msg2133_ts_pen_irq_work);

        msg2133a_power_init(msg2133_ts, true);
        msg2133a_power_on(msg2133_ts, true);
        spin_lock_init(&prox_lock);
        msg2133_ts_reset();
        fw_old.major = fw_old.minor = 0;
        while(time < 2){
            if(!ver_msg2133_get_version(&fw_old)){
                break;
            }
            time++;
            //if((fw_old.major & 0xff00) == 0)
            //	break;
            msleep(50);
        }
        printk("%s, time = %d, fw_old.version = [0x%x].[0x%x]\n", __func__, time, fw_old.major, fw_old.minor);   
		if(time == 2){
			printk("%s : time = %d, skip_update_fw, not update FW.\n", __func__, time);
			goto skip_update_fw;
		}
			
        /*<begin xiongyunmei 20130415  add tp msg2133 firmware update*/
#if MSG2133_AUTO_UPDATE
        //msg2133_auto_update_fw();//huangze
        fw_new.major =(unsigned short) ((GroupData[0x7f4f]<<8)|GroupData[0x7f4e]);
        fw_new.minor = (unsigned short)((GroupData[0x7f51]<<8)|GroupData[0x7f50]);
        printk("%s, fw_new.version = [0x%x].[0x%x]\n", __func__, fw_new.major, fw_new.minor);
        if((fw_old.major < fw_new.major) || ((fw_old.major == fw_new.major) && (fw_old.minor < fw_new.minor)))
        {
            firmware_data_store(GroupData,sizeof(GroupData));
            firmware_update_store(sizeof(GroupData));
            msleep(300);
            msg2133_ts_reset();	
        }else{
            MSG2133_DBG("msg2133a_ts_probe : fw no need to be updated.\n");
        }
#endif
        /* xiongyunmei 20130415  add tp msg2133 firmware update end>*/
skip_update_fw:

#if MSG2133_ESD
        msg2133_esd_resume_workqueue = create_singlethread_workqueue("msg2133_esd_resume");
        INIT_WORK(&resume_work, msg2133_esd_resume_callback);
#endif
        msg2133_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
        if (!msg2133_ts->ts_workqueue) {
            err = -ESRCH;
            goto exit_create_singlethread;
        }
        gpio_request(init_irq, "tp_int_gpio");
#if 0
        gpio_tlmm_config(GPIO_CFG(init_irq, 0, GPIO_CFG_INPUT, \
                    GPIO_CFG_PULL_UP, GPIO_CFG_4MA) , GPIO_CFG_ENABLE);
        //gpio_free(init_irq);
#else
        if (msm_gpiomux_write(init_irq, GPIOMUX_ACTIVE,
                    &mstar_int_act_cfg, &old_gpio_setting)) {
            pr_err("%s : init_irq GPIO pins have no active setting\n", __func__);
        }
#endif

        input_dev = input_allocate_device();
        if (!input_dev) {
            err = -ENOMEM;
            printk("%s: failed to allocate input device\n", __func__);
            goto exit_input_dev_alloc_failed;
        }
        msg2133_ts->input_dev = input_dev;	

        __set_bit(EV_ABS, input_dev->evbit);
        __set_bit(EV_KEY, input_dev->evbit);
        __set_bit(EV_SYN, input_dev->evbit);

        __set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
        __set_bit(ABS_MT_POSITION_X, input_dev->absbit);
        __set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
        __set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

#ifdef VIRTUAL_KEYS
        __set_bit(KEY_MENU,  input_dev->keybit);
        __set_bit(KEY_BACK,  input_dev->keybit);
        __set_bit(KEY_HOMEPAGE,  input_dev->keybit);
        __set_bit(KEY_SEARCH,  input_dev->keybit);
#else
		__set_bit(KEY_MENU,  input_dev->keybit);
        __set_bit(KEY_BACK,  input_dev->keybit);
        __set_bit(KEY_HOMEPAGE,  input_dev->keybit);
#endif
        /*<begin 2013-03-25 xiongyunmei add tp msg2133*/
        __set_bit(INPUT_PROP_DIRECT, msg2133_ts->input_dev->propbit);
        input_mt_init_slots(msg2133_ts->input_dev, 255);
        /* 2013-03-25 xiongyunmei add tp msg2133 end>*/
        input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TS_WIDTH_MAX, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TS_HEIGHT_MAX, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
        input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

        input_dev->name = MSG2133_TS_NAME;
        err = input_register_device(input_dev);
        if (err) {
            dev_err(&client->dev, "msg2133_ts_probe: failed to register input device: %s\n",
                    dev_name(&client->dev));
            goto exit_input_register_device_failed;
        }

        if(client->irq){
            err = request_irq(client->irq, msg2133_ts_interrupt, IRQF_TRIGGER_FALLING, client->name, msg2133_ts);
            if (err < 0) {
                printk("%s: msg2133_probe: request irq failed\n", __func__);
                goto exit_request_irq_failed;
            }
        }else{
            err = -1;
            printk("%s: tp irq must > 0\n", __func__);
            goto exit_request_irq_failed;
        }
        //disable_irq(client->irq);
#if defined(CONFIG_FB)
        msg2133_ts->fb_notif.notifier_call = fb_notifier_callback;
        err = fb_register_client(&msg2133_ts->fb_notif);
        if (err){
            dev_err(&client->dev, "Unable to register fb_notifier: %d\n", err);
        }
#elif defined(CONFIG_HAS_EARLYSUSPEND)
        MSG2133_DBG("register_early_suspend");
        msg2133_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FT_SUSPEND_LEVEL;
        msg2133_ts->early_suspend.suspend = msg2133_ts_early_suspend;
        msg2133_ts->early_suspend.resume	= msg2133_ts_late_resume;
        register_early_suspend(&msg2133_ts->early_suspend);
#endif

#if 0 //def MSG2133_SOFTWARE_UPDATE
        msg2133_init_fw_class();
#endif

#ifdef VIRTUAL_KEYS
        virtual_keys_init();
#endif

#if 0//def MSG2133_PROXIMITY
        msg2133_init_prox();
#endif

        //enable_irq(client->irq);
        /*<begin xiongyunmei 2013-04-07  add tpinfo with vender id*/
        create_proc_read_entry("tpinfo", 0, 0, read_tpinfo, NULL);
        /* xiongyunmei 2013-04-07 add tpinfo with vender id end>*/
        /*<begin xiongyunmei 20130415  add tp msg2133 firmware update*/
#if MSG2133_APK_UPDATE
        firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
        if (IS_ERR (firmware_class))
            pr_err("Failed to create class(firmware)!\n");

        firmware_cmd_dev = device_create(firmware_class, NULL, 0, NULL, "device");
        if (IS_ERR (firmware_cmd_dev))
            pr_err("Failed to create device(firmware_cmd_dev)!\n");

        //version
        if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);

        //update
        if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);

        //data
        if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

        //clear
        //  if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
        //  pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);

        dev_set_drvdata(firmware_cmd_dev, NULL);
#endif
        /* xiongyunmei 20130415  add tp msg2133 firmware update end>*/

        return 0;

exit_request_irq_failed:
        MSG2133_DBG("%s : exit_request_irq_failed\n", __func__);
        input_unregister_device(input_dev);
exit_input_register_device_failed:
        MSG2133_DBG("%s : exit_input_register_device_failed\n", __func__);
        input_free_device(input_dev);
exit_input_dev_alloc_failed:
        MSG2133_DBG("%s : exit_input_dev_alloc_failed\n", __func__);
        cancel_work_sync(&msg2133_ts->pen_event_work);
        destroy_workqueue(msg2133_ts->ts_workqueue);
exit_create_singlethread:
        MSG2133_DBG("%s : exit_create_singlethread\n", __func__);
        i2c_set_clientdata(client, NULL);
        kfree(msg2133_ts);
        gpio_free(client->irq);
        return err;
    }


    static int __devexit msg2133_ts_remove(struct i2c_client *client)
    {

        struct msg2133_ts_data *msg2133_ts = i2c_get_clientdata(client);

        MSG2133_DBG("%s : enter\n", __func__);

        unregister_early_suspend(&msg2133_ts->early_suspend);
        free_irq(client->irq, msg2133_ts);
        gpio_free(client->irq);
        input_unregister_device(msg2133_ts->input_dev);
        kfree(msg2133_ts);
        cancel_work_sync(&msg2133_ts->pen_event_work);
        destroy_workqueue(msg2133_ts->ts_workqueue);
        i2c_set_clientdata(client, NULL);

        //msg2133_device_power_off(); xym del
        return 0;
    }



    static const struct i2c_device_id msg2133_ts_id[] = {
        { MSG2133_TS_NAME, 0 },{ }
    };

    static const struct dev_pm_ops msg2133a_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
        .suspend = msg2133_ts_suspend,
        .resume = msg2133_ts_resume,
#endif
    };

    MODULE_DEVICE_TABLE(i2c, msg2133_ts_id);

    static struct i2c_driver msg2133_ts_driver = {
        .probe		= msg2133_ts_probe,
        .remove		= __devexit_p(msg2133_ts_remove),
        .id_table	= msg2133_ts_id,
        .driver	= {
            .name	= "msg2133a",
            .owner	= THIS_MODULE,
#ifdef CONFIG_PM
            .pm = &msg2133a_ts_pm_ops,
#endif		
        },
    };

    static int __init msg2133_init_module(void)
    {
        printk("%s\n", __func__);
        return i2c_add_driver(&msg2133_ts_driver);
    }

    static void __exit msg2133_exit_module(void)
    {
        MSG2133_DBG("%s\n", __func__);
        i2c_unregister_device(this_client);
        i2c_del_driver(&msg2133_ts_driver);
    }


#if MSG2133_ESD
    static void msg2133_esd_resume_expire(unsigned long data)
    {

        MSG2133_DBG("@hfs: 222  msg2133_esd_resume_expire \n");
        MSG2133_DBG("@hfs: 222 msg2133_close_flag :%d \n",msg2133_close_flag);
        if(msg2133_close_flag==1)
        {
            MSG2133_DBG("@hfs: 1111111\n");
            queue_work(msg2133_esd_resume_workqueue, &resume_work);

            MSG2133_DBG("@hfs: 111  prox_enable_prox \n");
            mod_timer(&msg2133_esd_resume_timer,jiffies + msecs_to_jiffies(10000));
        }
        else
        {	
            MSG2133_DBG("@hfs: 222222\n");		

        }
    }

    static void msg2133_esd_timer_start(void)
    {

        mod_timer(&msg2133_esd_resume_timer,jiffies + msecs_to_jiffies(10000));
    }


    void msg2133_esd_resume_callback(struct work_struct *work)
    {
        MSG2133_DBG("@hfs: msg2133_esd_resume_callback start ===\n");
        if(msg2133_close_flag==1)
        {prox_enable_prox(1);}
        else	
        {prox_enable_prox(0);}
        MSG2133_DBG("@hfs: msg2133_esd_resume_callback end ===\n");
    }
    //@hfs---end
#endif
#if MSG2133_PROXIMITY_FILTER
    static void msg2133_proximity_timer_start(void)
    {
        msg2133_proximity_report_lock=1;
        mod_timer(&msg2133_proximity_timer,jiffies + msecs_to_jiffies(200));
    }
    static void msg2133_proximity_timer_callback(void)
    {
        msg2133_proximity_report_lock=0;
        msg2133_close_num=0;
        msg2133_off_num=0;
        //input_report_abs(prox_input, ABS_DISTANCE, msg2133_proximity_report_value);
        MSG2133_DBG("%s, prox now can report next time \n", __func__);
    }

#endif

#if 0//def MSG2133_CHECK_INCALL
    static void msg2133_check_incall_expire(unsigned long data)
    {
        int android_mode = get_android_mode();

        MSG2133_DBG("==%s== android mode =%d, %d\n", __func__, android_mode, MODE_IN_CALL);
        if(prox_enable!=0)
        {

            if(android_mode != MODE_IN_CALL)
            {				
                input_report_abs(prox_input, ABS_DISTANCE, 1);								
                MSG2133_DBG(" check incall:send abs_distance 1 \n");
            }
            else 
            {			
                mod_timer(&msg2133_chek_incall_timer,jiffies + msecs_to_jiffies(1000));
                MSG2133_DBG(" check incall: mod_timer set again \n");
            }
        }		
    }

    static void msg2133_check_incall_timer_start(void)
    {
        mod_timer(&msg2133_chek_incall_timer,jiffies + msecs_to_jiffies(1000));
    }

#endif

    module_init(msg2133_init_module);
    module_exit(msg2133_exit_module);

    MODULE_LICENSE("GPL");

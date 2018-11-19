#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>

#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <mach/gpio.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/input/msg2138.h>
#define __FIRMWARE_UPDATE__

unsigned char MSG_FIRMWARE_MA[94*1024] =
{  
};

/*****************defination++++*******************/
#define u8         unsigned char
#define u32        unsigned int
#define s32        signed int
#define REPORT_PACKET_LENGTH    (8)

static u16 ctp_x_max = 480;
static u16 ctp_y_max = 854;

unsigned short ctp_id = 0;
static u8  x_y_swap = 0;
//#define SWAP_X_Y             (x_y_swap)
#define MSG21XX_INT_GPIO       (1)
#define MSG21XX_RESET_GPIO     (0)
#define MS_TS_MSG21XX_X_MAX  (ctp_x_max)
#define MS_TS_MSG21XX_Y_MAX  (ctp_y_max)

#define	TOUCH_KEY_HOME	      (102)
#define	TOUCH_KEY_MENU        (139)
#define	TOUCH_KEY_BACK        (158)
#define	TOUCH_KEY_SEARCH      (217)
static u8 g_dwiic_info_data[1024];   // Buffer for info data
//////////////////////////////////////////////////////
struct sprd_i2c_setup_data {
	unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
	unsigned short i2c_address;
	char type[I2C_NAME_SIZE];
};

struct msg2138_ts_data {
        struct i2c_client *client;
        struct input_dev *input_dev;
        struct regulator *vdd;
        struct regulator *vcc_i2c;
        bool loading_fw;
        u8 family_id;
        struct dentry *dir;
        u16 addr;
        bool suspended;
        char *ts_info;
        u8 *tch_data;
        u32 tch_data_len;
        u8 fw_ver[3];
	u8 power_on;
#if defined(CONFIG_FB)
        struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
        struct early_suspend early_suspend;
#endif
};
u8 irq_enabled = 1;

static struct msg2138_ts_data *msg2138_data;

extern  int call_flag_stk_ps;

static int msg21xx_irq = 0;
static struct i2c_client *msg21xx_i2c_client;
static struct work_struct msg21xx_wq;
//static struct early_suspend early_suspend;
static struct input_dev *input = NULL;

#define TP_DEVICE_INFO
#ifdef TP_DEVICE_INFO
    struct proc_dir_entry *tp_proc_file;
    char *tp_info;
#endif

/******* firmware update part *********/
#define	__FIRMWARE_UPDATE__

#ifdef __FIRMWARE_UPDATE__
#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
static  char *fw_version;
unsigned short fw_major_version=0,fw_minor_version=0;
#define FW_ADD	12405
int FWA_ADD = 32590;
int APK_UPDATE=0;
int IC_ID =0;
int PS_FUNC = 0;
static u8 temp[94][1024];
u8 *tempe;
u8  Fmr_Loader[1024];
u32 crc_tab[256];

static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;
typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static int drvTP_erase_emem_c33 (EMEM_TYPE_t emem_type);
static int drvTP_read_info_dwiic_c33 (void);

unsigned char MSG_2133A_BIN[94*1024] = 
{
#include "Yaris5NA_2138A_Each_V513_20140619.h"
};

#if defined(HQ_CTP_PS_TEST)
unsigned char MSG_PSEN_MA[94*1024] =
{  
#include "Beetlelite-mutto.h"//for psensor
};
unsigned char MSGA_PSEN_MA[94*1024] =
{  
#include "Yaris5NA_2138_v101.h"//for psensor
};
#define HQALSPS_DEVICE_NAME               		"apds990x_ps_dev"//"hq-sprd-alsps-tpd-dev"
#define HQALSPS_INPUT_NAME               		"Avago proximity sensor"//"hq-sprd-alsps-tpd-input"
#define APDS990X_MAGIC 'L'
#define APDS990X_IOCTL_PS_ENABLE		_IOW(APDS990X_MAGIC, 0x01, int)
#define APDS990X_IOCTL_PS_GET_ENABLE	_IOR(APDS990X_MAGIC, 0x02, unsigned int)
#define APDS990X_IOCTL_PS_GET_PDATA		_IOR(APDS990X_MAGIC, 0x03, unsigned int)
static int ps_state = 1;
static int mstar2133_pls_opened=0;
struct input_dev *input_dev;
struct timer_list ps_timer;
static void  msg2133_ps_mode_enable(bool enable);
#endif

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

	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if( rc < 0 )
	{
		printk("HalTscrCReadI2CSeq error %d\n", rc);
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
	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if( rc < 0 )
	{
		printk("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc,addr);
	}
}

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

static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
	return sprintf ( buf, "%s\n", fw_version );
}
/*reset the chip*/
static void _HalTscrHWReset(void)
{
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	gpio_set_value(MSG21XX_RESET_GPIO, 0);
	mdelay(10);  /* Note that the RST must be in LOW 10ms at least */
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	/* Enable the interrupt service thread/routine for INT after 50ms */
	mdelay(50);
}

static void msg21xx_release(void)
{
	TP_DEBUG("[%s]: Enter!\n", __func__);
	input_report_key(input, TOUCH_KEY_MENU, 0);
	input_report_key(input, TOUCH_KEY_HOME, 0);
	input_report_key(input, TOUCH_KEY_BACK, 0);
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
	input_report_key(input, BTN_TOUCH, 0);
	//input_mt_sync(input);
	input_sync(input);
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

#ifdef AUTO_UPDATE //having a problem that in order to update it should be regconized two different IC(Msg213/2133A)
static ssize_t c33_firmware_update ( EMEM_TYPE_t emem_type )
{
    //u8  dbbus_tx_data[4];
  //  u8  dbbus_rx_data[2] = {0};
	u32 i, j;
	u32 crc_main, crc_main_tp;

	int update_pass = 1;
	u16 reg_data = 0;
	printk("----%s----:",__func__);
	crc_main = 0xffffffff;

	drvTP_read_info_dwiic_c33();
	for (i=0;i<8;i++)
		printk("-----------%c------------\n", g_dwiic_info_data[i]);
	i =0;
		for (i = 0; i < 32; i++)   // total  33 KB : 1 byte per R/W
		{
			for ( j = 0; j < 1024; j++ )        //Read 1k bytes
			{
				//temp[i][j] = MSG_FIRMWARE_MA[(i*1024)+j]; // Read the bin files of slave firmware from the baseband file system
				temp[i][j] = MSG_2133A_BIN[(i*1024)+j];					
			}
		}
		printk("[MSG2133A] Date transf completed !\n");

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

	for ( i = 0; i < 32; i++ ) // total  33 KB : 2 byte per R/W
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
		if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

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
        }while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
    }
    TP_DEBUG ( "crc_main=0x%x,  crc_main_tp=0x%x, \n",
               crc_main, crc_main_tp);

    //drvDB_ExitDBBUS();

    update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	enable_irq(msg21xx_irq);
        return ( 0 );
    }

    printk ( "update OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
    enable_irq(msg21xx_irq);
    return 1;
}

static ssize_t firmware_update ( void )
{
  //  u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	disable_irq(msg21xx_irq);

    _HalTscrHWReset();

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
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
	printk("--##--%s:",__func__);
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
	printk("--^^^---%s:",__func__);
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
			printk("--***--%s:",__func__);
            return c33_firmware_update (  EMEM_MAIN );
        }
    }
	return 0;
}
#endif

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
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

static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
//    u8  dwiic_rx_data[4];
    int i;
    u16 reg_data=0;
    mdelay ( 300 );
printk("-----%s:",__func__);
    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );	
    mdelay ( 100 );

    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );
	
    dwiic_tx_data[0] = 0x72;
   dwiic_tx_data[3] = 0x00;
   dwiic_tx_data[4] = 0x80;	
   for(i=0;i<8;i++)
   	{
   	   dwiic_tx_data[1] = 0x80+(((i*128)&0xff00)>>8);
	   dwiic_tx_data[2] = ((i*128)&0x00ff);
		HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );

		mdelay ( 50 );

		// recive info data
		//printk("--AAAAAAAAA---%s:\n",__func__);
				HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[i*128], 128);
		//printk("--SSSSSSSSSSSS---%s:\n",__func__);
   	}
   printk("--AAAAAAAAAA---%d-----SSSSSSSSSSS---\n",i);
    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
    u32 i, j;
    u32 crc_main, crc_main_tp;
  
    int update_pass = 1;
    u16 reg_data = 0;
printk("-----%s:",__func__);
    crc_main = 0xffffffff;

    drvTP_read_info_dwiic_c33();
	for (i=0;i<8;i++)
			printk("-----------%c------------\n", g_dwiic_info_data[i]);
i =0;
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

    for ( i = 0; i < 32; i++ ) // total  33 KB : 2 byte per R/W
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
            if ( emem_type == EMEM_MAIN ) break;
        }

        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

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
        }while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
    }
    TP_DEBUG ( "crc_main=0x%x, crc_main_tp=0x%x\n",
               crc_main, crc_main_tp);

    //drvDB_ExitDBBUS();

    update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	enable_irq(msg21xx_irq);
        return ( 0 );
    }

    printk ( "update OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
    enable_irq(msg21xx_irq);
    return size;
}

#define _FW_UPDATE_C3_
#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
	//  u8 i;
	u8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	disable_irq(msg21xx_irq);

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
	//check id
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0xCC;
	HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
	printk("--##--%s:",__func__);
	HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
	if ( dbbus_rx_data[0] == 2 )
	{
        // check version
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x3C;
		dbbus_tx_data[2] = 0xEA;
		HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
		printk("--^^^---%s:",__func__);
		HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
		TP_DEBUG ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

		if ( dbbus_rx_data[0] == 3 ){
			printk("--***--%s:",__func__);
			return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
		}
	}
	return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
}
#else
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
	u8 i;
	u8 dbbus_tx_data[4];
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
	TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
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
	TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
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
	TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
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
	TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
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
	TP_DEBUG ( "update OK\n" );
	drvISP_ExitIspMode();
	FwDataCnt = 0;

	return size;
}
#endif
static DEVICE_ATTR(update, 0644, firmware_update_show, firmware_update_store);

static ssize_t version_cat(void)
{
	int ret = 0;
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ,local_fwa[4];
	unsigned short major=0, minor=0;

	fw_version = kzalloc(sizeof(char), GFP_KERNEL);
	local_fwa[0]=MSG_2133A_BIN[FWA_ADD];
	local_fwa[1]=MSG_2133A_BIN[FWA_ADD+1];
	local_fwa[2]=MSG_2133A_BIN[FWA_ADD+2];
	local_fwa[3]=MSG_2133A_BIN[FWA_ADD+3];
	printk("firmware version in .H file: major = %02d  minor = %02d\n", local_fwa[0], local_fwa[2]);
    //Get_Chip_Version();
	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x2A;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

	major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
	minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];
	TP_DEBUG("**FW_ADD = %d****\n",FW_ADD);

	printk("firmware version in TP:major = %02d minor = %02d\n", major, minor);
	sprintf(fw_version,"Mstar2138_YEJI_V%02d%02d", major, minor);

	if( (minor < local_fwa[2]) ||(minor > 100) )
		return 1;
	else
		return ret;

}
static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	TP_DEBUG("*** firmware_version_show fw_version = %s***\n", fw_version);
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
	version_cat();

	return size;
}
static DEVICE_ATTR(version, 0644, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	TP_DEBUG("***FwDataCnt = %d ***\n", FwDataCnt);
	for (i = 0; i < 1024; i++)
	{
		memcpy(temp[FwDataCnt], buf, 1024);
	}
	FwDataCnt++;
	APK_UPDATE =1;
	return size;
}
static DEVICE_ATTR(data, 0644, firmware_data_show, firmware_data_store);
#endif  //__FIRMWARE_UPDATE__
/***************firmware update----***********************/
/***********interrupter process++++**********************/
static u8 Calculate_8BitsChecksum( u8 *msg, s32 s32Length )
{
	s32 s32Checksum = 0;
	s32 i;

	for ( i = 0 ; i < s32Length; i++ )
	{
		s32Checksum += msg[i];
	}

	return (u8)( ( -s32Checksum ) & 0xFF );
}

static int keycode;
static int msg21xx_data_report(TouchScreenInfo_t *pinfo)
{
	int i;
	u8 finger_num=pinfo->nFingerNum;
	MSTAR_DBG("%s: finger_num =%x\n",__func__,finger_num);	
	if(finger_num) {
	for (i=0; i<finger_num; i++) {
		MSTAR_DBG("%s:pinfo.Point[i].X =%x,  pinfo.Point[i].Y=%x\n",__func__, pinfo->Point[i].X ,pinfo->Point[i].Y);	
		input_report_abs(input, ABS_MT_POSITION_X,  pinfo->Point[i].X);
		input_report_abs(input, ABS_MT_POSITION_Y,  pinfo->Point[i].Y);
		input_report_key(input, BTN_TOUCH, 1);
		input_mt_sync(input);
	}
	} else {
		input_report_key(input, BTN_TOUCH, 0);
	}
	input_sync(input); 

	return 0;

}
static void msg21xx_data_disposal(struct work_struct *work)
{
	int tempx;
	int tempy;

	u8 val[8] = {0};
	u8 Checksum = 0;
	//u8 reverse_x = 0,reverse_y = 0;
	u32 delta_x = 0, delta_y = 0;
	u32 u32X = 0;
	u32 u32Y = 0;
	//u8 touchkeycode = 0;
	TouchScreenInfo_t touchData;
	static u32 preKeyStatus=0;
	//static u32 preFingerNum=0;

	memset(&touchData, 0, sizeof(TouchScreenInfo_t));
//printk("interrupt........................................\n");
	
	i2c_master_recv(msg21xx_i2c_client,&val[0],REPORT_PACKET_LENGTH);
	Checksum = Calculate_8BitsChecksum(&val[0], 7); //calculate checksum

	if ((Checksum == val[7]) && (val[0] == 0x52))   //check the checksum  of packet
	{
		u32X = (((val[1] & 0xF0) << 4) | val[2]);         //parse the packet to coordinates
		u32Y = (((val[1] & 0x0F) << 8) | val[3]);

		delta_x = (((val[4] & 0xF0) << 4) | val[5]);
		delta_y = (((val[4] & 0x0F) << 8) | val[6]);

	if(x_y_swap)
	{
		tempy = u32X;
		tempx = u32Y;
		u32X = tempx;
		u32Y = tempy;
		
		tempy = delta_x;
		tempx = delta_y;
		delta_x = tempx;
		delta_y = tempy;
//		printk("------------------------\n");
	}
	else
	{
#ifdef REVERSE_1X
	  u32X = 2047 - u32X;
	  delta_x = 4095 - delta_x;
//	  		printk("-------^^^^^^^^^^^^^^-------------\n");
#endif
#ifdef REVERSE_1Y
	  u32Y = 2047 - u32Y;
	  delta_y = 4095 - delta_y;	  
#endif
	}
		if ((val[1] == 0xFF) && (val[2] == 0xFF) && (val[3] == 0xFF) && (val[4] == 0xFF) && (val[6] == 0xFF))
		{
			touchData.Point[0].X = 0; // final X coordinate
			touchData.Point[0].Y = 0; // final Y coordinate
				    if((val[5]==0x0)||(val[5]==0xFF))
			            {
			                touchData.nFingerNum = 0; //touch end
			                touchData.nTouchKeyCode = 0; //TouchKeyMode
			                touchData.nTouchKeyMode = 0; //TouchKeyMode
				                keycode = 0;
			              }
			            else
			            {
				                touchData.nTouchKeyMode = 1; //TouchKeyMode
						  touchData.nTouchKeyCode = val[5]; //TouchKeyCode
						  keycode=val[5];
							if(keycode==1)
							 {
								input_report_key(input, 158, 1);

							 }
							 else if(keycode==2)
							 {
								input_report_key(input, 102, 1);

							 }
							 else  if(keycode == 4)
							 {
								input_report_key(input, 139, 1);

							 }
				                touchData.nFingerNum = 1;
						input_sync(input);
				            }
		} else 	{
			touchData.nTouchKeyMode = 0; //Touch on screen...
			//if ((delta_x == 0) && (delta_y == 0))
			#if 1
			if(
			   #ifdef REVERSE_1X
			       (delta_x == 4095)
			   #else
			       (delta_x == 0) 
			   #endif
			   && 
			   #ifdef REVERSE_1Y
			       (delta_y == 4095)
			   #else
			       (delta_y == 0)
			   #endif
			  )
			  #endif
			{   /*one touch point*/
				touchData.nFingerNum = 1; //one touch
				touchData.Point[0].X = (u32X * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData.Point[0].Y = (u32Y * MS_TS_MSG21XX_Y_MAX) / 2048;
				MSTAR_DBG("****[%s]: u32X = %d, u32Y = %d\n", __func__, u32X, u32Y);
				MSTAR_DBG("[%s]: x = %d, y = %d\n", __func__, touchData.Point[0].X, touchData.Point[0].Y);
			}
			else
			{ /*two touch points*/
				u32 x2, y2;
				touchData.nFingerNum = 2; //two touch
				/* Finger 1 */
				touchData.Point[0].X = (u32X * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData.Point[0].Y = (u32Y * MS_TS_MSG21XX_Y_MAX) / 2048;
				MSTAR_DBG("[%s]: x = %d, y = %d\n", __func__, touchData.Point[0].X, touchData.Point[0].Y);
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

				touchData.Point[1].X = (x2 * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData.Point[1].Y = (y2 * MS_TS_MSG21XX_Y_MAX) / 2048;
			}
			msg21xx_data_report(&touchData);
		}
		if(0 == touchData.nFingerNum)
		{
			preKeyStatus = 0;
			msg21xx_release();
		}
	}
	
    enable_irq(msg21xx_irq);
}
/***********interrupter process++++**********************/

static int msg21xx_ts_open(struct input_dev *dev)
{
	return 0;
}

static void msg21xx_ts_close(struct input_dev *dev)
{
	printk("msg21xx_ts_close\n");
}

static int msg21xx_init_input(void)
{
	int err = 0;
	input = input_allocate_device();
	if (!input) {
		dev_err(&msg21xx_i2c_client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	input->name =msg21xx_i2c_client->name;// "ctp";
	input->phys = "I2C";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &msg21xx_i2c_client->dev;
	input->open = msg21xx_ts_open;
	input->close = msg21xx_ts_close;

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	
	__set_bit(BTN_TOUCH, input->keybit);
	
	__set_bit(TOUCH_KEY_BACK, input->keybit);
	__set_bit(TOUCH_KEY_MENU, input->keybit);
	__set_bit(TOUCH_KEY_HOME, input->keybit);
//	set_bit(TOUCH_KEY_SEARCH, input->keybit);
	__set_bit(ABS_MT_POSITION_X, input->absbit);
	__set_bit(ABS_MT_POSITION_Y, input->absbit);

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	
	//input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, MS_TS_MSG21XX_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, MS_TS_MSG21XX_Y_MAX, 0, 0);

	err = input_register_device(input);
	if (err) {
		dev_err(&msg21xx_i2c_client->dev,
		"msg_ts_probe: failed to register input device: %s\n",
		dev_name(&msg21xx_i2c_client->dev));
		goto exit_input_register_device_failed;
	}
	return 0;

exit_input_register_device_failed:
	input_free_device(input);
exit_input_dev_alloc_failed:
	return -1;
}
static irqreturn_t msg21xx_interrupt(int irq, void *dev_id)
{
	disable_irq_nosync(msg21xx_irq);

	schedule_work(&msg21xx_wq);
	return IRQ_HANDLED;
}
/////////////////////////

static int msg2133_i2c_test(struct i2c_client *client)
{
	int rc;
	char temp;
	struct i2c_msg msgs[] =
	{
		{
		.addr = 0xc4 >> 1,//client->addr,
		.flags = 0,//I2C_M_RD,
		.len = 1,
		.buf = &temp,
		},
	};
	printk("client addr : %x\n", client->addr);

	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	TP_DEBUG("i2c transfer rc = %d\n", rc);
	if( rc < 0 )
	{
		printk("HalTscrCReadI2CSeq error %d\n", rc);
		return rc;
	}

	msleep(10);
	return 0;
}
// proc file create tp_info...
#ifdef TP_DEVICE_INFO
static int tp_info_read_proc_t(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%s%s", "msg2133_", tp_info);
}
static int tp_info_write_proc_t(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	return 0;
}
#endif

static void tp_reset(void)
{

	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	gpio_set_value(MSG21XX_RESET_GPIO, 0);
	msleep(200);  /* Note that the RST must be in LOW 10ms at least */
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	/* Enable the interrupt service thread/routine for INT after 50ms */
	msleep(50);
}

static int tp_config_pins(void)
{
	int tp_irq;
	if (gpio_request(MSG21XX_RESET_GPIO, "gpio_tp_rst") < 0) {
		printk("gpio tp_rst request err\n");
		goto err_request_tprst;
	}
	gpio_tlmm_config(GPIO_CFG(MSG21XX_RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,	GPIO_CFG_2MA),	GPIO_CFG_ENABLE);
	
	if (gpio_request(MSG21XX_INT_GPIO, "gpio_tp_irq") < 0) {
		printk("gpio tp_irq request err\n");
		goto err_request_tpirq;
	}
	gpio_tlmm_config(GPIO_CFG(MSG21XX_INT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		
	tp_irq =  gpio_to_irq(MSG21XX_INT_GPIO);
	if (tp_irq < 0) {
		printk("sprd_alloc_gpio_irq err\n");
		goto err_alloc_gpio_irq;
	}

	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	//gpio_direction_input(sprd_3rdparty_gpio_tp_irq);
	
	return tp_irq;

err_alloc_gpio_irq:
	gpio_free(MSG21XX_INT_GPIO);
err_request_tpirq:
	gpio_free(MSG21XX_RESET_GPIO);
err_request_tprst:
 	return -1;
}

static void tp_unconfig_pins(int tp_irq)
{
	gpio_free(MSG21XX_RESET_GPIO);
	gpio_free(MSG21XX_INT_GPIO);
}

static int msg2138_power_init(bool on)
{
        int rc;

        if (!on)
                goto pwr_deinit;

        msg2138_data->vdd = regulator_get(&msg2138_data->client->dev, "vdd");
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");
        if (IS_ERR(msg2138_data->vdd)) {
                rc = PTR_ERR(msg2138_data->vdd);
                dev_err(&msg2138_data->client->dev,
                        "Regulator get failed vdd rc=%d\n", rc);
                return rc;
        }
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");

	if (regulator_count_voltages(msg2138_data->vdd) > 0) {
		rc = regulator_set_voltage(msg2138_data->vdd, 2600000,
					   3300000);
		if (rc) {
			dev_err(&msg2138_data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			//goto reg_vdd_put;
		}
	}
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");

        msg2138_data->vcc_i2c = regulator_get(&msg2138_data->client->dev, "vcc_i2c");
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");
        if (IS_ERR(msg2138_data->vcc_i2c)) {
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");
                rc = PTR_ERR(msg2138_data->vcc_i2c);
                dev_err(&msg2138_data->client->dev,
                        "Regulator get failed vcc_i2c rc=%d\n", rc);
                goto reg_vdd_set_vtg;
        }

	if (regulator_count_voltages(msg2138_data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(msg2138_data->vcc_i2c, 1800000,
					   1800000);
		if (rc) {
			dev_err(&msg2138_data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			//goto reg_vcc_i2c_put;
		}
	}
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");

        return 0;

reg_vdd_set_vtg:

pwr_deinit:
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");
//add by rongxiao.deng for disabel ldo when probe failed
	regulator_disable(msg2138_data->vdd);
        regulator_put(msg2138_data->vdd);

	regulator_disable(msg2138_data->vcc_i2c);
        regulator_put(msg2138_data->vcc_i2c);
//add end
        return 0;
}

//static int msg2138_power_on(struct msg2138_ts_data *data, bool on)
static int msg2138_power_on(bool on)
{
        int rc;
	if(call_flag_stk_ps)
	{
		return 0;
	}
	else
	{
	}

        if (!on)
                goto power_off;

	if(1 == msg2138_data->power_on)
		return 0;
	if(NULL == msg2138_data->vdd)
	{
		printk("step1.2\n");
		return 0;
	}
	
        rc = regulator_enable(msg2138_data->vdd);
        if (rc) {
                dev_err(&msg2138_data->client->dev,
                        "Regulator vdd enable failed rc=%d\n", rc);
                return rc;
        }

        rc = regulator_enable(msg2138_data->vcc_i2c);
        if (rc) {
                dev_err(&msg2138_data->client->dev,
                        "Regulator vcc_i2c enable failed rc=%d\n", rc);
                regulator_disable(msg2138_data->vdd);
        }
	msg2138_data->power_on = 1;

        return rc;

power_off:
	if(0 == msg2138_data->power_on)
		return 0;
	#if 0
        rc = regulator_disable(msg2138_data->vdd);
        if (rc) {
                dev_err(&msg2138_data->client->dev,
                        "Regulator vdd disable failed rc=%d\n", rc);
                return rc;
        }
	#endif

        rc = regulator_disable(msg2138_data->vcc_i2c);
        if (rc) {
                dev_err(&msg2138_data->client->dev,
                        "Regulator vcc_i2c disable failed rc=%d\n", rc);
        }

	msg2138_data->power_on = 0;
        return rc;
}

static int msg2138_ts_suspend(struct device *dev)
{
        int err;

	if(1==irq_enabled)
	{
		disable_irq_nosync(msg21xx_irq);
		irq_enabled = 0;
	}

	msg21xx_release();

	if (gpio_is_valid(MSG21XX_RESET_GPIO)) {
	   gpio_set_value_cansleep(MSG21XX_RESET_GPIO, 0);
	}
    err = msg2138_power_on(false);
    if (err) {
            dev_err(dev, "power off failed");
            goto pwr_off_fail;
    }

        return 0;
pwr_off_fail:
        return err;
}

static int msg2138_ts_resume(struct device *dev)
{
    int err;

    err = msg2138_power_on(true);
    if (err) {
            dev_err(dev, "power on failed");
            return err;
    }
    if (gpio_is_valid(MSG21XX_RESET_GPIO)) {
            gpio_set_value_cansleep(MSG21XX_RESET_GPIO, 1);
            msleep(20);
            gpio_set_value_cansleep(MSG21XX_RESET_GPIO, 0);
            msleep(20);
            gpio_set_value_cansleep(MSG21XX_RESET_GPIO, 1);
    }
    msleep(200);

	if(0==irq_enabled)
	{
		enable_irq(msg21xx_irq);
		irq_enabled = 1;
	}

        return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{

    struct fb_event *evdata = data;
    int *blank;

    if (evdata && evdata->data && event == FB_EVENT_BLANK &&
                    msg2138_data && msg2138_data->client) {
            blank = evdata->data;
            if (*blank == FB_BLANK_UNBLANK)
                    msg2138_ts_resume(&msg2138_data->client->dev);
            else if (*blank == FB_BLANK_POWERDOWN)
                    msg2138_ts_suspend(&msg2138_data->client->dev);
    }

    return 0;
}
#endif

//#define DMA_IIC //modify: MTK平台超过8byte必须使用DMA方式进行iic通信
#ifdef DMA_IIC
#include <linux/dma-mapping.h>
static unsigned char *I2CDMABuf_va = NULL;
static volatile unsigned int I2CDMABuf_pa = NULL;
static void _msg_dma_alloc(void)
{
    I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
}
static void _msg_dma_free(void)
{
    if(NULL!=I2CDMABuf_va)
    {
        dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
	    I2CDMABuf_va = NULL;
	    I2CDMABuf_pa = 0;
    }
}
#endif

#define ITO_TEST
#ifdef ITO_TEST

#include "open_test_ANA1_Each.h"
#include "open_test_ANA2_Each.h"
#include "open_test_ANA1_B_Each.h"
#include "open_test_ANA2_B_Each.h"
#include "open_test_ANA3_Each.h"

///////////////////////////////////////////////////////////////////////////
u8 bItoTestDebug = 0;
#define ITO_TEST_DEBUG(format, ...) \
{ \
    if(bItoTestDebug) \
    { \
        printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__); \
        mdelay(5); \
    } \
}
#define ITO_TEST_DEBUG_MUST(format, ...)	printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__);mdelay(5)



s16  s16_raw_data_1[48] = {0};
s16  s16_raw_data_2[48] = {0};
s16  s16_raw_data_3[48] = {0};
u8 ito_test_keynum = 0;
u8 ito_test_dummynum = 0;
u8 ito_test_trianglenum = 0;
u8 ito_test_2r = 0;
u8 g_LTP = 1;	
uint16_t *open_1 = NULL;
uint16_t *open_1B = NULL;
uint16_t *open_2 = NULL;
uint16_t *open_2B = NULL;
uint16_t *open_3 = NULL;
u8 *MAP1 = NULL;
u8 *MAP2=NULL;
u8 *MAP3=NULL;
u8 *MAP40_1 = NULL;
u8 *MAP40_2 = NULL;
u8 *MAP40_3 = NULL;
u8 *MAP40_4 = NULL;
u8 *MAP41_1 = NULL;
u8 *MAP41_2 = NULL;
u8 *MAP41_3 = NULL;
u8 *MAP41_4 = NULL;

static u8 g_fail_channel[48] = {0};
static int fail_channel_count = 0;

#define ITO_TEST_ADDR_TP  (0x4C>>1)
#define ITO_TEST_ADDR_REG (0xC4>>1)
#define REG_INTR_FIQ_MASK           0x04
#define FIQ_E_FRAME_READY_MASK      ( 1 << 8 )
#define MAX_CHNL_NUM (48)
#define BIT0  (1<<0)
#define BIT1  (1<<1)
#define BIT5  (1<<5)
#define BIT11 (1<<11)
#define BIT15 (1<<15)

static int ito_test_i2c_read(u8 addr, u8* read_data, u16 size)//modify : 根据项目修改 msg21xx_i2c_client
{
    int rc;
    u8 addr_before = msg21xx_i2c_client->addr;
    msg21xx_i2c_client->addr = addr;

    #ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
    {
        int i = 0;
        msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag | I2C_DMA_FLAG ;
        rc = i2c_master_recv(msg21xx_i2c_client, (unsigned char *)I2CDMABuf_pa, size);
        for(i = 0; i < size; i++)
   		{
        	read_data[i] = I2CDMABuf_va[i];
    	}
    }
    else
    {
        rc = i2c_master_recv(msg21xx_i2c_client, read_data, size);
    }
    msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag & (~I2C_DMA_FLAG);	
    #else
    rc = i2c_master_recv(msg21xx_i2c_client, read_data, size);
    #endif

    msg21xx_i2c_client->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_read error %d,addr=%d\n", rc,addr);
    }
    return rc;
}

static int ito_test_i2c_write(u8 addr, u8* data, u16 size)//modify : 根据项目修改 msg21xx_i2c_client
{
    int rc;
    u8 addr_before = msg21xx_i2c_client->addr;
    msg21xx_i2c_client->addr = addr;

#ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
	{
	    int i = 0;
	    for(i=0;i<size;i++)
    	{
    		 I2CDMABuf_va[i]=data[i];
    	}
		msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag | I2C_DMA_FLAG ;
		rc = i2c_master_send(msg21xx_i2c_client, (unsigned char *)I2CDMABuf_pa, size);
	}
	else
	{
		rc = i2c_master_send(msg21xx_i2c_client, data, size);
	}
    msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag & (~I2C_DMA_FLAG);	
#else
    rc = i2c_master_send(msg21xx_i2c_client, data, size);
#endif

    msg21xx_i2c_client->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_write error %d,addr = %d,data[0]=%d\n", rc, addr,data[0]);
    }
    return rc;
}

static void ito_test_reset(void)
{
	gpio_direction_output(MSG21XX_RESET_GPIO, 1);
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	gpio_set_value(MSG21XX_RESET_GPIO, 0);
	mdelay(100);  
    ITO_TEST_DEBUG("reset tp\n");
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	mdelay(200);
}
static void ito_test_disable_irq(void)
{
	disable_irq_nosync(msg21xx_irq);
}
static void ito_test_enable_irq(void)
{
	enable_irq(msg21xx_irq);
}

static void ito_test_set_iic_rate(u32 iicRate)
{
	#ifdef CONFIG_I2C_SPRD
        sprd_i2c_ctl_chg_clk(msg21xx_i2c_client->adapter->nr, iicRate);
        mdelay(100);
	#endif
    #ifdef MTK
        msg21xx_i2c_client->timing = iicRate/1000;
    #endif
}

static void ito_test_WriteReg( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 5 );
}
static void ito_test_WriteReg8Bit( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    ito_test_i2c_write ( ITO_TEST_ADDR_REG, &tx_data[0], 4 );
}
static unsigned short ito_test_ReadReg( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 3 );
    ito_test_i2c_read ( ITO_TEST_ADDR_REG, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}
static u32 ito_test_get_TpType(void)
{
    u8 tx_data[3] = {0};
    u8 rx_data[4] = {0};
    u32 Major = 0, Minor = 0;

    ITO_TEST_DEBUG("GetTpType\n");
        
    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x2A;
    ito_test_i2c_write(ITO_TEST_ADDR_TP, &tx_data[0], 3);
    mdelay(50);
    ito_test_i2c_read(ITO_TEST_ADDR_TP, &rx_data[0], 4);
    Major = (rx_data[1]<<8) + rx_data[0];
    Minor = (rx_data[3]<<8) + rx_data[2];

    ITO_TEST_DEBUG("***TpTypeMajor = %d ***\n", Major);
    ITO_TEST_DEBUG("***TpTypeMinor = %d ***\n", Minor);
    
    return Major;
    
}

#define TP_OF_EACH		(5)
static u32 ito_test_choose_TpType(void)
{
    u32 tpType = 0;
    u8 i = 0;
    open_1 = NULL;
    open_1B = NULL;
    open_2 = NULL;
    open_2B = NULL;
    open_3 = NULL;
    MAP1 = NULL;
    MAP2 = NULL;
    MAP3 = NULL;
    MAP40_1 = NULL;
    MAP40_2 = NULL;
    MAP40_3 = NULL;
    MAP40_4 = NULL;
    MAP41_1 = NULL;
    MAP41_2 = NULL;
    MAP41_3 = NULL;
    MAP41_4 = NULL;
    ito_test_keynum = 0;
    ito_test_dummynum = 0;
    ito_test_trianglenum = 0;
    ito_test_2r = 0;

    for(i=0;i<10;i++)
    {
        tpType = ito_test_get_TpType();
        ITO_TEST_DEBUG("tpType=%d;i=%d;\n",tpType,i);
        if(TP_OF_EACH==tpType)//modify:注意该项目tp数目
        {
            break;
        }
        else if(i<5)
        {
            mdelay(100);  
        }
        else
        {
            ito_test_reset();
        }
    }
    
    if(TP_OF_EACH==tpType)//modify:注意该项目tp数目
    {
        open_1 = open_1_Each;
        open_1B = open_1B_Each;
        open_2 = open_2_Each;
        open_2B = open_2B_Each;
        open_3 = open_3_Each;
        MAP1 = MAP1_Each;
        MAP2 = MAP2_Each;
        MAP3 = MAP3_Each;
        MAP40_1 = MAP40_1_Each;
        MAP40_2 = MAP40_2_Each;
        MAP40_3 = MAP40_3_Each;
        MAP40_4 = MAP40_4_Each;
        MAP41_1 = MAP41_1_Each;
        MAP41_2 = MAP41_2_Each;
        MAP41_3 = MAP41_3_Each;
        MAP41_4 = MAP41_4_Each;
        ito_test_keynum = NUM_KEY_EACH;
        ito_test_dummynum = NUM_DUMMY_EACH;
        ito_test_trianglenum = NUM_SENSOR_EACH;
        ito_test_2r = ENABLE_2R_EACH;
    }
    else
    {
        tpType = 0;
    }
    return tpType;
}
static void ito_test_EnterSerialDebugMode(void)
{
    u8 data[5];

    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 5);

    data[0] = 0x37;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x35;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x71;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);
}
static uint16_t ito_test_get_num( void )
{
    uint16_t    num_of_sensor,i;
    uint16_t 	RegValue1,RegValue2;
 
    num_of_sensor = 0;
        
    RegValue1 = ito_test_ReadReg( 0x11, 0x4A);
    ITO_TEST_DEBUG("ito_test_get_num,RegValue1=%d\n",RegValue1);
    if ( ( RegValue1 & BIT1) == BIT1 )
    {
    	RegValue1 = ito_test_ReadReg( 0x12, 0x0A);			
    	RegValue1 = RegValue1 & 0x0F;
    	
    	RegValue2 = ito_test_ReadReg( 0x12, 0x16);    		
    	RegValue2 = (( RegValue2 >> 1 ) & 0x0F) + 1;
    	
    	num_of_sensor = RegValue1 * RegValue2;
    }
	else
	{
	    for(i=0;i<4;i++)
	    {
	        num_of_sensor+=(ito_test_ReadReg( 0x12, 0x0A)>>(4*i))&0x0F;
	    }
	}
    ITO_TEST_DEBUG("ito_test_get_num,num_of_sensor=%d\n",num_of_sensor);
    return num_of_sensor;        
}
static void ito_test_polling( void )
{
    uint16_t    reg_int = 0x0000;
    uint8_t     dbbus_tx_data[5];
    uint8_t     dbbus_rx_data[4];
    uint16_t    reg_value;


    reg_int = 0;

    ito_test_WriteReg( 0x13, 0x0C, BIT15 );       
    ito_test_WriteReg( 0x12, 0x14, (ito_test_ReadReg(0x12,0x14) | BIT0) );         
            
    ITO_TEST_DEBUG("polling start\n");
    while( ( reg_int & BIT0 ) == 0x0000 )
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3D;
        dbbus_tx_data[2] = 0x18;
        ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
        ito_test_i2c_read(ITO_TEST_ADDR_REG,  dbbus_rx_data, 2);
        reg_int = dbbus_rx_data[1];
    }
    ITO_TEST_DEBUG("polling end\n");
    reg_value = ito_test_ReadReg( 0x3D, 0x18 ); 
    ito_test_WriteReg( 0x3D, 0x18, reg_value & (~BIT0) );      
}
static uint16_t ito_test_get_data_out( int16_t* s16_raw_data )
{
    uint8_t     i,dbbus_tx_data[8];
    uint16_t    raw_data[48]={0};
    uint16_t    num_of_sensor;
    uint16_t    reg_int;
    uint8_t		dbbus_rx_data[96]={0};
  
    num_of_sensor = ito_test_get_num();
    if(num_of_sensor*2>96)
    {
        ITO_TEST_DEBUG("danger,num_of_sensor=%d\n",num_of_sensor);
        return num_of_sensor;
    }

    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int & (uint16_t)(~FIQ_E_FRAME_READY_MASK) ) ); 
    ito_test_polling();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x13;
    dbbus_tx_data[2] = 0x40;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
    mdelay(20);
    ito_test_i2c_read(ITO_TEST_ADDR_REG, &dbbus_rx_data[0], (num_of_sensor * 2));
    mdelay(100);
    for(i=0;i<num_of_sensor * 2;i++)
    {
        ITO_TEST_DEBUG("dbbus_rx_data[%d]=%d\n",i,dbbus_rx_data[i]);
    }
 
    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int | (uint16_t)FIQ_E_FRAME_READY_MASK ) ); 


    for( i = 0; i < num_of_sensor; i++ )
    {
        raw_data[i] = ( dbbus_rx_data[ 2 * i + 1] << 8 ) | ( dbbus_rx_data[2 * i] );
        s16_raw_data[i] = ( int16_t )raw_data[i];
    }
    
    return(num_of_sensor);
}

static void ito_test_send_data_in( uint8_t step )
{
    uint16_t	i;
    uint8_t 	dbbus_tx_data[512];
    uint16_t 	*Type1=NULL;        

    ITO_TEST_DEBUG("ito_test_send_data_in step=%d\n",step);
	if( step == 4 )
    {
        Type1 = &open_1[0];        
    }
    else if( step == 5 )
    {
        Type1 = &open_2[0];      	
    }
    else if( step == 6 )
    {
        Type1 = &open_3[0];      	
    }
    else if( step == 9 )
    {
        Type1 = &open_1B[0];        
    }
    else if( step == 10 )
    {
        Type1 = &open_2B[0];      	
    } 
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0x00;    
    for( i = 0; i <= 0x3E ; i++ )
    {
        dbbus_tx_data[3+2*i] = Type1[i] & 0xFF;
        dbbus_tx_data[4+2*i] = ( Type1[i] >> 8 ) & 0xFF;    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+0x3F*2);
 
    dbbus_tx_data[2] = 0x7A * 2;
    for( i = 0x7A; i <= 0x7D ; i++ )
    {
        dbbus_tx_data[3+2*(i-0x7A)] = 0;
        dbbus_tx_data[4+2*(i-0x7A)] = 0;    	    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+8);  
    
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12;
      
    dbbus_tx_data[2] = 5 * 2;
    dbbus_tx_data[3] = Type1[128+5] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+5] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x0B * 2;
    dbbus_tx_data[3] = Type1[128+0x0B] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x0B] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x12 * 2;
    dbbus_tx_data[3] = Type1[128+0x12] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x12] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x15 * 2;
    dbbus_tx_data[3] = Type1[128+0x15] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x15] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        
}

static void ito_test_set_v( uint8_t Enable, uint8_t Prs)	
{
    uint16_t    u16RegValue;        
    
    
    u16RegValue = ito_test_ReadReg( 0x12, 0x08);   
    u16RegValue = u16RegValue & 0xF1; 							
    if ( Prs == 0 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0C); 		
    }
    else if ( Prs == 1 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0E); 		     	
    }
    else
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x02); 			
    }    
    
    if ( Enable )
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        ito_test_WriteReg( 0x11, 0x06, u16RegValue| 0x03);   	
    }
    else
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        u16RegValue = u16RegValue & 0xFC;					
        ito_test_WriteReg( 0x11, 0x06, u16RegValue);         
    }

}

static void ito_test_set_c( uint8_t Csub_Step )
{
    uint8_t i;
    uint8_t dbbus_tx_data[MAX_CHNL_NUM+3];
    uint8_t HighLevel_Csub = false;
    uint8_t Csub_new;
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;        
    dbbus_tx_data[2] = 0x84;        
    for( i = 0; i < MAX_CHNL_NUM; i++ )
    {
		Csub_new = Csub_Step;        
        HighLevel_Csub = false;   
        if( Csub_new > 0x1F )
        {
            Csub_new = Csub_new - 0x14;
            HighLevel_Csub = true;
        }
           
        dbbus_tx_data[3+i] =    Csub_new & 0x1F;        
        if( HighLevel_Csub == true )
        {
            dbbus_tx_data[3+i] |= BIT5;
        }
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);

    dbbus_tx_data[2] = 0xB4;        
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);
}

static void ito_test_sw( void )
{
    ito_test_WriteReg( 0x11, 0x00, 0xFFFF );
    ito_test_WriteReg( 0x11, 0x00, 0x0000 );
    mdelay( 50 );
}

static void ito_test_first( uint8_t item_id , int16_t* s16_raw_data)		
{
    uint8_t     loop;
    uint8_t     i,j;
    int16_t     s16_raw_data_tmp[48]={0};
    uint8_t     num_of_sensor, num_of_sensor2,total_sensor = 0;
    uint16_t	u16RegValue;
    uint8_t 	*pMapping=NULL;
    
    
	num_of_sensor = 0;
	num_of_sensor2 = 0;	
	
    ITO_TEST_DEBUG("ito_test_first item_id=%d\n",item_id);
	ito_test_WriteReg( 0x0F, 0xE6, 0x01 );

	ito_test_WriteReg( 0x1E, 0x24, 0x0500 );
	ito_test_WriteReg( 0x1E, 0x2A, 0x0000 );
	ito_test_WriteReg( 0x1E, 0xE6, 0x6E00 );
	ito_test_WriteReg( 0x1E, 0xE8, 0x0071 );
	    
    if ( item_id == 40 )    			
    {
        pMapping = &MAP1[0];
        if ( ito_test_2r )
		{
			total_sensor = ito_test_trianglenum/2; 
		}
		else
		{
		    total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
		}
    }
    else if( item_id == 41 )    		
    {
        pMapping = &MAP2[0];
        if ( ito_test_2r )
		{
			total_sensor = ito_test_trianglenum/2; 
		}
		else
		{
		    total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
		}
    }
    else if( item_id == 42 )    		
    {
        pMapping = &MAP3[0];      
        total_sensor =  ito_test_trianglenum + ito_test_keynum+ ito_test_dummynum; 
    }
        	    
	    
	loop = 1;
	if ( item_id != 42 )
	{
	    if(total_sensor>11)
        {
            loop = 2;
        }
	}	
    ITO_TEST_DEBUG("loop=%d\n",loop);
	for ( i = 0; i < loop; i++ )
	{
		if ( i == 0 )
		{
			ito_test_send_data_in( item_id - 36 );
		}
		else
		{ 
			if ( item_id == 40 ) 
				ito_test_send_data_in( 9 );
			else 		
				ito_test_send_data_in( 10 );
		}
	
		ito_test_set_v(1,0);    
		u16RegValue = ito_test_ReadReg( 0x11, 0x0E);    			
		ito_test_WriteReg( 0x11, 0x0E, u16RegValue | BIT11 );				 		
	
		if ( g_LTP == 1 )
	    	ito_test_set_c( 32 );	    	
		else	    	
	    	ito_test_set_c( 0 );
	    
		ito_test_sw();
		
		if ( i == 0 )	 
        {      
            num_of_sensor=ito_test_get_data_out(  s16_raw_data_tmp );
            ITO_TEST_DEBUG("num_of_sensor=%d;\n",num_of_sensor);
        }
		else	
        {      
            num_of_sensor2=ito_test_get_data_out(  &s16_raw_data_tmp[num_of_sensor] );
            ITO_TEST_DEBUG("num_of_sensor=%d;num_of_sensor2=%d\n",num_of_sensor,num_of_sensor2);
        }
	}
    for ( j = 0; j < total_sensor ; j ++ )
	{
		if ( g_LTP == 1 )
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j] + 4096;
		else
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j];	
	}	

	return;
}

typedef enum
{
	ITO_TEST_OK = 0,
	ITO_TEST_FAIL,
	ITO_TEST_GET_TP_TYPE_ERROR,
} ITO_TEST_RET;

ITO_TEST_RET ito_test_second (u8 item_id)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;

	u8  Th_Tri = 25;        
	u8  Th_bor = 25;        

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];
		}
    }
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2 ) * ( 100 - Th_bor) / 100 ;
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min);

	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
		{
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min)
			{
				g_fail_channel[fail_channel_count] = MAP40_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}

		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min)
			{ 
				g_fail_channel[fail_channel_count] = MAP41_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	return ret;
}
ITO_TEST_RET ito_test_second_2r (u8 item_id)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  s16_raw_data_jg_tmp3 = 0;
	s32  s16_raw_data_jg_tmp4 = 0;
	
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;
	s32  jg_tmp3_avg_Th_max =0;
	s32  jg_tmp3_avg_Th_min =0;
	s32  jg_tmp4_avg_Th_max =0;
	s32  jg_tmp4_avg_Th_min =0;

	u8  Th_Tri = 25;    // non-border threshold    
	u8  Th_bor = 25;    // border threshold    

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];  //first region: non-border 
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];  //first region: border
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_1[MAP40_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_1[MAP40_4[i]];  //second region: border
		}
    }



	
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];  //first region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];  //first region: border
		}
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_2[MAP41_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_2[MAP41_4[i]];  //second region: border
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2) * ( 100 - Th_bor) / 100 ;
		jg_tmp3_avg_Th_max = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp3_avg_Th_min = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp4_avg_Th_max = (s16_raw_data_jg_tmp4 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp4_avg_Th_min = (s16_raw_data_jg_tmp4 / 2) * ( 100 - Th_bor) / 100 ;
		
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d;sum3=%d;max3=%d;min3=%d;sum4=%d;max4=%d;min4=%d;\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min,s16_raw_data_jg_tmp3,jg_tmp3_avg_Th_max,jg_tmp3_avg_Th_min,s16_raw_data_jg_tmp4,jg_tmp4_avg_Th_max,jg_tmp4_avg_Th_min);




	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_2[i];				
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		} 
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_1[MAP40_3[i]] < jg_tmp3_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_3[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_1[MAP40_4[i]] < jg_tmp4_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_4[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_2[MAP41_3[i]] < jg_tmp3_avg_Th_min) 
			{	
				g_fail_channel[fail_channel_count] = MAP41_3[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_2[MAP41_4[i]] < jg_tmp4_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_4[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		} 
	}

	return ret;
}

static ITO_TEST_RET ito_test_interface(void)
{
    ITO_TEST_RET ret1 = ITO_TEST_OK, ret2 = ITO_TEST_OK, ret3 = ITO_TEST_OK;
    uint16_t i = 0;
#ifdef DMA_IIC
    _msg_dma_alloc();
#endif
    ito_test_set_iic_rate(50000);
    ITO_TEST_DEBUG("start\n");
    ito_test_disable_irq();
	ito_test_reset();
    if(!ito_test_choose_TpType())
    {
        ITO_TEST_DEBUG("choose tpType fail\n");
        ret1 = ITO_TEST_GET_TP_TYPE_ERROR;
        goto ITO_TEST_END;
    }
    ito_test_EnterSerialDebugMode();
    mdelay(100);
    ITO_TEST_DEBUG("EnterSerialDebugMode\n");
    ito_test_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );
    ito_test_WriteReg ( 0x3C, 0x60, 0xAA55 );
    ITO_TEST_DEBUG("stop mcu and disable watchdog V.005\n");   
    mdelay(50);
    
	for(i = 0;i < 48;i++)
	{
		s16_raw_data_1[i] = 0;
		s16_raw_data_2[i] = 0;
		s16_raw_data_3[i] = 0;
	}	
	
    fail_channel_count = 0; // Reset fail_channel_count to 0 before test start
	
    ito_test_first(40, s16_raw_data_1);
    ITO_TEST_DEBUG("40 get s16_raw_data_1\n");
    if(ito_test_2r)
    {
        ret2=ito_test_second_2r(40);
    }
    else
    {
        ret2=ito_test_second(40);
    }
    
    ito_test_first(41, s16_raw_data_2);
    ITO_TEST_DEBUG("41 get s16_raw_data_2\n");
    if(ito_test_2r)
    {
        ret3=ito_test_second_2r(41);
    }
    else
    {
        ret3=ito_test_second(41);
    }
    
    ito_test_first(42, s16_raw_data_3);
    ITO_TEST_DEBUG("42 get s16_raw_data_3\n");
    
    ITO_TEST_END:
#ifdef DMA_IIC
    _msg_dma_free();
#endif
    ito_test_set_iic_rate(100000);
	ito_test_reset();
    ito_test_enable_irq();
    ITO_TEST_DEBUG("end\n");
    
    if ((ret1 != ITO_TEST_OK) && (ret2 == ITO_TEST_OK) && (ret3 == ITO_TEST_OK))
    {
        return ITO_TEST_GET_TP_TYPE_ERROR;		
    }
    else if ((ret1 == ITO_TEST_OK) && ((ret2 != ITO_TEST_OK) || (ret3 != ITO_TEST_OK)))
    {
    		return ITO_TEST_FAIL;	
    }
    else
    {
    		return ITO_TEST_OK;	
    }
}

#include <linux/proc_fs.h>
#define ITO_TEST_AUTHORITY 0777 
static struct proc_dir_entry *msg_ito_test = NULL;
static struct proc_dir_entry *debug = NULL;
static struct proc_dir_entry *debug_on_off = NULL;
static struct proc_dir_entry *fail_channel = NULL;
static struct proc_dir_entry *data = NULL;
#define PROC_MSG_ITO_TESE      "msg-ito-test"
#define PROC_ITO_TEST_DEBUG      "debug"
#define PROC_ITO_TEST_DEBUG_ON_OFF     "debug-on-off"
#define PROC_ITO_TEST_FAIL_CHANNEL     "fail-channel"
#define PROC_ITO_TEST_DATA      "data"
ITO_TEST_RET g_ito_test_ret = ITO_TEST_OK;
static int ito_test_proc_read_debug(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt= 0;
    
    g_ito_test_ret = ito_test_interface();
    if(ITO_TEST_OK==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
    }
    else if(ITO_TEST_FAIL==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
    }
    else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
    }
    
    *eof = 1;
    cnt = sprintf(page,"%d",g_ito_test_ret);

    return cnt;
}

static int ito_test_proc_write_debug(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    u16 i = 0;
    mdelay(5);
    ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
    }
    mdelay(5);
    return count;
}
static int ito_test_proc_read_debug_on_off(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt= 0;
    
    bItoTestDebug = 1;
    ITO_TEST_DEBUG_MUST("on debug bItoTestDebug = %d",bItoTestDebug);
    
    *eof = 1;
    return cnt;
}

static int ito_test_proc_write_debug_on_off(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    bItoTestDebug = 0;
    ITO_TEST_DEBUG_MUST("off debug bItoTestDebug = %d",bItoTestDebug);
    return count;
}

static int ito_test_proc_read_fail_channel(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt = 0;
    int i;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_fail_channel()\n");
    ITO_TEST_DEBUG_MUST("fail_channel_count = %d\n", fail_channel_count);
    
    for (i = 0; i < fail_channel_count; i ++)
    {
    	  page[i] = g_fail_channel[i];
    }

    *eof = 1;

    cnt = fail_channel_count;

    return cnt;
}

static int ito_test_proc_write_fail_channel(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_fail_channel()\n");

    return count;
}

static int ito_test_proc_read_data(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt = 0;
    int i;
    u8 high_byte, low_byte;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_data()\n");
    
    for (i = 0; i < 48; i ++)
    {
    	  high_byte = (s16_raw_data_1[i] & 0xFF00) >> 8;
    	  low_byte = (s16_raw_data_1[i] & 0x00FF) << 8;
    	  
        page[i*2] = high_byte;
        page[i*2+1] = low_byte;
    }

    for (i = 0; i < 48; i ++)
    {
        high_byte = (s16_raw_data_2[i] & 0xFF00) >> 8;
        low_byte = (s16_raw_data_2[i] & 0x00FF) << 8;
        
        page[i*2+96] = high_byte;
        page[i*2+1+96] = low_byte;
    }

    *eof = 1;
    
    cnt = 96*2;

    return cnt;
}

static int ito_test_proc_write_data(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_data()\n");

    return count;
}

static void ito_test_create_entry(void)
{
    msg_ito_test = proc_mkdir(PROC_MSG_ITO_TESE, NULL);
    debug = create_proc_entry(PROC_ITO_TEST_DEBUG, ITO_TEST_AUTHORITY, msg_ito_test);
    debug_on_off= create_proc_entry(PROC_ITO_TEST_DEBUG_ON_OFF, ITO_TEST_AUTHORITY, msg_ito_test);
    fail_channel = create_proc_entry(PROC_ITO_TEST_FAIL_CHANNEL, ITO_TEST_AUTHORITY, msg_ito_test);
    data = create_proc_entry(PROC_ITO_TEST_DATA, ITO_TEST_AUTHORITY, msg_ito_test);

    if (NULL==debug) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG failed\n");
    } 
    else 
    {
        debug->read_proc = ito_test_proc_read_debug;
        debug->write_proc = ito_test_proc_write_debug;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG OK\n");
    }
    if (NULL==debug_on_off) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF failed\n");
    } 
    else 
    {
        debug_on_off->read_proc = ito_test_proc_read_debug_on_off;
        debug_on_off->write_proc = ito_test_proc_write_debug_on_off;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF OK\n");
    }

    if (NULL==fail_channel) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST FAIL CHANNEL failed\n");
    } 
    else 
    {
        fail_channel->read_proc = ito_test_proc_read_fail_channel;
        fail_channel->write_proc = ito_test_proc_write_fail_channel;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST FAIL CHANNEL OK\n");
    }

    if (NULL==data) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DATA failed\n");
    } 
    else 
    {
        data->read_proc = ito_test_proc_read_data;
        data->write_proc = ito_test_proc_write_data;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DATA OK\n");
    }
}
#endif

static int __devinit msg21xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int  err = 0;
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4]; 

	printk("[%s]: Enter NA!\n", __func__);

	msg2138_data = devm_kzalloc(&client->dev,
                sizeof(struct msg2138_ts_data), GFP_KERNEL);
	if (!msg2138_data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}
	msg2138_data->client = client;
	msg2138_data->power_on = 0;

	err = msg2138_power_init(true);
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto power_init_failed;
	}
	if(NULL == msg2138_data->vdd)
		printk("msg2138_data->vdd is NULL\n");

	err = msg2138_power_on(true);
	if (err) {
		dev_err(&client->dev, "power on failed");
		goto pwr_deinit;
	}


	msg21xx_irq = tp_config_pins();
	if(msg21xx_irq < 0)
	{
		printk("%s: msg_ts_config_pins failed\n",__func__);
		err = -ENOMEM;
		goto exit_config_pins_failed; 	
	}

	tp_reset();

	msg21xx_i2c_client = client;
	if(msg2133_i2c_test(client) < 0)
	{
		printk("msg2133 corressponding failed\n");
		goto msg2133_i2c_read_failed;
	}
/****************Check Mstar IC+++***************************/
	if(1){
		dbbusDWIICEnterSerialDebugMode();
		dbbusDWIICStopMCU();
		dbbusDWIICIICUseBus();
		dbbusDWIICIICReshape();
		mdelay ( 300 );
		//HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0xCC;
		HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
		HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
		TP_DEBUG("[MSG2133A]The CTP ID ---:0x%x !**********\n",dbbus_rx_data[0]);
		if ( dbbus_rx_data[0] == 2 )
		{
			// check version
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x3C;
			dbbus_tx_data[2] = 0xEA;
			HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
			HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
			TP_DEBUG ( "[MSG2133A] dbbus_rx version[0]=0x%x \n", dbbus_rx_data[0] );
			if ( dbbus_rx_data[0] == 3 ){
				IC_ID = 2;
				printk("[MSG2133A][The CTP is MSG2133A ---llf] OK!C33**********\n");
			}
			else
			{
				IC_ID = 3;
				printk("[MSG2133A][The CTP is   MSG2133A ---llf] OK! C32**********\n");
			}
		}
		else
		{
			IC_ID = 1;
			printk("[MSG2133A][The CTP is  MSG2133---llf] OK!**********\n");
		}
	}
	
	INIT_WORK(&msg21xx_wq, msg21xx_data_disposal);
	err = msg21xx_init_input();
	if(err < 0)
	{
		printk("msg2133 input init failed\n");
		goto msg2133_input_init_failed;
	}

#ifdef TP_DEVICE_INFO
	tp_proc_file = create_proc_entry("tp_info", 0666, NULL);
	if(tp_proc_file != NULL){
		tp_proc_file->read_proc = tp_info_read_proc_t;
		tp_proc_file->write_proc = tp_info_write_proc_t;
	}else {
		printk("tp device info proc file created failed..............\n");
		goto tp_info_proc_file_failed;
	}
#endif

	err = request_irq(msg21xx_irq, msg21xx_interrupt,
							IRQF_TRIGGER_FALLING, "msg21xx", NULL);
	if (err != 0) {
		printk("%s: cannot register irq\n", __func__);
		goto msg21xx_irq_request_failed;
	}
#if defined(CONFIG_FB)
        msg2138_data->fb_notif.notifier_call = fb_notifier_callback;
        err = fb_register_client(&msg2138_data->fb_notif);
        if (err)
                dev_err(&client->dev, "Unable to register fb_notifier: %d\n", err);
#endif

//	early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
//	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
//	early_suspend.suspend = msg21xx_suspend;
//	early_suspend.resume = msg21xx_resume;
//	register_early_suspend(&early_suspend);

	/*********  frameware upgrade *********/
	ito_test_create_entry();
	
#ifdef __FIRMWARE_UPDATE__
	firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
	if (IS_ERR(firmware_class))
		pr_err("Failed to create class(firmware)!\n");

	firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
	if (IS_ERR(firmware_cmd_dev))
		pr_err("Failed to create device(firmware_cmd_dev)!\n");

    // version
	if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
	if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
	if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

	dev_set_drvdata(firmware_cmd_dev, NULL);
	
#ifdef AUTO_UPDATE //having a problem that in order to update it should be regconized two different IC(Msg213/2133A)
	if ( version_cat()== 1 ){
		printk("%s: =========Start to upate=====\n", __func__);
		firmware_update();
		version_cat();
	}
	else
	{
		printk("%s: =============No need to update!\n", __func__);
	}
#endif
#endif //__FIRMWARE_UPDATE__
	printk("[TP] msg2331 probe OK!\n");
	return 0;

msg21xx_irq_request_failed:
#ifdef TP_DEVICE_INFO
	remove_proc_entry("tp_info", NULL);
tp_info_proc_file_failed:
#endif
	input_unregister_device(input);
	input_free_device(input);
msg2133_input_init_failed:
	cancel_work_sync(&msg21xx_wq);
msg2133_i2c_read_failed:
	msg21xx_i2c_client = NULL;
	tp_unconfig_pins(msg21xx_irq);
exit_config_pins_failed:
pwr_deinit:
	msg2138_power_init(false);
power_init_failed:
	return err;
}

static int __devexit msg21xx_remove(struct i2c_client *client)
{
   // unregister_early_suspend(&early_suspend);
#ifdef TP_DEVICE_INFO
	remove_proc_entry("tp_info", NULL);
	if(tp_info != NULL)
        kfree(tp_info);
#endif
	input_unregister_device(input);
	input_free_device(input);
	cancel_work_sync(&msg21xx_wq);
	msg21xx_i2c_client = NULL;
	tp_unconfig_pins(msg21xx_irq);
//	tp_pwroff();
	return 0;
}

static const struct i2c_device_id msg21xx_id[] = {
	{ "ms-msg21xx", 0},
	{ }
};

static struct of_device_id msg2138_match_table[] = {
        { .compatible = "m-star,msg2138",},
        { },
};

MODULE_DEVICE_TABLE(i2c, msg21xx_id);

static struct i2c_driver msg21xx_driver = {
	.driver = {
		   .name = "ms-msg21xx",
		   .owner = THIS_MODULE,
		   .of_match_table = msg2138_match_table,
	},
	.probe = msg21xx_probe,
	.remove = __devexit_p(msg21xx_remove),
	.id_table = msg21xx_id,
};

static int __init msg21xx_init(void)
{
	int err;
	printk("%s\n",__func__);
	err = i2c_add_driver(&msg21xx_driver);
	if (err) {
		printk(KERN_WARNING "msg21xx  driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          msg21xx_driver.driver.name);
	}
	return err;
}

static void __exit msg21xx_exit(void)
{
	printk("%s\n",__func__);
	i2c_del_driver(&msg21xx_driver);
}

module_init(msg21xx_init);
module_exit(msg21xx_exit);

MODULE_AUTHOR("Mstar semiconductor");
MODULE_DESCRIPTION("Driver for msg21xx Touchscreen Controller");
MODULE_LICENSE("GPL");

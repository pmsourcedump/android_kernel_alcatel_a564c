#ifndef __LINUX_MSG2133_TS_H__
#define __LINUX_MSG2133_TS_H__

#include <linux/earlysuspend.h>

#define TS_DEBUG_MSG 		0
#define TS_DEBUG_MSG_UPDATE 	0
#define VIRTUAL_KEYS			0
#define MSG2133_UPDATE		1
#define MSG2133_PROXIMITY       0

#ifdef MSG2133_UPDATE
#define MSG2133_SOFTWARE_UPDATE	 0

#define MSG2133_AUTO_UPDATE	 1
#define MSG2133_APK_UPDATE	 0
#define _FW_UPDATE_C3_              1
#endif


#define TS_WIDTH_MAX		480
#define	TS_HEIGHT_MAX		800

#define MSG2133_BUS_NUM		2
#define MSG2133_TS_NAME	   	"msg2133a"
#define MSG2133_TS_ADDR		0x26

#define MSG2133_FW_ADDR		0x62
#define MSG2133_FW_UPDATE_ADDR  0x49	


#define TPD_OK 			0
#define TPD_REG_BASE 		0x00
#define TPD_SOFT_RESET_MODE 	0x01
#define TPD_OP_MODE 		0x00
#define TPD_LOW_PWR_MODE 	0x04
#define TPD_SYSINFO_MODE 	0x10

#define GET_HSTMODE(reg)  ((reg & 0x70) >> 4)  // in op mode or not 
#define GET_BOOTLOADERMODE(reg) ((reg & 0x10) >> 4)  // in bl mode 


struct ts_event {
    u16	x1;
    u16	y1;
    u16	x2;
    u16	y2;
    u8  touch_point;
};
#if 0
struct msg2133_ts_data {
    struct i2c_client *client;	//zcf
    int(*power)(int on);
    struct input_dev	*input_dev;
    struct ts_event		event;
    struct work_struct 	pen_event_work;
    struct workqueue_struct *ts_workqueue;
    struct early_suspend	early_suspend;
};
#endif

struct msg2133_ts_data {
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct input_dev *keypad;
    struct regulator *vdd;
    struct regulator *vcc_i2c;	
    struct ts_event		event;
    //int use_irq;
    //struct hrtimer timer;
    struct work_struct  pen_event_work;
    struct workqueue_struct *ts_workqueue;
    int (*power)(int on);
    int suspended;
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
#elif defined (CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
};

struct fw_version {
    unsigned short major;
    unsigned short minor;
    unsigned short VenderID;
};

#if 1
struct msg2133_i2c_platform_data {
    int reset_gpio;
    int irq_gpio;
    u32 reset_gpio_flags;
    u32 irq_gpio_flags;
    int (*power) (int on);
    int x_num;
    int y_num;
    /*Add by reeson for keypad map-b-*/
    //char * kl_name;
    /*Add by reeson for keypad map-e-*/
};
#else
struct msg2133_i2c_platform_data {
    struct fw_upgrade_info info;
    const char *name;
    const char *fw_name;
    u32 irqflags;
    u32 irq_gpio;
    u32 irq_gpio_flags;
    u32 reset_gpio;
    u32 reset_gpio_flags;
    u32 family_id;
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 panel_minx;
    u32 panel_miny;
    u32 panel_maxx;
    u32 panel_maxy;
    u32 group_id;
    u32 hard_rst_dly;
    u32 soft_rst_dly;
    u32 num_max_touches;
    bool fw_vkey_support;
    bool no_force_update;
    bool i2c_pull_up;
    bool ignore_id_check;
    int (*power_init) (bool);
    int (*power_on) (bool);
}
#endif

enum {
    TP_HUANGZE = 1, //huangze
    TP_JUNDA,
    TP_MUDOND,
};

enum {
    PROX_NORMAL = 0,
    PROX_CLOSE,
    PROX_OFF,
};

enum {
    MODE_NORMAL = 0,
    MODE_RINGTONE,
    MODE_IN_CALL,
    MODE_IN_COMMUNICATION,
    MODE_WAITING,
    MODE_MAX
};

typedef enum bool_t
{
    FALSE = 0,
    TRUE  = 1
} bool_t;

typedef enum
{
    EMEM_ALL = 0,
    EMEM_MAIN,
    EMEM_INFO,
} EMEM_TYPE_t;


#endif

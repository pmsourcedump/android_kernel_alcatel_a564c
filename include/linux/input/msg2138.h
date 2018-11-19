#ifndef __LINUX_MSG21XX_TS_H__
#define __LINUX_MSG21XX_TS_H__

#define    MSG21XX_TOUCH_KEY
#define AUTO_UPDATE
//#define MSTAR_DEBUG		0
#ifdef MSTAR_DEBUG
#define MSTAR_DBG(format, ...)	\
		printk(KERN_INFO "MSTAR_TS " format "\n", ## __VA_ARGS__)
#define	TP_DEBUG(format, ...)	\
		printk(KERN_INFO "MSTAR_TS " format "\n", ## __VA_ARGS__)
#else
#define MSTAR_DBG(fmt, ...)
#define TP_DEBUG(fmt, ...)
#endif
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

struct msg21xx_ts_data
{
	uint16_t            addr;
	struct i2c_client  *client;
	struct input_dev   *input_dev;
	int                 use_irq;
	struct work_struct  work;
	int (*power)(int on);
	int (*get_int_status)(void);
	void (*reset_ic)(void);
};
#endif

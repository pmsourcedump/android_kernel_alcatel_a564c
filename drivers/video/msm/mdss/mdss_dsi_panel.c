/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>

#include "mdss_dsi.h"
#ifdef CONFIG_PROJECT_SOUL35EVDO
#include "dsi_v2.h"
#include "dsi_io_v2.h"
#include "dsi_host_v2.h"
#include "mdss_debug.h"
#endif
#include <linux/proc_fs.h> //add by yusen.ke.sz@tcl.com at 20140414 for display Mipi clk

#define DT_CMD_HDR 6

#ifdef CONFIG_PROJECT_SOUL35EVDO
struct wr_cmd_payload {
	u16 addr;
	u32 data;
} __packed;
static struct dsi_buf dsi_panel_tx_buf;
static struct dsi_buf dsi_panel_rx_buf;
#define DSI_BUF_SIZE	1024

#endif



DEFINE_LED_TRIGGER(bl_led_trigger);
//add by yusen.ke.sz@tcl.com for read board id begin
uint32_t socinfo_get_hw_version(void);
uint32_t iFramerate=0;
char sPanelName[20]= {0};

//add end
//#ifndef CONFIG_PROJECT_SOUL35EVDO
extern int qpnp_set_backlight_en(int value); //legen
//#endif
void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
	}
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled)
			pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_enabled) {
		pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
	}

	ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config_us() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
	ctrl->pwm_enabled = 1;
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: level=%d\n", __func__, level);

	led_pwm1[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

void mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
			gpio_set_value((ctrl_pdata->disp_en_gpio), 1);

		for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
			gpio_set_value((ctrl_pdata->rst_gpio),
				pdata->panel_info.rst_seq[i]);
			if (pdata->panel_info.rst_seq[++i])
				usleep(pdata->panel_info.rst_seq[i] * 1000);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
	}
}

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

static struct dsi_cmd_desc partial_update_enable_cmd[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(caset)}, caset},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static int mdss_dsi_panel_partial_update(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct dcs_cmd_req cmdreq;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	caset[1] = (((pdata->panel_info.roi_x) & 0xFF00) >> 8);
	caset[2] = (((pdata->panel_info.roi_x) & 0xFF));
	caset[3] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF00) >> 8);
	caset[4] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF));
	partial_update_enable_cmd[0].payload = caset;

	paset[1] = (((pdata->panel_info.roi_y) & 0xFF00) >> 8);
	paset[2] = (((pdata->panel_info.roi_y) & 0xFF));
	paset[3] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF00) >> 8);
	paset[4] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF));
	partial_update_enable_cmd[1].payload = paset;

	pr_debug("%s: enabling partial update\n", __func__);
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = partial_update_enable_cmd;
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return rc;
}
#ifndef CONFIG_PROJECT_SOUL35EVDO
extern int ESD_EHECK_BL;
#endif

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/*
	 * Some backlight controllers specify a minimum duty cycle
	 * for the backlight brightness. If the brightness is less
	 * than it, the controller can malfunction.
	 */

	if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
		bl_level = pdata->panel_info.bl_min;

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		//#ifndef CONFIG_PROJECT_SOUL35EVDO
		pr_info("%s: qpnp_set_backlight_en,bl_level=%d\n",__func__,bl_level);
		
		qpnp_set_backlight_en(bl_level);//legen
		
		//#endif
		//led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		break;
	case BL_GPIO_CMD:
		//pr_info("%s: BL_GPIO_CMD qpnp_set_backlight_en,bl_level=%d\n",__func__,bl_level);
		qpnp_set_backlight_en(bl_level);
	//	led_trigger_event(bl_led_trigger, bl_level);
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

#ifndef CONFIG_PROJECT_SOUL35EVDO
//baoqiang.qin add for display panel
#define MDSS_PANEL_ESD_CHECK_PERIOD msecs_to_jiffies(2600)
#define MDSS_PANEL_ESD_CHECK_START msecs_to_jiffies(16000)
#define TE_MONITOR_TO 168
//static int mdss_dsi_panel_on(struct mdss_panel_data *pdata);
//static int mdss_dsi_panel_off(struct mdss_panel_data *pdata);
static DECLARE_WAIT_QUEUE_HEAD(esd_thread_wq);
int esd_thread_timeout=0;
static bool queue_work_runing=false;

void wake_up_panel_to_suspend(void)
{
	esd_thread_timeout=1;
	wake_up(&esd_thread_wq);
}

static int mdss_dsi_panel_esd_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (ctrl->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
	
	pr_err("%s:-\n", __func__);//add by keys 20140305 for test
	
	return 0;
}

static int mdss_dsi_panel_esd_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;	

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	printk("[legen]--%s--\n",__func__);
	

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi  = &pdata->panel_info.mipi;

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

	//pr_debug("%s:-\n", __func__);//del by keys 20140305 for test
	pr_err("%s:-\n", __func__);//add by keys 20140305 for test
	return 0;
}

static int mdss_dsi_panel_regulator_on(struct mdss_panel_data *pdata, bool enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl=NULL;
	int ret;

	ctrl=container_of(pdata,struct mdss_dsi_ctrl_pdata, panel_data);

	mutex_lock(&ctrl->mutex);
	if(enable){
		ret=msm_dss_enable_vreg(
					ctrl->power_data.vreg_config,
					ctrl->power_data.num_vreg,
					1);
		if(ret){
			pr_err("%s:DSI power on failed\n",__func__);
			mutex_unlock(&ctrl->mutex);
			return ret;
		}
		pr_info("%s: success to power on panel.\n",__func__);
	}else{
		ret=msm_dss_enable_vreg(
				ctrl->power_data.vreg_config,
				ctrl->power_data.num_vreg,
				0);
		if(ret){
			pr_err("%s: DSI power off failed\n",__func__);
			mutex_unlock(&ctrl->mutex);
			return ret;
		}
		pr_info("%s: success to power off panel.\n",__func__);
	}
	mutex_unlock(&ctrl->mutex);
	return ret;
}

static int mdss_dsi_esd_panel_recovery(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl=NULL;
	static unsigned int esd_err_num;
	
	ctrl=container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	esd_err_num++;
	pr_info("%s: enter!\n",__func__);
	ctrl->panel_esd_data.esd_recovery_run=true;
	//dsi_set_tx_power_mode(0);
	mdss_dsi_panel_esd_off(pdata);
	//dsi_set_tx_power_mode(1);
	mdss_dsi_panel_reset(pdata, 0);
	mdss_dsi_panel_regulator_on(pdata,0);
	msleep(150);
	mdss_dsi_panel_regulator_on(pdata,1);
	mdss_dsi_panel_reset(pdata,1);
	//dsi_set_tx_power_mode(0);
	mdss_dsi_panel_esd_on(pdata);
	//dsi_set_tx_power_mode(1);
	pr_info("%s: done!\n",__func__);
	ctrl->panel_esd_data.esd_recovery_run=false;

	return 0;
}
static int mdss_dsi_te_monitor(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;
	
	if(ctrl==NULL){
		pr_err("%s: Invalid input data!\n",__func__);
		return -EINVAL;
	}
	INIT_COMPLETION(ctrl->panel_esd_data.te_detected);
	enable_irq(ctrl->panel_esd_data.te_irq);
	ret = wait_for_completion_timeout(&ctrl->panel_esd_data.te_detected,
						msecs_to_jiffies(TE_MONITOR_TO));
	if(ret==0){
		pr_warning("%s: No TE Sig for %d usec. Trigger ESD recovery\n",
				__func__,TE_MONITOR_TO);
		mdss_dsi_esd_panel_recovery(&ctrl->panel_data);
	}
	
	return 0;
}

static void mdss_dsi_esd_work(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	ctrl=container_of(work, struct mdss_dsi_ctrl_pdata, esd_work.work);
	if(ctrl==NULL)	{
		pr_err("%s: fail to get panel ctrl data!\n",__func__);
		goto end;
	}
	queue_work_runing=true;
	if(ctrl->panel_esd_data.esd_detect_mode==ESD_TE_DET){
		mdss_dsi_te_monitor(ctrl);
	}
	wake_up_panel_to_suspend();
	queue_delayed_work(ctrl->panel_esd_data.esd_wq, &ctrl->esd_work,
							MDSS_PANEL_ESD_CHECK_PERIOD);
end:
	pr_debug("%s: end of esd work!\n",__func__);
	queue_work_runing=false;
}

static irqreturn_t mdss_panel_esd_te_irq_handler(int irq, void *ctrl_ptr)
{
	struct mdss_dsi_ctrl_pdata *ctrl=
				(struct mdss_dsi_ctrl_pdata *)ctrl_ptr;
	complete(&ctrl->panel_esd_data.te_detected);
	disable_irq_nosync(ctrl->panel_esd_data.te_irq);
	//pr_info("%s: has been called!\n",__func__);
	return IRQ_HANDLED;
}

static int mdss_dsi_panel_esd_init(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;
	struct mdss_panel_esd_pdata *esd_data = &ctrl->panel_esd_data;

	INIT_DELAYED_WORK_DEFERRABLE(&ctrl->esd_work, mdss_dsi_esd_work);
	if(ctrl->disp_te_gpio!=12)
		ctrl->disp_te_gpio=12;
	//ctrl->disp_te_gpio=13;
	
	if(esd_data->esd_detect_mode == ESD_TE_DET){
		init_completion(&esd_data->te_detected);
		esd_data->te_irq = gpio_to_irq(ctrl->disp_te_gpio);
		ret = request_irq(esd_data->te_irq,
				mdss_panel_esd_te_irq_handler,
				IRQF_TRIGGER_RISING,"mdss_panel_esd_te",ctrl);
		if(ret<0){
			pr_err("%s: unable to request IRQ %d\n",__func__,ctrl->disp_te_gpio);
			return -EINVAL;
		}
		pr_info("%s: Successful to request IRQ, te_pin=%d\n",__func__,ctrl->disp_te_gpio);
	}
	return 0;
}

static int mdss_dsi_panel_esd(struct mdss_panel_data *pdata)
{
	int ret;
	static bool esd_work_queue_init;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	ctrl = container_of(pdata,struct mdss_dsi_ctrl_pdata, panel_data);
	if(ctrl->panel_esd_enable &&
		ctrl->panel_esd_data.esd_detection_run==false &&
		ctrl->panel_esd_data.esd_recovery_run==false){
		if(esd_work_queue_init==false){
			ret=mdss_dsi_panel_esd_init(ctrl);
			if(ret)
				return ret;
			esd_work_queue_init=true;
		}
		queue_delayed_work(ctrl->panel_esd_data.esd_wq, &ctrl->esd_work,
				MDSS_PANEL_ESD_CHECK_START);
		ctrl->panel_esd_data.esd_detection_run=true;
		pr_info("%s: start the ESD work queue.\n",__func__);
	}
	return 0;
}

static int mdss_dsi_panel_esd_workqueue_enable(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if(ctrl->panel_esd_enable){
		ctrl->panel_esd_data.esd_wq =
				create_singlethread_workqueue("mdss_panel_esd");
		if(ctrl->panel_esd_data.esd_wq == NULL){
			pr_err("%s: failed to create ESD work queue.\n",__func__);
			ctrl->panel_esd_enable = false;
		}
	}else
		pr_info("MDSS Panel: Esd detection is disable!\n");

	return 0;
}

//baoqiang.qin add end 20140324
#endif
bool panel_on_flag; //baoqiang add to fix backlight blink issue

#ifdef CONFIG_PROJECT_SOUL35EVDO
int mipi_d2l_write_reg(struct mdss_dsi_ctrl_pdata *ctrl, u16 reg, u32 data)
{
	struct wr_cmd_payload payload;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc cmd_write_reg = {
	{DTYPE_MAX_PKTSIZE, 1, 0, 1, 5, sizeof(payload)}, (char *)&payload};
	payload.addr = reg;
	payload.data = data;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &cmd_write_reg;
	cmdreq.cmds_cnt = 1;	
	cmdreq.flags =  CMD_REQ_COMMIT|CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return 0;
}


u32 mipi_d2l_read_reg(struct mdss_dsi_ctrl_pdata *ctrl, u16 reg)
{
	u32 data;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc cmd_read_reg = {	
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(reg)}, (char *) &reg};
	mdss_dsi_buf_init(&dsi_panel_rx_buf);
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &cmd_read_reg;
	cmdreq.cmds_cnt = 1;
	//cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT|CMD_REQ_LP_MODE;

	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT|CMD_REQ_LP_MODE;;

	cmdreq.rbuf = dsi_panel_rx_buf.data;
	cmdreq.rlen = 4;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	data = *(u32 *)dsi_panel_rx_buf.data;
	return data;

}

#endif

#if 1 //def CONFIG_PROJECT_SOUL35EVDO
int bkl_flag=0;
#endif


static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	//static unsigned int panel_on_first=0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	printk("[legen]--%s--\n",__func__);


	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (ctrl->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);

	
	#if 1 //def CONFIG_PROJECT_SOUL35EVDO
    bkl_flag=1;
    #endif

	panel_on_flag=true;
	
#ifndef CONFIG_PROJECT_SOUL35EVDO //baoqiang.qin add for esd detecting	
	if(ctrl->panel_esd_data.esd_detection_run==false)
		mdss_dsi_panel_esd(pdata);//legen
#endif //baoqiang add for esd detect end
	//pr_debug("%s:-\n", __func__);//del by keys 20140305 for test
	pr_err("%s:-\n", __func__);//add by keys 20140305 for test
	
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi  = &pdata->panel_info.mipi;
	#if 0 //ndef CONFIG_PROJECT_SOUL35EVDO
	if(ESD_EHECK_BL==1){
		qpnp_set_backlight_en(0);//legen
	}
    #endif
#ifndef CONFIG_PROJECT_SOUL35EVDO //baoqiang.qin add for esd
	if(queue_work_runing==true)
		wait_event(esd_thread_wq, esd_thread_timeout);
	
	if(ctrl->panel_esd_enable &&
			ctrl->panel_esd_data.esd_detection_run==true){
			cancel_delayed_work(&ctrl->esd_work);
			ctrl->panel_esd_data.esd_detection_run=false;
			pr_info("%s: cancel esd workqueue!\n",__func__);
	}
#endif//baoqiang.qin add end
	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

	//pr_debug("%s:-\n", __func__);//del by keys 20140305 for test
	pr_err("%s:-\n", __func__);//add by keys 20140305 for test
	return 0;
}

static void mdss_dsi_parse_lane_swap(struct device_node *np, char *dlane_swap)
{
	const char *data;

	*dlane_swap = DSI_LANE_MAP_0123;
	data = of_get_property(np, "qcom,mdss-dsi-lane-map", NULL);
	if (data) {
		if (!strcmp(data, "lane_map_3012"))
			*dlane_swap = DSI_LANE_MAP_3012;
		else if (!strcmp(data, "lane_map_2301"))
			*dlane_swap = DSI_LANE_MAP_2301;
		else if (!strcmp(data, "lane_map_1230"))
			*dlane_swap = DSI_LANE_MAP_1230;
		else if (!strcmp(data, "lane_map_0321"))
			*dlane_swap = DSI_LANE_MAP_0321;
		else if (!strcmp(data, "lane_map_1032"))
			*dlane_swap = DSI_LANE_MAP_1032;
		else if (!strcmp(data, "lane_map_2103"))
			*dlane_swap = DSI_LANE_MAP_2103;
		else if (!strcmp(data, "lane_map_3210"))
			*dlane_swap = DSI_LANE_MAP_3210;
	}
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	data = of_get_property(np, link_key, NULL);
	if (data && !strcmp(data, "dsi_hs_mode"))
		pcmds->link_state = DSI_HS_MODE;
	else
		pcmds->link_state = DSI_LP_MODE;

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


static int mdss_panel_dt_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

//add by yusen.ke.sz@tcl.com at 20140414 for display panel-framerate begin
static int ts_switch_read(char *page, char **start, off_t off,
			       int count, int *eof, void *data)
{
  int len = 0;

  len += sprintf(page + len, "%d\n", iFramerate);
   	if (len <= off+count)
		*eof = 1;

	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
   
}

static int ts_panel_name_read(char *page, char **start, off_t off,
			       int count, int *eof, void *data)
{
  int len = 0;

  len += sprintf(page + len, "%s\n", sPanelName);
   	if (len <= off+count)
		*eof = 1;

	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
   
}


static void init_adb_proc(void)
{
	struct proc_dir_entry *ts_switch_file;
	
	ts_switch_file = create_proc_read_entry("panel-framerate", 0, NULL, ts_switch_read,NULL);
	if (ts_switch_file) {
		printk(KERN_ERR "panel-framerate: init_log_proc create_proc_entry success\n");
	} 
	else
		printk(KERN_ERR "panel-framerate: init_log_proc create_proc_entry fails\n");

	ts_switch_file = create_proc_read_entry("panel-Name", 0, NULL, ts_panel_name_read,NULL);
	if (ts_switch_file) {
		printk(KERN_ERR "panel-Name: init_log_proc create_proc_entry success\n");
	} 
	else
		printk(KERN_ERR "panel-Name: init_log_proc create_proc_entry fails\n");
}

//add by yusen.ke.sz@tcl.com at 20140414 for display panel-framerate end

static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i, len;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	 pr_info("%s:%d, mdss_panel_parse_dt start\n",__func__, __LINE__);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->xres = (!rc ? tmp : 640);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		tmp = 1;
	rc = mdss_panel_dt_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, tmp,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	if (pdest) {
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strcmp(pdest, "display_1"))
			pinfo->pdest = DISPLAY_1;
		else if (!strcmp(pdest, "display_2"))
			pinfo->pdest = DISPLAY_2;
		else {
			pr_debug("%s: pdest not specified. Set Default\n",
								__func__);
			pinfo->pdest = DISPLAY_1;
		}
	} else {
		pr_err("%s: pdest not specified\n", __func__);
		return -EINVAL;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	pinfo->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} 
		else if (!strncmp(data, "bl_ctrl_gpio", 12)) 
					{
						led_trigger_register_simple("bkl-trigger", &bl_led_trigger);
					 printk(KERN_ERR"%s: SUCCESS-> GPIO TRIGGER register\n", __func__);
						ctrl_pdata->bklt_ctrl = BL_GPIO_CMD;
					}
		else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-bank-select", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, dsi lpg channel\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_lpg_chan = tmp;
			tmp = of_get_named_gpio(np,
				"qcom,mdss-dsi-pwm-gpio", 0);
			ctrl_pdata->pwm_pmic_gpio = tmp;
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);
	//add by yusen.ke.sz@tcl.com for read board id begin	
	iFramerate = pinfo->mipi.frame_rate ;		
	//add end
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clockrate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	pinfo->mipi.lp11_init = of_property_read_bool(np,
					"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_fbc_params(np, pinfo);
	#ifndef CONFIG_PROJECT_SOUL35EVDO
//baoqiang.qin add for esd detect
	ctrl_pdata->panel_esd_enable = of_property_read_bool(np,
					"qcom,mdss-dsi-esd-detect-enabled");
	rc = of_property_read_u32(np,"qcom,mdss-dsi-te-detect-mode",&tmp);
	ctrl_pdata->panel_esd_data.esd_detect_mode = (!rc ? tmp : 0);
//baoqiang.qin add for esd detect end
	#endif
	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_lane_swap(np, &(pinfo->mipi.dlane_swap));

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
		"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

	return 0;

error:
	return -EINVAL;
}

#ifdef CONFIG_PROJECT_SOUL35EVDO
int dsi_buf_alloc_gzm(struct dsi_buf *dp, int size)
{
	dp->start = kmalloc(size, GFP_KERNEL);
	if (dp->start == NULL) {
		pr_err("%s:%u\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dp->end = dp->start + size;
	dp->size = size;

	if ((int)dp->start & 0x07) {
		pr_err("%s: buf NOT 8 bytes aligned\n", __func__);
		return -EINVAL;
	}

	dp->data = dp->start;
	dp->len = 0;
	return 0;
}
#endif

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	bool cont_splash_enabled;
	bool partial_update_enabled;

	if (!node) {
		pr_err("%s: no panel node\n", __func__);
		return -ENODEV;
	}
#ifdef CONFIG_PROJECT_SOUL35EVDO
	rc = dsi_buf_alloc_gzm(&dsi_panel_tx_buf,ALIGN(DSI_BUF_SIZE,SZ_4K));
	rc = dsi_buf_alloc_gzm(&dsi_panel_rx_buf,ALIGN(DSI_BUF_SIZE,SZ_4K));
#endif

	pr_debug("%s:%d\n", __func__, __LINE__);
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name){
		ctrl_pdata->panel_esd_data.esd_panel_name=NONE_PANEL;
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	}else{
		if(0==strncmp(panel_name,"HX8379A",strlen("HX8379A")))
			ctrl_pdata->panel_esd_data.esd_panel_name=HX8379A_PANEL;
		else
			ctrl_pdata->panel_esd_data.esd_panel_name=ILI9806C_PANEL;
		//pr_info("panel name=%d\n",ctrl_pdata->panel_esd_data.esd_panel_name);
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);
		//add by yusen.ke.sz@tcl.com at 20140414 fro display panel name
		if(strncmp(panel_name,"ili9806c",strlen("ili9806c"))==0)
		{
			strncpy(sPanelName,"BYD:ILI9806C",12);
		}
		else{
			strncpy(sPanelName,"TDT:HX8379A",11);
		}
		//add end
	}

	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}
	init_adb_proc(); //add by yusen.ke.sz@tcl.com at20140414 for display panel information
	if (cmd_cfg_cont_splash)
		cont_splash_enabled = of_property_read_bool(node,
				"qcom,cont-splash-enabled");
	else
		cont_splash_enabled = false;
	if (!cont_splash_enabled) {
		pr_info("%s:%d Continuous splash flag not found.\n",
				__func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
	} else {
		pr_info("%s:%d Continuous splash flag enabled.\n",
				__func__, __LINE__);

		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 1;
	}

	partial_update_enabled = of_property_read_bool(node,
						"qcom,partial-update-enabled");
	if (partial_update_enabled) {
		pr_info("%s:%d Partial update enabled.\n", __func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.partial_update_enabled = 1;
		ctrl_pdata->partial_update_fnc = mdss_dsi_panel_partial_update;
	} else {
		pr_info("%s:%d Partial update disabled.\n", __func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.partial_update_enabled = 0;
		ctrl_pdata->partial_update_fnc = NULL;
	}
	#ifndef CONFIG_PROJECT_SOUL35EVDO //legen add for esd detect
	//ctrl_pdata->panel_esd_enable=true;
	//ctrl_pdata->panel_esd_data.esd_detect_mode=ESD_TE_DET;
	mdss_dsi_panel_esd_workqueue_enable(ctrl_pdata);
	#endif //legen add for esd detect

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;

	return 0;
}

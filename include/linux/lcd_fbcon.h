/*
 * add by jch for imei number check lcd show,watchdog and spinlock function defined
 */

#ifndef __DEV_FBCON_H
#define __DEV_FBCON_H

void fbcon_setup(u32 format);
void fb_update_parm(u32 width, u32 height,u32 stride);
void fb_update_base(void *base);
void fbcon_check_word(char *str);
void fbcon_putc(char c);
void fbcon_show(void );
void fbcon_clear(void);
void set_spinlock_delay_times(void);
void pet_watchdog_ext(void);

#endif /* __DEV_FBCON_H */

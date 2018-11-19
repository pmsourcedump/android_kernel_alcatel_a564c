/*
 * add by jch for imei number check lcd show text
 * copy text char point to fb dma.
 */



#include <linux/string.h>
#include <linux/msm_mdp.h>
#include <linux/lcd_fbcon.h>//add by jch for imei number check
#include "lcd_font_def.h"
#include "mdss_fb.h"
#include <linux/printk.h>


struct pos {
	u32 x;
	u32 y;
};

#define CONFIG_FONT_12x22

#define MAX_FB_SIZE 540*960*4
#define MAX_USR_BUF_SIZE 1016
 unsigned char fb_byte[MAX_FB_SIZE]={0};
void *fb_addr =NULL;
u32	width_fb;
u32	height_fb;
u32	fb_stride;
int	fb_bpp_a;
u32  x_start;
u32  y_start;


#define RGB565_BLACK		0x0000
#define RGB565_WHITE		0xffff

#define RGBA8888_BLACK            0x00000000
#define RGBA8888_WHITE            0xffffffff


#define RGBA8888_RED            	0xff0000ff
#define RGBA8888_GREEN            0x00ff00ff
#define RGBA8888_BLUE            	0x0000ffff

#ifdef CONFIG_FONT_12x22
#define FONT_WIDTH		12
#define FONT_HEIGHT		22
#else
#define FONT_WIDTH		5
#define FONT_HEIGHT		12
#endif

static uint32_t			BGCOLOR;
static uint32_t			FGCOLOR;


static struct pos		cur_pos;
static struct pos		max_pos;

extern u32 splash_addr;//legen
//bool show_flag = false;
void fb_update_base(void *base)
{       
	 fb_addr = base;
}

void fbcon_setup(u32 format)
{		
	
       switch (format) {
	   case MDP_RGB_565:
		FGCOLOR= RGB565_WHITE;
		BGCOLOR = RGB565_BLACK;
		fb_bpp_a = 2;
		break;

	   case MDP_RGB_888:
		fb_bpp_a = 3;
		break;

	   case MDP_ARGB_8888:		
		fb_bpp_a = 4;
		break;

	    case MDP_RGBA_8888:
	       FGCOLOR = RGBA8888_WHITE;
              BGCOLOR = RGBA8888_BLACK;
		fb_bpp_a = 4;
		break;

	   case MDP_YCRYCB_H2V1:		
		fb_bpp_a = 2;
		break;

	   default:
		return;
		
	}
   
}

void fbcon_show_white(void )
{       
       unsigned long count = width_fb * height_fb*fb_bpp_a;
	memset(fb_byte, 0xff, count);
	cur_pos.x = x_start;
	cur_pos.y = y_start;
	
}

void fbcon_clear(void)
{
	unsigned long count = width_fb * height_fb*fb_bpp_a;
	memset(fb_byte, 0x00, count);
	cur_pos.x = x_start;
	cur_pos.y = y_start;
}

void fb_update_parm(u32 width, u32 height,u32 stride)
{
       height_fb = height;
	width_fb = width;
       fb_stride=width;
	max_pos.x = width_fb/(FONT_WIDTH+1);
	max_pos.y = (height_fb-1)/FONT_HEIGHT;
       x_start = 0;
       y_start =max_pos.y /3;
	fbcon_clear();
}
#ifdef CONFIG_FONT_12x22
//add by yusen.ke.sz at 20140527 for fix Security error information can not be displayed
static void fbcon_drawglyph_lager_lk(unsigned char *pixels, uint32_t paint, uint32_t stride,
			    unsigned char *glyph)
{	
	unsigned x, y,id_x,id_y;
       unsigned char data = 0;
	id_x = 0;
	id_y = 0;
	//0x1f, 0xc0, /* 0001 1111 1100 */
	for (y = 0; y < FONT_HEIGHT; y++) {			
              for (x = 0; x < FONT_WIDTH; x++) {
			id_x =  (x/8);
			if(x%8==0)
		              data = glyph[id_y+id_x];
			if (data & 0x80)
			{
				*pixels = (paint&0xff);
				*(pixels+1) = ((paint>>8)&0xff);
				*(pixels+2) = ((paint>>16)&0xff);
				//*(pixels+3) = ((paint>>24)&0xff);

			}
			data <<= 1;
			pixels += fb_bpp_a;

		}
              id_y += 2;
			  
		//pixels += 3* fb_bpp_a;
		pixels += (stride-FONT_WIDTH)*fb_bpp_a;

	}

}
//add end
#if 0 //del by yusen.ke.sz at 20140527 for fix Security error information can not be displayed
static void fbcon_drawglyph_lager(unsigned char *pixels, uint32_t paint, uint32_t stride,
			    unsigned char *glyph)
{	
	unsigned x, y,id_x,id_y;
       unsigned char data = 0;
	id_x = 0;
	id_y = 0;
	//0x1f, 0xc0, /* 0001 1111 1100 */
	for (y = 0; y < FONT_HEIGHT; y++) {			
              for (x = 0; x < FONT_WIDTH; x++) {
			id_x =  (x/8);
			if(x%8==0)
		              data = glyph[id_y+id_x];
			if (data & 0x80)
			{
				*pixels = (paint&0xff);
				*(pixels+1) = ((paint>>8)&0xff);
				*(pixels+2) = ((paint>>16)&0xff);
				*(pixels+3) = ((paint>>24)&0xff);

			}
			data <<= 1;
			pixels += fb_bpp_a;

		}
              id_y += 2;
			  
		pixels += 4 * fb_bpp_a;
		pixels += (stride-FONT_WIDTH)*fb_bpp_a;

	}

}
#endif
#else
static void fbcon_drawglyph(unsigned char *pixels, uint32_t paint, uint32_t stride,
			    unsigned *glyph)
{
	unsigned x, y, data;

	data = glyph[0];
	for (y = 0; y < (FONT_HEIGHT / 2); y++) {
		for (x = 0; x < FONT_WIDTH; x++) {
			if (data & 1)
			{
				*pixels = (paint&0xff);
				*(pixels+1) = ((paint>>8)&0xff);
				*(pixels+2) = ((paint>>16)&0xff);
				*(pixels+3) = ((paint>>24)&0xff);
			}
			data >>= 1;
			pixels += fb_bpp_a;

		}

		pixels += (stride-FONT_WIDTH)*fb_bpp_a;

	}

	data = glyph[1];
	for (y = 0; y < (FONT_HEIGHT / 2); y++) {
		for (x = 0; x < FONT_WIDTH; x++) {
			if (data & 1)
			{
				*pixels = (paint&0xff);
				*(pixels+1) = ((paint>>8)&0xff);
				*(pixels+2) = ((paint>>16)&0xff);
				*(pixels+3) = ((paint>>24)&0xff);
			}
			data >>= 1;

			pixels += fb_bpp_a;

		}

		pixels += (stride-FONT_WIDTH)*fb_bpp_a;

	}

}
#endif

 void fbcon_show(void)
{
	// unsigned long i;
	// unsigned long count = width_fb * height_fb*fb_bpp_a;
	 
	 if (!fb_addr  ){
	 	 printk("JCH:fbcon_show, fb_addr is NULL!\n ");
		 
 //modify by yusen.ke.sz at 20140527 for fix Security error information can not be displayed
		 if(splash_addr)
		 	fb_addr=phys_to_virt(splash_addr);
		 else
			return;
//modify end
	 }
       // printk("JCH:show text,  fb addr:0x%lx\n",(unsigned long)fb_addr);
#if 0
	 for(i=0;i<=count;i+=MAX_USR_BUF_SIZE){
		memcpy(fb_addr+i,fb_byte+i,MAX_USR_BUF_SIZE);
	 }	 
#else
	 memcpy(fb_addr,fb_byte,sizeof(fb_byte));
#endif
}

/* TODO: Take stride into account */
static void fbcon_scroll_up(void)
{
	unsigned char *dst = fb_byte;
	unsigned char *src = dst + (width_fb * FONT_HEIGHT);
	unsigned long count = width_fb * (height_fb- FONT_HEIGHT)*fb_bpp_a;

	while(count--) {
		*dst++ = *src++;
	}

	count = width_fb * FONT_HEIGHT;
	while(count--) {
		*dst++ = BGCOLOR;
	}
	
}
void fbcon_check_word(char *str)
{     
	unsigned word_len=0;
	unsigned word_find =0;
       unsigned char_count = 0;
	unsigned str_len = strlen(str);
	while(*str != 0) {
		char_count++;
		if(char_count>max_pos.x){
			str_len = str_len - char_count;
			char_count = 0;
		}
		
		if(*str =='\n' ||*str =='\r' ){
			str_len = str_len - char_count;
			char_count = 0;
		}
		
		if((*str++)==' '){
			//find next space		
                     while (word_len <= (str_len-char_count))
                     {
			        word_len++;
				 word_find=0;
			        if(str[word_len]==' '){
                                   word_find=word_len;
					word_len=0;
					break;
				  }
			}
		      
			if((word_find>0) && (max_pos.x < (char_count+word_find)) && char_count!=0){				
				*(str-1)= '\n';
				str_len = str_len - char_count;
                            char_count = 0;
			}
		}      
		
	}	

}

int bShow_lk = 0;//add by yusen.ke.sz at 20140527 for fix Security error information can not be displayed
void fbcon_putc(char c)
{
	unsigned char *fb_tmp_byte;   
	//add by yusen.ke.sz at 20140527 for fix Security error information can not be displayed
	if(NULL==fb_addr)
 		bShow_lk =1;
	if(bShow_lk==1)
		fb_bpp_a = 3;
	//add end
	if((unsigned char)c > 127)
		return;
	if((unsigned char)c < 32) {
		if(c == '\n')
			goto newline;
		else if (c == '\r')
			cur_pos.x = 0;
		return;
	}
       if(c == ' ' && cur_pos.x == 0)
	   	return;
	   	
	fb_tmp_byte =fb_byte;
	
	//fbcon_show_white();
//modify by yusen.ke.sz at 20140527 for fix Security error information can not be displayed		
		 fb_tmp_byte += (cur_pos.y * FONT_HEIGHT * width_fb*fb_bpp_a);	
//modify end
	fb_tmp_byte += (cur_pos.x * (FONT_WIDTH + 1))*fb_bpp_a;
	
#ifdef CONFIG_FONT_12x22       
//modify by yusen.ke.sz at 20140527 for fix Security error information can not be displayed
	fbcon_drawglyph_lager_lk(fb_tmp_byte, FGCOLOR, fb_stride,
			font_12x22 +( c*44));
//modify end	

#else
	fbcon_drawglyph(fb_tmp_byte, FGCOLOR, fb_stride,
			font5x12 + (c - 32) * 2);
#endif

	cur_pos.x++;
	if (cur_pos.x < max_pos.x)
		return;

newline:
	cur_pos.y++;
	cur_pos.x = 0;
	if(cur_pos.y >= max_pos.y) {
		cur_pos.y = max_pos.y - 1;
		fbcon_scroll_up();
	} 
}



#include <linux/ppp-ioctl.h>

/*add by luo.jun@tcl.com for PR607536 2014-03-08 begin*/ 
/*[VPN]The L2TP/IPSec PSK vpn can not connect normally*/ 
#define PPP_MTU     1500    /* Default MTU (size of Info field) */ 
#define PPP_MAXMRU  65000   /* Largest MRU we allow */ 
/*add by luo.jun@tcl.com for PR607536 2014-03-08 end*/

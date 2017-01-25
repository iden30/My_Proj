


#ifndef __RC5_H
#define __RC5_H 

#include <stdint.h>
 

extern void rc5_init(void);
extern void rc5_send(uint16_t * data);
extern void rc5_get(uint16_t * data);
extern void calc_rc5 (uint16_t *period_buf, uint16_t *data_buf);

#endif

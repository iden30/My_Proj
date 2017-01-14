


#ifndef __RC5_H
#define __RC5_H 

#include <stdint.h>



uint32_t ir_period_get(uint32_t ir_clk_hZ);
uint32_t pwm_duty_get(uint32_t duty_precent);
extern void rc5_send(uint32_t *data);
extern void rc5_get(uint32_t *data);
extern void rc5_init(void);

#endif

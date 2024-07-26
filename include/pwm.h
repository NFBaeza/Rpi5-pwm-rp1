#ifndef PWM_H
#define PWM_H

#include <string>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <bitset>
#include <time.h>
#include <unistd.h>
#include "rp1_reg.h"
#include "simpĺe_logger.h"

using namespace std;

typedef struct
{
    uint8_t number;
    volatile uint32_t *status;
    volatile uint32_t *ctrl;
    volatile uint32_t *pad;
} gpio_pin_t;

typedef struct
{
    volatile uint32_t *ctl;
    volatile uint32_t *range;
    volatile uint32_t *phase;
    volatile uint32_t *duty;
} pwm_t;

typedef struct
{
    uint32_t mode;
    uint32_t range;
    uint32_t phase;
    uint32_t duty;
    bool invert;
} pwm_config_t;

typedef struct
{
    volatile void *rp1_peripherial_base;
    volatile void *gpio_base;
    volatile void *pads_base;
    volatile uint32_t *pwm0_base;
    volatile uint32_t *pwm1_base;
    volatile uint32_t *sys_clock;
    volatile uint32_t *rio_out;
    volatile uint32_t *rio_output_enable;
    volatile uint32_t *rio_nosync_in;

    gpio_pin_t *pins[27];
    pwm_t *pwms[4];

} rp1_t;

static Logger PWM_RPI5_LOGGER("PWM LOG");
const uint32_t PWM_CHANNEL[4]={PWM_CHAN0_CTRL/4,PWM_CHAN1_CTRL/4,PWM_CHAN2_CTRL/4,PWM_CHAN3_CTRL/4};
static rp1_t *rp1;

class PWM_PI5 {

public:
    PWM_PI5(int pin, pwm_config_t pwm_conf);

    int  pwm_enable(int id, bool Enable, bool all_pwm = false);

private:
    void printbinary(uint32_t value);

    void *mapgpio(off_t dev_base, off_t dev_size);
    bool create_rp1(rp1_t **rp1);
    int  clk_create();
    bool create_pin(uint8_t pinnumber,  uint32_t funcmask);
    int  pin_enable_output(uint8_t pinnumber);
    int  getPWMIdx(int pin);
    int  pwm_create(int id);
    int  pwm_configuration(int id, pwm_config_t config);
};

#endif 
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
    uint32_t pin;
    uint32_t mode;
    uint32_t range;
    uint32_t phase = 0;
    uint32_t duty;
    bool invert = false;
    bool use_common_config = true;
} pwm_config_t;

typedef struct
{
    volatile uint32_t *ctl;
    volatile uint32_t *range;
    volatile uint32_t *phase;
    volatile uint32_t *duty;
    pwm_config_t pwm_config;
} pwm_t;


typedef struct
{
    uint32_t clk_div_int = 1;
    uint32_t clk_div_frac = 0;
    uint32_t common_range = 0;
    uint32_t common_duty = 0;
    bool debug =  false;
} pwm_common_config_t;

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
static uint32_t clk_main = 50000000;

class PWM_PI5 {

public:
    PWM_PI5(int pin, pwm_config_t pwm_config, pwm_common_config_t pwm_common_conf);
    ~PWM_PI5();

    int pwm_enable(bool Enable);
    void update_config(pwm_config_t pwm_config);
    void release();
    int pwm_enable_all(bool Enable);

private:
    static pwm_common_config_t pwm_common_config;
    static rp1_t *rp1;
    static uint32_t clk_pwm;
    pwm_config_t pwm_conf;
    int pwm_idx;
    int pwm_pin;
    
    void printbinary(uint32_t value);
    uint32_t clk_create();
    void *mapgpio(off_t dev_base, off_t dev_size);
    bool create_rp1();
    int  pwm_create();
    void release(int id);
    int  pwm_configuration(pwm_config_t pwm_config);
    bool create_pin(uint32_t funcmask);
    int  pin_enable_output();
    int  getPWMIdx();
    void  pwm_delete_all();
};

#endif  // PWM_PI5_H

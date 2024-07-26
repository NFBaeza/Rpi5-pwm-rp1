#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>
#include "simpĺe_logger.h"
#include "rp1_reg.h"
#include "pwm.h"

using namespace std;

const uint8_t pins[2] = {12, 13};

int main(void)
{
    printf("hola desde el main\n");
    pwm_config_t config_pwm_0;
    config_pwm_0.mode   = 0x06;
    config_pwm_0.range  = 1249;
    config_pwm_0.phase  = 0;
    config_pwm_0.duty   = 1250/2;
    config_pwm_0.invert = false;

    PWM_PI5 pwm12 = PWM_PI5(12, config_pwm_0);
    PWM_PI5 pwm13 = PWM_PI5(13, config_pwm_0);
    PWM_PI5 pwm14 = PWM_PI5(14, config_pwm_0);
    PWM_PI5 pwm15 = PWM_PI5(15, config_pwm_0);

    pwm12.pwm_enable(0,true,true);

    printf("adios desde el main\n");

}
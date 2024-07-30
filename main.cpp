#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "simpĺe_logger.h"
#include "pwm.h"

using namespace std;

const uint8_t pins[2] = {12, 13};

int main(void)
{
    printf("hello from main\n");
      pwm_config_t config_pwm_0 = { 
    .mode   = 0x02,
    .range = 1250/2,
    .duty = (1250)/4,
    .use_common_config = false,
  };

  pwm_config_t config_pwm_1 = {
    .mode   = 0x02,
    .range = 1250/2,
    .duty = (1250+24)/4,
    .invert = true,
    .use_common_config = false,
  };

  pwm_common_config_t global_conf;

  PWM_PI5 pwm0(12, config_pwm_0, global_conf);
  PWM_PI5 pwm1(13, config_pwm_1, global_conf);
  PWM_PI5 pwm2(14, config_pwm_0, global_conf);
  PWM_PI5 pwm3(15, config_pwm_1, global_conf);

    pwm0.enable(true);
    pwm2.enable(true);
    
    sleep(60);
    

    printf("bye bye from main\n");

}

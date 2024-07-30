    #include "pwm.h"
    ///----

    rp1_t* PWM_PI5::rp1 = nullptr;
    pwm_common_config_t PWM_PI5::pwm_common_config = pwm_common_config;
    uint32_t PWM_PI5::clk_pwm = 0;

    PWM_PI5::PWM_PI5(int pin, pwm_config_t pwm_config, pwm_common_config_t pwm_common_conf)
    {
        if (rp1 == nullptr){
            if(!create_rp1()){
                if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(ERROR, "error creating rp1");}
            };
            pwm_common_config = pwm_common_conf;
            clk_pwm = clk_create();
            *(rp1->pwm0_base + 2) = pwm_common_conf.common_range;
            *(rp1->pwm0_base + 3) = pwm_common_conf.common_duty;
        }

        pwm_pin = pin;
        pwm_idx = getPWMIdx();
        
        if (pwm_idx >= 0){
            if(!pwm_create()) {
                if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(ERROR, "unable to create pwm %d",pwm_idx);}
                return;
            };
            pwm_configuration(pwm_config);
        } else {
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(ERROR, "unable to create pin %d",pwm_pin);}
            return;
        }

        int func = (pwm_pin > 15) ? 0x03 : 0x00;
        if(!create_pin(func)) {
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(ERROR,"unable to create pin %d", pwm_pin);}
            return;
        }; 
        pin_enable_output();
    }

    PWM_PI5::~PWM_PI5() {
        // Log the beginning of destruction process
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "Destructing PWM_PI5 instance");}

        pwm_delete_all();

        // Unmap the memory mapped for GPIO access
        if (rp1->rp1_peripherial_base != NULL) {
            if (munmap((void *)rp1->rp1_peripherial_base, RP1_BAR1_LEN) == -1) {
                if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(ERROR, "Error unmapping memory");}
            } else {
                if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "Memory unmapped successfully");}
            }
        }

        // Free the rp1 structure
        if (rp1 != NULL) {
            free(rp1);
            rp1 = NULL;
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "Resource struct rp1 freed");}
        }
    }

    void PWM_PI5::release(){
        if (rp1->pwms[pwm_idx] != nullptr) {
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(WARNING, "deleting pwm channel %d).",pwm_idx);}
            uint32_t temp = (1 << pwm_idx);
            temp = (~temp) & 0xF;
            temp = (*(rp1->pwm0_base) & temp);

            *(rp1->pwms[pwm_idx]->ctl    + RP1_ATOM_CLR_OFFSET / 4) = 0xFF;  //limpieza
            *(rp1->pwms[pwm_idx]->range  + RP1_ATOM_SET_OFFSET / 4) = 0;
            *(rp1->pwms[pwm_idx]->phase  + RP1_ATOM_SET_OFFSET / 4) = 0;
            *(rp1->pwms[pwm_idx]->duty   + RP1_ATOM_SET_OFFSET / 4) = 0;

            *(rp1->pwm0_base) = (temp | 0x80000000);
            free(rp1->pwms[pwm_idx]);
            rp1->pwms[pwm_idx] = nullptr;
        }
       
        if (rp1->pins[pwm_pin] != nullptr) {
            *(rp1->pins[pwm_pin]->ctrl + RP1_ATOM_CLR_OFFSET / 4) = 0x9f; // Clear function select bits
            *(rp1->pins[pwm_pin]->pad + RP1_ATOM_CLR_OFFSET / 4) = 0x96; // Clear pad configuration bits
            free(rp1->pins[pwm_pin]);
            rp1->pins[pwm_pin] = nullptr;
        }

        //debería baja los puntos
        *(rp1->rio_out + RP1_ATOM_CLR_OFFSET / 4) = 1 << pwm_pin;
        *(rp1->rio_output_enable + RP1_ATOM_CLR_OFFSET / 4) = 1 << pwm_pin;
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(WARNING, "pwm %d delete", pwm_idx);}
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(WARNING, "gpio %d off", pwm_pin);}
    } 

     void PWM_PI5::release(int id){
        if (rp1->pwms[id] != nullptr) {
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(WARNING, "deleting pwm channel %d).",id);}
            uint32_t temp = (1 << id);
            temp = (~temp) & 0xF;
            temp = (*(rp1->pwm0_base) & temp);

            *(rp1->pwms[id]->ctl    + RP1_ATOM_CLR_OFFSET / 4) = 0xFF;  //limpieza
            *(rp1->pwms[id]->range  + RP1_ATOM_SET_OFFSET / 4) = 0;
            *(rp1->pwms[id]->phase  + RP1_ATOM_SET_OFFSET / 4) = 0;
            *(rp1->pwms[id]->duty   + RP1_ATOM_SET_OFFSET / 4) = 0;

            *(rp1->pwm0_base) = (temp | 0x80000000);
            free(rp1->pwms[id]);
            rp1->pwms[id] = nullptr;
        }
       
        if (rp1->pins[pwm_pin] != nullptr) {
            *(rp1->pins[pwm_pin]->ctrl + RP1_ATOM_CLR_OFFSET / 4) = 0x9f; // Clear function select bits
            *(rp1->pins[pwm_pin]->pad + RP1_ATOM_CLR_OFFSET / 4) = 0x96; // Clear pad configuration bits
            free(rp1->pins[pwm_pin]);
            rp1->pins[pwm_pin] = nullptr;
        }

        //debería baja los puntos
        *(rp1->rio_out + RP1_ATOM_CLR_OFFSET / 4) = 1 << pwm_pin;
        *(rp1->rio_output_enable + RP1_ATOM_CLR_OFFSET / 4) = 1 << pwm_pin;
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(WARNING, "pwm %d delete", id);}
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(WARNING, "gpio %d off", pwm_pin);}
    }

    void PWM_PI5::pwm_delete_all(){
         for(int  i = 0; i < 4 ; i++){
            if(rp1->pwms[i] != nullptr){
                    release(i);
            }
         }
    }

    void *PWM_PI5::mapgpio(off_t dev_base, off_t dev_size)
    {
        int fd;
        void *mapped;
        
        if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
        {
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(ERROR, "Can't open /dev/mem");}
            return (void *)0;
        }

        mapped = mmap(0, dev_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base);
        
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "base address: %llx, size: %x, mapped: %p", dev_base, dev_size, mapped);}

        if (mapped == (void *)-1)
        {
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(ERROR, "Can't map the memory to user space.");}
            return (void *)0;
        }

        close(fd);
        return mapped;
    }


    bool PWM_PI5::create_rp1()
    {

        rp1_t *r = (rp1_t *)calloc(1, sizeof(rp1_t));
        if (r == NULL){
            PWM_RPI5_LOGGER.log(ERROR, "no se creó el rpi1");
            return false;
        }

        void *base = mapgpio(RP1_BAR1, RP1_BAR1_LEN);
        if (base == NULL){
            PWM_RPI5_LOGGER.log(ERROR, "base es empty");
            return false;
        }

        r->rp1_peripherial_base = base;
        r->gpio_base = base + RP1_IO_BANK0_BASE;
        r->pads_base = base + RP1_PADS_BANK0_BASE;
        r->pwm0_base = (volatile uint32_t *)(base + RP1_PWM0_BASE);
        r->pwm1_base = (volatile uint32_t *)(base + RP1_PWM1_BASE);
        r->sys_clock = (volatile uint32_t *)(base + RP1_CLOCK_BASE);
        r->rio_out = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OUT_OFFSET);
        r->rio_output_enable = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OE_OFFSET);
        r->rio_nosync_in = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_NOSYNC_IN_OFFSET);

        rp1 = r;
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "device connected");}
        return true;
    }


    uint32_t PWM_PI5::clk_create()
    {
        *(rp1->sys_clock + CLK_PWM_CTRL/4) = 0x1000840;
        *(rp1->sys_clock + CLK_PWM_DIV_INT/4) = pwm_common_config.clk_div_int;
        *(rp1->sys_clock + CLK_PWM_DIV_FRAC/4) = pwm_common_config.clk_div_frac;
        *(rp1->sys_clock + CLK_PWM_SEL/4) = 1;

        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "clk config setted as 0x%llx", *(rp1->sys_clock + CLK_PWM_CTRL/4));}
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "clk config at %lu MHz", 50/ *(rp1->sys_clock + CLK_PWM_DIV_INT/4));}
        return clk_main/pwm_common_config.clk_div_int;
    }

    int PWM_PI5::pwm_create()
    { 
        pwm_t *newpwm = (pwm_t *)calloc(1, sizeof(pwm_t));
        if (newpwm == NULL)
            return false;

        newpwm->ctl     = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[pwm_idx]);
        newpwm->range   = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[pwm_idx] + PWM_CHAN_RANGE);
        newpwm->phase   = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[pwm_idx] + PWM_CHAN_PHASE);
        newpwm->duty    = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[pwm_idx] + PWM_CHAN_DUTY);

        rp1->pwms[pwm_idx] = newpwm;

        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwm %d created", pwm_idx);}

        return 1;
    }

    int PWM_PI5::pwm_configuration(pwm_config_t pwm_config)
    {
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO,"pwm %d configuring", pwm_idx);}

        *(rp1->pwms[pwm_idx]->ctl    + RP1_ATOM_CLR_OFFSET / 4) = 0xFF;  //limpieza
        *(rp1->pwms[pwm_idx]->range  + RP1_ATOM_CLR_OFFSET / 4) = 0;
        *(rp1->pwms[pwm_idx]->phase  + RP1_ATOM_CLR_OFFSET / 4) = 0;
        *(rp1->pwms[pwm_idx]->duty   + RP1_ATOM_CLR_OFFSET / 4) = 0;

        *(rp1->pwms[pwm_idx]->ctl    + RP1_ATOM_SET_OFFSET / 4) = pwm_config.mode;
        *(rp1->pwms[pwm_idx]->ctl    + RP1_ATOM_SET_OFFSET / 4) = (pwm_config.invert) ? (1<<3):(0<<3);
        *(rp1->pwms[pwm_idx]->range  + RP1_ATOM_SET_OFFSET / 4) = pwm_config.range; //1249;
        *(rp1->pwms[pwm_idx]->phase  + RP1_ATOM_SET_OFFSET / 4) = pwm_config.phase;//delay;
        *(rp1->pwms[pwm_idx]->duty   + RP1_ATOM_SET_OFFSET / 4) = pwm_config.duty; //1250/2;

        pwm_conf = pwm_config;
        rp1->pwms[pwm_idx]->pwm_config = pwm_config;
   
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwm %d config setted as 0x%llx", pwm_idx, *(rp1->pwms[pwm_idx]->ctl));}
        return 1;
    }

    void PWM_PI5::update_config(pwm_config_t pwm_config)
    {
        pwm_enable(false);
        pwm_configuration(pwm_config);
        pwm_enable(true);
    }

    bool PWM_PI5::create_pin(uint32_t funcmask)
    {
        gpio_pin_t *newpin = (gpio_pin_t *)calloc(1, sizeof(gpio_pin_t));
        if(newpin == NULL) return false;

        newpin->number = pwm_pin;

        newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pwm_pin);
        newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pwm_pin + 4);
        newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pwm_pin * 4);

        // set the function
        *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
        *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = funcmask;  // now set the value we need

        *(newpin->pad +  RP1_ATOM_SET_OFFSET / 4) = 0x3B; 
        
        rp1->pins[pwm_pin] = newpin;

        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pin %d stored in pins array %p", pwm_pin, rp1->pins[pwm_pin]);}
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO,"gpio %d set function %d",pwm_pin, funcmask);}
        return true;
    }

    int PWM_PI5::pin_enable_output()
    {
        volatile uint32_t *clr_add = rp1->pins[pwm_pin]->pad + RP1_ATOM_CLR_OFFSET / 4;
        volatile uint32_t *set_add = rp1->pins[pwm_pin]->pad + RP1_ATOM_SET_OFFSET / 4;

        // Limpiar los bits IE[6] y OD[7]
        *clr_add = 0xC4;  // Limpiar bits 6 y 7 (IE[6] y OD[7]
        *set_add = 0x3B;  // Configurar bits necesarios para salida

        *(rp1->rio_out + RP1_ATOM_SET_OFFSET / 4) = 1 << rp1->pins[pwm_pin]->number;
        *(rp1->rio_output_enable + RP1_ATOM_SET_OFFSET / 4) = 1 << rp1->pins[pwm_pin]->number;

        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "Pin %d pad config as output", pwm_pin);}
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "Pin %d pad register 0x%llx", pwm_pin, *(rp1->sys_clock + CLK_PWM_CTRL/4));}

        return 1;
    }

    int PWM_PI5::pwm_enable(bool Enable)
    {
        uint32_t temp = 1 << pwm_idx;

        if(pwm_conf.invert && !Enable){
            *(rp1->pwms[pwm_idx]->ctl + RP1_ATOM_CLR_OFFSET / 4) = 0x8;
            //printbinary(*(rp1->pwms[pwm_idx]->ctl));
        }

        if(pwm_conf.invert && Enable){
            *(rp1->pwms[pwm_idx]->ctl + RP1_ATOM_SET_OFFSET / 4) = 0x8;
            //printbinary(*(rp1->pwms[pwm_idx]->ctl));
        }

        if(!Enable){
            temp = (~temp) & 0xF;
            temp = (*(rp1->pwm0_base) & temp);
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwm disable");}
        } else {
            temp = (*(rp1->pwm0_base) | temp);
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwm enable");}
        }
        
        // Asignar el bit de actualización (bit 31)
        *(rp1->pwm0_base) = temp | 0x80000000;
        

        return 1;
    }
    void PWM_PI5::printbinary(uint32_t value){
        for (int i = 31; i >= 0; i--) {
            printf("%d", (value >> i) & 1);
            if (i % 8 == 0) {
                printf(" ");  // Espacio para separar bytes
            }
        }
        printf("\n");
    }

    int PWM_PI5::pwm_enable_all(bool Enable)
    {
        uint32_t temp = 0x0000;
        for(int  i = 0; i < 4 ; i++){
            if(rp1->pwms[i] != nullptr){
                temp |= 1 << i;
                if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwn %d will be put %s", i, (Enable)? "on":"off");}

                if(rp1->pwms[i]->pwm_config.invert && !Enable){
                    *(rp1->pwms[i]->ctl + RP1_ATOM_CLR_OFFSET / 4) = 0x8;
                    //printbinary(*(rp1->pwms[pwm_idx]->ctl));
                }

                if(rp1->pwms[i]->pwm_config.invert && Enable){
                    *(rp1->pwms[i]->ctl + RP1_ATOM_SET_OFFSET / 4) = 0x8;
                    //printbinary(*(rp1->pwms[i]->ctl));
                }

            }else{
                if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwn %d is not created", i);}
            }
        }
        if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "temp %d", temp);}
        if(!Enable){
            temp = (~temp) & 0xF;
            temp = (*(rp1->pwm0_base) & temp);
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwm disable");}
        } else {
            temp = (*(rp1->pwm0_base) | temp);
            if(pwm_common_config.debug){PWM_RPI5_LOGGER.log(INFO, "pwm enable");}
        }

        // Asignar el bit de actualización (bit 31)
        *(rp1->pwm0_base) = temp | 0x80000000;

        return 1;
    }

    int PWM_PI5::getPWMIdx()
    {
        switch(pwm_pin)
        {
            case 12:  return 0;
            case 13:  return 1;
            case 14:
            case 18:  return 2;
            case 15:
            case 19:  return 3;
        }
        return -1;
    }


/*     int PWM_PI5::duty(uint32_t Duty)
    {
    uint32_t  _Duty =  (uint32_t) ((config.range-1)*Duty)/100;
    *(rp1->pwms[id]->duty + RP1_ATOM_SET_OFFSET / 4) = _Duty;
    return 1;
    }

    int PWM_PI5::frequency(uint32_t freq)
    {
    printf("clock main : %d\n", clk_pwm);
    uint32_t  _Range =  (uint32_t) (clk_pwm/freq)-1;
    *(rp1->pwms[id]->range   + RP1_ATOM_SET_OFFSET / 4) = _Range;
    printf("range new : %d\n", _Range);
    duty(config.duty);
    printf("range new : %d\n", _Range);

    return 1;
    } */
    
    /* int Pi5PWM::Channel::range(uint32_t Range)
    {
    uint32_t  _Range =  (uint32_t) (factor_us * Range);
    pwm0[PWM_CHANNEL[currentPWM]+PWM_CHAN_RANGE]= _Range;
    return 1;
    } */



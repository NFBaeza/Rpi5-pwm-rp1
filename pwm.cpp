#include "pwm.h"


void PWM_PI5::printbinary(uint32_t value){
    for (int i = 31; i >= 0; i--) {
        printf("%d", (value >> i) & 1);
        if (i % 8 == 0) {
            printf(" ");  // Espacio para separar bytes
        }
    }
    printf("\n");
}


void *PWM_PI5::mapgpio(off_t dev_base, off_t dev_size)
{
    int fd;
    void *mapped;
    
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        PWM_RPI5_LOGGER.log(ERROR, "Can't open /dev/mem");
        return (void *)0;
    }

    mapped = mmap(0, dev_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base);
    
    PWM_RPI5_LOGGER.log(INFO, "base address: %llx, size: %x, mapped: %p", dev_base, dev_size, mapped);

    if (mapped == (void *)-1)
    {
        PWM_RPI5_LOGGER.log(ERROR, "Can't map the memory to user space.");
        return (void *)0;
    }

    close(fd);
    return mapped;
}

bool PWM_PI5::create_rp1(rp1_t **rp1)
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

    *rp1 = r;

    return true;
}

int PWM_PI5::clk_create()
{
    *(rp1->sys_clock + CLK_PWM_CTRL/4) = 0x1000840;
    *(rp1->sys_clock + CLK_PWM_DIV_INT/4) = 1;
    *(rp1->sys_clock + CLK_PWM_DIV_FRAC/4) = 0;
    *(rp1->sys_clock + CLK_PWM_SEL/4) = 1;

    PWM_RPI5_LOGGER.log(INFO, "clk config setted as 0x%llx", *(rp1->sys_clock + CLK_PWM_CTRL/4));
    PWM_RPI5_LOGGER.log(INFO, "clk config at %lu MHz", 50/ *(rp1->sys_clock + CLK_PWM_SEL/4));
    return 1;
}

bool PWM_PI5::create_pin(uint8_t pinnumber,  uint32_t funcmask)
{
    gpio_pin_t *newpin = (gpio_pin_t *)calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = pinnumber;

    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pinnumber);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pinnumber + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pinnumber * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = funcmask;  // now set the value we need

    *(newpin->pad +  RP1_ATOM_SET_OFFSET / 4) = 0x07; 
    
    rp1->pins[pinnumber] = newpin;

    PWM_RPI5_LOGGER.log(INFO, "pin %d stored in pins array %p\n", pinnumber, rp1->pins[pinnumber]);
    PWM_RPI5_LOGGER.log(INFO,"gpio %d set function %d",pinnumber, funcmask);
    return true;
}

int PWM_PI5::pin_enable_output(uint8_t pinnumber)
{
    volatile uint32_t *clr_add = rp1->pins[pinnumber]->pad + RP1_ATOM_CLR_OFFSET / 4;
    volatile uint32_t *set_add = rp1->pins[pinnumber]->pad + RP1_ATOM_SET_OFFSET / 4;

    // Limpiar los bits IE[6] y OD[7]
    *clr_add = 0xC4;  // Limpiar bits 6 y 7 (IE[6] y OD[7]
    *set_add = 0x3B;  // Configurar bits necesarios para salida

    *(rp1->rio_output_enable + RP1_ATOM_SET_OFFSET / 4) = 1 << rp1->pins[pinnumber]->number;

    PWM_RPI5_LOGGER.log(INFO, "Pin %d pad config as output", pinnumber);
    PWM_RPI5_LOGGER.log(INFO, "Pin %d pad register 0x%llx", pinnumber, *(rp1->sys_clock + CLK_PWM_CTRL/4));

    return 1;
}


int PWM_PI5::getPWMIdx(int pin)
{
    switch(pin)
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

int PWM_PI5::pwm_create(int id)
{ 
    pwm_t *newpwm = (pwm_t *)calloc(1, sizeof(pwm_t));
    if (newpwm == NULL)
        return false;

    newpwm->ctl     = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id]);
    newpwm->range   = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id] + PWM_CHAN_RANGE);
    newpwm->phase   = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id] + PWM_CHAN_PHASE);
    newpwm->duty    = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id] + PWM_CHAN_DUTY);

    rp1->pwms[id] = newpwm;

    PWM_RPI5_LOGGER.log(INFO, "pwm %d created\n", id);

    return 1;
}

int PWM_PI5::pwm_configuration(int id, pwm_config_t config){
    PWM_RPI5_LOGGER.log(INFO,"pwm %d configuring\n", id);

    *(rp1->pwms[id]->ctl    + RP1_ATOM_CLR_OFFSET / 4)&= ~0x07;  //limpieza
    *(rp1->pwms[id]->range  + RP1_ATOM_CLR_OFFSET / 4) = 0;
    *(rp1->pwms[id]->phase  + RP1_ATOM_CLR_OFFSET / 4) = 0;
    *(rp1->pwms[id]->duty   + RP1_ATOM_CLR_OFFSET / 4) = 0;

    *(rp1->pwms[id]->ctl    + RP1_ATOM_SET_OFFSET / 4) = config.mode;
    *(rp1->pwms[id]->ctl    + RP1_ATOM_SET_OFFSET / 4) = (config.invert) ? (1<<3):(0<<3);
    *(rp1->pwms[id]->range  + RP1_ATOM_SET_OFFSET / 4) = config.range; //1249;
    *(rp1->pwms[id]->phase  + RP1_ATOM_SET_OFFSET / 4) = config.phase;//delay;
    *(rp1->pwms[id]->duty   + RP1_ATOM_SET_OFFSET / 4) = config.duty; //1250/2;

    PWM_RPI5_LOGGER.log(INFO, "pwm %d config setted as 0x%llx\n", id, *(rp1->pwms[id]->ctl));
    return 1;
}

int PWM_PI5::pwm_enable(int id, bool Enable, bool all_pwm)
{
    uint32_t temp = 0x0000;
    if(all_pwm){
        temp = (Enable) ? ~temp : temp;
        PWM_RPI5_LOGGER.log(INFO, "all pwms will be turn %s", ((Enable)? "on" : "off"));
    } else {
        // Verificar que el id está en el rango correcto
        if (id < 0 || id > 3) {
            PWM_RPI5_LOGGER.log(WARNING, "Invalid PWM channel ID");
            return 0;
        }
        // Desplazar 1 hacia la izquierda por 'id' bits.
        temp = 1 << id;
    }

    if(!Enable){
        temp = (~temp) & 0xF;
        temp = (*(rp1->pwm0_base) & temp);
        PWM_RPI5_LOGGER.log(INFO, "pwm disable");
    } else {
        temp = (*(rp1->pwm0_base) | temp);
        PWM_RPI5_LOGGER.log(INFO, "pwm enable");
    }

    // Asignar el bit de actualización (bit 31)
    temp |= 0x80000000;
    *(rp1->pwm0_base) = temp;

    return 1;
}

PWM_PI5::PWM_PI5(int pin, pwm_config_t pwm_conf)
{
    if (!create_rp1(&rp1))
    {
        PWM_RPI5_LOGGER.log(ERROR,"unable to create rp1");
    }

    clk_create();

    int id_pwm = getPWMIdx(pin);
    if (id_pwm >= 0){
        if(!pwm_create(id_pwm)) {
            PWM_RPI5_LOGGER.log(ERROR, "unable to create pin %d",pin);
            return;
        };
    } else {
        PWM_RPI5_LOGGER.log(ERROR, "unable to create pin %d",pin);
        return;
    }

    int func = (pin > 15) ? 0x03 : 0x00;
    if(!create_pin(pin, func)) {
       PWM_RPI5_LOGGER.log(ERROR,"unable to create pin %d\n", pin);
       return;
    }; 
    pin_enable_output(pin);

    pwm_configuration(id_pwm, pwm_conf);
}
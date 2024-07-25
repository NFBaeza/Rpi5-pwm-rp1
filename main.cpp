#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>

void delay_ms(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

using namespace std;

// Define base addresses
#define RP1_BAR1 0x1f00000000
#define RP1_BAR1_LEN 0x400000

#define RP1_IO_BANK0_BASE       0x0d0000
#define RP1_RIO0_BASE           0x0e0000
#define RP1_PADS_BANK0_BASE     0x0f0000
#define RP1_CLOCK_BASE          0x018000
#define RP1_PWM0_BASE           0x098000
#define RP1_PWM1_BASE           0x09C000

#define CLK_PWM_CTRL           0x00074
#define CLK_PWM_DIV_INT        0x00078
#define CLK_PWM_DIV_FRAC       0x0007c
#define CLK_PWM_SEL            0x00080

#define PWM_CHAN0_CTRL          0x14
#define PWM_CHAN1_CTRL          0x24
#define PWM_CHAN2_CTRL          0x34
#define PWM_CHAN3_CTRL          0x44

#define PWM_CHAN_CTRL           0
#define PWM_CHAN_RANGE          1
#define PWM_CHAN_PHASE          2
#define PWM_CHAN_DUTY           3

#define RP1_ATOM_XOR_OFFSET 0x1000
#define RP1_ATOM_SET_OFFSET 0x2000
#define RP1_ATOM_CLR_OFFSET 0x3000

#define PADS_BANK0_VOLTAGE_SELECT_OFFSET 0
#define PADS_BANK0_GPIO_OFFSET 0x4

#define RIO_OUT_OFFSET 0x00
#define RIO_OE_OFFSET 0x04
#define RIO_NOSYNC_IN_OFFSET 0x08
#define RIO_SYNC_IN_OFFSET 0x0C

//                          10987654321098765432109876543210
#define CTRL_MASK_FUNCSEL 0b00000000000000000000000000011111
#define PADS_MASK_OUTPUT  0b00000000000000000000000011000000

const uint32_t CLOCK = 30303030;
const double factor_us =  ((double) CLOCK + 500000) / 1000000.0;

#define CTRL_FUNCSEL_RIO 0x00

const uint32_t PWM_CHANNEL[4]={PWM_CHAN0_CTRL/4,PWM_CHAN1_CTRL/4,PWM_CHAN2_CTRL/4,PWM_CHAN3_CTRL/4};

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

void print_reg(uint32_t data){
    for (int i = 31; i >= 0; i--) {
        printf("%d", (data >> i) & 1);
        if (i % 8 == 0) {
            printf(" ");  // Espacio para separar bytes
        }
    }
    printf("\n\n");
}

void *mapgpio(off_t dev_base, off_t dev_size)
{
    int fd;
    void *mapped;
    
    printf("sizeof(off_t) %d\n", sizeof(off_t));

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        printf("Can't open /dev/mem\n");
        return (void *)0;
    }

    mapped = mmap(0, dev_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base);
    
    printf("base address: %llx, size: %x, mapped: %p\n", dev_base, dev_size, mapped);

    if (mapped == (void *)-1)
    {
        printf("Can't map the memory to user space.\n");
        return (void *)0;
    }

    return mapped;
}

bool create_rp1(rp1_t **rp1)
{

    rp1_t *r = (rp1_t *)calloc(1, sizeof(rp1_t));
    if (r == NULL){
        printf("no se creó el rpi1\n");
        return false;
    }

    void *base = mapgpio(RP1_BAR1, RP1_BAR1_LEN);
    if (base == NULL){
        printf("base es empty\n");
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

bool create_pin(uint8_t pinnumber, rp1_t *rp1, uint32_t funcmask)
{
    printf("gpio create\n");
    gpio_pin_t *newpin = (gpio_pin_t *)calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = pinnumber;

    // each gpio has a status and control register
    // adjacent to each other. control = status + 4 (uint8_t)
    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pinnumber);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pinnumber + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pinnumber * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = funcmask;  // now set the value we need

    *(newpin->pad +  RP1_ATOM_SET_OFFSET / 4) = 0x56;  // now set the value we need
    
    rp1->pins[pinnumber] = newpin;
    printf("pin %d stored in pins array %p\n", pinnumber, rp1->pins[pinnumber]);
    printf("pin %d config setted as :\n" , pinnumber);
    print_reg(*(rp1->pins[pinnumber]->ctrl));
    return true;
}


int pwm_create(rp1_t *rp1, int id){
    printf("pwm create\n");
    
    pwm_t *newpwm = (pwm_t *)calloc(1, sizeof(pwm_t));
    if (newpwm == NULL)
        return false;

    newpwm->ctl     = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id]);
    newpwm->range   = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id] + PWM_CHAN_RANGE);
    newpwm->phase   = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id] + PWM_CHAN_PHASE);
    newpwm->duty    = (uint32_t *)(rp1->pwm0_base + PWM_CHANNEL[id] + PWM_CHAN_DUTY);

    rp1->pwms[id] = newpwm;

    printf("pwm create done\n");

    return 1;
}

int pwm_enable(rp1_t *rp1, int id, bool Enable)
{
    printf("pwm enable function\n");

    // Verificar que el id está en el rango correcto
    if (id < 0 || id > 3) {
        printf("Invalid PWM channel ID\n");
        return 0;
    }

    // Desplazar 1 hacia la izquierda por 'id' bits.
    uint32_t temp = 1 << id;
    printf("pwm %d\n", temp);

    if(!Enable)
    {
        // Invertir temp y hacer AND con el valor actual en pwm0_base
        temp = (~temp) & 0xF;
        temp = (*(rp1->pwm0_base) & temp);
        printf("pwm disable\n");
    }
    else
    {
        // Hacer OR con el valor actual en pwm0_base
        temp = (*(rp1->pwm0_base) | temp);
        printf("pwm enable\n");
    }

    // Asignar el bit de actualización (bit 31)
    temp |= 0x80000000;

    // Asignar el valor calculado a la dirección correcta
    *(rp1->pwm0_base) = temp;

    printf("pwm status setted as :\n");
    print_reg(*(rp1->pwm0_base));

    return 1;
}

int pin_enable_output(uint8_t pinnumber, rp1_t *rp1)
{
    printf("Attempting to enable output\n");

    // Primero, habilitar el pad para salida.
    // Los pads necesitan tener OD[7] -> 0 (no deshabilitar salida)
    // y IE[6] -> 0 (no habilitar entrada).
    // Usamos acceso atómico a la dirección con una máscara.

    volatile uint32_t *clr_add = rp1->pins[pinnumber]->pad + RP1_ATOM_CLR_OFFSET / 4;
    volatile uint32_t *set_add = rp1->pins[pinnumber]->pad + RP1_ATOM_SET_OFFSET / 4;

    // Limpiar los bits IE[6] y OD[7]
    *clr_add = 0xC4;  // Limpiar bits 6 y 7 (IE[6] y OD[7]
    *set_add = 0x3B;  // Configurar bits necesarios para salida

    *(rp1->rio_output_enable + RP1_ATOM_SET_OFFSET / 4) = 1 << rp1->pins[pinnumber]->number;

    printf("Pin %d pad config as:\n", pinnumber);
    print_reg(*(rp1->pins[pinnumber]->pad));

    return 1;
}


int pwm_configuration(rp1_t *rp1, int id){
    printf("pwm config\n");

    *(rp1->pwms[id]->ctl    + RP1_ATOM_CLR_OFFSET / 4)&= ~0x07;  //limpieza

    *(rp1->pwms[id]->ctl    + RP1_ATOM_SET_OFFSET / 4) = 0x2;
    *(rp1->pwms[id]->range  + RP1_ATOM_SET_OFFSET / 4) = 1024;
    *(rp1->pwms[id]->phase  + RP1_ATOM_SET_OFFSET / 4) = 0;
    *(rp1->pwms[id]->duty   + RP1_ATOM_SET_OFFSET / 4) = 512;

    printf("pwm %d config setted as :\n", id);
    print_reg(*(rp1->pwms[id]->ctl));
    return 1;
}

int clk_create(rp1_t *rp1){
    printf("creando clk\n");
    *(rp1->sys_clock + CLK_PWM_CTRL/4) = 0x1000000;

    *(rp1->sys_clock + CLK_PWM_CTRL/4) = 0x1000840;
    *(rp1->sys_clock + CLK_PWM_DIV_INT/4) = 100000;
    *(rp1->sys_clock + CLK_PWM_DIV_FRAC/4) = 0;
    *(rp1->sys_clock + CLK_PWM_SEL/4) = 1;

    printf("clk config setted as :\n");
    print_reg(*(rp1->sys_clock + CLK_PWM_CTRL/4));
    return 1;
}


const uint8_t pins[2] = {12, 13};

int main(void)
{

    int i, j;
    // create a rp1 device
    printf("creating rp1\n");
    rp1_t *rp1;
    if (!create_rp1(&rp1))
    {
        printf("unable to create rp1\n");
    }
    clk_create(rp1);

    // let's see if we can dump the iobank registers
    uint32_t *p;

    for (i = 0; i < 27; i++)
    {

        p = (uint32_t *)(rp1->gpio_base + i * 8);

        printf(
            "gpio %0d: status @ p = %lx, ctrl @ p = %lx\n",
            i,
            *p, *(p + 1));
    }

    for(i=0;i<2;i++) {
        if(!pwm_create(rp1, i)) {
            printf("unable to create pin %d\n",pins[i]);
            return 3;
        };
    }

    for(i=0;i<2;i++) {
         pwm_enable(rp1, i, false);
    }

    for(i=0;i<2;i++) {
        if(!create_pin(pins[i], rp1, 0x00)) {
            printf("unable to create pin %d\n", pins[i]);
            return 3;
        }; 
        pwm_configuration(rp1, i);
        pin_enable_output(pins[i], rp1);
    }

    for(i=0;i<2;i++) {
        pwm_enable(rp1, i, true);
    }

    while (true)
    {
        
    }

    printf("done\n\n");

    return 0;
}

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "sysctl_pll.h"
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "driverlib/fpu.h"

#include "sampling.h"

volatile uint16_t draw_buffer[128];

void sample_init() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3); // GPIO setup for analog input AIN3


    // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE)
    + 1; // round up
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
    pll_divisor);

    // choose ADC1 sequence 0; disable before configuring
    ADCSequenceDisable(ADC1_BASE, 0);

    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger

    // in the 0th step, sample channel 3 (AIN3)
    // enable interrupt, and make it the end of sequence
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

    // enable the sequence. it is now sampling
    ADCSequenceEnable(ADC1_BASE, 0);

    // enable sequence 0 interrupt in the ADC1 peripheral
    ADCIntEnable(ADC1_BASE,0);

    IntPrioritySet(INT_ADC1SS0, SAMPLING_INT_PRIORITY); // set ADC1 sequence 0 interrupt priority

    // enable ADC1 sequence 0 interrupt in int. controller
    //setup the draw buffer
    IntEnable(INT_ADC1SS0);
    uint8_t i;
    for(i=0;i<128;++i) {
        draw_buffer[i] = 0;
    }
}


#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
// index wrapping macro
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
// latest sample index
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines
#define VIN_RANGE 4086
#define ADC_BITS 12
volatile uint32_t ADC_OFFSET = VIN_RANGE /2; //constent for now
volatile uint32_t fVoltsPerDiv = 1238;

void ADC_ISR(void)
{
    // clear ADC1 sequence0 interrupt flag in the ADCISC register
    ADCIntClear(ADC1_BASE,0);
    //check for missed intrupts
    if(ADC1_OSTAT_R & ADC_OSTAT_OV0) {
        gADCErrors++; // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
    }
    //get the index for sample.
    // ADC_BUFFER_WRAP is a macro that handles wrapping the buffer
    gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1);
    
    // read sample from the ADC1 sequence 0 FIFO using direct address access
    uint16_t sample = (uint16_t)(ADC1_SSFIFO0_R);
    gADCBuffer[gADCBufferIndex] = sample; // read from sample sequence 0
}

int RisingTrigger(void) // search for rising edge trigger
{
    int init_index =  gADCBufferIndex - (LCD_HORIZONTAL_MAX /2);
    // Step 1
    int x = init_index;/* half screen width; don’t use a
    magic number */;
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
    if ( gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
            gADCBuffer[ADC_BUFFER_WRAP(x-1)] < ADC_OFFSET) {
            break;
        }
    }
    // Step 3
    if (x == x_stop) { // for loop ran to the end
        x = init_index; // reset x back to how it was initialized
    }
    return x;
}


void draw_grid(tContext* context,tRectangle* rectFullScreen,uint16_t max_width,uint16_t max_height) {
    GrContextForegroundSet(context, ClrBlack);
    GrRectFill(context, rectFullScreen); // fill screen with black
    GrContextForegroundSet(context, ClrRoyalBlue); // blue line
    uint8_t i;
    for(i=4; i<max_width;i+=PIXELS_PER_DIV) {
        //this works because we are working with a square display
        GrLineDrawV(context,i,0,max_height);
        GrLineDrawH(context,0,max_width,i);
    }
    GrContextForegroundSet(context, ClrYellow); // blue line
    for(i=1;i<LCD_VERTICAL_MAX;++i) {
       uint16_t prev_y = (LCD_HORIZONTAL_MAX - 1) - draw_buffer[i-1]/32;
       uint16_t y = (LCD_HORIZONTAL_MAX - 1) - draw_buffer[i]/32;
       GrLineDraw(context,i-1,prev_y,i,y);
    }
    GrFlush(context); // flush the frame buffer to the LCD
}

void update_draw_buffer() {
    int8_t i;
    int32_t search_index = RisingTrigger();
    for(i=(LCD_HORIZONTAL_MAX/2) * -1;i<LCD_HORIZONTAL_MAX/2;++i) {
        int index = ADC_BUFFER_WRAP(search_index +i);
        int32_t sample = gADCBuffer[index];

        float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv);
        int y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)sample - ADC_OFFSET));

        draw_buffer[(LCD_HORIZONTAL_MAX/2)+i] = y;
    }
}

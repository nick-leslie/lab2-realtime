#define SAMPLING_INT_PRIORITY 0  // button interrupt priority (higher number is lower priority)
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates
#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define PIXELS_PER_DIV 20

void sample_init();
void draw_grid(tContext* context,tRectangle* rectFullScreen,uint16_t max_width,uint16_t max_height);
void update_draw_buffer();

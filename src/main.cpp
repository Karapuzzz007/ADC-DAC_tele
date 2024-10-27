#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>

//==================================================================
void indicate_adc_val(uint16_t val){

if (val > 2048) gpio_set(GPIOD, GPIO13)
else {gpio_clear (GPIOD, GPIO13)};

if (val > 1024) gpio_set(GPIOD, GPIO14)
else {gpio_clear (GPIOD, GPIO14)};

if (val > 512) gpio_set(GPIOD, GPIO15)
else {gpio_clear (GPIOD, GPIO15)};

}
//==================================================================
uint16_t start_wait_and_read_adc(){
	uint8_t channel_array[16];
	channel_array[0] = 0;
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	uint16_t reg16 = adc_read_regular(ADC1);
	return reg16;
}

void output_dac(uint16_t val){
dac_load_data_buffer_single(DAC1, val, DAC_ALIGN_RIGHT12, DAC_CHANNEL2);
dac_software_trigger(DAC1, DAC_CHANNEL2);
}
//==================================================================
void blocking_delay_parrots () {
    for (volatile uint32_t i = 0; i < 500; i++); 
}
//==================================================================
void setup_ADC () {
    	rcc_periph_clock_enable(RCC_ADC1);
        adc_power_off(ADC1);
        rcc_periph_reset_pulse(RST_ADC);

        rcc_periph_clock_enable(RCC_GPIOA);
        gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

        //adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
        adc_set_single_conversion_mode(ADC1);
        adc_set_sample_time(ADC1, 1, ADC_SMPR_SMP_3CYC);
        adc_power_on(ADC1);

}
//==================================================================
void setup_DAC(){
rcc_periph_clock_enable(RCC_DAC);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);
	dac_disable(DAC1, DAC_CHANNEL2);
	dac_disable_waveform_generation(DAC1, DAC_CHANNEL2);
	dac_enable(DAC1, DAC_CHANNEL2);
	dac_set_trigger_source(DAC1, DAC_CR_TSEL2_SW);
}
//==================================================================
void setup () {
    setup_ADC();
    setup_DAC();

    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}
//==================================================================
void loop(){
uint16_t temp = start_wait_and_read_adc();
indicate_adc_val(temp);
output_dac(temp);

blocking_delay_parrots();
gpio_toggle(GPIOD, GPIO12);

}
//==================================================================
int main (){

    setup();

   while(true){

    loop();

   } 
}
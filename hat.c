#include "mcu.h"
#include "pwm.h"
#include "nrf24.h"
#include "pio.h"
#include "pacer.h"
#include "stdio.h"
#include "delay.h"
#include "panic.h"
#include "target.h"
#include "usb_serial.h"
#include "adxl345.h"
#include "piezo_beep.h"
#include "ledbuffer.h"

#define PACER_RATE 20
#define ACCEL_POLL_RATE 1
#define ADXL345_ADDRESS 0x53

#define RADIO_CHANNEL_1 10
#define RADIO_CHANNEL_2 20
#define RADIO_CHANNEL_3 30
#define RADIO_CHANNEL_4 40
#define RADIO_ADDRESS 0xABCDEF1234LL 

#define RADIO_PAYLOAD_SIZE 32
#define PWM_FREQ_HZ 1e3 //PWM for buzzer


static const pwm_cfg_t pwm_cfg =
{
    .pio = BUZZER_PIO,
    .period = PWM_PERIOD_DIVISOR (PWM_FREQ_HZ),
    .duty = PWM_DUTY_DIVISOR (PWM_FREQ_HZ, 50),
    .align = PWM_ALIGN_LEFT,
    .polarity = PWM_POLARITY_HIGH,
    .stop_state = PIO_OUTPUT_LOW
};

static twi_cfg_t adxl345_twi_cfg =
{
    .channel = TWI_CHANNEL_0,
    .period = TWI_PERIOD_DIVISOR (100000), // 100 kHz
    .slave_addr = 0
};

struct PWM_motors {
    int32_t left_motor;
    int32_t right_motor;

} PWM_motors_t;


void radio_channel_init(void)
{
    pio_init(RADIO_CH0_PIO);
    pio_init(RADIO_CH1_PIO);
    pio_init(RADIO_CH2_PIO);
    pio_init(RADIO_CH3_PIO);

    pio_config_set(RADIO_CH0_PIO, PIO_PULLUP);
    pio_config_set(RADIO_CH1_PIO, PIO_PULLUP);
    pio_config_set(RADIO_CH2_PIO, PIO_PULLUP);
    pio_config_set(RADIO_CH3_PIO, PIO_PULLUP);
}
void radio_switch(nrf24_t *nrf)
{
    if (!pio_input_get(RADIO_CH0_PIO))
        nrf24_set_channel(nrf, RADIO_CHANNEL_1);
    if (!pio_input_get(RADIO_CH1_PIO))
        nrf24_set_channel(nrf, RADIO_CHANNEL_2);
    if (!pio_input_get(RADIO_CH2_PIO))
        nrf24_set_channel(nrf, RADIO_CHANNEL_3);
    if (!pio_input_get(RADIO_CH3_PIO))
        nrf24_set_channel(nrf, RADIO_CHANNEL_4);

}

int main (void)
{
    mcu_jtag_disable();
    twi_t adxl345_twi;
    adxl345_t *adxl345;
    pwm_t pwm;
    int ticks = 0;
    int counta = 0;

    bool blue = false;
    int count = 0;
    ledbuffer_t *leds = ledbuffer_init (LEDTAPE_PIO, NUM_LEDS);

    double c = 300;
    double dc = 450;
    double q = 150;
    double m = 600;
    double dm = 900;
    const double duration[] = {c,dc,q,c,m,c,dm,dm,dc,q,c,m,c,dm

    };
    const double note_freq[] = {0.247, 0.330, 0.392, 0.370, 0.329, 0.494, 0.440, 
                                0.370, 0.329, 0.392, 0.370, 0.311, 0.349, 0.247
    };
    usb_serial_stdio_init ();
    pwm = pwm_init (&pwm_cfg);
    if (! pwm)
        panic (LED_ERROR_PIO, 1);

    pio_config_set (LED_ERROR_PIO, PIO_OUTPUT_LOW);
    pio_output_set (LED_ERROR_PIO, ! LED_ACTIVE);
    pio_config_set (LED_STATUS_PIO, PIO_OUTPUT_LOW);
    pio_output_set (LED_STATUS_PIO, ! LED_ACTIVE);
    adxl345_twi = twi_init (&adxl345_twi_cfg);

    if (! adxl345_twi)
        panic (LED_ERROR_PIO, 1);
    adxl345 = adxl345_init (adxl345_twi, ADXL345_ADDRESS);

    if (! adxl345)
        panic (LED_ERROR_PIO, 2);

    pacer_init (PACER_RATE);

    spi_cfg_t spi_cfg =
    {
        .channel = 0,
        .clock_speed_kHz = 1000,
        .cs = RADIO_CS_PIO,
        .mode = SPI_MODE_0,
        .cs_mode = SPI_CS_MODE_FRAME,
        .bits = 8
    };
    nrf24_cfg_t nrf24_cfg =
    {
        //.channel = RADIO_CHANNEL,
        .address = RADIO_ADDRESS,
        .payload_size = RADIO_PAYLOAD_SIZE,
        .ce_pio = RADIO_CE_PIO,
        .irq_pio = RADIO_IRQ_PIO,
        .spi = spi_cfg, 
    };
 
    uint8_t countb = 0;
    nrf24_t *nrf;
    pio_config_set (LED_ERROR_PIO, PIO_OUTPUT_HIGH);
    pio_config_set (LED_STATUS_PIO, PIO_OUTPUT_HIGH);
    pio_config_set (BUZZER_PIO, PIO_OUTPUT_LOW);
    pacer_init (10);
    printf("main loaded");

#ifdef RADIO_POWER_ENABLE_PIO
    pio_config_set (RADIO_POWER_ENABLE_PIO, PIO_OUTPUT_HIGH);
    delay_ms (10);
#endif


    char tx_buffer[RADIO_PAYLOAD_SIZE + 1];
    char rx_buffer[RADIO_PAYLOAD_SIZE + 1];
    nrf = nrf24_init (&nrf24_cfg);

    if (! nrf)
        panic (LED_ERROR_PIO, 2);
    pacer_wait();
    radio_channel_init();


    pio_config_set (SLEEP_PIO, PIO_PULLUP);
    while (1)
    {
        radio_switch(nrf);

        pio_output_toggle (LED_STATUS_PIO);

        ticks++;
        if (ticks < PACER_RATE / ACCEL_POLL_RATE)
            continue;
        ticks = 0;

        if (! adxl345_is_ready (adxl345))
        {
            counta++;
        }
        else
        {
            int16_t accel[3];
            if (adxl345_accel_read (adxl345, accel))
            {
                int16_t center = 0;
                int16_t deadzone = 50;
                int32_t map_t = (accel[0]*100)/255;
                int32_t map_s = (accel[1]*100)/255;
              
                if (abs(accel[0]) < deadzone)
                    map_t = 0;
                if (abs(accel[1]) < deadzone)
                    map_s = 0;

                PWM_motors_t.left_motor = map_t + map_s;
                PWM_motors_t.right_motor = map_t - map_s;
            }
        }
        snprintf (tx_buffer, sizeof (tx_buffer), "%ld %ld \n", PWM_motors_t.left_motor *10, PWM_motors_t.right_motor *10);
        printf("%ld %ld \n", PWM_motors_t.left_motor *10, PWM_motors_t.right_motor *10);

        if (! nrf24_write (nrf, tx_buffer, RADIO_PAYLOAD_SIZE))
            pio_output_set (LED_ERROR_PIO, 0);
        else
            pio_output_set (LED_ERROR_PIO, 1);

        uint8_t bytes;
        bytes = nrf24_read (nrf, rx_buffer, RADIO_PAYLOAD_SIZE);
        if (bytes != 0)
        {
            rx_buffer[bytes] = 0;
            printf ("%s", rx_buffer);
            // Harry potter theme
            const uint16_t num_notes = sizeof(note_freq) / sizeof(note_freq[0]);
            for (uint16_t j = 0; j < num_notes; j++) {
                for (uint16_t i = 0; i < duration[j] * note_freq[j] * 2; i++) {    
                    pio_output_toggle(BUZZER_PIO);   
                    DELAY_US(500 / note_freq[j]);  
                }
                delay_ms(50);
            }
        }
        
        if (count++ == NUM_LEDS)
        {
            // wait for a revolution
            ledbuffer_clear(leds);
            if (blue)
            {
                ledbuffer_set(leds, 0, 0, 0, 255);
                ledbuffer_set(leds, NUM_LEDS / 2, 0, 0, 255);
            }
            else
            {
                ledbuffer_set(leds, 0, 255, 0, 0);
                ledbuffer_set(leds, NUM_LEDS / 2, 255, 0, 0);
            }
            blue = !blue;
            count = 0;
        }

        ledbuffer_write (leds);
        ledbuffer_advance (leds, 1);
        pio_output_toggle (LED_STATUS_PIO);
    }
}
/*
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define CONTROL_OUTPUT_SHIFT    9
#define PWM_OUTPUT_MAX          1023
#define ADC_SAMPLES             8192
#define ADC_AVERAGE_SHIFT       13
#define CURRENT_MAX             5
#define SET_POINT_MULTIPLIER    1
#define ERROR_INTEGRAL_MAX      32000

#define ROTATE_CCW               PORTB|=(1<<1)
#define ROTATE_CW              PORTB&=~(1<<1)

#define SET_ERR_LED             PORTC|=(1<<3)
#define CLEAR_ERR_LED           PORTC&=~(1<<3)
#define SET_ERR_OUTPUT          PORTD|=(1<<1)
#define CLEAR_ERR_OUTPUT        PORTD&=~(1<<1)
#define PULLUP_RES              PORTD|=(1<<0);
#define PULLUP_A                PORTD|=(1<<2);
#define PULLUP_B                PORTD|=(1<<3);
#define PULLUP_STP              PORTD|=(1<<4);
#define PULLUP_DIR              PORTD|=(1<<5);
#define PID_SAMPLING_TIME       OCR0A
#define DUTY_CYCLE              OCR1B

volatile uint8_t state_change, encoder_state, encoder_state_prev;
volatile int16_t error, error_prev, error_integral, error_derivative;
volatile uint16_t adc_value, adc_samples_counter;

int main(void) {

    uint8_t adc_average=0;
    uint16_t Kp=6000, Ki=13, Kd=00;
    uint16_t pwm_output=0;
    int32_t control_output=0;

    DDRB|=(1<<1)|(1<<2);                                                        //PH/EN output
    DDRC|=(1<<3);                                                               //ERR LED output
    PORTC|=(1<<1)|(1<<2);                                                       //OVC/FAULT pull ups
    DDRD|=(1<<1);                                                               //ERR output

    PULLUP_RES;
    PULLUP_STP;
    PULLUP_DIR;

    TCCR1A|=(1<<WGM11)|(1<<WGM10)|(1<<COM1B1);	                                //Phase correct PWM 10-bit non inverting mode
    TCCR1B|=(1<<CS10);	                                                        //Set prescaler 1, PWM freq 10 kHz

    TCCR0A|=(1<<WGM01);                                                         //CTC mode, set prescaler 1024
    TCCR0B|=(1<<CS02)|(1<<CS00);
    TIMSK0|=(1<<OCIE0A);

    PID_SAMPLING_TIME=5;

    ADMUX|=(1<<ADLAR);                                                          //AREF, 8-bit conversion, channel 0
    ADCSRA|=(1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS0);     //Auto trigger, ADC interrupts, 625 kHz clock

    EICRA|=(1<<ISC10)|(1<<ISC00);                                               //enable A/B interrupts (any edge)
    EIMSK|=(1<<INT1)|(1<<INT0);

    PCICR|=(1<<PCIE2)|(1<<PCIE1);                                               //enable STP/FAULT interrupts
    PCMSK2|=(1<<PCINT20);
    PCMSK1|=(1<<PCINT10);

    if(PIND&(1<<2)) encoder_state|=(1<<0);                                        //zero encoder_state/encoder_state_prew
    else encoder_state&=~(1<<0);
    if(PIND&(1<<3)) encoder_state|=(1<<1);
    else encoder_state&=~(1<<1);
    encoder_state_prev=encoder_state;

    sei();

    while(1) {

        SET_ERR_LED;
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            //9.8 us
            control_output=((int32_t)Kp*error)+((int32_t)Ki*error_integral)-((int32_t)Kd*error_derivative);
        }
        if(control_output>0) {
            ROTATE_CW;
        }
        else if(control_output<0) {
            ROTATE_CCW;
            control_output*=-1;
        }
        //3.4 us
        pwm_output=(uint16_t)(control_output>>CONTROL_OUTPUT_SHIFT);
        if(pwm_output>PWM_OUTPUT_MAX) pwm_output=PWM_OUTPUT_MAX;
        DUTY_CYCLE=pwm_output;

        if(error<15&& error>-15) CLEAR_ERR_OUTPUT;
        else SET_ERR_OUTPUT;


        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            //0.7 us
            if(adc_samples_counter==ADC_SAMPLES) {
                adc_average=(uint8_t)(adc_value>>ADC_AVERAGE_SHIFT);
                adc_value=0;
                adc_samples_counter=0;
            }
        }
        //if(adc_average>CURRENT_MAX) SET_ERR_LED;
        //else CLEAR_ERR_LED;
        CLEAR_ERR_LED;
    }

    return 0;
}

ISR(INT0_vect) {                                                                //encoder A
    //2.4us
    if(PIND&(1<<2)) encoder_state|=(1<<0);
    else encoder_state&=~(1<<0);

    switch(encoder_state) {
    case 0:
        //if(encoder_state_prev==2) error++;
        if(encoder_state_prev==1) error--;
        //else if(encoder_state_prev==0) ;
        else SET_ERR_OUTPUT;
        encoder_state_prev=0;
        break;
    case 1:
        if(encoder_state_prev==0) error++;
        //if(encoder_state_prev==3) error--;
        //else if(encoder_state_prev==1) ;
        else SET_ERR_OUTPUT;
        encoder_state_prev=1;
        break;
    case 3:
        //if(encoder_state_prev==1) error++;
        if(encoder_state_prev==2) error--;
        //else if(encoder_state_prev==3) ;
        else SET_ERR_OUTPUT;
        encoder_state_prev=3;
        break;
    case 2:
        if(encoder_state_prev==3) error++;
        //if(encoder_state_prev==0) error--;
        //else if(encoder_state_prev==2) ;
        else SET_ERR_OUTPUT;
        encoder_state_prev=2;
        break;
    }
}

ISR(INT1_vect) {                                                                //encoder B
    //2.4 us
    if(PIND&(1<<3)) encoder_state|=(1<<1);
    else encoder_state&=~(1<<1);

    switch(encoder_state) {
    case 0:
        if(encoder_state_prev==2) error++;
        //if(encoder_state_prev==1) error--;
        //else if(encoder_state_prev==0) ;
        else SET_ERR_OUTPUT;
        encoder_state_prev=0;
        break;
    case 1:
        //if(encoder_state_prev==0) error++;
        if(encoder_state_prev==3) error--;
        //else if(encoder_state_prev==1) ;
        else SET_ERR_LED;
        encoder_state_prev=1;
        break;
    case 3:
        if(encoder_state_prev==1) error++;
        //if(encoder_state_prev==2) error--;
        //else if(encoder_state_prev==3) ;
        else SET_ERR_OUTPUT;
        encoder_state_prev=3;
        break;
    case 2:
        //if(encoder_state_prev==3) error++;
        if(encoder_state_prev==0) error--;
        //else if(encoder_state_prev==2) ;
        else SET_ERR_OUTPUT;
        encoder_state_prev=2;
        break;
    }
}

ISR(PCINT1_vect) {                                                              //driver FAULT

}

ISR(PCINT2_vect) {                                                              //input STEP
    //1.75 us
    if(((PIND&(1<<4))==0)&&state_change==1) {                                      //check for falling edge
        if(PIND&(1<<5)) {                                                       //check DIR
            error+=SET_POINT_MULTIPLIER;
            error_prev+=SET_POINT_MULTIPLIER;
        }
        else {
            error-=SET_POINT_MULTIPLIER;
            error_prev-=SET_POINT_MULTIPLIER;
        }
        state_change=0;
    }
    else state_change=1;
}

ISR(TIMER0_COMPA_vect) {
    //2.7 us
    error_integral+=error;
    if(error_integral>ERROR_INTEGRAL_MAX) error_integral=ERROR_INTEGRAL_MAX;
    else if(error_integral<-ERROR_INTEGRAL_MAX) error_integral=-ERROR_INTEGRAL_MAX;
    error_derivative=error_prev-error;
    error_prev=error;
}

ISR(ADC_vect) {
    //1.2 us
    adc_value+=ADCH;
    adc_samples_counter++;
}

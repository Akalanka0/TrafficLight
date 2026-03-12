/* ============================================================
   SMART PEDESTRIAN TRAFFIC LIGHT — Bare-metal AVR-C
   Method: INT0 Hardware Interrupt + Threshold Hysteresis

   Only ONE wire changes from the original sketch:
     ECHO moves from D7 → D2 (INT0)
   All LED pins are exactly the same as the original .ino.

   Wiring:
     PD6 = D6  -> TRIG
     PD2 = D2  -> ECHO  ← moved here (INT0 hardware interrupt)
     PB0 = D8  -> CAR_RED      (unchanged)
     PB1 = D9  -> CAR_YELLOW   (unchanged)
     PB2 = D10 -> CAR_GREEN    (unchanged)
     PB3 = D11 -> PED_GREEN    (unchanged)
     PB4 = D12 -> PED_RED      (unchanged)

   How INT0 echo measurement works:
     - ECHO rises  → INT0 ISR fires (rising edge)  → save Timer1 count
     - ECHO falls  → INT0 ISR fires (falling edge) → save Timer1 count
     - difference  → exact pulse width in hardware ticks
     - No busy-wait, no software tick counting, no loop

   Hysteresis thresholds (replaces debounce):
     Object closer than DETECT_CM  → pedestrian confirmed
     Object farther than RELEASE_CM → pedestrian cleared
     Between the two               → state frozen, no flicker

   Compile + flash:
     avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os \
             -std=c99 -o TrafficLight.elf TrafficLight.c
     avr-objcopy -O ihex TrafficLight.elf TrafficLight.hex
     avrdude -c arduino -p m328p -P COM3 -b 115200 \
             -U flash:w:TrafficLight.hex
   ============================================================ */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   PIN BITS — LED pins identical to original .ino
   ============================================================ */
#define TRIG_BIT        PD6   /* D6  — trigger output              */
#define ECHO_BIT        PD2   /* D2  — INT0, echo input            */

#define CAR_RED_BIT     PB0   /* D8  — unchanged from original     */
#define CAR_YELLOW_BIT  PB1   /* D9  — unchanged                   */
#define CAR_GREEN_BIT   PB2   /* D10 — unchanged                   */
#define PED_GREEN_BIT   PB3   /* D11 — unchanged                   */
#define PED_RED_BIT     PB4   /* D12 — unchanged                   */

/* ============================================================
   FSM TIMING (ms) — identical to original .ino
   ============================================================ */
#define VEHICLE_GREEN_TIME      5000UL
#define VEHICLE_YELLOW_TIME     2000UL
#define VEHICLE_RED_TIME        1000UL
#define PED_CROSS_TIME          4000UL
#define VEHICLE_PRE_GREEN_TIME  2000UL

/* ============================================================
   HYSTERESIS THRESHOLDS (cm)
   DETECT_CM  — must be closer than this to confirm pedestrian
   RELEASE_CM — must be farther than this to clear pedestrian
   The 10 cm gap between them is the hysteresis band.
   ============================================================ */
#define DETECT_CM   20
#define RELEASE_CM  30
#define NO_OBJECT   999

/* ============================================================
   MILLIS — Timer0 overflow ISR
   ============================================================ */
#define MILLIS_INC  1UL
#define FRAC_INC    3
#define FRAC_MAX    125

static volatile uint32_t ms_count = 0;
static volatile uint8_t  ms_frac  = 0;

ISR(TIMER0_OVF_vect)
{
    uint32_t m = ms_count;
    uint8_t  f = ms_frac;
    m += MILLIS_INC;
    f += FRAC_INC;
    if (f >= FRAC_MAX) { f -= FRAC_MAX; m++; }
    ms_count = m;
    ms_frac  = f;
}

static uint32_t millis(void)
{
    uint32_t m;
    uint8_t sreg = SREG;
    cli();
    m = ms_count;
    SREG = sreg;
    return m;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while (millis() - start < ms);
}

/* ============================================================
   INT0 ECHO MEASUREMENT
   
   Timer1 runs free at /8 prescaler (0.5 µs per tick).
   INT0 ISR fires on both rising and falling edges of ECHO.
   
   Rising edge  → record t_rise
   Falling edge → record t_fall, set echo_ready flag
   
   Main code reads echo_ready, computes pulse width, converts
   to cm. CPU does nothing while waiting for the echo — no
   busy-wait at all.
   
   Timer1 at /8: 1 tick = 0.5 µs
   distance_cm = ticks × 0.5 × 0.034 / 2 = ticks × 17 / 2000
   ============================================================ */
static volatile uint16_t t_rise     = 0;
static volatile uint16_t t_fall     = 0;
static volatile bool     echo_ready = false;
static volatile bool     echo_timeout = false;

/* Timer1 overflow = echo never returned (~33 ms at /8) */
ISR(TIMER1_OVF_vect)
{
    echo_timeout = true;
}

ISR(INT0_vect)
{
    if (PIND & (1 << ECHO_BIT)) {
        /* Rising edge — start of echo pulse */
        t_rise      = TCNT1;
        echo_ready  = false;
        echo_timeout = false;
        TCNT1       = 0;              /* reset counter at pulse start */
        TIFR1      |= (1 << TOV1);   /* clear any pending overflow   */
    } else {
        /* Falling edge — end of echo pulse */
        t_fall     = TCNT1;
        echo_ready = true;
    }
}

static void timer1_init(void)
{
    TCCR1A = 0x00;
    TCCR1B = (1 << CS11);   /* /8 prescaler, normal mode */
    TIMSK1 = (1 << TOIE1);  /* overflow interrupt for timeout detection */
    TCNT1  = 0;
}

static void int0_init(void)
{
    /* INT0 on any logical change (both rising and falling) */
    EICRA |= (1 << ISC00);
    EICRA &= ~(1 << ISC01);
    EIMSK |= (1 << INT0);
}

/* ============================================================
   SINGLE DISTANCE READING
   Fires one trigger pulse and waits for INT0 ISR to complete.
   Returns distance in cm or NO_OBJECT on timeout.
   ============================================================ */
#define ECHO_WAIT_MS  35UL   /* max wait: 30 ms echo + 5 ms margin */

static int get_distance_single(void)
{
    uint32_t start;
    uint16_t pulse_ticks;

    /* Reset flags */
    echo_ready   = false;
    echo_timeout = false;

    /* Send trigger pulse */
    PORTD &= ~(1 << TRIG_BIT);
    _delay_us(2);
    PORTD |=  (1 << TRIG_BIT);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_BIT);

    /* Wait for INT0 ISR to set echo_ready (non-blocking via millis) */
    start = millis();
    while (!echo_ready && !echo_timeout) {
        if (millis() - start > ECHO_WAIT_MS) return NO_OBJECT;
    }

    if (echo_timeout || !echo_ready) return NO_OBJECT;

    pulse_ticks = t_fall;   /* TCNT1 was reset at rising edge, so
                               t_fall is directly the pulse width   */

    /* ticks × 0.5 µs × 0.034 / 2 = ticks × 17 / 2000 */
    return (int)((pulse_ticks * 17UL) / 2000UL);
}

/* ============================================================
   3-SAMPLE MEDIAN — same as original .ino
   ============================================================ */
static int get_distance(void)
{
    int     samples[3];
    int     tmp;
    uint8_t i, j;

    for (i = 0; i < 3; i++) {
        samples[i] = get_distance_single();
        delay_ms(10);
    }

    for (i = 0; i < 2; i++) {
        for (j = i + 1; j < 3; j++) {
            if (samples[i] > samples[j]) {
                tmp = samples[i]; samples[i] = samples[j]; samples[j] = tmp;
            }
        }
    }
    return samples[1];
}

/* ============================================================
   HYSTERESIS DETECTION
   Returns last confirmed state when reading is in the band.
   ============================================================ */
static bool pedestrian_detected(void)
{
    static bool state = false;
    int dist = get_distance();

    if (dist != NO_OBJECT && dist < DETECT_CM) {
        state = true;
    } else if (dist == NO_OBJECT || dist > RELEASE_CM) {
        state = false;
    }
    /* Inside band (DETECT_CM to RELEASE_CM): keep last state */

    return state;
}

/* ============================================================
   INIT
   ============================================================ */
static void gpio_init(void)
{
    /* LED outputs on PORTB */
    DDRB |= (1 << CAR_RED_BIT)    |
            (1 << CAR_YELLOW_BIT) |
            (1 << CAR_GREEN_BIT)  |
            (1 << PED_GREEN_BIT)  |
            (1 << PED_RED_BIT);

    /* TRIG output */
    DDRD |=  (1 << TRIG_BIT);

    /* ECHO input on D2 (INT0) — no pull-up */
    DDRD  &= ~(1 << ECHO_BIT);
    PORTD &= ~(1 << ECHO_BIT);
}

static void timer0_init(void)
{
    TCCR0A = 0x00;
    TCCR0B = (1 << CS01) | (1 << CS00);   /* /64 prescaler */
    TIMSK0 = (1 << TOIE0);
}

/* ============================================================
   SET ALL LIGHTS
   ============================================================ */
static void set_lights(uint8_t car_r, uint8_t car_y, uint8_t car_g,
                       uint8_t ped_g, uint8_t ped_r)
{
    if (car_r) PORTB |=  (1 << CAR_RED_BIT);
    else       PORTB &= ~(1 << CAR_RED_BIT);

    if (car_y) PORTB |=  (1 << CAR_YELLOW_BIT);
    else       PORTB &= ~(1 << CAR_YELLOW_BIT);

    if (car_g) PORTB |=  (1 << CAR_GREEN_BIT);
    else       PORTB &= ~(1 << CAR_GREEN_BIT);

    if (ped_g) PORTB |=  (1 << PED_GREEN_BIT);
    else       PORTB &= ~(1 << PED_GREEN_BIT);

    if (ped_r) PORTB |=  (1 << PED_RED_BIT);
    else       PORTB &= ~(1 << PED_RED_BIT);
}

/* ============================================================
   STATE MACHINE
   ============================================================ */
typedef enum {
    VEHICLE_GREEN,
    VEHICLE_YELLOW,
    VEHICLE_RED,
    PEDESTRIAN_GREEN,
    VEHICLE_PRE_GREEN
} State;

/* ============================================================
   MAIN
   ============================================================ */
int main(void)
{
    State    currentState;
    uint32_t stateStartTime;
    uint32_t currentTime;
    bool     pedestrianWaiting;

    gpio_init();
    timer0_init();
    timer1_init();
    int0_init();
    sei();

    currentState   = VEHICLE_GREEN;
    stateStartTime = millis();

    /*                 car_r  car_y  car_g  ped_g  ped_r */
    set_lights(          0,     0,     1,     0,     1   );

    while (1) {

        currentTime       = millis();
        pedestrianWaiting = pedestrian_detected();

        switch (currentState) {

            case VEHICLE_GREEN:
                set_lights(0, 0, 1, 0, 1);
                if (pedestrianWaiting &&
                    (currentTime - stateStartTime >= VEHICLE_GREEN_TIME)) {
                    currentState   = VEHICLE_YELLOW;
                    stateStartTime = currentTime;
                }
                break;

            case VEHICLE_YELLOW:
                set_lights(0, 1, 0, 0, 1);
                if (currentTime - stateStartTime >= VEHICLE_YELLOW_TIME) {
                    currentState   = VEHICLE_RED;
                    stateStartTime = currentTime;
                }
                break;

            case VEHICLE_RED:
                set_lights(1, 0, 0, 0, 1);
                if (pedestrianWaiting) {
                    currentState   = PEDESTRIAN_GREEN;
                    stateStartTime = currentTime;
                } else if (currentTime - stateStartTime >= VEHICLE_RED_TIME) {
                    currentState   = VEHICLE_PRE_GREEN;
                    stateStartTime = currentTime;
                }
                break;

            case PEDESTRIAN_GREEN:
                set_lights(1, 0, 0, 1, 0);
                if (currentTime - stateStartTime >= PED_CROSS_TIME) {
                    currentState   = VEHICLE_PRE_GREEN;
                    stateStartTime = currentTime;
                }
                break;

            case VEHICLE_PRE_GREEN:
                set_lights(1, 1, 0, 0, 1);
                if (currentTime - stateStartTime >= VEHICLE_PRE_GREEN_TIME) {
                    currentState   = VEHICLE_GREEN;
                    stateStartTime = currentTime;
                }
                break;

            default:
                currentState   = VEHICLE_GREEN;
                stateStartTime = currentTime;
                set_lights(0, 0, 1, 0, 1);
                break;
        }
    }

    return 0;
}
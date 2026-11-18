/* Fire Bird V ATmega2560 I/O bridge for the Raspberry Pi.
 *
 * The Pi decides everything. This firmware only:
 *   - applies motor PWM/direction and servo pulses the Pi sends,
 *   - streams raw sensors (encoders, Sharp IR, IR proximity 1-8, white line, battery) at 50 Hz,
 *     IR proximity 1-5 from its own ADC, 6-8 from the ATmega8 slave over SPI,
 *   - stops on its own: command timeout, E-stop, battery cutoff, optional front reflex,
 *     and a 60 ms hardware watchdog.
 * Pins from the Fire Bird V ATMEGA2560 Hardware Manual V1.08. Crystal 14.7456 MHz.
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "protocol.h"

#define FW_VERSION 4
#define TICKS_PER_MS 230UL           /* Timer3, prescaler 64: 230.4 ticks per ms */
#define TELEM_PERIOD_MS 20
#define MIN_EDGE_TICKS (5 * TICKS_PER_MS)   /* glitch filter: real pulses are >= 25 ms apart */
#define STOPPED_TICKS (300 * TICKS_PER_MS)  /* no edge for 300 ms = wheel stopped */

/* ---------------------------------------------------------------- config */
static uint16_t cfg_timeout_ms = 250;
static uint16_t cfg_reflex_sharp_adc = 238; /* ~25 cm with NEX's Sharp formula; Pi overrides */
static uint8_t cfg_reflex_sharp_ch = 2;     /* Sharp sensor 3 = front */
static uint8_t cfg_reflex_prox_mask = 0;    /* off until the Pi sends calibrated values */
static uint16_t cfg_reflex_prox_adc = 300;
static uint16_t cfg_batt_cutoff_mv = 8000;
static uint16_t cfg_dwell_ms = 300;
static uint16_t cfg_ir_off_ms = 10; /* LEDs off this long before the ambient reading; 0 = LEDs always on */
static uint16_t cfg_ir_on_ms = 2;   /* LEDs on this long before the lit reading */
static uint16_t cfg_ir_win_ms = 10; /* each reading averages this window (10 ms = one 100 Hz flicker cycle) */
static uint8_t cfg_ir_led_pins = 3; /* bit0 PH2, bit1 PH3: NEX's manual says PH2, its example code PH3 */

/* ----------------------------------------------------------------- clock */
static volatile uint16_t t3_hi;
ISR(TIMER3_OVF_vect) { t3_hi++; }

static uint32_t now_ticks(void) {
    uint8_t s = SREG;
    cli();
    uint16_t lo = TCNT3, hi = t3_hi;
    if ((TIFR3 & (1 << TOV3)) && lo < 0x8000) hi++; /* overflow pending, ISR not run yet */
    SREG = s;
    return ((uint32_t)hi << 16) | lo;
}

static uint32_t ticks_to_us(uint32_t t) { return (uint32_t)(((uint64_t)t * 625) / 144); }
static uint32_t ticks_to_ms(uint32_t t) { return (uint32_t)(((uint64_t)t * 5) / 1152); }

/* -------------------------------------------------------------- encoders */
static volatile int32_t enc[2];
static volatile uint32_t last_edge[2], period[2];
static volatile int8_t enc_dir[2] = {1, 1}; /* sign of the applied drive; kept while coasting */

static inline void encoder_edge(uint8_t i) {
    uint32_t t = now_ticks(), dt = t - last_edge[i];
    if (dt < MIN_EDGE_TICKS) return;
    enc[i] += enc_dir[i];
    period[i] = dt;
    last_edge[i] = t;
}
ISR(INT4_vect) { encoder_edge(0); } /* left,  PE4 */
ISR(INT5_vect) { encoder_edge(1); } /* right, PE5 */

/* ------------------------------------------------------------------- ADC */
/* 0 battery, 1 line R, 2 line C, 3 line L, 4-7 prox 1-4, 8 prox 5, 9-13 Sharp 1-5 */
#define N_ADC 14
static volatile uint16_t adc[N_ADC];
static volatile uint8_t adc_ch, adc_discard;
/* IR proximity window accumulators: 0-4 filled by the ADC ISR, 5-7 by spi_poll() */
static volatile uint32_t acc_sum[8];
static volatile uint16_t acc_n[8];

static void adc_select(uint8_t ch) {
    ADMUX = ch & 7; /* REFS = 00: external AREF (tied to 5V uC on the board) */
    if (ch >= 8) ADCSRB |= (1 << MUX5); else ADCSRB &= ~(1 << MUX5);
}

ISR(ADC_vect) {
    uint16_t v = ADC;
    if (adc_discard) adc_discard = 0; /* first result after a mux change is unreliable */
    else {
        adc[adc_ch] = v;
        if (adc_ch >= 4 && adc_ch <= 8) { acc_sum[adc_ch - 4] += v; acc_n[adc_ch - 4]++; }
        adc_ch = (adc_ch + 1) % N_ADC;
        adc_select(adc_ch);
        adc_discard = 1;
    }
    ADCSRA |= (1 << ADSC);
}

/* ------------------------------------------- IR proximity 1-8, ambient light rejected */
/* Hardware manual V1.08 §3.11: each sensor is an IR LED + photodiode; ~3.3 V with nothing
 * near, falling as an obstacle comes closer. The photodiode also sees room light, so the
 * manual recommends reading with the LEDs on and off and taking the difference. All 8 LEDs
 * share one switch (T3/T4, logic 1 = off). It only works with the IRP jumper (J1-2 on the
 * main board) removed - robots ship with it inserted, which keeps the LEDs on for good.
 * The manual says the switch is PH2, NEX's example code says PH3: cfg_ir_led_pins drives
 * either or both (default both; PH2 also powers Sharp 1+5, PH3 Sharp 1-5 per the manual).
 *
 * Cycle: LEDs off, wait ir_off_ms (the LED supply has to discharge), average ir_win_ms ->
 * dark; LEDs on, wait ir_on_ms, average ir_win_ms -> lit. A 10 ms window spans one 100 Hz
 * flicker cycle of mains lighting, so the flicker averages out of both readings.
 * Sensors 1-5: ADC4-8, averaged by the ADC ISR. Sensors 6-8: the ATmega8 over SPI (NEX
 * examples 14_A/14_B: send channel 5/6/7, wait for its conversion, clock a dummy byte, the
 * reply is its 8-bit ADC). The SPI needs the three J4 jumpers on the adaptor board (open
 * by default; remove them only for ISP). Everything is non-blocking. */
static uint16_t ir_lit[8], ir_dark[8];
static uint8_t ir_phase, ir_epoch; /* epoch changes at every window start */
static uint32_t ir_t;

static void ir_leds(uint8_t on) {
    uint8_t m = ((cfg_ir_led_pins & 1) ? (1 << PH2) : 0) | ((cfg_ir_led_pins & 2) ? (1 << PH3) : 0);
    if (on) PORTH &= ~m; else PORTH |= m;
}

static void acc_reset(void) {
    uint8_t s = SREG;
    cli();
    for (uint8_t i = 0; i < 8; i++) { acc_sum[i] = 0; acc_n[i] = 0; }
    ir_epoch++;
    SREG = s;
}

static void acc_take(uint16_t *out) { /* keeps the old value if a sensor got no sample */
    uint8_t s = SREG;
    cli();
    for (uint8_t i = 0; i < 8; i++) if (acc_n[i]) out[i] = acc_sum[i] / acc_n[i];
    SREG = s;
}

static void ir_poll(uint32_t now) {
    uint32_t dt = now - ir_t;
    if (!cfg_ir_off_ms) { /* LEDs always on, as in NEX's examples: no ambient reading */
        ir_leds(1);
        if (dt >= cfg_ir_win_ms * TICKS_PER_MS) {
            acc_take(ir_lit); acc_reset(); ir_t = now;
            for (uint8_t i = 0; i < 8; i++) ir_dark[i] = 0xFFFF; /* tells the Pi: no ambient */
        }
        return;
    }
    switch (ir_phase) {
    case 0: if (dt >= cfg_ir_off_ms * TICKS_PER_MS) { acc_reset(); ir_t = now; ir_phase = 1; } break;
    case 1: if (dt >= cfg_ir_win_ms * TICKS_PER_MS) { acc_take(ir_dark); ir_leds(1); ir_t = now; ir_phase = 2; } break;
    case 2: if (dt >= cfg_ir_on_ms * TICKS_PER_MS) { acc_reset(); ir_t = now; ir_phase = 3; } break;
    default: if (dt >= cfg_ir_win_ms * TICKS_PER_MS) { acc_take(ir_lit); ir_leds(0); ir_t = now; ir_phase = 0; } break;
    }
}

/* Each rear channel is asked twice in a row and only the second answer is kept: the ATmega8
 * switches its ADC mux just before converting, and a photodiode's high source impedance
 * cannot recharge the sample capacitor that fast, so a first answer carries some of the
 * previous sensor's voltage (the same effect made unconnected inputs copy their neighbour). */
static uint8_t spi_ch, spi_rep, spi_phase, spi_tag;
static uint32_t spi_t;

static void spi_poll(uint32_t now) {
    switch (spi_phase) {
    case 0: PORTB &= ~(1 << PB0); spi_tag = ir_epoch; SPDR = 5 + spi_ch; spi_phase = 1; break; /* SS low, ask */
    case 1: if (SPSR & (1 << SPIF)) { (void)SPDR; spi_t = now; spi_phase = 2; } break;
    case 2: if (now - spi_t >= TICKS_PER_MS / 2) { SPDR = 0x50; spi_phase = 3; } break; /* dummy byte */
    default:
        if (!(SPSR & (1 << SPIF))) break;
        uint8_t v = SPDR;
        PORTB |= (1 << PB0); /* SS high */
        if (spi_rep && spi_tag == ir_epoch) { acc_sum[5 + spi_ch] += v; acc_n[5 + spi_ch]++; } /* same window only */
        if (spi_rep) spi_ch = spi_ch == 2 ? 0 : spi_ch + 1;
        spi_rep ^= 1;
        spi_phase = 0;
        break;
    }
}

/* ------------------------------------------------------------------ UART2 */
static volatile uint8_t rx_buf[128], rx_head, rx_tail;
static volatile uint8_t tx_buf[256], tx_head, tx_tail;

ISR(USART2_RX_vect) {
    uint8_t b = UDR2, n = (rx_head + 1) & 127;
    if (n != rx_tail) { rx_buf[rx_head] = b; rx_head = n; }
}
ISR(USART2_UDRE_vect) {
    if (tx_head == tx_tail) { UCSR2B &= ~(1 << UDRIE2); return; }
    UDR2 = tx_buf[tx_tail++];
}

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t f[FB_MAX_PAYLOAD + 5], n = fb_encode(type, payload, len, f);
    if ((uint8_t)(tx_head - tx_tail) > (uint8_t)(255 - n)) return; /* no room: drop, Pi sees the gap */
    for (uint8_t i = 0; i < n; i++) tx_buf[tx_head++] = f[i];
    UCSR2B |= (1 << UDRIE2);
}

/* ----------------------------------------------------------------- outputs */
static int16_t cmd_pwm[2];     /* requested by the Pi */
static int16_t applied[2];     /* what the motors get */
static uint32_t zero_since[2]; /* tick when the wheel last went to 0 */
static uint16_t servo_us[3];

static void motors_apply(int16_t l, int16_t r) {
    uint8_t dir = 0;
    if (l > 0) dir |= 1 << PA1; else if (l < 0) dir |= 1 << PA0;
    if (r > 0) dir |= 1 << PA2; else if (r < 0) dir |= 1 << PA3;
    OCR5A = abs(l);
    OCR5B = abs(r);
    PORTA = (PORTA & 0xF0) | dir;
    applied[0] = l;
    applied[1] = r;
    if (l) enc_dir[0] = l > 0 ? 1 : -1;
    if (r) enc_dir[1] = r > 0 ? 1 : -1;
}

static void servo_set(uint8_t ch, uint16_t us) {
    if (ch > 2) return;
    if (us) us = us < 500 ? 500 : (us > 2500 ? 2500 : us);
    servo_us[ch] = us;
    uint16_t ocr = (uint16_t)(((uint32_t)us * 2304) / 1250); /* 1.8432 ticks per us */
    if (ch == 0) OCR1A = ocr; else if (ch == 1) OCR1B = ocr; else OCR1C = ocr;
}

static void init(void) {
    /* JTAG off. IR proximity 1-4 sit on PF4-7 = TCK/TMS/TDO/TDI. With the JTAGEN fuse set (the
     * ATmega2560 factory default) the JTAG port owns those pins: the sensor voltages clock its
     * state machine at random and it drives TDO (IR3 read 1022 = 5 V from a 3.3 V sensor).
     * JTD must be written twice within 4 cycles; two back-to-back stores, interrupts still off. */
    uint8_t mcucr = MCUCR | (1 << JTD);
    MCUCR = mcucr;
    MCUCR = mcucr;

    /* Motors: PA0-PA3 direction, PL3/PL4 PWM (Timer5 fast PWM 8-bit, /64 = 900 Hz) */
    DDRA |= 0x0F; PORTA &= 0xF0;
    DDRL |= (1 << PL3) | (1 << PL4);
    TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << WGM50);
    TCCR5B = (1 << WGM52) | (1 << CS51) | (1 << CS50);
    OCR5A = OCR5B = 0;

    /* Servos: PB5/PB6/PB7 on Timer1 mode 14, /8, ICR1 = 20 ms -> 0.54 us steps */
    DDRB |= (1 << PB5) | (1 << PB6) | (1 << PB7);
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 36863;
    OCR1A = OCR1B = OCR1C = 0; /* no pulse until the Pi asks */

    /* Clock: Timer3 normal mode, /64 */
    TCCR3A = 0;
    TCCR3B = (1 << CS31) | (1 << CS30);
    TIMSK3 = (1 << TOIE3);

    /* Encoders: PE4/PE5 inputs with pull-ups, INT4/INT5 falling edge. PE7 = E-stop button. */
    DDRE &= ~((1 << PE4) | (1 << PE5) | (1 << PE7));
    PORTE |= (1 << PE4) | (1 << PE5) | (1 << PE7);
    EICRB |= (1 << ISC41) | (1 << ISC51);
    EIMSK |= (1 << INT4) | (1 << INT5);

    /* Sensor power, low = on: PG2 Sharp 2-4 + white line LEDs; PH2/PH3 IR proximity LEDs and
     * Sharp 1/5 (NEX's manual and example disagree which is which) - switched by ir_poll() */
    DDRH |= (1 << PH2) | (1 << PH3); PORTH &= ~((1 << PH2) | (1 << PH3));
    DDRG |= (1 << PG2); PORTG &= ~(1 << PG2);

    /* ADC: /128 = 115 kHz, interrupt-driven scan */
    adc_select(0);
    adc_discard = 1;
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);

    /* SPI master to the ATmega8: PB0 SS, PB1 SCK, PB2 MOSI out (SS must be an output or a low
     * level on it drops us out of master mode). Mode 0, fosc/128 = 115 kHz, as in NEX's example. */
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2);
    PORTB |= (1 << PB0);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);

    /* UART2 (FT232 USB): 115200 8N1, UBRR = 7 at 14.7456 MHz (0 % error) */
    UBRR2 = 7;
    UCSR2A = 0;
    UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);
    UCSR2B = (1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2);
}

/* ------------------------------------------------------- LCD: motion direction */
/* 16x2 HD44780 in 4-bit mode, wired as in NEX's lcd.c: RS PC0, RW PC1 (always 0 = write),
 * EN PC2, D4-D7 PC4-PC7. PC3 is the buzzer and is never touched. NEX waits 5 ms per nibble;
 * the controller needs ~40 us, so a full 2-line refresh here takes ~4 ms, done at most every
 * LCD_PERIOD_MS and only when the text changed. */
#define LCD_PERIOD_MS 250

static void lcd_nibble(uint8_t v, uint8_t rs) {
    PORTC = (PORTC & (1 << PC3)) | (v & 0xF0) | (rs ? (1 << PC0) : 0);
    PORTC |= (1 << PC2); _delay_us(2); PORTC &= ~(1 << PC2);
    _delay_us(60);
}
static void lcd_byte(uint8_t v, uint8_t rs) { lcd_nibble(v, rs); lcd_nibble(v << 4, rs); }

static void lcd_line(uint8_t row, const char *s) { /* pads to 16 chars so old text is overwritten */
    lcd_byte(row ? 0xC0 : 0x80, 0);
    for (uint8_t i = 0; i < 16; i++) lcd_byte(*s ? *s++ : ' ', 1);
}

static void lcd_init(void) { /* HD44780 4-bit start-up; runs before the watchdog is enabled */
    DDRC |= 0xF7;
    _delay_ms(40);
    for (uint8_t i = 0; i < 3; i++) { lcd_nibble(0x30, 0); _delay_ms(5); }
    lcd_nibble(0x20, 0); _delay_ms(1);
    lcd_byte(0x28, 0); /* 4-bit, 2 lines */
    lcd_byte(0x0C, 0); /* display on, no cursor */
    lcd_byte(0x06, 0); /* left to right */
    lcd_byte(0x01, 0); _delay_ms(2); /* clear */
    lcd_line(0, "FIRE BIRD V");
    lcd_line(1, "READY");
}

/* Direction of what the motors actually get (after every fail-safe). The heading turns by
 * (right - left): positive = left. Within 25 % of each other counts as straight. */
static const char *motion_name(int16_t l, int16_t r) {
    if (!l && !r) return "STOPPED";
    if ((l > 0 && r < 0) || (l < 0 && r > 0)) return r > 0 ? "SPIN LEFT" : "SPIN RIGHT";
    int16_t al = abs(l), ar = abs(r), mx = al > ar ? al : ar, turn = r - l;
    uint8_t straight = (uint16_t)abs(al - ar) * 4 <= (uint16_t)mx;
    if (l >= 0 && r >= 0) return straight ? "FORWARD" : turn > 0 ? "FORWARD LEFT" : "FORWARD RIGHT";
    return straight ? "BACKWARD" : turn > 0 ? "BACKWARD LEFT" : "BACKWARD RIGHT";
}

static void lcd_motion(int16_t l, int16_t r, uint16_t flags) {
    static char shown[2][17];
    char line2[17] = "L", *p = line2 + 1;
    if (!l && !r && (flags & (FLAG_ESTOP | FLAG_BATT_LOW | FLAG_UNARMED | FLAG_TIMEOUT))) {
        const char *why = (flags & FLAG_ESTOP) ? "E-STOP" : (flags & FLAG_BATT_LOW) ? "BATTERY LOW"
                        : (flags & FLAG_UNARMED) ? "WAITING FOR PI" : "NO COMMAND";
        for (uint8_t i = 0; (line2[i] = why[i]); i++);
    } else { /* "L+120  R-120" */
        *p++ = l < 0 ? '-' : '+'; itoa(abs(l), p, 10); p += strlen(p);
        *p++ = ' '; *p++ = ' '; *p++ = 'R'; *p++ = r < 0 ? '-' : '+'; itoa(abs(r), p, 10);
    }
    const char *line1 = motion_name(l, r);
    if (strcmp(line1, shown[0])) { lcd_line(0, line1); strcpy(shown[0], line1); }
    if (strcmp(line2, shown[1])) { lcd_line(1, line2); strcpy(shown[1], line2); }
}

static uint16_t batt_mv(void) { return 700 + (uint16_t)(((uint32_t)adc[0] * 20151) / 1024); }

int main(void) {
    uint8_t mcusr = MCUSR;
    MCUSR = 0;
    wdt_disable();
    init();
    lcd_init(); /* ~60 ms of start-up delays: before the 60 ms watchdog is enabled */
    sei();
    wdt_enable(WDTO_60MS);

    uint8_t boot[4] = {FW_VERSION, mcusr};
    put_u16(boot + 2, 14746);
    send_frame(FB_BOOT, boot, 4);

    fb_parser parser = {0};
    uint16_t crc_seen = 0;
    uint8_t seq_ack = 0, armed = 0, estop = 0, batt_low = 0;
    uint32_t last_cmd = now_ticks(), last_telem = last_cmd, last_lcd = last_cmd, batt_bad_since = 0, estop_since = 0;

    for (;;) {
        wdt_reset();
        uint32_t now = now_ticks();
        ir_poll(now);
        spi_poll(now);

        /* --- receive */
        while (rx_tail != rx_head) {
            uint8_t b = rx_buf[rx_tail];
            rx_tail = (rx_tail + 1) & 127;
            if (!fb_feed(&parser, b)) continue;
            const uint8_t *p = parser.payload;
            switch (parser.type) {
            case FB_DRIVE:
                if (parser.len != 5) break;
                seq_ack = p[0];
                cmd_pwm[0] = (int16_t)get_u16(p + 1);
                cmd_pwm[1] = (int16_t)get_u16(p + 3);
                last_cmd = now;
                armed = 1;
                break;
            case FB_SERVO:
                if (parser.len == 3) servo_set(p[0], get_u16(p + 1));
                break;
            case FB_CONFIG: {
                if (parser.len != 3) break;
                uint16_t v = get_u16(p + 1);
                switch (p[0]) {
                case CFG_TIMEOUT_MS: cfg_timeout_ms = v < 50 ? 50 : (v > 1000 ? 1000 : v); break;
                case CFG_REFLEX_SHARP_ADC: cfg_reflex_sharp_adc = v; break;
                case CFG_REFLEX_SHARP_CH: if (v < 5) cfg_reflex_sharp_ch = v; break;
                case CFG_REFLEX_PROX_MASK: cfg_reflex_prox_mask = v & 0x1F; break;
                case CFG_REFLEX_PROX_ADC: cfg_reflex_prox_adc = v; break;
                case CFG_BATT_CUTOFF_MV: cfg_batt_cutoff_mv = v; break;
                case CFG_DWELL_MS: cfg_dwell_ms = v > 2000 ? 2000 : v; break;
                case CFG_IR_OFF_MS: cfg_ir_off_ms = v > 200 ? 200 : v; break;
                case CFG_IR_ON_MS: cfg_ir_on_ms = v > 200 ? 200 : v; break;
                case CFG_IR_WIN_MS: cfg_ir_win_ms = v < 1 ? 1 : (v > 200 ? 200 : v); break;
                case CFG_IR_LED_PINS: cfg_ir_led_pins = v & 3; break;
                }
                break;
            }
            case FB_STOP: estop = 1; break;
            case FB_CLEAR: estop = 0; batt_low = 0; break;
            }
        }

        /* --- E-stop button (PE7 low), held 5 ms */
        if (!(PINE & (1 << PE7))) {
            if (!estop_since) estop_since = now | 1;
            else if (now - estop_since > 5 * TICKS_PER_MS) estop = 1;
        } else estop_since = 0;

        /* --- battery cutoff after 2 s below the limit (motor surges dip it briefly) */
        if (cfg_batt_cutoff_mv && batt_mv() < cfg_batt_cutoff_mv) {
            if (!batt_bad_since) batt_bad_since = now | 1;
            else if (now - batt_bad_since > 2000 * TICKS_PER_MS) batt_low = 1;
        } else batt_bad_since = 0;

        /* --- decide what the motors get (stop-only rules) */
        uint16_t flags = 0;
        int16_t want[2] = {cmd_pwm[0], cmd_pwm[1]};
        if (!armed) flags |= FLAG_UNARMED;
        if (now - last_cmd > cfg_timeout_ms * TICKS_PER_MS) flags |= FLAG_TIMEOUT;
        if (estop) flags |= FLAG_ESTOP;
        if (batt_low) flags |= FLAG_BATT_LOW;
        if (flags) want[0] = want[1] = 0;

        uint8_t close = cfg_reflex_sharp_adc && adc[9 + cfg_reflex_sharp_ch] >= cfg_reflex_sharp_adc;
        for (uint8_t i = 0; i < 5; i++) { /* reflection = ambient - lit (voltage falls as light grows) */
            int16_t refl = ir_dark[i] == 0xFFFF ? 675 - (int16_t)ir_lit[i] : (int16_t)ir_dark[i] - (int16_t)ir_lit[i];
            if ((cfg_reflex_prox_mask & (1 << i)) && refl >= (int16_t)cfg_reflex_prox_adc) close = 1;
        }
        if (close && want[0] + want[1] > 0) { /* keep only the turn-in-place part */
            int16_t turn = (want[1] - want[0]) / 2;
            want[0] = -turn;
            want[1] = turn;
            flags |= FLAG_REFLEX;
        }

        for (uint8_t i = 0; i < 2; i++) { /* reversal dwell: pass through 0 for cfg_dwell_ms */
            if (applied[i] == 0) {
                if (!zero_since[i]) zero_since[i] = now | 1;
            } else zero_since[i] = 0;
            if ((want[i] > 0 && applied[i] < 0) || (want[i] < 0 && applied[i] > 0)) want[i] = 0;
            else if (want[i] && applied[i] == 0 && zero_since[i] &&
                     now - zero_since[i] < cfg_dwell_ms * TICKS_PER_MS &&
                     ((want[i] > 0) != (enc_dir[i] > 0))) {
                want[i] = 0;
                flags |= FLAG_DWELL;
            }
            if (want[i] > 255) want[i] = 255;
            if (want[i] < -255) want[i] = -255;
        }
        motors_apply(want[0], want[1]);

        /* --- LCD: show the direction the robot is actually driving */
        if (now - last_lcd >= LCD_PERIOD_MS * TICKS_PER_MS) { last_lcd = now; lcd_motion(applied[0], applied[1], flags); }

        /* --- telemetry */
        if (now - last_telem >= TELEM_PERIOD_MS * TICKS_PER_MS) {
            last_telem = now;
            if (parser.crc_errors != crc_seen) { crc_seen = parser.crc_errors; flags |= FLAG_CRC_ERR; }
            uint8_t t[TELEM_LEN];
            uint8_t s = SREG;
            cli();
            int32_t e0 = enc[0], e1 = enc[1];
            uint32_t p0 = now - last_edge[0] > STOPPED_TICKS ? 0 : period[0];
            uint32_t p1 = now - last_edge[1] > STOPPED_TICKS ? 0 : period[1];
            uint16_t a[N_ADC];
            for (uint8_t i = 0; i < N_ADC; i++) a[i] = adc[i];
            SREG = s;
            put_u32(t + TELEM_T_MS, ticks_to_ms(now));
            t[TELEM_SEQ_ACK] = seq_ack;
            put_u16(t + TELEM_FLAGS, flags);
            put_u32(t + TELEM_ENC_L, (uint32_t)e0);
            put_u32(t + TELEM_ENC_R, (uint32_t)e1);
            put_u32(t + TELEM_PER_L, ticks_to_us(p0));
            put_u32(t + TELEM_PER_R, ticks_to_us(p1));
            for (uint8_t i = 0; i < 5; i++) put_u16(t + TELEM_SHARP + 2 * i, a[9 + i]);
            for (uint8_t i = 0; i < 5; i++) put_u16(t + TELEM_PROX + 2 * i, ir_lit[i]);
            put_u16(t + TELEM_LINE + 0, a[3]);
            put_u16(t + TELEM_LINE + 2, a[2]);
            put_u16(t + TELEM_LINE + 4, a[1]);
            put_u16(t + TELEM_BATT_MV, batt_mv());
            for (uint8_t i = 0; i < 3; i++) put_u16(t + TELEM_SERVO + 2 * i, servo_us[i]);
            for (uint8_t i = 0; i < 3; i++) put_u16(t + TELEM_PROX_REAR + 2 * i, ir_lit[5 + i]);
            for (uint8_t i = 0; i < 8; i++) put_u16(t + TELEM_PROX_DARK + 2 * i, ir_dark[i]);
            t[TELEM_FW] = FW_VERSION;
            send_frame(FB_TELEM, t, TELEM_LEN);
        }
    }
}

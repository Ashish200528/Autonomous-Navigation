/* Pi <-> ATmega2560 frame protocol. Pure C, no AVR headers: also compiled on
 * the PC by test_protocol.c. Mirror of backend/src/protocol.js.
 *
 * Frame: 0xAA 0x55 | type | len | payload[len] | crc8(type, len, payload)
 * CRC-8, poly 0x07, init 0x00 (CRC-8/SMBUS). Multi-byte fields little-endian.
 */
#ifndef FB_PROTOCOL_H
#define FB_PROTOCOL_H

#include <stdint.h>

#define FB_SYNC0 0xAA
#define FB_SYNC1 0x55
#define FB_MAX_PAYLOAD 96

/* Pi -> MCU */
#define FB_DRIVE  0x01 /* seq u8, pwm_l i16, pwm_r i16 (-255..255) */
#define FB_SERVO  0x02 /* ch u8 (0..2), pulse_us u16 (0 = off) */
#define FB_CONFIG 0x03 /* key u8, value i16 */
#define FB_STOP   0x04 /* latch software E-stop */
#define FB_CLEAR  0x05 /* clear E-stop + battery latches */

/* MCU -> Pi */
#define FB_BOOT   0x80 /* version u8, mcusr u8, f_cpu_khz u16 */
#define FB_TELEM  0x81 /* see TELEM_* offsets */

/* CONFIG keys */
#define CFG_TIMEOUT_MS      1
#define CFG_REFLEX_SHARP_ADC 2 /* 0 = off; Sharp ADC >= value means too close */
#define CFG_REFLEX_SHARP_CH  3 /* 0..4 = Sharp sensor 1..5 */
#define CFG_REFLEX_PROX_MASK 4 /* bit i = IR proximity sensor i+1 counts as front */
#define CFG_REFLEX_PROX_ADC  5 /* reflection (ambient - lit) >= value means obstacle (v3) */
#define CFG_BATT_CUTOFF_MV   6 /* 0 = off */
#define CFG_DWELL_MS         7
#define CFG_IR_OFF_MS        8  /* v3: IR LEDs off before the ambient reading; 0 = LEDs always on */
#define CFG_IR_ON_MS         9  /* v3: IR LEDs on before the lit reading */
#define CFG_IR_WIN_MS        10 /* v3: averaging window of each reading */
#define CFG_IR_LED_PINS      11 /* v3: bit0 PH2, bit1 PH3 switch the IR LEDs */

/* TELEM flags */
#define FLAG_TIMEOUT  (1u << 0)
#define FLAG_REFLEX   (1u << 1)
#define FLAG_ESTOP    (1u << 2)
#define FLAG_BATT_LOW (1u << 3)
#define FLAG_DWELL    (1u << 4)
#define FLAG_CRC_ERR  (1u << 5)
#define FLAG_UNARMED  (1u << 6)

/* TELEM payload offsets (80 bytes). IR proximity values are raw ADC with the IR LEDs on
 * ("lit") plus, from v2, the ambient level with them off ("dark"); the reflection is the
 * difference. v1 sent 57 bytes: prox only, LEDs always on, ambient included. */
#define TELEM_T_MS     0  /* u32 */
#define TELEM_SEQ_ACK  4  /* u8  */
#define TELEM_FLAGS    5  /* u16 */
#define TELEM_ENC_L    7  /* i32 */
#define TELEM_ENC_R    11 /* i32 */
#define TELEM_PER_L    15 /* u32 us, 0 = stopped */
#define TELEM_PER_R    19 /* u32 us */
#define TELEM_SHARP    23 /* 5 x u16 raw 10-bit ADC */
#define TELEM_PROX     33 /* 5 x u16, IR proximity 1-5 lit */
#define TELEM_LINE     43 /* 3 x u16 (left, centre, right) */
#define TELEM_BATT_MV  49 /* u16 */
#define TELEM_SERVO    51 /* 3 x u16 us */
#define TELEM_PROX_REAR 57 /* 3 x u16: IR proximity 6-8 lit, from the ATmega8 over SPI (8-bit) */
#define TELEM_PROX_DARK 63 /* 8 x u16: IR proximity 1-8 ambient (LEDs off); 6-8 are 8-bit; 0xFFFF = LEDs kept on */
#define TELEM_FW        79 /* u8 firmware version (v3+); the BOOT frame alone can be missed */
#define TELEM_LEN      80

uint8_t fb_crc8(const uint8_t *data, uint8_t len, uint8_t crc);

/* Writes a full frame into out (needs len + 5 bytes). Returns frame length. */
uint8_t fb_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);

typedef struct {
    uint8_t state, type, len, idx;
    uint8_t payload[FB_MAX_PAYLOAD];
    uint16_t crc_errors;
} fb_parser;

/* Feed one byte. Returns 1 when a valid frame is in p->type/len/payload. */
uint8_t fb_feed(fb_parser *p, uint8_t b);

static inline void put_u16(uint8_t *b, uint16_t v) { b[0] = v; b[1] = v >> 8; }
static inline void put_u32(uint8_t *b, uint32_t v) { put_u16(b, v); put_u16(b + 2, v >> 16); }
static inline uint16_t get_u16(const uint8_t *b) { return b[0] | ((uint16_t)b[1] << 8); }

#endif

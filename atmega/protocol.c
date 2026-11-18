#include "protocol.h"

uint8_t fb_crc8(const uint8_t *data, uint8_t len, uint8_t crc) {
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t fb_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out) {
    out[0] = FB_SYNC0;
    out[1] = FB_SYNC1;
    out[2] = type;
    out[3] = len;
    for (uint8_t i = 0; i < len; i++) out[4 + i] = payload[i];
    out[4 + len] = fb_crc8(out + 2, len + 2, 0);
    return len + 5;
}

enum { S_SYNC0, S_SYNC1, S_TYPE, S_LEN, S_DATA, S_CRC };

uint8_t fb_feed(fb_parser *p, uint8_t b) {
    switch (p->state) {
    case S_SYNC0:
        if (b == FB_SYNC0) p->state = S_SYNC1;
        return 0;
    case S_SYNC1:
        p->state = (b == FB_SYNC1) ? S_TYPE : (b == FB_SYNC0 ? S_SYNC1 : S_SYNC0);
        return 0;
    case S_TYPE:
        p->type = b;
        p->state = S_LEN;
        return 0;
    case S_LEN:
        if (b > FB_MAX_PAYLOAD) { p->state = S_SYNC0; p->crc_errors++; return 0; }
        p->len = b;
        p->idx = 0;
        p->state = b ? S_DATA : S_CRC;
        return 0;
    case S_DATA:
        p->payload[p->idx++] = b;
        if (p->idx == p->len) p->state = S_CRC;
        return 0;
    default: { /* S_CRC */
        uint8_t hdr[2] = {p->type, p->len};
        uint8_t crc = fb_crc8(p->payload, p->len, fb_crc8(hdr, 2, 0));
        p->state = S_SYNC0;
        if (crc == b) return 1;
        p->crc_errors++;
        return 0;
    }
    }
}

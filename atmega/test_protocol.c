/* Host check (gcc, not avr-gcc): make test
 * Same vectors as backend/test/protocol.test.js, so both sides agree byte for byte. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "protocol.h"

int main(void) {
    assert(fb_crc8((const uint8_t *)"123456789", 9, 0) == 0xF4);

    /* DRIVE seq=1, left=100, right=-100 */
    const uint8_t payload[] = {0x01, 0x64, 0x00, 0x9C, 0xFF};
    const uint8_t expect[] = {0xAA, 0x55, 0x01, 0x05, 0x01, 0x64, 0x00, 0x9C, 0xFF, 0xD3};
    uint8_t f[16];
    assert(fb_encode(FB_DRIVE, payload, 5, f) == sizeof expect);
    assert(memcmp(f, expect, sizeof expect) == 0);

    /* parser: garbage before, one good frame, one corrupted frame */
    fb_parser p = {0};
    const uint8_t junk[] = {0x00, 0xAA, 0x13, 0x55};
    int frames = 0;
    for (unsigned i = 0; i < sizeof junk; i++) frames += fb_feed(&p, junk[i]);
    for (unsigned i = 0; i < sizeof expect; i++) frames += fb_feed(&p, expect[i]);
    assert(frames == 1 && p.type == FB_DRIVE && p.len == 5 && memcmp(p.payload, payload, 5) == 0);
    f[6] ^= 0x01;
    for (unsigned i = 0; i < sizeof expect; i++) frames += fb_feed(&p, f[i]);
    assert(frames == 1 && p.crc_errors == 1);

    puts("protocol ok");
    return 0;
}

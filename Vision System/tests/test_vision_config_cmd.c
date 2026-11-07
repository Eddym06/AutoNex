#include "vision_config_cmd.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static vision_cmd_event_t feed_text(vision_cmd_parser_t *p, vision_config_t *cfg, const char *text)
{
    vision_cmd_event_t last = VISION_CMD_NONE;
    for (; *text; ++text) {
        const vision_cmd_event_t ev = vision_cmd_feed(p, (uint8_t)*text, cfg);
        if (ev != VISION_CMD_NONE) last = ev;
    }
    return last;
}

int main(void)
{
    vision_cmd_parser_t p;
    vision_config_t cfg;
    vision_cmd_reset(&p);
    vision_config_default(&cfg);

    /* Text commands, as typed in a serial monitor (CRLF included). */
    assert(feed_text(&p, &cfg, "set pass_side_red -1\r\n") == VISION_CMD_CHANGED);
    assert(cfg.pass_side_red == -1);
    assert(feed_text(&p, &cfg, "set camera.exposure 640\n") == VISION_CMD_CHANGED);
    assert(cfg.camera.exposure == 640);
    assert(feed_text(&p, &cfg, "set no.such.key 1\n") == VISION_CMD_ERROR);
    assert(strstr(p.reply, "no.such.key") != NULL && cfg.camera.exposure == 640);
    assert(feed_text(&p, &cfg, "save\n") == VISION_CMD_SAVE);
    assert(feed_text(&p, &cfg, "defaults\n") == VISION_CMD_DEFAULTS);
    assert(feed_text(&p, &cfg, "dump\n") == VISION_CMD_DUMP);
    assert(feed_text(&p, &cfg, "help\n") == VISION_CMD_REPLY);
    assert(feed_text(&p, &cfg, "bogus\n") == VISION_CMD_ERROR);
    assert(feed_text(&p, &cfg, "\n\n") == VISION_CMD_NONE);

    /* A binary frame in the middle of the same stream. */
    vision_config_t other;
    vision_config_default(&other);
    other.pass_side_green = 1;
    other.hsv[VISION_CLASS_RED].sat_min = 70;
    uint8_t wire[VISION_CONFIG_WIRE_MAX];
    const size_t n = vision_config_encode(&other, wire, sizeof(wire));
    assert(n > 0);
    vision_cmd_event_t ev = VISION_CMD_NONE;
    for (size_t i = 0; i < n; ++i) {
        const vision_cmd_event_t e = vision_cmd_feed(&p, wire[i], &cfg);
        if (i + 1 < n) assert(e == VISION_CMD_NONE);
        ev = e;
    }
    assert(ev == VISION_CMD_CHANGED && cfg.pass_side_green == 1);
    assert(cfg.hsv[VISION_CLASS_RED].sat_min == 70);

    /* A corrupted frame is rejected and the parser recovers for the next line. */
    wire[20] ^= 0x01;
    for (size_t i = 0; i < n; ++i) ev = vision_cmd_feed(&p, wire[i], &cfg);
    assert(ev == VISION_CMD_ERROR);
    assert(feed_text(&p, &cfg, "set pass_side_green -1\n") == VISION_CMD_CHANGED);
    assert(cfg.pass_side_green == -1);

    /* Overlong lines cannot overrun the buffer. */
    for (int i = 0; i < 400; ++i) ev = vision_cmd_feed(&p, 'x', &cfg);
    assert(ev == VISION_CMD_ERROR || ev == VISION_CMD_NONE);
    assert(feed_text(&p, &cfg, "\nsave\n") == VISION_CMD_SAVE);
    puts("PASS: live configuration commands (text + binary frames, recovery, bounds)");
    return 0;
}

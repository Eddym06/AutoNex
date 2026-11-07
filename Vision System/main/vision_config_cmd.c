#include "vision_config_cmd.h"

#include <stdio.h>
#include <string.h>

void vision_cmd_reset(vision_cmd_parser_t *p)
{
    if (p) memset(p, 0, sizeof(*p));
}

static vision_cmd_event_t reply(vision_cmd_parser_t *p, vision_cmd_event_t ev, const char *text)
{
    snprintf(p->reply, sizeof(p->reply), "%s", text);
    return ev;
}

static vision_cmd_event_t run_line(vision_cmd_parser_t *p, vision_config_t *cfg)
{
    char *line = (char *)p->buf;
    line[p->used] = '\0';
    while (p->used && (line[p->used - 1] == '\r' || line[p->used - 1] == ' ')) line[--p->used] = '\0';
    char *start = line;
    while (*start == ' ' || *start == '\t') ++start;
    if (!*start) return VISION_CMD_NONE;
    if (!strcmp(start, "save")) return VISION_CMD_SAVE;
    if (!strcmp(start, "defaults")) return VISION_CMD_DEFAULTS;
    if (!strcmp(start, "dump")) return VISION_CMD_DUMP;
    if (!strcmp(start, "help"))
        return reply(p, VISION_CMD_REPLY, "set <key> <value> | save | defaults | dump");
    if (!strncmp(start, "set ", 4)) {
        char error[96];
        vision_config_t work = *cfg;
        if (!vision_config_parse_text(&work, start + 4, error, sizeof(error)))
            return reply(p, VISION_CMD_ERROR, error);
        *cfg = work;
        return VISION_CMD_CHANGED;
    }
    return reply(p, VISION_CMD_ERROR, "unknown command (try: help)");
}

vision_cmd_event_t vision_cmd_feed(vision_cmd_parser_t *p, uint8_t byte, vision_config_t *cfg)
{
    if (!p || !cfg) return VISION_CMD_NONE;
    if (p->used == 0) {
        if (byte == '\n' || byte == '\r') return VISION_CMD_NONE;
        p->binary = byte == 0xA6u;
    }
    if (p->used >= sizeof(p->buf) - 1u) {
        vision_cmd_reset(p);
        return reply(p, VISION_CMD_ERROR, "line too long");
    }
    if (p->binary) {
        p->buf[p->used++] = byte;
        if (p->used == 2u && byte != 0x6Au) { vision_cmd_reset(p); return reply(p, VISION_CMD_ERROR, "bad frame"); }
        if (p->used < 4u) return VISION_CMD_NONE;
        const size_t total = (size_t)p->buf[3] + 6u;
        if (total > VISION_CONFIG_WIRE_MAX) { vision_cmd_reset(p); return reply(p, VISION_CMD_ERROR, "frame too long"); }
        if (p->used < total) return VISION_CMD_NONE;
        const bool ok = vision_config_decode(p->buf, total, cfg);
        vision_cmd_reset(p);
        return ok ? reply(p, VISION_CMD_CHANGED, "configuration frame accepted")
                  : reply(p, VISION_CMD_ERROR, "configuration frame rejected (CRC or version)");
    }
    if (byte == '\n') {
        const vision_cmd_event_t ev = run_line(p, cfg);
        p->used = 0;
        return ev;
    }
    p->buf[p->used++] = byte;
    return VISION_CMD_NONE;
}

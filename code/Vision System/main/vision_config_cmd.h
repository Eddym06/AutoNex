#ifndef WRO_VISION_CONFIG_CMD_H
#define WRO_VISION_CONFIG_CMD_H

#include "vision.h"

/* Byte-stream parser for live tuning. It accepts two things on the same wire:
 *   - a binary configuration frame (vision_config_encode), as a host tool or the
 *     P4 would send; and
 *   - text lines typed in a serial monitor:
 *       set <key> <value>   change one field, e.g. "set pass_side_red -1"
 *       save                write the active configuration to flash
 *       defaults            drop the saved copy, back to the built-in values
 *       dump                print the active configuration
 *       help
 * It does no I/O: the caller owns the configuration and acts on the event. */

typedef enum {
    VISION_CMD_NONE = 0,
    VISION_CMD_CHANGED,    /* cfg was modified: apply it */
    VISION_CMD_SAVE,
    VISION_CMD_DEFAULTS,
    VISION_CMD_DUMP,
    VISION_CMD_REPLY,      /* reply holds a message for the person */
    VISION_CMD_ERROR       /* reply says what was wrong; cfg is unchanged */
} vision_cmd_event_t;

typedef struct {
    uint8_t buf[VISION_CONFIG_WIRE_MAX + 8];
    size_t used;
    bool binary;
    char reply[128];
} vision_cmd_parser_t;

void vision_cmd_reset(vision_cmd_parser_t *parser);
vision_cmd_event_t vision_cmd_feed(vision_cmd_parser_t *parser, uint8_t byte,
                                   vision_config_t *cfg);

#endif

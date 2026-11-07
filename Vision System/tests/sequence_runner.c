#include "vision.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint8_t frame[VISION_WIDTH * VISION_HEIGHT * 2];
static vision_workspace_t workspace;
int main(void)
{
    char line[4096];
    while (fgets(line, sizeof(line), stdin)) {
        if (strlen(line) < 3) return 2;
        int parking = line[0] == '1';
        char *path = line + 2;
        path[strcspn(path, "\r\n")] = '\0';
        FILE *f = fopen(path, "rb");
        if (!f) { fprintf(stderr, "Cannot open frame: %s\n", path); return 2; }
        size_t length = fread(frame, 1, sizeof(frame), f);
        fclose(f);
        vision_result_t r;
        const int width = length == (size_t)160 * 120 * 2 ? 160 : 320;
        const int height = width * 3 / 4;
        vision_process(frame, length, width, height, parking != 0, &workspace, &r);
        printf("%d,%s", r.valid, vision_action_name(r.action));
        const vision_box_t *boxes[] = {&r.red, &r.green, &r.parking};
        for (int i = 0; i < 3; ++i) {
            const vision_box_t *b = boxes[i];
            printf(",%d,%d,%d,%d,%d,%d,%.6f", b->detected, b->min_x, b->max_x,
                   b->min_y, b->max_y, b->count, (double)b->distance_cm);
        }
        puts(""); fflush(stdout);
    }
    return ferror(stdin) ? 2 : 0;
}

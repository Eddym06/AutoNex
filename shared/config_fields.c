#include "config_fields.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool cf_set(void *cfg, const cf_field_t *fields, size_t count, const char *key, const char *value)
{
    if (!cfg || !fields || !key || !value) return false;
    for (size_t i = 0; i < count; ++i) {
        if (strcmp(fields[i].name, key) != 0) continue;
        void *p = (uint8_t *)cfg + fields[i].offset;
        char *end = NULL;
        if (fields[i].type == CF_F32) {
            const float f = strtof(value, &end);
            if (end == value || !isfinite(f)) return false;
            memcpy(p, &f, sizeof(f));
            return true;
        }
        long long n;
        if (fields[i].type == CF_BOOL && (!strcmp(value, "true") || !strcmp(value, "on"))) n = 1;
        else if (fields[i].type == CF_BOOL && (!strcmp(value, "false") || !strcmp(value, "off"))) n = 0;
        else {
            n = strtoll(value, &end, 10);
            if (end == value) return false;
        }
        switch (fields[i].type) {
        case CF_BOOL: *(bool *)p = n != 0; break;
        case CF_U8: *(uint8_t *)p = (uint8_t)(n < 0 ? 0 : n > 255 ? 255 : n); break;
        case CF_I8: *(int8_t *)p = (int8_t)(n < -128 ? -128 : n > 127 ? 127 : n); break;
        case CF_U16: *(uint16_t *)p = (uint16_t)(n < 0 ? 0 : n > 65535 ? 65535 : n); break;
        case CF_I16: *(int16_t *)p = (int16_t)(n < -32768 ? -32768 : n > 32767 ? 32767 : n); break;
        case CF_INT: *(int *)p = (int)(n < -1000000 ? -1000000 : n > 1000000 ? 1000000 : n); break;
        case CF_U32: *(uint32_t *)p = (uint32_t)(n < 0 ? 0 : n > 4294967295LL ? 4294967295LL : n); break;
        default: return false;
        }
        return true;
    }
    return false;
}

size_t cf_format(const void *cfg, const cf_field_t *fields, size_t count, char *out, size_t capacity)
{
    if (!cfg || !fields || !out || capacity == 0) return 0;
    size_t used = 0;
    out[0] = '\0';
    for (size_t i = 0; i < count; ++i) {
        const void *p = (const uint8_t *)cfg + fields[i].offset;
        char line[112];
        int n;
        switch (fields[i].type) {
        case CF_BOOL: n = snprintf(line, sizeof(line), "%s %s\n", fields[i].name, *(const bool *)p ? "true" : "false"); break;
        case CF_U8: n = snprintf(line, sizeof(line), "%s %u\n", fields[i].name, *(const uint8_t *)p); break;
        case CF_I8: n = snprintf(line, sizeof(line), "%s %d\n", fields[i].name, *(const int8_t *)p); break;
        case CF_U16: n = snprintf(line, sizeof(line), "%s %u\n", fields[i].name, *(const uint16_t *)p); break;
        case CF_I16: n = snprintf(line, sizeof(line), "%s %d\n", fields[i].name, *(const int16_t *)p); break;
        case CF_INT: n = snprintf(line, sizeof(line), "%s %d\n", fields[i].name, *(const int *)p); break;
        case CF_U32: n = snprintf(line, sizeof(line), "%s %lu\n", fields[i].name, (unsigned long)*(const uint32_t *)p); break;
        default: n = snprintf(line, sizeof(line), "%s %.6g\n", fields[i].name, (double)*(const float *)p); break;
        }
        if (n < 0 || used + (size_t)n + 1u > capacity) break;
        memcpy(out + used, line, (size_t)n + 1u);
        used += (size_t)n;
    }
    return used;
}

bool cf_parse_text(void *cfg, void *work, size_t cfg_size, const cf_field_t *fields, size_t count,
                   const char *text, char *error, size_t error_size)
{
    if (!cfg || !work || !text) return false;
    memcpy(work, cfg, cfg_size);
    char line[128];
    int number = 0;
    while (*text) {
        size_t n = 0;
        while (text[n] && text[n] != '\n' && n + 1 < sizeof(line)) { line[n] = text[n]; ++n; }
        line[n] = '\0';
        text += n;
        while (*text && *text != '\n') ++text;
        if (*text == '\n') ++text;
        ++number;
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        char *key = line;
        while (*key == ' ' || *key == '\t' || *key == '\r') ++key;
        if (!*key) continue;
        char *value = key;
        while (*value && *value != ' ' && *value != '\t' && *value != '=') ++value;
        if (*value) { *value++ = '\0'; }
        while (*value == ' ' || *value == '\t' || *value == '=') ++value;
        char *tail = value + strlen(value);
        while (tail > value && (tail[-1] == ' ' || tail[-1] == '\t' || tail[-1] == '\r')) *--tail = '\0';
        if (!cf_set(work, fields, count, key, value)) {
            if (error && error_size) snprintf(error, error_size, "line %d: cannot set '%s' to '%s'", number, key, value);
            return false;
        }
    }
    memcpy(cfg, work, cfg_size);
    return true;
}

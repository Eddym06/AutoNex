#include "hw_mapstore.h"

#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "mapstore"
#define SLOT_SIZE 0x10000u                 /* 64 KB por ranura (16 sectores de 4 KB) */
#define SECTOR 0x1000u
#define HDR_SIZE 16u
#define HDR_MAGIC 0x3153544Du              /* "MTS1" */

typedef struct { uint32_t magic, seq, len, seq_inv; } slot_header_t;

static const esp_partition_t *s_part;
static uint32_t s_seq;                     /* ultima copia valida */
static int s_slot = -1;                    /* ranura de esa copia */
static EXT_RAM_BSS_ATTR uint8_t s_buf[SLOT_SIZE];   /* copia en la PSRAM: no ocupa la RAM interna */
static const trace_map_t *s_map;
static bool (*s_finished)(void);

static bool read_header(int slot, slot_header_t *h)
{
    if (esp_partition_read(s_part, (size_t)slot * SLOT_SIZE, h, sizeof *h) != ESP_OK) return false;
    return h->magic == HDR_MAGIC && h->seq_inv == ~h->seq && h->len > 0 && h->len <= SLOT_SIZE - HDR_SIZE;
}

bool hw_mapstore_save(const trace_map_t *m)
{
    if (!s_part || !m) return false;
    const size_t len = trace_map_save(m, s_buf, sizeof s_buf);
    if (len == 0) { ESP_LOGE(TAG, "el mapa no cabe en la ranura de %u bytes", (unsigned)(SLOT_SIZE - HDR_SIZE)); return false; }
    const int slot = s_slot == 0 ? 1 : 0;                        /* la otra ranura: la anterior queda intacta */
    const size_t base = (size_t)slot * SLOT_SIZE;
    const size_t erase = ((HDR_SIZE + len) + SECTOR - 1u) / SECTOR * SECTOR;
    if (esp_partition_erase_range(s_part, base, erase) != ESP_OK) return false;
    if (esp_partition_write(s_part, base + HDR_SIZE, s_buf, len) != ESP_OK) return false;
    const slot_header_t h = {.magic = HDR_MAGIC, .seq = s_seq + 1u, .len = (uint32_t)len, .seq_inv = ~(s_seq + 1u)};
    if (esp_partition_write(s_part, base, &h, sizeof h) != ESP_OK) return false;   /* la cabecera se escribe la ultima: confirma la copia */
    s_seq = h.seq;
    s_slot = slot;
    ESP_LOGI(TAG, "mapa guardado: copia %u, %u bytes, %u vueltas, %u objetos", (unsigned)s_seq, (unsigned)len,
             (unsigned)m->laps_mapped, (unsigned)m->object_count);
    return true;
}

bool hw_mapstore_load(trace_map_t *m)
{
    if (!s_part || !m || s_slot < 0) return false;
    slot_header_t h;
    if (!read_header(s_slot, &h)) return false;
    if (esp_partition_read(s_part, (size_t)s_slot * SLOT_SIZE + HDR_SIZE, s_buf, h.len) != ESP_OK) return false;
    return trace_map_load(m, s_buf, h.len) != 0;
}

uint32_t hw_mapstore_sequence(void) { return s_seq; }

static void store_task(void *arg)
{
    (void)arg;
    bool saved = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        const bool fin = s_finished && s_finished();
        if (!fin) { saved = false; continue; }                   /* nueva mision: habra otra copia al terminar */
        if (!saved) saved = hw_mapstore_save(s_map) || (s_part == NULL);
    }
}

void hw_mapstore_init(const trace_map_t *map, bool (*finished)(void))
{
    s_map = map;
    s_finished = finished;
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "tracemap");
    if (!s_part) { ESP_LOGW(TAG, "sin particion 'tracemap': el mapa no se guardara (usa partitions.csv)"); return; }
    for (int slot = 0; slot < 2; ++slot) {
        slot_header_t h;
        if (read_header(slot, &h) && h.seq > s_seq) { s_seq = h.seq; s_slot = slot; }
    }
    if (s_slot >= 0) ESP_LOGI(TAG, "ultima copia guardada: %u (ranura %d)", (unsigned)s_seq, s_slot);
    else ESP_LOGI(TAG, "aun no hay mapa guardado");
    xTaskCreate(store_task, "mapstore", 4096, NULL, 1, NULL);
}

#ifndef WRO_CONFIG_FIELDS_H
#define WRO_CONFIG_FIELDS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Acceso por nombre a los campos de una estructura de configuracion.
 *
 * Una tabla {nombre, tipo, desplazamiento} por estructura basta para editar la
 * configuracion como texto ("clave valor", un campo por linea), mostrarla y
 * cargarla desde un archivo o una consola serie. Anadir un parametro nuevo es
 * una linea en la tabla, y la herramienta de ajuste lo ve sin cambiar nada. */
typedef enum { CF_BOOL, CF_U8, CF_I8, CF_U16, CF_I16, CF_INT, CF_U32, CF_F32 } cf_type_t;

typedef struct {
    const char *name;
    cf_type_t type;
    size_t offset;
} cf_field_t;

#define CF_FIELD(name_, type_, struct_, member_) {name_, type_, offsetof(struct_, member_)}

/* false si la clave no existe o el valor no es del tipo. Los enteros se
 * saturan al rango de su tipo; sanitize() de cada modulo hace el resto. */
bool cf_set(void *cfg, const cf_field_t *fields, size_t count, const char *key, const char *value);
size_t cf_format(const void *cfg, const cf_field_t *fields, size_t count, char *out, size_t capacity);
/* Aplica un texto entero o nada: `work` es un espacio de cfg_size bytes. */
bool cf_parse_text(void *cfg, void *work, size_t cfg_size, const cf_field_t *fields, size_t count,
                   const char *text, char *error, size_t error_size);

#endif

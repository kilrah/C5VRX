#include "common.h"
typedef void *async_memcpy_handle_t;
typedef struct { void *data; } async_memcpy_event_t;
typedef bool (*async_memcpy_isr_cb_t)(async_memcpy_handle_t, async_memcpy_event_t *, void *);
typedef struct { uint32_t backlog; } async_memcpy_config_t;
static inline esp_err_t esp_async_memcpy_install_gdma_axi(const async_memcpy_config_t *c, async_memcpy_handle_t *h) { (void)c; *h = NULL; return 1; }
static inline esp_err_t esp_async_memcpy(async_memcpy_handle_t h, void *d, void *s, size_t n, async_memcpy_isr_cb_t cb, void *a) { (void)h; (void)d; (void)s; (void)n; (void)cb; (void)a; return 1; }

#include "../common.h"
/* BitScrambler stand-ins: the host test never selects raw IQ. */
typedef void *bitscrambler_handle_t;
typedef enum { BITSCRAMBLER_DIR_TX, BITSCRAMBLER_DIR_RX } bitscrambler_direction_t;
typedef struct { bitscrambler_direction_t dir; int attach_to; } bitscrambler_config_t;
#define SOC_BITSCRAMBLER_ATTACH_PARL_IO 0
#define BITSCRAMBLER_PROGRAM(VAR, NAME) static const uint8_t VAR[1]
static inline esp_err_t bitscrambler_new(const bitscrambler_config_t *c, bitscrambler_handle_t *h){(void)c;*h=NULL;return ESP_ERR_NOT_FOUND;}
static inline esp_err_t bitscrambler_enable(bitscrambler_handle_t h){(void)h;return ESP_OK;}
static inline esp_err_t bitscrambler_disable(bitscrambler_handle_t h){(void)h;return ESP_OK;}
static inline void bitscrambler_free(bitscrambler_handle_t h){(void)h;}
static inline esp_err_t bitscrambler_load_program(bitscrambler_handle_t h, const void *p){(void)h;(void)p;return ESP_OK;}
static inline esp_err_t bitscrambler_reset(bitscrambler_handle_t h){(void)h;return ESP_OK;}
static inline esp_err_t bitscrambler_start(bitscrambler_handle_t h){(void)h;return ESP_OK;}

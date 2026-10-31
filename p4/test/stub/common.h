/* Minimal ESP-IDF stand-ins for the host test of cvbs.c (test_cvbs.c). */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_NOT_FOUND 0x105
#define IRAM_ATTR
#define ESP_LOGW(...) do{}while(0)
#define ESP_LOGI(...) do{}while(0)
#define ESP_LOGE(...) do{}while(0)
static inline const char *esp_err_to_name(esp_err_t e){(void)e;return "err";}
typedef void *TaskHandle_t; typedef int BaseType_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#define configMAX_PRIORITIES 25
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
static inline void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *w){(void)t;(void)w;}
static inline uint32_t ulTaskNotifyTake(int a,int b){(void)a;(void)b;return 0;}
static inline void vTaskDelay(int t){(void)t;}
static inline void xTaskNotifyGive(TaskHandle_t t){(void)t;}
static inline int xTaskCreatePinnedToCore(void (*f)(void*),const char*n,int s,void*a,int p,TaskHandle_t*h,int c){(void)f;(void)n;(void)s;(void)a;(void)p;(void)h;(void)c;return 1;}
extern int64_t g_fake_time;
static inline int64_t esp_timer_get_time(void){return g_fake_time;}
static inline esp_err_t esp_cache_msync(void*a,size_t s,int f){(void)a;(void)s;(void)f;return 0;}
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C 0
#define ESP_CACHE_MSYNC_FLAG_DIR_C2M 1
static inline esp_err_t esp_cache_get_alignment(uint32_t c,size_t*o){(void)c;*o=64;return 0;}
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_DMA 2
#define MALLOC_CAP_SPIRAM 4
static inline void *heap_caps_aligned_calloc(size_t a,size_t n,size_t s,uint32_t c){(void)a;(void)c;return calloc(n,s);}
static inline void esp_rom_install_channel_putc(int c, void (*f)(char)){(void)c;(void)f;}
static inline void esp_rom_install_uart_printf(void){}
typedef struct { struct { uint32_t length; } dw0; void *buffer; void *next; } dma_descriptor_align8_t;
typedef struct { struct { struct { struct { uint32_t peri_in_sel_chn; } in_peri_sel; struct { uint32_t val; } in_dscr_bf0; } conf; } in[3]; } axi_dma_dev_t;
extern axi_dma_dev_t AXI_DMA;
#define SOC_GDMA_TRIG_PERIPH_PARLIO0 3
typedef void *parlio_rx_unit_handle_t; typedef void *parlio_rx_delimiter_handle_t;
typedef struct { size_t recv_bytes; } parlio_rx_event_data_t;
typedef enum { PARLIO_SAMPLE_EDGE_POS, PARLIO_SAMPLE_EDGE_NEG } parlio_sample_edge_t;
#define PARLIO_BIT_PACK_ORDER_LSB 0
#define PARLIO_CLK_SRC_EXTERNAL 0
typedef struct { int sample_edge, bit_pack_order; uint32_t eof_data_len, timeout_ticks; } parlio_rx_soft_delimiter_config_t;
typedef struct { void *delimiter; struct { uint32_t partial_rx_en:1; } flags; } parlio_receive_config_t;
typedef struct { size_t trans_queue_depth, max_recv_size, data_width; int clk_src; uint32_t ext_clk_freq_hz, exp_clk_freq_hz; int clk_in_gpio_num, clk_out_gpio_num, valid_gpio_num; int data_gpio_nums[16]; struct { uint32_t free_clk:1; } flags; } parlio_rx_unit_config_t;
typedef struct { bool (*on_partial_receive)(parlio_rx_unit_handle_t, const parlio_rx_event_data_t*, void*); } parlio_rx_event_callbacks_t;
static inline esp_err_t parlio_new_rx_soft_delimiter(const void*c,void*d){(void)c;(void)d;return 0;}
static inline esp_err_t parlio_rx_unit_enable(void*u,bool b){(void)u;(void)b;return 0;}
static inline esp_err_t parlio_rx_unit_disable(void*u){(void)u;return 0;}
static inline esp_err_t parlio_rx_unit_receive(void*u,void*b,size_t s,const void*c){(void)u;(void)b;(void)s;(void)c;return 0;}
static inline esp_err_t parlio_rx_soft_delimiter_start_stop(void*u,void*d,bool b){(void)u;(void)d;(void)b;return 0;}
static inline esp_err_t parlio_del_rx_delimiter(void*d){(void)d;return 0;}
static inline esp_err_t parlio_new_rx_unit(const void*c,void*u){(void)c;(void)u;return 0;}
static inline esp_err_t parlio_rx_unit_register_event_callbacks(void*u,const void*c,void*a){(void)u;(void)c;(void)a;return 0;}

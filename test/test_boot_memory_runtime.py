#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: fault-inject actual boot memory functions.
Host checks cover allocation/startup contracts, not RF or board acceptance.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def run(source, raster=False):
    with tempfile.TemporaryDirectory() as tmp:
        c = Path(tmp) / 'boot.c'
        c.write_text(source)
        command = [os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                   '-Werror', '-I' + str(ROOT / 'main'), str(c)]
        if raster:
            command += [str(ROOT / 'main/menu_raster.c'), '-lm']
        subprocess.run(command + ['-o', str(Path(tmp) / 'boot')], check=True)
        subprocess.run([str(Path(tmp) / 'boot')], check=True)


class BootMemoryTest(unittest.TestCase):
    def test_receiver_reservation_and_failures(self):
        source = (ROOT / 'main/receiver.c').read_text()
        storage = source[source.index('#define OBSERVER_STACK_BYTES'):source.index('static volatile uint8_t s_gain_mode')]
        functions = source[source.index('bool receiver_prepare(void)'):source.index('void receiver_print(void)')]
        run(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define portBYTE_ALIGNMENT 16
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define ESP_TIMER_TASK 0
#define ESP_OK 0
#define DG_MAP_VERSION 1
/* A byte-addressed IDF stack, with a nontrivial TCB and alignment. */
typedef uint8_t StackType_t;
typedef struct { uint8_t opaque[320]; } StaticTask_t;
typedef void *TaskHandle_t;
typedef void *esp_timer_handle_t;
typedef struct { void (*callback)(void *); int dispatch_method;
 const char *name; bool skip_unhandled_events; } esp_timer_create_args_t;
static unsigned allocations, creates, starts, deletes, task_calls, fail_task;
static bool fail_alloc, fail_timer, fail_start;
static void *heap_caps_aligned_alloc(unsigned align, size_t size, unsigned caps) {
 assert(align==portBYTE_ALIGNMENT && caps==(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
 ++allocations;return fail_alloc?NULL:malloc(size);
}
static void heap_caps_free(void *p) {free(p);}
static void tick_cb(void *p) {(void)p;}
static int esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *h) {
 assert(a->callback==tick_cb && a->skip_unhandled_events);++creates;
 if(fail_timer)return -1;
 *h=(void *)1;return ESP_OK;
}
static int esp_timer_start_periodic(esp_timer_handle_t h, unsigned us) {
 assert(h && us==200);++starts;return fail_start?-1:ESP_OK;
}
static void observer_task(void *p) {(void)p;}
static void sentinel_task(void *p) {(void)p;}
static TaskHandle_t xTaskCreateStatic(void (*fn)(void *),const char *name,unsigned bytes,
 void *arg,unsigned priority,StackType_t *stack,StaticTask_t *task) {
 ++task_calls;assert(!arg && stack && task);
 assert(((uintptr_t)stack % portBYTE_ALIGNMENT)==0);
 if(!strcmp(name,"gain_obs"))assert(fn==observer_task && bytes==4096 && priority==3);
 else assert(!strcmp(name,"gain_fast") && fn==sentinel_task && bytes==3072 && priority==4);
 return task_calls==fail_task?NULL:task;
}
static void vTaskDelete(TaskHandle_t h) {assert(h);++deletes;}
static struct {int version;bool context_valid;} s_gain_map_saved;
static bool s_gain_map_saved_valid;
static bool blob_load(const char *key,void *p,size_t size) {
 assert(!strcmp(key,"gain_map") && p==&s_gain_map_saved && size==sizeof(s_gain_map_saved));
 s_gain_map_saved.version=DG_MAP_VERSION;s_gain_map_saved.context_valid=true;return true;
}
''' + storage + functions + r'''
int main(void) {
 fail_alloc=true;assert(!receiver_prepare());assert(!s_receiver_memory && !s_gain_timer);
 fail_alloc=false;fail_timer=true;assert(!receiver_prepare());assert(!s_receiver_memory && !s_gain_timer);
 fail_timer=false;assert(receiver_prepare());assert(!s_observer && !s_sentinel && !starts && !task_calls);
 unsigned reserved=allocations,timers=creates;
 assert(receiver_prepare());assert(allocations==reserved && creates==timers);
 /* No heap remains after splash: startup must use only the reservation. */
 fail_alloc=true;assert(receiver_start());assert(s_gain_map_saved_valid);
 assert(allocations==reserved && creates==timers && starts==1 && task_calls==2);
 assert(receiver_start());assert(starts==1 && task_calls==2);
 vTaskDelete(s_sentinel);vTaskDelete(s_observer);s_observer=s_sentinel=NULL;
 unsigned removed=deletes;
 fail_task=task_calls+1;assert(!receiver_start());assert(!s_observer && !s_sentinel && deletes==removed);
 fail_task=task_calls+2;assert(!receiver_start());assert(!s_observer && !s_sentinel && deletes==removed+1);
 fail_task=0;fail_start=true;assert(!receiver_start());
 assert(!s_observer && !s_sentinel && deletes==removed+3);
 fail_start=false;assert(receiver_start());assert(allocations==reserved && creates==timers);
 vTaskDelete(s_sentinel);vTaskDelete(s_observer);heap_caps_free(s_receiver_memory);
}
''')

    def test_progressive_fallback_releases_excess(self):
        source = (ROOT / 'main/menu.c').read_text()
        functions = source[source.index('static void free_nodes(void)'):source.index('static esp_err_t build_chain(')]
        run(r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "menu_raster.h"
#define NODE_CHUNK 32u
#define ESP_OK 0
#define ESP_ERR_NO_MEM 1
#define ESP_ERR_INVALID_SIZE 2
#define MALLOC_CAP_DMA_DESC_AHB 1
#define MALLOC_CAP_INTERNAL 2
typedef int esp_err_t;
typedef struct {uint32_t words[3];} dma_descriptor_t;
static dma_descriptor_t *s_chunks[58];
static unsigned s_chunk_count,s_node_capacity,s_node_count;
static bool s_raster_flat;
static menu_raster_t s_raster;
static unsigned allocated,limit;
static void *heap_caps_aligned_alloc(unsigned align,size_t size,unsigned caps) {
 assert(align==4 && size==384 && caps==3);
 if(allocated==limit)return NULL;
 ++allocated;return malloc(size);
}
static void heap_caps_free(void *p){assert(p && allocated);--allocated;free(p);}
static bool count_segment(void *ctx,const uint8_t *data,unsigned bytes) {
 assert(data && bytes);++*(unsigned *)ctx;return true;
}
''' + functions + r'''
int main(void) {
 unsigned required;
 for(unsigned pal=0;pal<2;++pal) {
  /* Monochrome raster (shared tail+prefix nodes): interlaced NTSC 994 /
   * PAL 1203 nodes (32 / 38 chunks), progressive 496 / 601 (16 / 19). */
  limit=30; /* Interlaced fails after partial allocation, progressive fits. */
  assert(reserve_chain(pal,true,&required)==ESP_ERR_NO_MEM && allocated==30);
  assert(reserve_chain(pal,false,&required)==ESP_OK);
  assert(required==(pal?601u:496u));
  assert(allocated==(pal?19u:16u) && s_chunk_count==allocated);
  assert(s_node_capacity==allocated*NODE_CHUNK);
  for(unsigned i=s_chunk_count;i<58;++i)assert(!s_chunks[i]);
  free_nodes();assert(!allocated && !s_node_capacity && !s_node_count);
  limit=10; /* Neither chain fits: caller can release every partial chunk. */
  assert(reserve_chain(pal,true,&required)==ESP_ERR_NO_MEM);
  assert(reserve_chain(pal,false,&required)==ESP_ERR_NO_MEM);
  free_nodes();assert(!allocated);
 }
}
''', raster=True)

    def test_boot_preserves_task_start_order(self):
        source = (ROOT / 'main/main.c').read_text()
        self.assertLess(source.index('receiver_prepare()'), source.index('rf_start()'))
        self.assertLess(source.index('transport_start()'), source.index('menu_splash_begin()'))
        self.assertLess(source.index('menu_splash_begin()'), source.index('receiver_start()'))
        self.assertLess(source.index('receiver_start()'), source.index('control_start()'))
        failure = source[source.index('if (!receiver_start())'):source.index('    control_start();')]
        for statement in ['menu_splash_end();', 'transport_stop_tx();', 'return;']:
            self.assertIn(statement, failure)
        control = (ROOT / 'main/control.c').read_text()
        self.assertIn('xQueueCreateStatic(16, sizeof(int)', control)
        self.assertNotIn('xQueueCreate(16, sizeof(int)', control)


if __name__ == '__main__':
    unittest.main()

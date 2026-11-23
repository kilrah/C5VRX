#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: execute the production calibration searches
with deterministic PHY/probe boundaries. Host evidence, not RF timing proof.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def run_c(source):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        (path / 'test.c').write_text(source)
        (path / 'esp_err.h').write_text('typedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_NOT_SUPPORTED 1\n#define ESP_ERR_INVALID_ARG 2\n#define ESP_ERR_INVALID_RESPONSE 3\n')
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I', str(path), '-I', str(ROOT / 'main'), str(path / 'test.c'), '-lm', '-o', str(path / 'run')], check=True)
        subprocess.run([str(path / 'run')], check=True)


class CalibrationSearchTest(unittest.TestCase):
    def test_actual_dc_searches(self):
        phy = (ROOT / 'main/phy.c').read_text()
        helpers = phy[phy.index('#define DCO_PROBE'):phy.index('/* phy_pbus_workmode()')]
        # The hold-only globals are outside the search under test.
        helpers = helpers.replace('static int s_dco_codes[2];', '').replace('static bool s_dco_held;', '')
        body = phy[phy.index('esp_err_t phy_dco_search(bool'):phy.index('esp_err_t phy_dco_hold(')]
        run_c(r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dsp.h"
#include "phy.h"
#define C5VRX_PHY_PINNED
static int codes[2]={220,200}, target[2]={225,186};
static int probes, checks, writes, fail_probe, fail_check, offset_check;
static void phy_pbus_force_test(unsigned b, unsigned bank, unsigned v) {
    ++writes; if (bank==2 && (b==2 || b==3)) codes[b-2]=(int)v;
}
void phy_begin(void) {}
void phy_end(void) {}
''' + helpers + r'''
static void read_live(uint16_t live[PBUS_BLOCKS][2]) {
    memset(live,0,sizeof(uint16_t)*PBUS_BLOCKS*2);
    live[2][0]=live[2][1]=codes[0]; live[3][0]=live[3][1]=codes[1];
}
static void enter_debug(uint16_t live[PBUS_BLOCKS][2]) {(void)live;}
static void pbus_workmode_keep_gain(void) {}
''' + body + r'''
static bool probe(int dc[2]) {
    ++probes;
    dc[0]=(codes[0]-target[0])*50+(codes[1]-target[1])*5;
    dc[1]=(codes[1]-target[1])*50;
    return !fail_probe;
}
static bool verify(int dc[2]) {
    ++checks; dc[0]=(codes[0]-target[0])*50+offset_check;
    dc[1]=(codes[1]-target[1])*50;
    return !fail_check;
}
static bool flat(int dc[2]) {
    ++probes; dc[0]=codes[0]<=target[0] ? -500 : 500;
    dc[1]=codes[1]<=target[1] ? -500 : 500;
    return !fail_probe;
}
static void reset(void) {
    codes[0]=220;codes[1]=200;probes=checks=writes=0;
    fail_probe=fail_check=offset_check=0;
}
static void restored(void) {assert(codes[0]==220 && codes[1]==200);}
int main(void) {
    phy_dco_result_t r;
    reset(); int seed[2]={225,186};
    assert(phy_dco_search_verified(probe,verify,seed,&r)==ESP_OK);
    assert(r.measured && probes==1 && checks==1);restored();
    reset();assert(phy_dco_search_verified(probe,verify,NULL,&r)==ESP_OK);
    assert(r.measured && checks==1 && abs(r.residual_mcells[0])<=300);restored();
    reset();fail_check=1;phy_dco_search_verified(probe,verify,seed,&r);
    assert(!r.measured);restored();
    reset();offset_check=1000;phy_dco_search_verified(probe,verify,seed,&r);
    assert(!r.measured);restored();
    reset();fail_probe=1;phy_dco_search_verified(probe,verify,seed,&r);
    assert(!r.measured);restored();
    reset();target[0]=225;target[1]=186;
    phy_dco_search_flat_from(flat,seed,&r);
    assert(r.measured && r.codes[0]==225 && r.codes[1]==186 && probes<35);restored();
    reset();target[0]=130;target[1]=290;
    phy_dco_search_flat(flat,&r);
    assert(r.measured && r.codes[0]==130 && r.codes[1]==290);restored();
    reset();target[0]=20;phy_dco_search_flat(flat,&r);
    assert(!r.measured);restored();
    puts("PASS: production DC centred seed, coupled response, failed/noisy verification, flat local/full bracket, rollback");
}
''')

    def test_actual_filter_measurement(self):
        source = ''.join(p.read_text() for p in sorted((ROOT/'main').glob('calibration*.c')))
        body = source[source.index('static bool measure_setting('):source.index('/* A channel is quiet')]
        run_c(r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "calibration_noise.h"
typedef struct {int code;unsigned skirt;bool bw20;} rf_filter_t;
#define CAL_WINDOWS 96u
#define FILTER_MIN_KHZ 12000u
#define FILTER_NARROW_KHZ 10000u
#define FILTER_WIDE_KHZ 16000u
#define pdMS_TO_TICKS(x) (x)
static unsigned s_cal_total, s_cal_done, waits, full, screen;
static unsigned test_width=12000,test_nbw=12000,test_sw=12000,test_sn=12000;
static bool applied=true,copied=true,carrier_input;
static void cal_status(void) {}
static void vTaskDelay(unsigned n) {waits+=n;}
static bool rf_set_filter(rf_filter_t f) {(void)f;return applied;}
static bool noise_collect(cal_noise_t *a, unsigned n) {
    if(!copied)return false;
    a->got=n;a->env_sum=(carrier_input ? 1000u : 100u)*n;
    /* Construct a 64-bin rectangular PSD with known width/NBW. The test
     * below uses helper substitutions only at the FFT summary boundary. */
    if(n==24)++screen;else ++full;
    a->psd[0]=(float)n;return true;
}
#define dsp_psd_width_khz(x) ((x)[0]==24 ? test_sw : test_width)
#define dsp_psd_nbw_khz(x) ((x)[0]==24 ? test_sn : test_nbw)
static bool finish(const cal_noise_t *a,unsigned *w,unsigned *n,unsigned *e,int *c) {
    assert(a->got==96);*w=test_width;*n=test_nbw;*e=a->env_sum/a->got;*c=0;return true;
}
#define cal_noise_finish finish
static unsigned quiet_limit(int clip) {return dsp_quiet_limit(clip);}
''' + body + r'''
int main(void) {
    unsigned w,n;bool q,c,fault=false;
    const unsigned best[3]={12000,10000,16000};rf_filter_t f={0};
    test_sw=test_width=5000;test_sn=test_nbw=5000;
    assert(measure_setting(f,&w,&n,&q,&c,best,&fault));
    assert(!q && !n && screen==1 && full==0);
    test_sw=test_width=12000;test_sn=test_nbw=12000;
    assert(measure_setting(f,&w,&n,&q,&c,best,&fault));
    assert(q && n==12000 && full==1 && !fault);
    /* Near width boundaries must reach full precision. */
    test_sw=9500;test_sn=9000;test_width=10000;test_nbw=10000;
    assert(measure_setting(f,&w,&n,&q,&c,best,&fault) && full==2 && q);
    test_sw=12000;test_sn=12000;test_width=18000;test_nbw=18000;
    assert(measure_setting(f,&w,&n,&q,&c,best,&fault) && fault);
    applied=false;assert(measure_setting(f,&w,&n,&q,&c,best,&fault) && !q && !n);
    applied=true;copied=false;assert(!measure_setting(f,&w,&n,&q,&c,best,&fault));
    copied=true;carrier_input=true;assert(measure_setting(f,&w,&n,&q,&c,best,&fault) && c && !q);
    puts("PASS: production filter rejects are ineligible, contenders/boundaries get 96 windows, discrepancy/failed copies/carrier gates");
}
''')

    def test_actual_sampling_scan(self):
        source = ''.join(p.read_text() for p in sorted((ROOT/'main').glob('calibration*.c')))
        body = source[source.index('sphase_verdict_t sphase_scan_n('):source.index('static sphase_live_t s_sphase_live')]
        run_c(r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "sphase_logic.h"
#define SPHASE_POSITIONS 9u
#define SPHASE_TRIES 12u
#define pdMS_TO_TICKS(x) (x)
typedef struct {unsigned windows;int excess;} window_stats_t;
static sphase_scan_t s_sphase_last;
static int64_t s_probe_deadline;
static int64_t esp_timer_get_time(void) {return 1;}
static unsigned position, pauses, resumes, copies;
static int pattern[3]={900,1500,3000};
static bool partial,unstable;
static bool receiver_pause(const char *tag) {(void)tag;++pauses;return true;}
static void receiver_resume(void) {++resumes;}
static void dco_hold_for_scan(void) {}
static bool phase_slip(void) {++position;return true;}
static bool menu_splash_active(void) {return true;}
static void vTaskDelay(unsigned n) {(void)n;}
static bool collect(unsigned windows,window_stats_t *w) {
    ++copies;w->windows=partial ? windows/2 : windows;
    w->excess=pattern[position%3];
    if(unstable && copies==6)w->excess+=1000;
    return true;
}
static int sphase_metric(const window_stats_t *w) {return w->excess;}
static int sphase_record(const window_stats_t *w) {return w->excess;}
''' + body + r'''
static void reset(void) {position=pauses=resumes=copies=0;partial=unstable=false;}
int main(void) {
    reset();assert(sphase_scan()==SPHASE_SETTLED);
    assert(pauses==1 && resumes==1 && s_sphase_last.positions==3 && position%3==0);
    reset();pattern[1]=pattern[2]=900;
    assert(sphase_scan()==SPHASE_NOISE_LIMITED);
    assert(pauses==2 && resumes==2 && s_sphase_last.positions==9);
    reset();pattern[1]=1500;pattern[2]=3000;partial=true;
    sphase_scan();assert(pauses==2 && resumes==2); /* incomplete fast scan must fall back */
    reset();unstable=true;sphase_scan();assert(pauses==2 && resumes==2);
    puts("PASS: production three-position scan confirms twice, ambiguous/partial/drifting data fall back to nine");
}
''')

    def test_actual_noise_cache(self):
        source = ''.join(p.read_text() for p in sorted((ROOT/'main').glob('calibration*.c')))
        body = source[source.index('void noise_map_startup('):source.index('/* One-sided clipping', source.index('void noise_map_startup('))]
        run_c(r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "cn_auto.h"
#define NOISE_CACHE_LO 54u
#define NOISE_CACHE_ROWS 30u
#define DG_STATES 84
#define PROBE_BYTES 256u
#define pdMS_TO_TICKS(x) (x)
static struct {int64_t time;uint16_t freq,key;float m2,kurt;} s_noise_cache[30];
static unsigned s_current_gain=83, copies, stored, writes;
static int64_t now=10000000;
static bool quiet=true;
static int64_t esp_timer_get_time(void) {return now;}
static void vTaskDelay(unsigned n) {now+=(int64_t)n*1000;}
static bool noise_like(void) {return quiet;}
static bool receiver_pause(const char *tag) {(void)tag;return true;}
static void receiver_resume(void) {}
static void receiver_set_gain(uint8_t g) {(void)g;++writes;}
static struct {int max_index;} table={83};
#define rf_gain_table() (&table)
static unsigned receiver_gain_top(void) {return (unsigned)table.max_index;}   /* bare board: the table maximum */
static unsigned rf_frequency_mhz(void) {return 5865;}
static unsigned live_filter_key(void) {return 42;}
static bool copy_probe_window(uint8_t *s,void *n) {
    (void)n;++copies;for(unsigned k=0;k<256;++k)s[k]=(k&1)?0x11:0;return true;
}
static void receiver_cn_noise_store(uint8_t g,float m2) {(void)g;assert(m2>=0);++stored;}
static void receiver_cn_noise_save(void) {}
static void receiver_gain_table_measured(float *m) {(void)m;}
''' + body + r'''
static void reset(void) {
    now=10000000;copies=stored=writes=0;quiet=true;
    for(unsigned k=0;k<30;++k) {
        s_noise_cache[k].time=now-1;s_noise_cache[k].freq=5865;s_noise_cache[k].key=42;
        s_noise_cache[k].m2=.5f;s_noise_cache[k].kurt=2;
    }
}
int main(void) {
    reset();noise_map_startup(now+10000000);assert(stored==10 && copies==0 && writes==1);
    reset();for(unsigned k=0;k<30;++k)s_noise_cache[k].key=41;
    noise_map_startup(now+10000000);assert(copies && writes>1);
    reset();for(unsigned k=0;k<30;++k)s_noise_cache[k].freq=5805;
    noise_map_startup(now+10000000);assert(copies);
    reset();now+=31000000;noise_map_startup(now+10000000);assert(copies);
    reset();for(unsigned k=0;k<30;++k){s_noise_cache[k].m2=2;s_noise_cache[k].kurt=3;}
    noise_map_startup(now+10000000);assert(copies);
    reset();quiet=false;noise_map_startup(now+10000000);assert(!copies && !stored && !writes);
    puts("PASS: production noise reuse requires fresh exact frequency/filter and quiet gate; invalid cache remeasures");
}
''')

    def test_actual_deviation_wait(self):
        source = ''.join(p.read_text() for p in sorted((ROOT/'main').glob('calibration*.c')))
        body = source[source.index('#define DEVIATION_WAIT_US'):source.index('static void calib_record(bool manual, int64_t t0);')]
        run_c(r'''
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#define FILTER_NARROW 1
#define FILTER_WIDE 2
#define pdMS_TO_TICKS(x) (x)
typedef int rf_filter_t;
static int64_t now,arrival;
static int result=-1,begin,end,commit;
static const char *reason;
static int64_t esp_timer_get_time(void) {return now;}
static void vTaskDelay(unsigned n) {now+=(int64_t)n*1000;if(begin && now>=arrival+1000000)result=170;}
static bool video_recent(void) {return arrival>=0 && now>=arrival;}
static bool filter_setting(int c,rf_filter_t *f) {(void)c;*f=0;return true;}
static void profile_set_reason(const char *s) {reason=s;}
static void profile_measure_begin(void) {++begin;}
static void profile_measure_end(void) {++end;}
static int profile_measure_result(void) {return result;}
static int profile_active(void) {return 0;}
static const char *profile_name(int p) {(void)p;return "DEFAULT";}
static void profile_commit(int d) {assert(d==170);++commit;}
''' + body + r'''
int main(void) {
    now=0;arrival=-1;deviation_startup(100000000);
    assert(now==2000000 && !begin && !commit && reason);
    now=0;arrival=1000000;deviation_startup(100000000);
    assert(begin==1 && end==1 && commit==1 && now==2000000);
    puts("PASS: absent VTX exits after 2 s; arriving VTX still needs the existing measurement result");
}
''')


if __name__ == '__main__':
    unittest.main()

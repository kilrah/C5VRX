/* C5VRX-5 by Twotoz and contributors: receiver DC correction of the
 * calibration task: per-channel DC tables, quiet/carrier searches, health
 * monitor and the live DC servo. */
#include "calibration_internal.h"
#include "experimental.h"
#include "driver/temperature_sensor.h"
#include "dc_trim.h"
/* The servo rows are the RF-stage rows G54..G89 only, and its state lives in
 * LP RAM: HP SRAM has under ~5 KB of boot headroom (board 2026-10-26: 5.2 KB
 * more static HP RAM left the USB console silent from boot). */
#define DC_SERVO_ROW0 54u
#define DC_SERVO_TABLE_ROWS 36u        /* servo slots of the table rows G54..G89 */
#define DC_LOW_STAGES 8u               /* RF stages under the table: G0, 10, 17, 25, 30, 38, 43, 47 */
/* LOW DC serves stages 4-7 (G30-G53): behind an LNA medium carriers sit
 * there; G0-G29 only carry very strong carriers (a 5+ cell circle), where
 * the chip's own DC does no harm, and RAM is tight (heap_min fell to 840 B
 * with all eight stages and their rows). One servo slot per served stage
 * after the table rows (LP RAM: ~64 B a slot). */
#define DC_LOW_STAGE_FIRST 4u
#define DC_LOW_SLOTS (DC_LOW_STAGES - DC_LOW_STAGE_FIRST)
#define DC_SERVO_ROWS (DC_SERVO_TABLE_ROWS + DC_LOW_SLOTS)
#define DC_MODEL_ROWS DC_SERVO_TABLE_ROWS   /* the DC table rows G54..G89, index g - DC_SERVO_ROW0 (HP RAM) */
#include "dc_servo.h"
#include "flightlog_format.h"

extern unsigned char phy_param[];

cal_watch_t s_basemap_watch;
static uint32_t s_basemap_context;
unsigned s_health_blocks, s_health_rejected;
bool s_cal_notice;
static uint16_t s_cal_notice_freq, s_cal_notice_key;
static uint8_t s_cal_notice_gain;
int64_t s_health_us;

/* ---- Per-gain RX DC correction --------------------------------------------
 * Near the range edge the carrier is a few cells; a receiver DC of ~1 cell
 * biases every origin-adjacent phase. The DC DAC pair is searched per gain
 * row in confirmed quiet (no burst-confirmed sync for 3 s, a noise-like
 * envelope for 5 s, never on a channel where a carrier was seen this boot)
 * and held right after every gain write by the post-gain hook (board,
 * 2026-10-07: DC median ~134 LSB without the hook, ~11 with it). */
#define DCO_VERSION      8u   /* 8: 11p RF/filter context */
typedef struct {
    int16_t code[2];
    int16_t residual_mcells[2];
    uint8_t valid;           /* DCO_ROW_* */
} dco_entry_t;
/* A row is OK (codes measured) or FLOOR (measured: receiver noise below
 * one quantizer step there, nothing to correct); 0 = still missing:
 * DCO_ROW_OK / DCO_ROW_FLOOR (calibration_internal.h). */
/* A row whose search left more than this is not converged (clipped start,
 * see phy_dco_search_from): never OK, never a template, searched again from
 * a converged neighbour. The live servo trims what a good row leaves. */
#define DCO_BAD_MCELLS 1200
static bool dco_entry_bad(const dco_entry_t *e)
{
    return e->valid == 1u && (abs(e->residual_mcells[0]) > DCO_BAD_MCELLS || abs(e->residual_mcells[1]) > DCO_BAD_MCELLS);
}
static uint32_t s_dco_bad_dropped, s_dco_repairs, s_dco_repair_fails;
typedef struct {
    uint16_t freq;
    uint8_t version, lo, hi, band5, iq_scale_sel, filter_bw20;
    int8_t filter_code, filter_skirt;
    dco_entry_t e[ARC_VENDOR_GAIN_MAX + 1u];
} dco_table_t;
/* CPU-only state in LP RAM: internal SRAM is the menu's DMA heap. */
static RTC_FAST_ATTR dco_table_t s_dco;
static RTC_FAST_ATTR int64_t s_dco_found_us[ARC_VENDOR_GAIN_MAX + 1u];
/* One calibration-task scratch buffer: the noise envelope probe (startup,
 * CALIBRATE, DC searches) and the live DC trim window never run nested and
 * never on another task. Sharing it frees 4092 bytes of RTC_FAST, which
 * doubles as boot heap. */
static RTC_FAST_ATTR uint8_t s_cal_scratch[24u * PROBE_BYTES];
_Static_assert(sizeof(s_cal_scratch) >= 4092u, "DC trim needs one control window");
#define s_env_buf s_cal_scratch
static uint8_t s_dco_banned[ARC_VENDOR_GAIN_MAX + 1u];
/* A ban expires: released for good, a row that railed once (often its own
 * DC, which the servo would have fixed) stayed on the chip's DC with the
 * servo skipping it (board 2026-10-20: G83 banned, 4 cells of DC, every
 * probe clipped, KEEL holding G83 on one-sided clipping: a deadlock). The
 * hold is tried again after DCO_BAN_US, doubling per repeat up to 32 s. */
#define DCO_BAN_US     2000000LL
#define DCO_BAN_MAX_US 32000000LL
static int64_t s_dco_ban_until[ARC_VENDOR_GAIN_MAX + 1u], s_dco_ban_len[ARC_VENDOR_GAIN_MAX + 1u];
static uint32_t s_dco_unbans, s_dco_guard_deferred;
static bool dc_servo_can_fix(unsigned g);
static uint8_t s_dco_floor;
static bool s_dco_dirty;
uint32_t s_dco_searches;
static uint32_t s_dco_hook_holds, s_dco_aborts, s_dco_carrier_searches;

static void dco_context(dco_table_t *t, uint16_t freq)
{
    const arc_gain_table_t *table = rf_gain_table();
    rf_filter_t f = rf_filter();
    t->version = DCO_VERSION;
    t->freq = freq;
    t->lo = arc_gain_highest_rf_stage_start(table);
    t->hi = table->max_index;
    t->band5 = table->band5;
    t->iq_scale_sel = phy_param[650];   /* phy_rxiq_scale_set() selector */
    t->filter_code = f.code;
    t->filter_skirt = (int8_t)f.skirt;
    t->filter_bw20 = f.bw20;
}

/* Only the header of the wanted context is ever compared, so it is no
 * longer a full 910-byte table (it lived in RTC_FAST to stay off the task
 * stack: two full tables there took 1840 bytes before NVS frames). */
typedef struct {
    uint16_t freq;
    uint8_t lo, hi, band5, iq_scale_sel;
    uint16_t key;            /* dco_filter_key() of the live filter */
} dco_want_t;
static uint16_t live_filter_key(void)
{
    const rf_filter_t f = rf_filter();
    return dco_filter_key(f.code, (int)f.skirt, f.bw20);
}
static dco_want_t dco_want_now(void)
{
    /* The compared fields of dco_context(), without a table on the stack. */
    const arc_gain_table_t *table = rf_gain_table();
    return (dco_want_t){rf_frequency_mhz(), arc_gain_highest_rf_stage_start(table), table->max_index,
                        table->band5, phy_param[650], live_filter_key()};
}
/* The header without the filter: a table of another filter can still seed
 * this one (its rows then stay template rows until measured here). Which
 * filter a table serves is s_dco_serving; its own filter fields say what it
 * was measured with (dco_startup re-measures when they differ). */
static bool dco_matches(const dco_table_t *t, const dco_want_t *w)
{
    return t->version == DCO_VERSION && t->freq == w->freq && t->lo == w->lo && t->hi == w->hi &&
           t->band5 == w->band5 && t->iq_scale_sel == w->iq_scale_sel;
}

/* Per-channel tables (dco_slots.h): slot blobs "dco0".."dco5" plus the
 * index "dco_idx". */
static dco_index_t s_dco_index;
static bool s_dco_index_loaded;
static uint16_t s_dco_seed_mhz;     /* the current table was seeded from this channel, 0 = own */
static uint32_t s_dco_seeds;
static dco_mask_t s_dco_templ;      /* rows of the current table that come from a template */
static uint16_t s_dco_serving;      /* dco_filter_key() of the filter the current table is for */
static bool s_dco_serving_valid;
/* Live trim per row (dco_post_gain): the shared DC model (dc_servo.h),
 * shared + local; indexed by gain row. */
static dc_model_t s_dc_model;
/* Model row of gain row g; DC_MODEL_ROWS (no row) under the DC table. */
static inline unsigned mrow(unsigned g) { return g >= DC_SERVO_ROW0 && g - DC_SERVO_ROW0 < DC_MODEL_ROWS ? g - DC_SERVO_ROW0 : DC_MODEL_ROWS; }
/* DC under the table (2026-11-14, EXPERIMENTAL > LOW DC): the rows of the
 * lower RF stages had no DC hold at all, so the chip's own DC applied there,
 * 1-2.6 cells off on the board at G46-G51 (blackbox session 389: 12 s
 * without picture at 7-10 dB C/N behind a ~22 dB LNA, which puts medium
 * carriers on those rows). Each low row is held on the chip's own bank-2
 * codes, read once per table context in vendor mode, plus one live trim per
 * RF stage, servoed on the carrier like the table rows. Bank 1 stays the
 * chip's own there (phy.c pins it only inside the table span). RAM only,
 * nothing stored. */
#define DC_LOW_ROW0 30u                /* first row of stage 4 on the 5 GHz table (rows under it decode lower) */
#define DC_LOW_ROWS (DC_SERVO_ROW0 - DC_LOW_ROW0)
static int16_t s_low_base[DC_LOW_ROWS][2];
static uint8_t s_low_base_ok[(DC_LOW_ROWS + 7u) / 8u];
static int16_t s_low_trim[DC_LOW_SLOTS][2];
static uint8_t s_low_slot_row[DC_LOW_SLOTS];
static uint32_t s_low_holds, s_low_steps, s_low_read_fails;
static uint32_t s_dco_reloads;      /* dco_select() loads of another table (each drops the trims) */
static uint8_t s_dco_reload_why;    /* 1 none yet, 2 filter, 3 freq, 4 rows, 5 band, 6 IQ scale, 7 version */

unsigned dco_template_rows(void) { return dco_mask_count(s_dco_templ, s_dco.lo, s_dco.hi); }

/* A row measured on the own channel is no longer a template row. */
static void dco_row_measured(unsigned g) { dco_mask_put(s_dco_templ, g, false); }

/* Rows stored before this check: a bad row becomes missing on load. */
static unsigned dco_drop_bad(dco_table_t *t)
{
    unsigned n = 0;
    for (unsigned g = t->lo; g <= t->hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
        if (dco_entry_bad(&t->e[g])) { t->e[g].valid = 0; ++n; }
    s_dco_bad_dropped += n;
    return n;
}

static void dco_slot_key(unsigned slot, char key[8]) { snprintf(key, 8, "dco%u", slot); }
static bool dco_table_complete(const dco_table_t *t, const dco_mask_t templ, uint16_t freq, uint16_t key);
static void dco_ready_put(uint16_t freq, uint16_t key, bool ready);
static void dco_ready_load(void);

static bool dco_load_slot(unsigned slot, dco_table_t *out)
{
    char key[8];
    dco_slot_key(slot, key);
    return blob_load(key, out, sizeof(*out)) && out->version == DCO_VERSION && out->freq == s_dco_index.freq[slot];
}

static bool dco_save(void)
{
    char key[8];
    const uint16_t filt = s_dco_serving_valid ? s_dco_serving :
                          dco_filter_key(s_dco.filter_code, s_dco.filter_skirt, s_dco.filter_bw20);
    const unsigned slot = dco_slot_for_store(&s_dco_index, s_dco.freq, filt);
    const uint16_t old_freq = s_dco_index.freq[slot], old_filt = s_dco_index.filt[slot];
    dco_slot_key(slot, key);
    if (experimental_transfer_active()) return false;
    if (!blob_store(key, &s_dco, sizeof(s_dco))) return false;
    dco_index_touch(&s_dco_index, slot, s_dco.freq, filt);
    memcpy(s_dco_index.templ[slot], s_dco_templ, sizeof(s_dco_templ));
    if (!blob_store("dco_idx", &s_dco_index, sizeof(s_dco_index))) return false;
    if (old_freq && (old_freq != s_dco.freq || old_filt != filt)) dco_ready_put(old_freq, old_filt, false);
    dco_ready_put(s_dco.freq, filt, dco_table_complete(&s_dco, s_dco_templ, s_dco.freq, filt));
    s_dco_seed_mhz = 0;
    return true;
}

static void dco_index_load(void)
{
    if (s_dco_index_loaded) return;
    s_dco_index_loaded = true;
    if (blob_load("dco_idx", &s_dco_index, sizeof(s_dco_index)) && s_dco_index.version == DCO_INDEX_VERSION) {
        dco_ready_load();
        return;
    }
    /* No index of this version: start empty. The version 2 index and the
     * single "dco" table of earlier builds predate calibration generation
     * 5, which discards every older calibration, so nothing is upgraded. */
    dco_index_init(&s_dco_index);
}

static void dco_load_table(const dco_want_t *w);
static void dco_preset_fill(void);

/* Live DC fault: a row the servo cannot hold. It used to be silent (board
 * 2026-10-27: G83 off the ADC for minutes after a walk, the menu clean and
 * the calibration answering "VTX on"). Reasons: 1 the guard released a
 * held row, 2 a row's trim sits at its limit with the DC still off. Shown
 * as DC CAL! in the menu for DC_FAULT_SHOW_US after the last evidence, in
 * T (DC_FAULT) and once per episode in the flight log (DC_CAL_NEEDED). */
#define DC_FAULT_SHOW_US 10000000LL
#define DC_FAULT_DC_MCELLS 1500
static uint8_t s_dcf_reason, s_dcf_gain;
static int64_t s_dcf_us;
static uint32_t s_dcf_episodes;
static const char *dc_fault_name(uint8_t r) { return r == 1 ? "released" : r == 2 ? "trim_limit" : "none"; }
static bool dc_fault_live(void) { return s_dcf_reason && esp_timer_get_time() - s_dcf_us < DC_FAULT_SHOW_US; }
static void dc_fault_mark(uint8_t reason, uint8_t g)
{
    if (!dc_fault_live()) {
        ++s_dcf_episodes;
        printf("DC_FAULT reason=%s gain=%u (the servo cannot hold this row: calibrate with the VTX off)\n",
               dc_fault_name(reason), g);
        const uint8_t data[2] = {reason, g};
        flightlog_event(FLOG_E_DC_CAL_NEEDED, data, 2);
    }
    s_dcf_reason = reason;
    s_dcf_gain = g;
    s_dcf_us = esp_timer_get_time();
}

void dco_select(void)
{
    const dco_want_t want = dco_want_now();
    if (s_dco_serving_valid && s_dco_serving == want.key && dco_matches(&s_dco, &want)) return;
    (void)phy_dco_release();
    phy_dco_bank1_reset(want.lo, want.hi);   /* this table's bank 1 (phy.c pin_bank1) */
    memset(s_dco_found_us, 0, sizeof(s_dco_found_us));
    memset(s_dco_banned, 0, sizeof(s_dco_banned));
    memset(s_dco_ban_until, 0, sizeof(s_dco_ban_until));
    memset(s_dco_ban_len, 0, sizeof(s_dco_ban_len));
    s_dco_floor = 0;
    s_dco_seed_mhz = 0;
    dco_index_load();
    memset(s_dco_templ, 0, sizeof(s_dco_templ));
    ++s_dco_reloads;
    s_dco_reload_why = !s_dco_serving_valid ? 1u : s_dco_serving != want.key ? 2u : s_dco.freq != want.freq ? 3u :
                       s_dco.lo != want.lo || s_dco.hi != want.hi ? 4u : s_dco.band5 != want.band5 ? 5u :
                       s_dco.iq_scale_sel != want.iq_scale_sel ? 6u : 7u;
    s_dco_serving = want.key;
    s_dco_serving_valid = true;
    dc_model_reset(&s_dc_model);   /* trims belong to the table they were made on */
    /* the chip's own low-row codes belong to the channel/filter too */
    memset(s_low_base_ok, 0, sizeof(s_low_base_ok));
    memset(s_low_trim, 0, sizeof(s_low_trim));
    memset(s_low_slot_row, 0, sizeof(s_low_slot_row));
    receiver_dc_reset_all();
    dco_load_table(&want);
    dco_preset_fill();
}

/* This channel's own table, else the nearest one as a template, else empty. */
static void dco_load_table(const dco_want_t *w)
{
    const uint16_t freq = w->freq;
    const int own = dco_slot_of(&s_dco_index, freq, w->key);
    if (own >= 0 && dco_load_slot((unsigned)own, &s_dco) && dco_matches(&s_dco, w)) {
        memcpy(s_dco_templ, s_dco_index.templ[own], sizeof(s_dco_templ));
        if (dco_drop_bad(&s_dco)) printf("DCO table freq=%u: unconverged rows dropped (searched again)\n", freq);
        return;
    }
    /* No own table: start from the nearest one, the same filter first.
     * Preset filters have their own per-class firmware base map, so they only
     * borrow from a stored table of the exact same filter on another channel. */
    const int near = dco_nearest_slot(&s_dco_index, freq, w->key);
    const bool allow_near = near >= 0 && (s_dco_index.filt[near] == w->key || !DCO_PRESET_BOARDS || dco_preset_class_of(w->key) < 0);
    if (allow_near && dco_load_slot((unsigned)near, &s_dco)) {
        const uint16_t from = s_dco.freq;
        s_dco.freq = freq;
        (void)dco_drop_bad(&s_dco);
        if (dco_matches(&s_dco, w)) {
            s_dco_seed_mhz = from;
            ++s_dco_seeds;
            unsigned ok = 0;
            for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g) {
                dco_mask_put(s_dco_templ, g, s_dco.e[g].valid != 0);
                ok += s_dco.e[g].valid != 0;
            }
            printf("DCO template freq=%u from=%u rows=%u (nearest table, same_filter=%u; replaced by own measurements)\n",
                   freq, from, ok, s_dco_index.filt[near] == w->key);
            return;
        }
    }
    memset(&s_dco, 0, sizeof(s_dco));
    dco_context(&s_dco, freq);
}

/* Live trim on top of the stored codes, per row, for this table (a gain
 * write used to re-hold the bare table codes and drop it: with Direct Gain
 * moving between G82 and G83 every trim was undone within a second). */
/* A row without its own trim takes the nearest trimmed row's: the DC shift
 * after a channel setup (a profile switch) is common to the rows. Board
 * 2026-10-19, weak VTX after the live switch to MATCHED: G71 trimmed +8/+13
 * to 0.3 cells, while G77, never trimmed, sat at 3.9/3.5 cells for minutes
 * as Direct Gain moved between them, and the picture was gone. */
/* Since 2026-11-14 the common part is a model term of its own (dc_servo.h
 * dc_model_t): shared by every row, also after rows have their own trims. */
static int dco_offset(unsigned g, unsigned axis)
{
    if (g > ARC_VENDOR_GAIN_MAX) return 0;
    return mrow(g) < DC_MODEL_ROWS ? dc_model_offset(&s_dc_model, mrow(g), axis, dco_mask_get(s_dco_templ, g)) : 0;
}

static int dco_trimmed(unsigned g, unsigned axis)
{
    const int v = s_dco.e[g].code[axis] + dco_offset(g, axis);
    return v < 0 ? 0 : v > 511 ? 511 : v;
}

/* DC base map in the firmware (main/dco_preset.h, tools/dco_preset.py):
 * measured boards' codes of the baseline filter fill the rows this table
 * has no codes for, as template rows: held, trimmed by the live servo and
 * adopted as own rows once the servo holds them (operator decision
 * 2026-10-26: an uncalibrated board held the chip's own DC, 3.7/1.7 cells at
 * G83). Only for the exact baseline context; never calibration evidence. */
static dco_mask_t s_dco_preset;     /* rows of the current table that come from the base map */
uint32_t s_dco_preset_holds;
static void dco_preset_fill(void)
{
    memset(s_dco_preset, 0, sizeof(s_dco_preset));
    const int cls = dco_preset_class_of(s_dco_serving);
    if (!DCO_PRESET_BOARDS || cls < 0 || s_dco.lo != DCO_PRESET_LO || s_dco.hi != DCO_PRESET_HI ||
        s_dco.iq_scale_sel != DCO_PRESET_IQ_SCALE || DCO_VERSION != DCO_PRESET_VERSION)
        return;
    unsigned n = 0;
    for (unsigned g = DCO_PRESET_LO; g <= DCO_PRESET_HI; ++g) {
        const int16_t *row = c5vrx5_dco_preset_class[cls][g - DCO_PRESET_LO];
        if (s_dco.e[g].valid || row[0] < 0 || row[1] < 0) continue;
        s_dco.e[g] = (dco_entry_t){{row[0], row[1]}, {0, 0}, DCO_ROW_OK};
        dco_mask_put(s_dco_templ, g, true);
        dco_mask_put(s_dco_preset, g, true);
        ++n;
    }
    if (n) printf("DCO preset freq=%u class=%d rows=%u boards=%u (firmware base map: trimmed live, adopted when held)\n",
                  s_dco.freq, cls, n, (unsigned)DCO_PRESET_BOARDS);
}

/* The LOW DC slot of row g (0..DC_LOW_SLOTS-1: its RF stage minus
 * DC_LOW_STAGE_FIRST), -1 for the table rows, the stages LOW DC does not
 * serve, a row the table cannot decode or LOW DC switched off. */
static int dc_low_stage(unsigned g)
{
    if (experimental_get(EXP_LOW_DC) || g < DC_LOW_ROW0 || g >= DC_SERVO_ROW0 || g >= s_dco.lo) return -1;
    arc_gain_tuple_t tuple;
    if (!arc_gain_tuple_decode(rf_gain_table(), (uint8_t)g, &tuple) || tuple.rf_stage < DC_LOW_STAGE_FIRST ||
        tuple.rf_stage >= DC_LOW_STAGES) return -1;
    return (int)(tuple.rf_stage - DC_LOW_STAGE_FIRST);
}
static inline unsigned low_idx(unsigned g) { return g - DC_LOW_ROW0; }
static inline bool low_base_ok(unsigned g) { return (s_low_base_ok[low_idx(g) >> 3] >> (low_idx(g) & 7u)) & 1u; }

/* A low row: the chip's own bank-2 codes (read once, in vendor mode right
 * after the gain write) plus its stage's live trim. */
static bool dco_low_codes(uint8_t g, int code[2])
{
    const int st = dc_low_stage(g);
    if (st < 0 || s_dco.freq != rf_frequency_mhz()) return false;
    if (!low_base_ok(g)) {
        int pair[2];
        if (phy_dco_held() || !phy_dco_read_pair(pair)) { ++s_low_read_fails; return false; }
        s_low_base[low_idx(g)][0] = (int16_t)pair[0];
        s_low_base[low_idx(g)][1] = (int16_t)pair[1];
        s_low_base_ok[low_idx(g) >> 3] |= (uint8_t)(1u << (low_idx(g) & 7u));
    }
    for (unsigned a = 0; a < 2; ++a) {
        const int v = s_low_base[low_idx(g)][a] + s_low_trim[st][a];
        code[a] = v < 0 ? 0 : v > 511 ? 511 : v;
    }
    ++s_low_holds;
    return true;
}

/* The codes gain g holds: its row (own, template or base map) with its live
 * trim, or a low row (dco_low_codes). False = nothing to hold (the chip's own DC). */
static bool dco_hold_codes(uint8_t g, int code[2], bool *preset)
{
    *preset = false;
    if (g < s_dco.lo && g < DC_SERVO_ROW0) return g >= DC_LOW_ROW0 && dco_low_codes(g, code);
    if (g < s_dco.lo || g > s_dco.hi || s_dco_banned[g] || s_dco.freq != rf_frequency_mhz() ||
        s_dco.e[g].valid != DCO_ROW_OK) return false;
    code[0] = dco_trimmed(g, 0);
    code[1] = dco_trimmed(g, 1);
    *preset = dco_mask_get(s_dco_preset, g);
    return true;
}

void dco_post_gain(uint8_t g)
{
    if (receiver_paused()) return;
    int code[2];
    bool preset;
    if (!dco_hold_codes(g, code, &preset)) return;
    if (phy_dco_hold(code[0], code[1]) == ESP_OK) {
        ++s_dco_hook_holds;
        s_dco_preset_holds += preset;
    }
}

/* Sampling-phase probes run with the gain worker paused, where the post-gain
 * hook stays silent: hold the current gain's codes explicitly so the scan
 * sees the same DC as live video. */
void dco_hold_for_scan(void)
{
    dco_select();
    int code[2];
    bool preset;
    if (dco_hold_codes(s_current_gain, code, &preset)) (void)phy_dco_hold(code[0], code[1]);   /* the live trim too */
}

/* Bench labs (sense_lab.c) write gains while paused too: the same explicit
 * hold, so TEST and STOCK rows keep their calibrated DC. */
bool calibration_dc_hold_current(void)
{
    dco_hold_for_scan();
    return phy_dco_held();
}

/* Reuse the exact noise statistics of accepted, quiet DC verification
 * instead of immediately measuring the same gain/filter again. Volatile,
 * startup-only, exact-context and at most 30 s old; no NVS format change. */
#define NOISE_CACHE_LO 54u
#define NOISE_CACHE_ROWS (ARC_VENDOR_GAIN_MAX - NOISE_CACHE_LO + 1u)
static RTC_FAST_ATTR struct {
    int64_t time;
    uint16_t freq, key;
    float m2, kurt;
} s_noise_cache[NOISE_CACHE_ROWS];
static float s_noise_last_m2, s_noise_last_kurt;
void cal_dc_noise_cache_clear(void) { memset(s_noise_cache, 0, sizeof(s_noise_cache)); }

static bool dco_measure(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!collect_stats(64, &w, true) || w.windows != 64u) return false;
    const double m2 = w.noise_m2 / w.windows;
    s_noise_last_kurt = m2 > 0 ? (float)((w.noise_m4 / w.windows) / (m2 * m2)) : 0;
    float mi, mq;
    double M2, M4;
    cna_mean(&w.noise_sums, &mi, &mq);
    cna_moments(&w.noise_sums, mi, mq, &M2, &M4);
    s_noise_last_m2 = (float)M2;
    dc[0] = w.dc_i;
    dc[1] = w.dc_q;
    return true;
}

/* Search at one quarter of the window count; the final callback above
 * still measures 64 windows. Preserve the board-proven two-tick IDLE yield
 * and 50 ms settling, rather than trading stability for a shorter delay. */
static bool dco_measure_fast(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!collect(16, &w) || w.windows != 16u) return false;
    dc[0] = w.dc_i; dc[1] = w.dc_q;
    return true;
}

/* Receiver noise: flat envelope statistic, and not dead or railing IQ
 * (which also has one; board 2026-10-07). NULL when it is noise, else the
 * reason. */
const char *noise_check(unsigned *env_out, int *clip_out)
{
    unsigned got = 0;
    for (unsigned tries = 0; tries < 72u && got < 24u; ++tries) {
        vTaskDelay(1);
        if (copy_probe_window(s_env_buf + got * PROBE_BYTES, NULL)) ++got;
    }
    return cal_channel_quiet(s_env_buf, got, false, env_out, clip_out);
}

bool noise_like(void) { return noise_check(NULL, NULL) == NULL; }
/* Quiet enough for a DC search: noise, or IQ pushed onto one rail per axis
 * by the DC the search is about to correct (dsp_quiet_reason "dc_rails"). */
static bool dc_searchable(void)
{
    const char *why = noise_check(NULL, NULL);
    return !why || !strcmp(why, "dc_rails");
}
static bool carrier_like(void)
{
    unsigned env = 0;
    int clip = 0;
    const char *why = noise_check(&env, &clip);
    return cal_dc_gate_carrier(why, env, clip);
}

/* A search needs noise wide enough to dither the quantizer: >= 3 cells
 * per axis each with >= 5 % of the samples. A failed probe is not a floor:
 * it used to mark every lower row FLOOR in NVS and stop the DC bar half way. */
enum { NOISE_MEASURABLE, NOISE_BELOW_STEP, NOISE_PROBE_FAILED };
static int noise_measurable(void)
{
    uint8_t buf[PROBE_BYTES];
    unsigned hi[16] = {0}, hq[16] = {0}, n = 0;
    for (unsigned tries = 0; tries < 24u && n < 8u * sizeof(buf); ++tries) {
        vTaskDelay(1);
        if (!copy_probe_window(buf, NULL)) continue;
        for (unsigned k = 0; k < sizeof(buf); ++k) { ++hi[dsp_i(buf[k]) & 15]; ++hq[dsp_q(buf[k]) & 15]; }
        n += sizeof(buf);
    }
    if (n < 4u * sizeof(buf)) return NOISE_PROBE_FAILED;
    unsigned ci = 0, cq = 0;
    for (unsigned c = 0; c < 16u; ++c) { ci += hi[c] * 20u >= n; cq += hq[c] * 20u >= n; }
    return ci >= 3u && cq >= 3u ? NOISE_MEASURABLE : NOISE_BELOW_STEP;
}

/* With a VTX on: a strong FM carrier lies on a circle around the true DC,
 * and the circle centre does not depend on the picture (dsp_circle_dc).
 * The current gain row is searched on such a carrier, but only on the
 * bench: a clean, strong circle and the same gain held for 10 s, never in
 * flight (a search steps the DC for about two seconds). */
#define DCO_CARRIER_RATIO_X100 300u
static bool dco_measure_carrier(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!collect(64, &w) || !w.dc_circle || w.envelope_x100 < DCO_CARRIER_RATIO_X100) return false;
    dc[0] = w.dc_i;
    dc[1] = w.dc_q;
    return true;
}

static bool dco_carrier_search(bool manual)
{
    const uint8_t g = s_current_gain;
    if (!manual && g <= ARC_VENDOR_GAIN_MAX && s_dco.e[g].valid == DCO_ROW_OK) return false;
    if (s_gain.state != DG_HOLD || g < s_dco.lo || g > s_dco.hi || s_obs_clip_pm) return false;
    window_stats_t w;
    if (!collect(16, &w) || !w.dc_circle || w.envelope_x100 < DCO_CARRIER_RATIO_X100) return false;
    if (!receiver_pause("DCO_CARRIER")) return false;
    (void)phy_dco_release();
    phy_dco_result_t res;
    (void)phy_dco_search(dco_measure_carrier, &res);
    if (abs(res.residual_mcells[0]) > DCO_BAD_MCELLS || abs(res.residual_mcells[1]) > DCO_BAD_MCELLS)
        res.measured = false;
    if (res.measured) {
        s_dco.e[g] = (dco_entry_t){{(int16_t)res.codes[0], (int16_t)res.codes[1]},
            {(int16_t)res.residual_mcells[0], (int16_t)res.residual_mcells[1]}, DCO_ROW_OK};
        dco_row_measured(g);
        if (dco_save()) s_dco_dirty = false;
    }
    s_dco_found_us[g] = esp_timer_get_time();
    ++s_dco_searches;
    ++s_dco_carrier_searches;
    receiver_resume();
    return res.measured;
}

/* Noise below one quantizer step: the IQ is a fixed code, so a short
 * window is an exact DC measurement (no averaging needed). */
static bool dco_measure_flat(int dc[2])
{
    window_stats_t w;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!collect(8, &w)) return false;
    dc[0] = w.dc_i;
    dc[1] = w.dc_q;
    return true;
}

/* Row g with the receiver noise below one step: codes from the flat search.
 * Before, such a row and all below it were FLOOR (left at the chip's DC). */
static bool dco_flat_row(uint8_t g, const int *seed)
{
    phy_dco_result_t res;
    (void)phy_dco_search_flat_from(dco_measure_flat, seed, &res);
    if (!res.measured && seed) (void)phy_dco_search_flat(dco_measure_flat, &res);
    if (!res.measured) return false;
    s_dco.e[g] = (dco_entry_t){{(int16_t)res.codes[0], (int16_t)res.codes[1]},
        {(int16_t)res.residual_mcells[0], (int16_t)res.residual_mcells[1]}, DCO_ROW_OK};
    dco_row_measured(g);
    s_dco_found_us[g] = esp_timer_get_time();
    ++s_dco_searches;
    return true;
}

/* One quiet search of gain row g (receiver noise). NOISE_BELOW_STEP when
 * the noise is below one quantizer step there (no codes at or below g),
 * NOISE_PROBE_FAILED when nothing could be measured (rows untouched). */
static bool s_dco_batch;
static int dco_quiet_search(uint8_t g, const int *seed)
{
    if (!s_dco_batch && !receiver_pause("DCO")) return NOISE_PROBE_FAILED;
    const uint8_t restore = s_current_gain;
    (void)phy_dco_release();
    receiver_set_gain(g);
    vTaskDelay(pdMS_TO_TICKS(20));
    int measurable = noise_measurable();
    if (measurable == NOISE_PROBE_FAILED) {
        printf("DCO row=%u probe_failed (rows kept for the next boot)\n", g);
    } else if (measurable == NOISE_BELOW_STEP && dco_flat_row(g, seed)) {
        measurable = NOISE_MEASURABLE;          /* this row has codes now; go on downward */
    } else if (measurable == NOISE_BELOW_STEP) {
        /* Less gain, even less noise, and no flip found: the rows below
         * cannot be searched either. Codes measured on a carrier stay. */
        for (int k = g; k >= (int)s_dco.lo; --k) {
            /* Own codes measured on a carrier stay; template codes (another
             * channel or another filter) are not this context's and go. */
            if (s_dco.e[k].valid != DCO_ROW_OK || dco_mask_get(s_dco_templ, (unsigned)k))
                s_dco.e[k].valid = DCO_ROW_FLOOR;
            dco_row_measured((unsigned)k);   /* measured: nothing to correct here */
        }
        s_dco_floor = (uint8_t)(g + 1);
    } else {
        phy_dco_result_t res;
        (void)phy_dco_search_verified(dco_measure_fast, dco_measure, seed, &res);
        if (!res.measured) {
            printf("DCO row=%u fast_search=fallback_full\n", g);
            (void)phy_dco_search_from(dco_measure, seed, &res);
        }
        /* A neighbouring seed may be outside this row's response range. */
        if ((!res.measured || abs(res.residual_mcells[0]) > DCO_BAD_MCELLS ||
             abs(res.residual_mcells[1]) > DCO_BAD_MCELLS) && seed)
            (void)phy_dco_search_from(dco_measure, NULL, &res);
        if (res.measured) {
            const dco_entry_t e = {{(int16_t)res.codes[0], (int16_t)res.codes[1]},
                {(int16_t)res.residual_mcells[0], (int16_t)res.residual_mcells[1]}, DCO_ROW_OK};
            if (!dco_entry_bad(&e)) {
                s_dco.e[g] = e;
                dco_row_measured(g);
                if (g >= NOISE_CACHE_LO) {
                    const unsigned k = g - NOISE_CACHE_LO;
                    s_noise_cache[k].time = esp_timer_get_time();
                    s_noise_cache[k].freq = rf_frequency_mhz();
                    s_noise_cache[k].key = live_filter_key();
                    s_noise_cache[k].m2 = s_noise_last_m2;
                    s_noise_cache[k].kurt = s_noise_last_kurt;
                }
            } else {
                s_dco.e[g].valid = 0;                /* not converged: missing, repaired from a neighbour */
                printf("DCO row=%u not_converged residual=%d/%d\n", g, e.residual_mcells[0], e.residual_mcells[1]);
            }
        }
        s_dco_found_us[g] = esp_timer_get_time();
        ++s_dco_searches;
    }
    if (measurable != NOISE_PROBE_FAILED) s_dco_dirty = true;
    if (!s_dco_batch) { receiver_set_gain(restore); receiver_resume(); }
    return measurable;
}

unsigned dco_rows(uint8_t state)
{
    unsigned n = 0;
    for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g) n += s_dco.e[g].valid == state;
    return n;
}
unsigned dco_valid_rows(void) { return dco_rows(DCO_ROW_OK); }

bool calibration_dc_ok(uint8_t g)
{
    if (phy_filter_calibrated_code() < 0) return true;     /* unpinned PHY: nothing to hold */
    if (s_dco.freq != rf_frequency_mhz() || s_dco_serving != live_filter_key() ||
        g > ARC_VENDOR_GAIN_MAX || dco_mask_get(s_dco_templ, g)) return false;
    if (g < s_dco.lo || g > s_dco.hi) return true;
    if (s_dco.e[g].valid == DCO_ROW_OK) return !s_dco_banned[g] && phy_dco_held();
    return s_dco.e[g].valid == DCO_ROW_FLOOR || g < s_dco_floor;
}

/* Continuation: every missing row next to a converged one is searched from
 * that row's codes, upward from below and downward from above, until no row
 * changes. The gain steps between rows are ~1 dB, so the start is within the
 * search's linear range wherever the chip's own codes were not. */
static void dco_repair_rows(int64_t deadline)
{
    for (bool changed = true; changed && esp_timer_get_time() < deadline;) {
        changed = false;
        for (int pass = 0; pass < 2; ++pass) {
            for (int k = 0; k <= (int)(s_dco.hi - s_dco.lo); ++k) {
                const int g = pass == 0 ? (int)s_dco.lo + k : (int)s_dco.hi - k;
                const int n = pass == 0 ? g - 1 : g + 1;
                /* A template neighbour seeds too: the seed only has to start
                 * the search inside its linear range. */
                if (n < (int)s_dco.lo || n > (int)s_dco.hi || s_dco.e[g].valid || s_dco.e[n].valid != DCO_ROW_OK ||
                    g < (int)s_dco_floor) continue;
                if (esp_timer_get_time() > deadline || carrier_like()) return;
                const int seed[2] = {s_dco.e[n].code[0], s_dco.e[n].code[1]};
                ++s_dco_repairs;
                if (dco_quiet_search((uint8_t)g, seed) == NOISE_MEASURABLE && s_dco.e[g].valid == DCO_ROW_OK) {
                    printf("DCO row=%u repaired from row=%d codes=%d/%d\n", g, n, s_dco.e[g].code[0], s_dco.e[g].code[1]);
                    changed = true;
                } else {
                    ++s_dco_repair_fails;
                }
            }
        }
    }
}

/* The quiet-noise row search, from the top row down; shared by the own
 * channel (VTX off) and a quiet channel (VTX on). */
/* refine: also measure rows that still come from a template (own channel
 * quiet at boot), replacing them by this channel's own codes. */
static const char *dco_quiet_rows(bool manual, bool refine, int64_t deadline, const char *line1, unsigned *done_out)
{
    if (!receiver_pause("DCO_BATCH")) return "busy";
    const uint8_t restore_gain = s_current_gain;
    s_dco_batch = true;
    const int64_t t0 = esp_timer_get_time();
    const unsigned rows = manual ? (unsigned)(s_dco.hi - s_dco.lo + 1u) :
                          dco_rows(0) + (refine ? dco_template_rows() : 0u);
    unsigned done = 0;
    const char *why = "done";
    /* Rows over the useful top (receiver_gain_top): behind an external LNA
     * the receiver noise folds there (board 2026-11-14: G72 and up), so a
     * quiet search reads no DC and the carrier gate stops the whole run
     * before the rows the edge uses (G50-G67). No gain model uses them. */
    const int gtop = receiver_gain_top();
    for (int g = s_dco.hi; g >= (int)s_dco.lo; --g) {
        if (g > gtop) continue;
        if (!manual && s_dco.e[g].valid && !(refine && dco_mask_get(s_dco_templ, (unsigned)g)))
            continue;                                             /* own codes or floor */
        char line2[48];
        unsigned left = done ? (unsigned)((esp_timer_get_time() - t0) / done * (rows - done) / 1000000) + 1u : rows * 2u;
        snprintf(line2, sizeof(line2), "ABOUT %u S LEFT, THEN LIVE VIDEO", left);
        status_screen("CALIBRATING DC", line1, line2, rows ? (int)(done * 100u / rows) : 0);
        if (esp_timer_get_time() > deadline) { why = "timeout"; break; }
        unsigned env = 0;
        int clip = 0;
        const char *quiet = g >= 70 ? noise_check(&env, &clip) : NULL;
        if (cal_dc_gate_carrier(quiet, env, clip)) {   /* VTX switched on / leaking */
            printf("DCO gate row=%d gain=%u reason=%s env_x100=%u clip_pm=%d\n", g, s_current_gain, quiet, env, clip);
            why = "carrier_detected";
            break;
        }
        /* Only own, converged, unbanned rows of this exact filter/channel
         * seed the next row; a template is never calibration evidence. */
        const bool own = !manual && s_dco.e[g].valid == DCO_ROW_OK && !dco_mask_get(s_dco_templ, (unsigned)g) &&
            !s_dco_banned[g];
        const bool top_linear = manual && g == (int)s_dco.hi && s_dco.e[g].valid == DCO_ROW_OK && !s_dco_banned[g];
        const bool neighbour = g < (int)s_dco.hi && s_dco.e[g + 1].valid == DCO_ROW_OK &&
            !dco_mask_get(s_dco_templ, (unsigned)g + 1u) && !s_dco_banned[g + 1];
        const int seed[2] = {own ? s_dco.e[g].code[0] : top_linear ? s_dco.e[g].code[0] + 6 : neighbour ? s_dco.e[g + 1].code[0] : 0,
                             own ? s_dco.e[g].code[1] : top_linear ? s_dco.e[g].code[1] - 6 : neighbour ? s_dco.e[g + 1].code[1] : 0};
        const int r = dco_quiet_search((uint8_t)g, own || top_linear || neighbour ? seed : NULL);
        if (r == NOISE_PROBE_FAILED) { why = "probe_failed"; break; }
        if (r == NOISE_BELOW_STEP) { why = "floor"; break; }     /* the rest needs no correction */
        ++done;
    }
    if (!strcmp(why, "done") || !strcmp(why, "floor")) dco_repair_rows(deadline);
    *done_out = done;
    s_dco_batch = false;
    receiver_set_gain(restore_gain);
    receiver_resume();
    printf("CAL_STEP step=dc rows=%u ms=%lld result=%s\n", done, (esp_timer_get_time() - t0) / 1000, why);
    return why;
}

/* With the VTX on, the own channel never shows plain receiver noise, so the
 * high gain rows (the ones used far out) stayed without DC codes: board
 * DC without correction is ~1.1 cell (C5VRX-4 video_dco.c), and in the
 * stress matrix a 1-cell DC at 4 dB C/N broke the goggle line lock
 * (picture 0.25-0.97: lines shifting in bands). Like the filter sweep, the
 * rows are measured on a quiet channel >= 40 MHz away and the exact tuning
 * is restored. The DC DAC offsets are baseband offsets per gain row, so the
 * codes are stored for the own channel; the live drift nudges trim the
 * remaining difference. Startup only, behind the status screen. */
/* ---- DC health in flight ------------------------------------------------
 * Passive (drift_service): reads the table state, the filter registers and
 * the DC mean; never writes the receiver. A sustained finding the servo
 * cannot fix shows DC CAL NEEDED; no search or automatic repair is attached.
 * (The NVS repair flags "dc_recal" of earlier builds are no longer read:
 * nothing has written them since the manual calibration policy, and
 * calibration generation 5 discarded the old ones.) */
unsigned calibration_dc_needed(char *buf, size_t size)
{
    /* Keep a proven recommendation visible while the user navigates the
     * menu (where passive measurement is intentionally suspended). */
    const bool bad = s_cal_notice && s_cal_notice_freq == rf_frequency_mhz() && s_cal_notice_key == live_filter_key();
    const bool fault = dc_fault_live();
    if (size) snprintf(buf, size, "%s", fault ? "FAULT" : bad ? "BASEMAP" : "");
    return bad || fault;
}

static void dc_health_print(void)
{
    printf("CAL_HEALTH state=%s qualified=%u rejected=%u dc_mcells=%d/%d reason=%u "
           "auto_cal=0 auto_dc_trim=1 boot_phase_budget_ms=600 last_us=%lld\n",
           calibration_dc_needed(NULL, 0) ? "bad" : s_basemap_watch.state == CAL_GOOD ? "good" : "unknown",
           s_health_blocks, s_health_rejected, s_basemap_watch.last_i, s_basemap_watch.last_q,
           s_basemap_watch.reason, s_basemap_watch.last_us);
}

/* Startup: only the rows of the channel that are still missing (all rows
 * on a manual request). Rows that have codes or a measured noise floor are
 * kept, so a boot after a complete calibration does nothing here. */
/* Result of the last DC pass on the live filter ("done", "floor", or why it
 * stopped), for the end screen of a manual calibration. */
const char *s_dco_last_why = "not_run";

void dco_startup(bool manual, int64_t deadline)
{
    dco_select();
    const unsigned missing = dco_rows(0);
    const bool carrier = video_recent();
    /* Codes measured with another analog filter do not fit the live one
     * (dco_slots.h dco_filter_same: 3.7 cells of DC left on the board, and
     * the gain parked on it). Every row with a result (codes or FLOOR) is
     * then marked as a template row, i.e. not measured in this context; a
     * row leaves that mask only on a new result (dco_row_measured), the mask
     * is stored with the table, so an interrupted pass goes on at the next
     * boot, and the table takes the live filter only when no template row
     * is left. Template rows are measured on the own channel (VTX off) or a
     * quiet channel (VTX on). */
    const rf_filter_t live = rf_filter();
    const unsigned results = dco_valid_rows() + dco_rows(DCO_ROW_FLOOR);
    const bool stale = dco_filter_stale(results,
        dco_filter_same(s_dco.filter_code, s_dco.filter_skirt, s_dco.filter_bw20, live.code, (int)live.skirt,
                        live.bw20));
    if (stale && !dco_template_rows()) {
        for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
            if (s_dco.e[g].valid) dco_mask_put(s_dco_templ, g, true);
        s_dco_dirty = true;
    }
    const unsigned templ = dco_template_rows();
    if (stale)
        printf("DCO startup filter_stale=1 table_bw20=%u table_code=%d table_skirt=%d live_bw20=%u live_code=%d "
               "live_skirt=%u rows_to_measure=%u\n", (unsigned)s_dco.filter_bw20, (int)s_dco.filter_code,
               (int)s_dco.filter_skirt, (unsigned)live.bw20, live.code, (unsigned)live.skirt, templ);
    /* Template rows from another channel are replaced only when the own
     * channel can be measured without a detour (VTX off at boot); rows of
     * another filter also go to a quiet channel. */
    const bool refine = !manual && !missing && templ && (stale || (!carrier && dc_searchable()));
    if (!manual && !missing && !refine) {
        printf("DCO startup skip=complete rows=%u floor_rows=%u template_rows=%u\n", dco_valid_rows(),
               dco_rows(DCO_ROW_FLOOR), templ);
        return;
    }
    unsigned done = 0;
    const char *why;
    const char *where = refine ? "own_refine" : "own";
    if (!carrier && dc_searchable()) {
        why = dco_quiet_rows(manual, refine, deadline, refine ? "OWN CHANNEL, VTX OFF" : "KEEP THE VTX OFF", &done);
    } else {
        /* A carrier on the own channel: measure on a quiet channel; only if
         * there is none, the current row on the carrier's circle. */
        const bool ok = carrier && dco_carrier_search(manual);
        where = "carrier";
        why = ok ? "carrier_row_only" : "need_vtx_off";
        printf("DCO startup carrier=%u result=%s gain=%u missing_rows=%u (VTX off measures all rows)\n",
               carrier, ok ? "stored_current_row" : "skip", s_current_gain, missing);
    }
    const bool complete = !strcmp(why, "floor") || !strcmp(why, "done");
    s_dco_last_why = why;
    /* The noise floor ends the search before the last row: that is the
     * normal end, not a stall, so the bar completes and says so. */
    if (complete)
        status_screen("DC CALIBRATED", "LOWER GAINS NEED NO DC CORRECTION", "", 100);
    else
        status_screen("DC NOT COMPLETE", why, "USE MENU > CALIBRATE", -1);
    vTaskDelay(pdMS_TO_TICKS(800));
    if (stale && complete && !dco_template_rows()) {
        s_dco.filter_code = live.code;          /* every row now has a result in the live filter */
        s_dco.filter_skirt = (int8_t)live.skirt;
        s_dco.filter_bw20 = live.bw20;
        s_dco_dirty = true;
    }
    if (s_dco_dirty && dco_save()) s_dco_dirty = false;
    printf("DCO startup where=%s result=%s searched=%u rows=%u floor_rows=%u missing=%u template_rows=%u "
           "floor_from_gain=%u\n", where, why, done, dco_valid_rows(), dco_rows(DCO_ROW_FLOOR), dco_rows(0),
           dco_template_rows(), s_dco_floor);
}

/* DC readiness per physical filter on the current channel, loaded from the
 * stored NVS slots at boot and updated after every DC pass / save: a profile
 * switch or menu FILTER choice may only select a filter whose table is
 * complete there (no missing or template rows, measured with that filter);
 * otherwise it would fly on codes of another filter. */
#define DCO_READY_MAX DCO_SLOTS
static struct { uint16_t freq, key; bool ready; } s_dco_ready[DCO_READY_MAX];

static bool dco_table_complete(const dco_table_t *t, const dco_mask_t templ, uint16_t freq, uint16_t key)
{
    const arc_gain_table_t *table = rf_gain_table();
    const dco_want_t want = {freq, arc_gain_highest_rf_stage_start(table), table->max_index,
                             table->band5, phy_param[650], key};
    if (!dco_matches(t, &want) || dco_filter_key(t->filter_code, t->filter_skirt, t->filter_bw20) != key ||
        dco_mask_count(templ, t->lo, t->hi)) return false;
    for (unsigned g = t->lo; g <= t->hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
        if (!t->e[g].valid || dco_entry_bad(&t->e[g])) return false;
    return true;
}

bool dco_current_complete(void)
{
    return dco_table_complete(&s_dco, s_dco_templ, rf_frequency_mhz(), live_filter_key());
}

static void dco_ready_put(uint16_t freq, uint16_t key, bool ready)
{
    int empty = -1, unready = -1, other = -1;
    for (unsigned k = 0; k < DCO_READY_MAX; ++k) {
        if (s_dco_ready[k].freq == freq && s_dco_ready[k].key == key) {
            s_dco_ready[k].ready = ready;
            return;
        }
        if (!s_dco_ready[k].freq && empty < 0) empty = (int)k;
        else if (s_dco_ready[k].freq && !s_dco_ready[k].ready && unready < 0) unready = (int)k;
        if (s_dco_ready[k].freq != freq && other < 0) other = (int)k;
    }
    const int slot = empty >= 0 ? empty : unready >= 0 ? unready : ready ? (other >= 0 ? other : 0) : -1;
    if (slot < 0) return;
    s_dco_ready[slot].freq = freq;
    s_dco_ready[slot].key = key;
    s_dco_ready[slot].ready = ready;
}

static void dco_ready_load(void)
{
    memset(s_dco_ready, 0, sizeof(s_dco_ready));
    for (unsigned k = 0; k < DCO_SLOTS; ++k) {
        if (!s_dco_index.freq[k] || !dco_load_slot(k, &s_dco)) continue;
        dco_ready_put(s_dco_index.freq[k], s_dco_index.filt[k],
                      dco_table_complete(&s_dco, s_dco_index.templ[k], s_dco_index.freq[k], s_dco_index.filt[k]));
    }
    memset(&s_dco, 0, sizeof(s_dco));
    for (unsigned k = 0; k < DCO_READY_MAX; ++k)
        if (s_dco_ready[k].freq)
            printf("DCO_READY freq=%u filter_key=%04x ready=%u\n", s_dco_ready[k].freq, s_dco_ready[k].key,
                   s_dco_ready[k].ready);
}

void dco_ready_note(void)
{
    dco_ready_put(rf_frequency_mhz(), live_filter_key(), dco_current_complete());
}

bool calibration_dc_ready(rf_filter_t f)
{
    const uint16_t key = dco_filter_key(f.code, (int)f.skirt, f.bw20), freq = rf_frequency_mhz();
    for (unsigned k = 0; k < DCO_READY_MAX; ++k)
        if (s_dco_ready[k].freq == freq && s_dco_ready[k].key == key)
            return s_dco_ready[k].ready || (DCO_PRESET_BOARDS > 0 && dco_preset_class_of(key) >= 0);
    return DCO_PRESET_BOARDS > 0 && dco_preset_class_of(key) >= 0;
}

/* Startup: the DC tables of the other profile filters too (NORMAL, NARROW,
 * WIDE, each distinct physical setting once), so a profile switch in flight
 * finds the codes of its own filter. Each filter is set up in its own
 * analog branch (rf_set_filter) and goes through dco_startup(); a pass cut
 * by the budget continues at the next boot (template/missing rows). */
void dco_startup_profiles(bool manual, int64_t deadline)
{
    const rf_filter_t live = rf_filter();
    dco_ready_note();
    rf_filter_t todo[4];
    unsigned n = 0;
    static const int classes[4] = {FILTER_NORMAL, FILTER_NARROW, FILTER_WIDE, FILTER_XNARROW};
    for (unsigned c = 0; c < 4u; ++c) {
        rf_filter_t f;
        if (!filter_setting(classes[c], &f) || rf_filter_equal(f, live)) continue;
        bool dup = false;
        for (unsigned k = 0; k < n; ++k) dup |= rf_filter_equal(todo[k], f);
        if (!dup) todo[n++] = f;
    }
    for (unsigned k = 0; k < n && esp_timer_get_time() < deadline; ++k) {
        printf("DCO startup profile_filter bw20=%u code=%d skirt=%u\n", (unsigned)todo[k].bw20, todo[k].code,
               (unsigned)todo[k].skirt);
        if (!rf_set_filter(todo[k])) continue;
        receiver_set_gain(receiver_gain_top());
        vTaskDelay(pdMS_TO_TICKS(50));
        dco_startup(manual, deadline);
        dco_ready_note();
    }
    if (n) {
        if (!rf_set_filter(live)) printf("DCO startup error: live filter not restored\n");
        dco_select();
    }
    for (unsigned k = 0; k < DCO_READY_MAX; ++k)
        if (s_dco_ready[k].freq)
            printf("DCO_READY freq=%u filter_key=%04x ready=%u\n", s_dco_ready[k].freq, s_dco_ready[k].key,
                   s_dco_ready[k].ready);
}

/* Live: hold the stored codes after gain writes and guard them; never a
 * search (a search steps the DC for seconds). */
void dco_service(void)
{
    if (!quiet_owner()) return;
    dco_select();
    if (s_dco_dirty && s_gain.state == DG_HOLD && dco_save()) s_dco_dirty = false;
    const int64_t now = esp_timer_get_time();
    for (unsigned r = 0; r <= ARC_VENDOR_GAIN_MAX; ++r)
        if (s_dco_banned[r] && now >= s_dco_ban_until[r]) { s_dco_banned[r] = 0; ++s_dco_unbans; }
    /* Hold guard: held codes the servo cannot correct must not keep the IQ
     * dead or on a rail. Railing IQ on a row the servo can still trim is the
     * servo's job (sign-bit bracket, dc_servo.h): releasing it fell back to
     * the chip's own DC (cells off on this board) and the ban kept the servo
     * off that row for 2-32 s. Blackbox 2026-10-27 (sessions 246-249): edge
     * rows G73-G83 sat 2-4.4 cells off for tens of seconds with their held
     * trims unchanged, and RECOVER lasted 3-10 s. */
    static unsigned bad_ticks;
    const uint8_t g = s_current_gain;
    if (phy_dco_held()) {
        const bool bad = s_obs_origin_pm >= 900 || s_obs_clip_pm >= 500;
        if (bad && dc_servo_can_fix(g)) { calibration_dc_urgent(); bad_ticks = 0; ++s_dco_guard_deferred; }
        else bad_ticks = bad ? bad_ticks + 1u : 0u;
        if (bad_ticks >= 3u) {
            (void)phy_dco_release();
            s_dco_banned[g] = 1u;
            s_dco_ban_len[g] = s_dco_ban_len[g] && now - s_dco_ban_until[g] < 60000000LL ?
                               (s_dco_ban_len[g] * 2 > DCO_BAN_MAX_US ? DCO_BAN_MAX_US : s_dco_ban_len[g] * 2) : DCO_BAN_US;
            s_dco_ban_until[g] = now + s_dco_ban_len[g];
            bad_ticks = 0;
            ++s_dco_aborts;
            printf("DCO hold abort gain=%u\n", g);
            dc_fault_mark(1u, g);
        }
    } else if (s_gain.state != DG_SETTLE) {
        dco_post_gain(g);
    }
}

/* One passive collector for health and phase evidence. Independent blocks
 * use identical gain/filter/DC/TX context. No PHY ownership, writes, pause,
 * NVS learning, or raster takeover: the DC actuator is servo_service(). */
static int s_drift_avg[2];
static float s_temp_c = -100.0f;
static uint32_t s_servo_writes;
TaskHandle_t s_cal_task;
static volatile bool s_dc_urgent;
uint32_t s_dc_urgent_wakes;

/* Live DC servo (dc_servo.h, restored from main by operator decision
 * 2026-10-26: boot on the base map, then tune live with the VTX on or off).
 * The DC of each gain row is the mean of every settled observer probe in
 * that row (receiver_dc_take_row); a computed step on both DAC codes of the
 * row corrects it, without stopping video. Board, main 2026-10-19/20: six
 * boots 0.3-0.8 cells after 60 s and a 2-4 dB range limit (2-8 dB without),
 * folding-safe since PR #228. Boot itself still never calibrates. */
/* Codes around the stored ones, as far as the calibration search spans:
 * calibrations of one board scatter by tens of codes. */
#define DC_TRIM_MAX_OFFSET 96
/* Template and base-map rows (another channel, another board) may need more. */
#define DC_TRIM_MAX_TEMPLATE 192
static int dc_trim_max(unsigned g) { return dco_mask_get(s_dco_templ, g) ? DC_TRIM_MAX_TEMPLATE : DC_TRIM_MAX_OFFSET; }
/* Self-calibration in use: a template or base-map row the servo holds within
 * DC_ADOPT_MCELLS for a run of estimates becomes this channel's own row
 * (codes + trim), stored with the table. On receiver noise as in main; on a
 * carrier (the mean of a full FM circle is its centre) after a longer run. */
#define DC_ADOPT_MCELLS 300
#define DC_ADOPT_RUN 8u
#define DC_ADOPT_RUN_CARRIER 16u
static uint8_t s_adopt_run[ARC_VENDOR_GAIN_MAX + 1u];
uint32_t s_dco_adopted;
static bool s_dc_fault;
static RTC_FAST_ATTR dc_servo_t s_servo;
_Static_assert(DC_SERVO_ROW0 + DC_SERVO_TABLE_ROWS == ARC_VENDOR_GAIN_MAX + 1u, "table servo rows end at the table maximum");
static bool s_servo_ready;
/* No step: state (menu, scan, startup), row without codes or another table,
 * sampling phase not settled, PHY busy, write refused. */
static uint32_t s_servo_skip[5], s_servo_estimates, s_servo_fast;
static int s_servo_dc[2];      /* last mean of the active row (T DC_SERVO) */
static int64_t s_servo_dc_us;  /* when s_servo_dc was measured */
static bool s_servo_phase_hold; /* waiting for a settled sampling phase */
static struct { uint32_t control, reloads; uint16_t freq, key; } s_servo_ctx;

static bool dc_trim_snapshot(dc_trim_context_t *c)
{
    arc_gain_tuple_t tuple;
    if (!arc_gain_tuple_decode(rf_gain_table(), s_current_gain, &tuple)) return false;
    const rf_filter_t filter = rf_filter();
    *c = (dc_trim_context_t){s_current_gain, s_gain_epoch, s_control_epoch, phy_generation(),
        rf_table_generation(), rf_frequency_mhz(), (uint32_t)rf_offset_khz(), (uint32_t)filter.code,
        filter.skirt, filter.bw20, rf_lane(), phy_param[650], tuple.packed_state};
    return true;
}

static void dc_trim_invalidate(void)
{
    s_last_gain_write_us = esp_timer_get_time();
    ++s_gain_epoch; /* A physical DC change invalidates amplitude/CN evidence too. */
}

/* A row's codes changed (its own offset, the offset a template row inherits,
 * or the shared part every row carries): their sums no longer describe them,
 * and their last servo step is no evidence for the next mean. */
static void servo_rows_changed(unsigned g, bool shared)
{
    receiver_dc_reset_row((uint8_t)g);
    for (unsigned r = s_dco.lo; r <= s_dco.hi && r <= ARC_VENDOR_GAIN_MAX; ++r) {
        if (r == g || (!shared && ((mrow(r) < DC_MODEL_ROWS && s_dc_model.own[mrow(r)]) || !dco_mask_get(s_dco_templ, r)))) continue;
        receiver_dc_reset_row((uint8_t)r);
        if (r >= DC_SERVO_ROW0) dc_servo_forget_step(&s_servo, r - DC_SERVO_ROW0);
    }
}

/* Write a step to the active row's DAC pair (one axis per write, checked
 * against the gain and the held pair). Returns the codes applied. */
static bool servo_write_active(uint8_t g, const int step[2], int applied[2], bool *fault)
{
    applied[0] = applied[1] = 0;
    dc_trim_context_t context;
    int pair[2];
    if (!dc_trim_snapshot(&context) || context.gain != g || !phy_try_actuator(context.phy)) return false;
    if (!phy_dco_read_pair(pair)) { phy_end_actuator(); return false; }
    int cur[2] = {pair[0], pair[1]};
    for (int a = 0; a < 2 && !*fault; ++a) {
        /* Verified trials of at most PHY_DCO_TRY_MAX codes each: the sign
         * bracket asks for up to DC_SERVO_FOLD_STEP_MAX. */
        int left = step[a];
        bool stop = false;
        while (left && !stop) {
            const int part = left > PHY_DCO_TRY_MAX ? PHY_DCO_TRY_MAX : left < -PHY_DCO_TRY_MAX ? -PHY_DCO_TRY_MAX : left;
            int target[2] = {cur[0], cur[1]};
            target[a] += part;
            if (target[a] < 0 || target[a] > 511) break;
            const esp_err_t err = phy_dco_try_pair(context.packed_gain, cur, target);
            if (err == ESP_OK) { cur[0] = target[0]; cur[1] = target[1]; left -= part; }
            else { *fault = err == ESP_ERR_INVALID_RESPONSE; ++s_servo_skip[4]; stop = true; }
        }
        if (stop) break;
    }
    applied[0] = cur[0] - pair[0];
    applied[1] = cur[1] - pair[1];
    if (applied[0] || applied[1]) dc_trim_invalidate();
    phy_end_actuator();
    return applied[0] || applied[1];
}

/* Whether the servo can still correct row g: running, a held row, and trim
 * left on both axes. A recommendation waits for it otherwise. */
static bool dc_servo_can_fix(unsigned g)
{
    if (!s_servo_ready || s_dc_fault || s_servo_phase_hold || g < s_dco.lo || g > s_dco.hi || g > ARC_VENDOR_GAIN_MAX ||
        s_dco.e[g].valid != DCO_ROW_OK || s_dco_banned[g]) return false;
    const int lim = dc_trim_max(g);
    return abs(dco_offset(g, 0)) < lim && abs(dco_offset(g, 1)) < lim;
}

/* The rows under the table (dco_low_codes): one servo slot and one trim per
 * RF stage. The trim moves every row of the stage, so a step resets the
 * stage's sums; a visit to another row of the stage is no evidence for the
 * slot's last step (forgotten). The same means, sign-bit bracketing and
 * trim limit as the template rows; no adoption, nothing stored. */
static void servo_low_rows(void)
{
    for (unsigned g = DC_LOW_ROW0; g < s_dco.lo && g < DC_SERVO_ROW0; ++g) {
        const int st = dc_low_stage(g);
        if (st < 0 || !low_base_ok(g)) continue;
        const unsigned slot = DC_SERVO_TABLE_ROWS + (unsigned)st;
        int64_t si, sq;
        uint32_t samples, neg[2], need = DC_SERVO_MIN_SAMPLES, peek_neg[2], peek_n;
        if (receiver_dc_peek_row((uint8_t)g, peek_neg, &peek_n) && peek_n >= DC_SERVO_FAST_SAMPLES &&
            (dc_servo_saturated(dc_servo_pneg(peek_neg[0], peek_n)) || dc_servo_saturated(dc_servo_pneg(peek_neg[1], peek_n))))
            need = DC_SERVO_FAST_SAMPLES;
        if (!receiver_dc_take_row((uint8_t)g, need, &si, &sq, neg, &samples)) continue;
        if (samples < DC_SERVO_MIN_SAMPLES) {
            si = si * (int64_t)DC_SERVO_MIN_SAMPLES / (int64_t)samples;
            sq = sq * (int64_t)DC_SERVO_MIN_SAMPLES / (int64_t)samples;
            neg[0] = (uint32_t)((uint64_t)neg[0] * DC_SERVO_MIN_SAMPLES / samples);
            neg[1] = (uint32_t)((uint64_t)neg[1] * DC_SERVO_MIN_SAMPLES / samples);
            samples = DC_SERVO_MIN_SAMPLES;
        }
        int dc[2];
        if (!dc_servo_mean(si, sq, samples, dc)) continue;
        const int pneg[2] = {dc_servo_pneg(neg[0], samples), dc_servo_pneg(neg[1], samples)};
        if (g == s_current_gain) { s_servo_dc[0] = dc[0]; s_servo_dc[1] = dc[1]; s_servo_dc_us = esp_timer_get_time(); }
        if (s_low_slot_row[st] != g) { dc_servo_forget_step(&s_servo, slot); s_low_slot_row[st] = (uint8_t)g; }
        int step[2];
        dc_servo_decide_sign(&s_servo, slot, dc, pneg, step);
        for (int a = 0; a < 2; ++a) {
            if (s_low_trim[st][a] + step[a] > DC_TRIM_MAX_TEMPLATE) step[a] = DC_TRIM_MAX_TEMPLATE - s_low_trim[st][a];
            if (s_low_trim[st][a] + step[a] < -DC_TRIM_MAX_TEMPLATE) step[a] = -DC_TRIM_MAX_TEMPLATE - s_low_trim[st][a];
        }
        if (!step[0] && !step[1]) continue;
        int applied[2] = {step[0], step[1]};
        bool fault = false;
        if (g == s_current_gain && phy_dco_held() && !servo_write_active((uint8_t)g, step, applied, &fault)) {
            if (!fault) { ++s_servo_skip[3]; continue; }
            s_dc_fault = true;
            (void)receiver_pause("DC_LIVE_RESTORE_FAILED");
            printf("DC_SERVO fault=restore_failed gain=%u (low stage %d); gain owner paused\n", g, st + (int)DC_LOW_STAGE_FIRST);
            const uint8_t code = 1;
            flightlog_event(FLOG_E_FAULT, &code, 1);
            return;
        }
        s_low_trim[st][0] = (int16_t)(s_low_trim[st][0] + applied[0]);
        s_low_trim[st][1] = (int16_t)(s_low_trim[st][1] + applied[1]);
        for (unsigned r = DC_LOW_ROW0; r < s_dco.lo && r < DC_SERVO_ROW0; ++r)
            if (dc_low_stage(r) == st) receiver_dc_reset_row((uint8_t)r);
        dc_servo_applied(&s_servo, slot, dc, applied);
        ++s_low_steps;
        ++s_servo_writes;
    }
}

void servo_service(void)
{
    if (!s_servo_ready) { dc_servo_init(&s_servo); s_servo_ready = true; }
    if (experimental_transfer_active() || s_dc_fault || s_startup || !quiet_owner() || rf_lane() != RF_LANE_ULTRAFINE) {
        ++s_servo_skip[0];
        return;
    }
    /* A sampling phase that reads mixed IQ bits turns every mean into noise
     * of its own: board 2026-10-26, one boot in five came up unverified
     * (excess 1883 against 300) and the servo walked I to its -96 limit with
     * the IQ fully clipped. Correct DC only on a settled phase; F repairs it.
     * NOISE_LIMITED is no phase fault (every position reads the same noise). */
    if (!calibration_sphase_settled()) {
        if (!s_servo_phase_hold) dc_servo_forget_all(&s_servo);
        s_servo_phase_hold = true;
        ++s_servo_skip[2];
        return;
    }
    s_servo_phase_hold = false;
    /* Another channel, offset or table since the last tick: no row's last
     * step is evidence for its next mean (dco_select restarts the sums). */
    if (s_servo_ctx.control != s_control_epoch || s_servo_ctx.reloads != s_dco_reloads ||
        s_servo_ctx.freq != rf_frequency_mhz() || s_servo_ctx.key != live_filter_key()) {
        dc_servo_forget_all(&s_servo);
        s_servo_ctx.control = s_control_epoch; s_servo_ctx.reloads = s_dco_reloads;
        s_servo_ctx.freq = rf_frequency_mhz(); s_servo_ctx.key = live_filter_key();
    }
    if (s_dco.freq != rf_frequency_mhz()) dco_select();
    if (s_dco.freq != rf_frequency_mhz()) { ++s_servo_skip[1]; return; }
    for (unsigned r = s_dco.lo; r <= s_dco.hi && r <= ARC_VENDOR_GAIN_MAX; ++r) {
        const uint8_t g = (uint8_t)r;
        if (g < DC_SERVO_ROW0) continue;
        const unsigned sr = g - DC_SERVO_ROW0;   /* servo state row */
        int64_t si, sq;
        uint32_t samples, neg[2];
        int dc[2];
        /* A saturated sign share is decided on DC_SERVO_FAST_SAMPLES: a row
         * on a rail is corrected in one short visit, not after 40000 samples
         * that a row hopped through at the edge rarely collects. */
        uint32_t need = DC_SERVO_MIN_SAMPLES, peek_neg[2], peek_n;
        if (receiver_dc_peek_row(g, peek_neg, &peek_n) && peek_n >= DC_SERVO_FAST_SAMPLES &&
            (dc_servo_saturated(dc_servo_pneg(peek_neg[0], peek_n)) || dc_servo_saturated(dc_servo_pneg(peek_neg[1], peek_n))))
            need = DC_SERVO_FAST_SAMPLES;
        if (!receiver_dc_take_row(g, need, &si, &sq, neg, &samples)) continue;
        if (samples < DC_SERVO_MIN_SAMPLES) {     /* dc_servo_mean wants the full count: scale */
            si = si * (int64_t)DC_SERVO_MIN_SAMPLES / (int64_t)samples;
            sq = sq * (int64_t)DC_SERVO_MIN_SAMPLES / (int64_t)samples;
            neg[0] = (uint32_t)((uint64_t)neg[0] * DC_SERVO_MIN_SAMPLES / samples);
            neg[1] = (uint32_t)((uint64_t)neg[1] * DC_SERVO_MIN_SAMPLES / samples);
            samples = DC_SERVO_MIN_SAMPLES;
            ++s_servo_fast;
        }
        if (!dc_servo_mean(si, sq, samples, dc)) continue;
        const int pneg[2] = {dc_servo_pneg(neg[0], samples), dc_servo_pneg(neg[1], samples)};
        ++s_servo_estimates;
        if (g == s_current_gain) {
            s_servo_dc[0] = dc[0]; s_servo_dc[1] = dc[1]; s_servo_dc_us = esp_timer_get_time();
            const int lim = dc_trim_max(g);
            for (int a = 0; a < 2; ++a)
                if (abs(dc[a]) > DC_FAULT_DC_MCELLS && abs(dco_offset(g, (unsigned)a)) >= lim) dc_fault_mark(2u, g);
        }
        if (s_dco.e[g].valid != DCO_ROW_OK || s_dco_banned[g]) {
            ++s_servo_skip[1];
            dc_servo_forget_step(&s_servo, sr);
            continue;
        }
        const bool carrier = video_recent();
        if (dco_mask_get(s_dco_templ, g) && abs(dc[0]) <= DC_ADOPT_MCELLS && abs(dc[1]) <= DC_ADOPT_MCELLS) {
            if (++s_adopt_run[g] >= (carrier ? DC_ADOPT_RUN_CARRIER : DC_ADOPT_RUN)) {
                /* The trim goes into the row's codes; the model keeps its
                 * offset and what its template neighbours inherit (no more
                 * pinning every row: that stopped all sharing). */
                for (int a = 0; a < 2; ++a) {
                    s_dco.e[g].code[a] = (int16_t)dco_trimmed(g, (unsigned)a);
                    s_dco.e[g].residual_mcells[a] = (int16_t)dc[a];
                }
                dc_model_adopt(&s_dc_model, mrow(g), true);
                const bool was_preset = dco_mask_get(s_dco_preset, g);
                dco_row_measured(g);
                dco_mask_put(s_dco_preset, g, false);
                s_dco_dirty = true;
                s_adopt_run[g] = 0;
                ++s_dco_adopted;
                printf("DCO row=%u adopted freq=%u codes=%d/%d from=%s on=%s (live servo)\n", g, s_dco.freq,
                       s_dco.e[g].code[0], s_dco.e[g].code[1], was_preset ? "base_map" : "template",
                       carrier ? "carrier" : "noise");
            }
        } else {
            s_adopt_run[g] = 0;
        }
        const bool templ = dco_mask_get(s_dco_templ, g);
        dc_model_note(&s_dc_model, mrow(g), dc, pneg[0] < DC_SERVO_PNEG_LO || pneg[0] > DC_SERVO_PNEG_HI ||
                                          pneg[1] < DC_SERVO_PNEG_LO || pneg[1] > DC_SERVO_PNEG_HI);
        int step[2];
        dc_servo_decide_sign(&s_servo, sr, dc, pneg, step);
        const int base[2] = {dco_offset(g, 0), dco_offset(g, 1)};
        for (int a = 0; a < 2; ++a) {
            const int lim = dc_trim_max(g);
            if (base[a] + step[a] > lim) step[a] = lim - base[a];
            if (base[a] + step[a] < -lim) step[a] = -lim - base[a];
        }
        if (!step[0] && !step[1]) continue;
        int applied[2] = {step[0], step[1]};
        bool fault = false;
        /* The active, held row is written now; any other row takes its new
         * codes at its next gain write (dco_post_gain). */
        if (g == s_current_gain && phy_dco_held() && !servo_write_active(g, step, applied, &fault)) {
            if (!fault) { ++s_servo_skip[3]; continue; }
            s_dc_fault = true;
            (void)receiver_pause("DC_LIVE_RESTORE_FAILED");
            printf("DC_SERVO fault=restore_failed gain=%u; gain owner paused\n", g);
            const uint8_t code = 1;
            flightlog_event(FLOG_E_FAULT, &code, 1);
            return;
        }
        const bool shared = dc_model_apply(&s_dc_model, mrow(g), applied, templ);
        servo_rows_changed(g, shared);
        dc_servo_applied(&s_servo, sr, dc, applied);
        ++s_servo_writes;
    }
    servo_low_rows();
}
static cal_persist_t s_dc_persist;

uint32_t health_context(void)
{
    /* Epochs cover gain/DC/retunes. TX ownership matters because the menu
     * raster and live output can produce different self-noise. */
    uint32_t h = 2166136261u;
    const uint32_t part[] = {s_gain_epoch, s_control_epoch, phy_generation(), rf_table_generation(),
        rf_frequency_mhz(), (uint32_t)rf_offset_khz(), live_filter_key(), s_current_gain,
        menu_splash_active(), s_menu_active};
    for (unsigned k = 0; k < sizeof(part) / sizeof(part[0]); ++k) h = (h ^ part[k]) * 16777619u;
    return h;
}

void drift_service(void)
{
    s_dc_urgent = false;
    if (s_startup || s_scan_active || receiver_paused() || phy_busy() ||
        (s_menu_active && !menu_splash_active()) || rf_lane() != RF_LANE_ULTRAFINE) return;
    const int64_t now = esp_timer_get_time();
    static int64_t next_us;
    if (now < next_us || now - s_last_gain_write_us < 20000) return;
    next_us = now + 250000;
    const uint32_t context = health_context();
    window_stats_t w;
    const bool copied = collect_stats(24, &w, true);
    if (context != health_context()) { ++s_health_rejected; return; }
    const int mismatch = phy_filter_hw_mismatch(NULL, NULL);
    if (context != health_context()) { ++s_health_rejected; return; }
    const double m2 = w.windows ? w.noise_m2 / w.windows : 0;
    const double kurt = m2 > 0 && w.windows ? (w.noise_m4 / w.windows) / (m2 * m2) : 0;
    const bool noise = w.envelope_x100 <= dsp_quiet_limit(w.m.clip_pm) && kurt >= 1.85 && kurt <= 2.25;
    const bool carrier = w.dc_circle && video_recent();
    /* Noise needs the same >=3 cells/axis with >=5% occupancy as the
     * quiet DC search. Requiring a full cell of variance would miss the
     * known low-gain DC park (3.7-cell offset, 0.68-cell noise, #223). */
    const bool qualified = mismatch > 0 || (copied && w.windows == 24u &&
        ((noise && w.dc_noise_dither) || (carrier && w.dc_qualified)));
    if (qualified) { ++s_health_blocks; s_drift_avg[0] = w.dc_i; s_drift_avg[1] = w.dc_q; }
    else ++s_health_rejected;
    s_basemap_context = context;
    s_health_us += esp_timer_get_time() - now;
    const bool watch = cal_watch_add(&s_basemap_watch, context, now, qualified,
                                    mismatch > 0 ? CAL_REASON_FILTER : 0u, w.dc_i, w.dc_q);
    /* The live servo corrects DC: stable bad DC recommends calibration only
     * where it cannot (trim limit, no row, fault) or after it failed for
     * CAL_SERVO_GIVEUP_US on this channel and filter. */
    const bool persistent = cal_persist_add(&s_dc_persist, (uint32_t)rf_frequency_mhz() << 16 | live_filter_key(), now,
                                            qualified && mismatch <= 0, w.dc_i, w.dc_q);
    const bool warn = (watch && (mismatch > 0 || !dc_servo_can_fix(s_current_gain))) || persistent;
    if (warn) {
        const bool already = s_cal_notice && s_cal_notice_freq == rf_frequency_mhz() && s_cal_notice_key == live_filter_key();
        s_cal_notice = true;
        s_cal_notice_freq = rf_frequency_mhz(); s_cal_notice_key = live_filter_key(); s_cal_notice_gain = s_current_gain;
        if (!already) printf("CAL_NEEDED reason=%s dc_mcells=%d/%d action=MENU_CALIBRATE auto_cal=0\n",
                             mismatch > 0 ? "filter_readback" : !calibration_sphase_settled() ? "sampling_phase" :
                             persistent ? "dc_servo_failed" : "dc_servo_limit",
                             w.dc_i, w.dc_q);
    } else if (s_basemap_watch.state == CAL_GOOD && s_cal_notice && s_cal_notice_gain == s_current_gain &&
               s_cal_notice_freq == rf_frequency_mhz() && s_cal_notice_key == live_filter_key()) {
        s_cal_notice = false;
        printf("CAL_HEALTH state=good recommendation=cleared auto_cal=0\n");
    }
    /* A high absolute HF statistic alone is intrinsic noise or phase mixing,
     * not enough evidence to label the base map bad. No automatic slips.
     * A clean block proves the phase (a mixing read cannot read low); a high
     * one does not disprove it, so it never demotes the verdict: receiver
     * noise on the top rows reads ~725 on a clean position (limit 300), and
     * every block there used to mark the phase unverified, which stopped the
     * DC servo on exactly the rows of the range edge (blackbox session 286:
     * G83 without a carrier, 2.4-4.2 cells of DC for 12 s, 0 servo writes,
     * until a gain change). Demotion is a scan's verdict (phase_service,
     * sphase_qualify, the carrier scans). */
    if (copied && w.windows == 24u && !menu_splash_active()) {
        (void)sphase_record(&w);
        if (s_sphase_excess < (carrier ? SPHASE_LIVE_LIMIT : SPHASE_LIMIT)) s_sphase_state = "settled";
    }
}


/* C/N-AUTO noise map (cn_auto.h): with no carrier, the noise power of the
 * rows 3 apart from the table maximum down to G54, ~25 ms each, every boot
 * (also on a channel quiet now but measured before: temperature and drift).
 * Startup only, never in flight; skipped with a carrier on the channel. */
void noise_map_startup(int64_t deadline)
{
    if (!noise_like()) { printf("CN_NOISE startup skip=carrier_or_not_quiet (map from NVS / learned later)\n"); return; }
    if (!receiver_pause("CNMAP")) return;
    const uint8_t restore = s_current_gain;
    unsigned rows = 0, rejected = 0;
    float measured[DG_STATES] = {0};   /* the gain table (receiver_gain_table_measured) */
    /* Every row while the noise is real (it is the gain measurement of the
     * edge rows, gain.c dg_apply_measured_gain), every 3rd below. */
    bool fine = true;
    for (int g = (int)receiver_gain_top(); g >= 54 && esp_timer_get_time() < deadline; g -= fine ? 1 : 3) {
        const unsigned ck = (unsigned)g - NOISE_CACHE_LO;
        if (ck < NOISE_CACHE_ROWS && s_noise_cache[ck].time &&
            esp_timer_get_time() - s_noise_cache[ck].time <= 30000000LL &&
            s_noise_cache[ck].freq == rf_frequency_mhz() && s_noise_cache[ck].key == live_filter_key() &&
            (s_noise_cache[ck].m2 < CNA_QLIM_CELLS2 ||
             (s_noise_cache[ck].kurt >= 1.85f && s_noise_cache[ck].kurt <= 2.25f))) {
            const float m2 = s_noise_cache[ck].m2;
            receiver_cn_noise_store((uint8_t)g, m2);
            if (g < DG_STATES) measured[g] = m2;
            ++rows;
            if (m2 < CNA_QLIM_CELLS2) fine = false;
            printf("CN_NOISE row=%d noise_cells2=%.3f source=dc_verify\n", g, (double)m2);
            continue;
        }
        receiver_set_gain((uint8_t)g);
        vTaskDelay(pdMS_TO_TICKS(20));
        cna_sums_t s = {0};
        uint8_t buf[PROBE_BYTES];
        double m2s = 0.0, m4s = 0.0;
        unsigned got = 0;
        for (unsigned tries = 0; tries < 48u && got < 24u; ++tries) {
            vTaskDelay(1);
            if (!copy_probe_window(buf, NULL)) continue;
            cna_sums_t p = {0};
            for (unsigned k = 0; k < sizeof(buf); ++k) cna_sums_add(&p, buf[k]);
            float mi, mq;
            cna_mean(&p, &mi, &mq);
            double m2, m4;
            cna_moments(&p, mi, mq, &m2, &m4);
            m2s += m2; m4s += m4;
            cna_sums_merge(&s, &p);
            ++got;
        }
        if (got < 8u) continue;
        /* Noise only (Gaussian: M4 ~ 2 M2^2); a row below one 4-bit step
         * reads ~0.25-0.5 cells^2 and is kept (cn_auto marks it
         * quantisation-limited). */
        const double m2 = m2s / got, kurt = (m4s / got) / (m2 * m2);
        /* Receiver noise reads 1.88-2.13 on a clean channel (G83 folds a
         * little: ~1.9); a carrier lowers it, bursts raise it: B2 (5752 MHz,
         * next to Wi-Fi 149/151) read 2.3-14 on 2026-10-20, and row 72 came
         * out above row 73. Neither is the receiver's noise. */
        if (m2 >= CNA_QLIM_CELLS2 && (kurt < 1.85 || kurt > 2.25)) {
            ++rejected;
            printf("CN_NOISE row=%d rejected kurtosis=%.2f (carrier or interference)\n", g, kurt);
            continue;
        }
        float mi, mq;
        cna_mean(&s, &mi, &mq);
        double M2, M4;
        cna_moments(&s, mi, mq, &M2, &M4);
        receiver_cn_noise_store((uint8_t)g, (float)M2);
        if (g < DG_STATES) measured[g] = (float)M2;
        ++rows;
        if (M2 < CNA_QLIM_CELLS2) fine = false;
        printf("CN_NOISE row=%d noise_cells2=%.3f kurtosis=%.2f\n", g, M2, kurt);
    }
    receiver_set_gain(restore);
    receiver_resume();
    /* The gain table only from a clean pass (the stored one stays otherwise):
     * any rejected row above the 4-bit step means interference or a carrier. */
    if (rejected == 0u) receiver_gain_table_measured(measured);
    else printf("CN_NOISE gain_table=kept (rejected rows on this channel)\n");
    receiver_cn_noise_save();
    printf("CN_NOISE startup rows=%u rejected=%u (per-row noise for C/N-AUTO)\n", rows, rejected);
}

/* One-sided clipping seen by the gain observer: run the DC servo now. */
/* Woken early, the servo still decides on the full DC_SERVO_MIN_SAMPLES of
 * the row (they accumulate continuously): faster, never noisier. Deciding
 * on 10000 samples (~8 ms) chased the flicker noise (board 2026-10-19: 804
 * wakes, 335 steps, 25 sign rejects in a minute without a VTX). */
void calibration_dc_urgent(void)
{
    static int64_t last;
    const int64_t now = esp_timer_get_time();
    if (s_dc_urgent || now - last < 100000) return;
    last = now;
    s_dc_urgent = true;
    if (s_cal_task) xTaskNotifyGive(s_cal_task);
}

static void cal_dump_table(cal_dump_t *d, const dco_table_t *t, const dco_mask_t templ, int slot)
{
    const unsigned key = dco_filter_key(t->filter_code, t->filter_skirt, t->filter_bw20);
    for (unsigned g = t->lo; g <= t->hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
        cal_dump_line(d, "{\"type\":\"dc_row\",\"slot\":%d,\"freq_mhz\":%u,\"filter_key\":%u,"
            "\"code\":%d,\"skirt\":%d,\"bw20\":%u,\"iq_scale\":%u,\"table_version\":%u,"
            "\"gain\":%u,\"i\":%d,\"q\":%d,\"residual_i_mcells\":%d,\"residual_q_mcells\":%d,"
            "\"valid\":%u,\"template\":%s}", slot, t->freq, key, t->filter_code, t->filter_skirt,
            t->filter_bw20, t->iq_scale_sel, t->version, g, t->e[g].code[0], t->e[g].code[1],
            t->e[g].residual_mcells[0], t->e[g].residual_mcells[1], t->e[g].valid,
            dco_mask_get(templ, g) ? "true" : "false");
}
void calibration_flog_live(int dc[2], int off[2], uint32_t *servo_writes, uint8_t *sphase, int8_t *temp_c)
{
    /* The active row's DC as the servo measured it (>= 4096 observer samples,
     * the means of every settled probe), while fresh. The health block's
     * circle fit is kept only as the fallback: on a weak carrier it read
     * -4.15 cells for 9 s where the IQ held -0.09 (blackbox session 249). */
    const bool fresh = s_servo_dc_us && esp_timer_get_time() - s_servo_dc_us < 2000000;
    dc[0] = fresh ? s_servo_dc[0] : s_drift_avg[0];
    dc[1] = fresh ? s_servo_dc[1] : s_drift_avg[1];
    off[0] = dco_offset(s_current_gain, 0); off[1] = dco_offset(s_current_gain, 1);
    *servo_writes = s_servo_writes;
    *sphase = sphase_code();
    *temp_c = (int8_t)(s_temp_c <= -99.f ? -128 : s_temp_c > 127.f ? 127 : (int)s_temp_c);   /* -128: not read yet */
}


/* Discard every DC table of an earlier calibration generation
 * (calibration_generation), then store the empty index. */
void cal_dc_discard(void)
{
    dco_index_init(&s_dco_index);
    s_dco_index_loaded = true;                 /* never adopt or upgrade the old tables */
    memset(s_dco_ready, 0, sizeof(s_dco_ready));
}

bool cal_dc_store_discarded(void)
{
    return blob_store("dco_idx", &s_dco_index, sizeof(s_dco_index));
}

/* Flight-log calibration record, bytes 30..48 and 50..54: DC state. */
void cal_dc_record(uint8_t *p)
{
    for (unsigned k = 0; k < 3u; ++k) {
        uint8_t *e = p + 30 + k * 5;
        flog_put16(e, s_dco_ready[k].freq); flog_put16(e + 2, s_dco_ready[k].key); e[4] = s_dco_ready[k].ready;
    }
    p[45] = (uint8_t)dco_valid_rows(); p[46] = (uint8_t)dco_rows(0); p[47] = (uint8_t)dco_template_rows();
    p[48] = 0;   /* repair flags (retired; the byte stays for the record layout) */
    flog_put16(p + 50, (uint16_t)(int16_t)s_drift_avg[0]); flog_put16(p + 52, (uint16_t)(int16_t)s_drift_avg[1]);
    p[54] = (uint8_t)(int8_t)(s_temp_c < -100.f ? -128 : (int)s_temp_c);
}

/* Report (calibration_dump): the active table, then the saved slots. */
void cal_dc_dump_tables(cal_dump_t *d)
{
    /* -1 is the active table, slots 0..5 are saved observations. They may
     * duplicate it: consumers group by board/context/gain, not row count. */
    cal_dump_table(d, &s_dco, s_dco_templ, -1);
    dco_table_t table;
    for (unsigned k = 0; k < DCO_SLOTS; ++k)
        if (s_dco_index_loaded && s_dco_index.freq[k] && dco_load_slot(k, &table))
            cal_dump_table(d, &table, s_dco_index.templ[k], (int)k);
}

void cal_dc_print(rf_filter_t f)
{
    const unsigned valid = dco_valid_rows();
    printf("DCO held=%u gains_valid=%u floor_rows=%u missing_rows=%u floor=%u searches=%lu carrier_searches=%lu hook_holds=%lu aborts=%lu "
           "drift_avg_mcells=%d/%d nudges=%lu temp_c=%.1f\n", phy_dco_held(), valid, dco_rows(DCO_ROW_FLOOR), dco_rows(0),
           s_dco_floor,
           (unsigned long)s_dco_searches, (unsigned long)s_dco_carrier_searches, (unsigned long)s_dco_hook_holds,
           (unsigned long)s_dco_aborts, s_drift_avg[0], s_drift_avg[1], (unsigned long)s_servo_writes,
           (double)s_temp_c);
    {
        int b1[2], b2[2];
        const uint8_t g = s_current_gain;
        int pin[2], moved[2];
        const uint32_t overrides = phy_dco_bank1_status(pin, moved);
        if (phy_dco_banks(b1, b2))
            printf("DCO_BANKS bank1=%d/%d bank2=%d/%d gain=%u row_codes=%d/%d row_valid=%u banned=%u "
                   "bank1_pin=%d/%d vendor_moved_to=%d/%d overrides=%lu\n", b1[0], b1[1],
                   b2[0], b2[1], g, g <= ARC_VENDOR_GAIN_MAX ? s_dco.e[g].code[0] : -1,
                   g <= ARC_VENDOR_GAIN_MAX ? s_dco.e[g].code[1] : -1,
                   g <= ARC_VENDOR_GAIN_MAX ? s_dco.e[g].valid : 0u, g <= ARC_VENDOR_GAIN_MAX ? s_dco_banned[g] : 0u,
                   pin[0], pin[1], moved[0], moved[1], (unsigned long)overrides);
        rf_bank1_check(true);
        printf("DC_FAULT live=%u reason=%s gain=%u age_ms=%lld episodes=%lu\n", dc_fault_live(),
               dc_fault_name(s_dcf_reason), s_dcf_gain,
               s_dcf_us ? (long long)((esp_timer_get_time() - s_dcf_us) / 1000) : -1LL, (unsigned long)s_dcf_episodes);
    }
    printf("CAL_POLICY auto_cal=0 auto_dc_trim=1 manual=F full=B health_cpu_us=%lld context=%lu\n",
           s_health_us, (unsigned long)s_basemap_context);
    dc_health_print();
    unsigned stored = 0;
    for (unsigned k = 0; k < DCO_SLOTS; ++k) stored += s_dco_index.freq[k] != 0;
    printf("DCO tables freq=%u template_rows=%u seeded_from=%u seeds=%lu stored_channels=%u/%u:", s_dco.freq,
           dco_template_rows(), s_dco_seed_mhz, (unsigned long)s_dco_seeds, stored, DCO_SLOTS);
    for (unsigned k = 0; k < DCO_SLOTS; ++k) if (s_dco_index.freq[k]) printf(" %u", s_dco_index.freq[k]);
    printf("\n");
    printf("DCO_REPAIR bad_dropped=%lu repairs=%lu repair_fails=%lu (unconverged rows dropped, "
           "searched from a converged neighbour)\n",
           (unsigned long)s_dco_bad_dropped, (unsigned long)s_dco_repairs, (unsigned long)s_dco_repair_fails);
    printf("DCO_PRESET boards=%u gains=%u..%u rows=%u holds=%lu (firmware base map in rows without own codes)\n",
           (unsigned)DCO_PRESET_BOARDS, DCO_PRESET_LO, DCO_PRESET_HI,
           dco_mask_count(s_dco_preset, s_dco.lo, s_dco.hi), (unsigned long)s_dco_preset_holds);
    printf("DC_SERVO dc_mcells=%d/%d slope_mcells_per_code=%d/%d known=%u/%u pol_votes=%d/%d offset=%d/%d own_trim=%u "
           "steps=%lu writes=%lu estimates=%lu learned=%lu relearned=%lu sign_rejects=%lu turns=%lu forgotten=%lu "
           "adopted=%lu urgent_wakes=%lu fast=%lu brackets=%lu skip=%lu/%lu/%lu/%lu/%lu fault=%u\n", s_servo_dc[0], s_servo_dc[1],
           s_servo.slope[0], s_servo.slope[1], s_servo.known[0], s_servo.known[1], s_servo.pol_votes[0],
           s_servo.pol_votes[1], dco_offset(s_current_gain, 0), dco_offset(s_current_gain, 1),
           mrow(s_current_gain) < DC_MODEL_ROWS ? (unsigned)s_dc_model.own[mrow(s_current_gain)] : 0u,
           (unsigned long)s_servo.steps, (unsigned long)s_servo_writes, (unsigned long)s_servo_estimates,
           (unsigned long)s_servo.learned, (unsigned long)s_servo.relearned, (unsigned long)s_servo.sign_rejects,
           (unsigned long)s_servo.turns, (unsigned long)s_servo.forgotten, (unsigned long)s_dco_adopted,
           (unsigned long)s_dc_urgent_wakes, (unsigned long)s_servo_fast, (unsigned long)s_servo.brackets,
           (unsigned long)s_servo_skip[0], (unsigned long)s_servo_skip[1],
           (unsigned long)s_servo_skip[2], (unsigned long)s_servo_skip[3], (unsigned long)s_servo_skip[4], s_dc_fault);
    printf("DC_MODEL shared=%d/%d shared_steps=%lu local_steps=%lu settled=%u strikes=%u (dc_servo.h: one DC shift common to "
           "the rows plus a local part per template row)\n", s_dc_model.shared[0], s_dc_model.shared[1],
           (unsigned long)s_dc_model.shared_steps, (unsigned long)s_dc_model.local_steps,
           mrow(s_current_gain) < DC_MODEL_ROWS ? (unsigned)s_dc_model.settled[mrow(s_current_gain)] : 0u,
           mrow(s_current_gain) < DC_MODEL_ROWS ? (unsigned)s_dc_model.strikes[mrow(s_current_gain)] : 0u);
    {
        const int st = dc_low_stage(s_current_gain);
        printf("DC_LOW on=%u holds=%lu steps=%lu read_fails=%lu stage=%d trim=%d/%d base=%d/%d (rows G30-G53 under the DC table: "
               "the chip's own codes plus one trim per RF stage; EXPERIMENTAL > LOW DC)\n",
               !experimental_get(EXP_LOW_DC), (unsigned long)s_low_holds, (unsigned long)s_low_steps,
               (unsigned long)s_low_read_fails, st >= 0 ? st + (int)DC_LOW_STAGE_FIRST : -1, st >= 0 ? s_low_trim[st][0] : 0, st >= 0 ? s_low_trim[st][1] : 0,
               st >= 0 && low_base_ok(s_current_gain) ? s_low_base[low_idx(s_current_gain)][0] : -1,
               st >= 0 && low_base_ok(s_current_gain) ? s_low_base[low_idx(s_current_gain)][1] : -1);
    }
    printf("DCO_BAN unbans=%lu deferred_to_servo=%lu (a hold is released for 2-32 s only where the servo cannot trim it)\n",
           (unsigned long)s_dco_unbans, (unsigned long)s_dco_guard_deferred);
    printf("DCO_ADOPT disabled=manual_only\n");
    for (unsigned g = s_dco.lo; g <= s_dco.hi && g <= ARC_VENDOR_GAIN_MAX; ++g)
        printf("CAL_ROW freq=%u filter_key=%04x gain=%u codes=%d/%d residual_mcells=%d/%d valid=%u template=%u\n",
               s_dco.freq, live_filter_key(), g, s_dco.e[g].code[0], s_dco.e[g].code[1],
               s_dco.e[g].residual_mcells[0], s_dco.e[g].residual_mcells[1], s_dco.e[g].valid,
               dco_mask_get(s_dco_templ, g));
    printf("DCO_CONTEXT table_freq=%u table_bw20=%u table_code=%d table_skirt=%d "
           "active_freq=%u active_bw20=%u active_code=%d active_skirt=%u filter_same=%u\n",
           s_dco.freq, (unsigned)s_dco.filter_bw20, (int)s_dco.filter_code, (int)s_dco.filter_skirt,
           rf_frequency_mhz(), (unsigned)f.bw20, f.code, (unsigned)f.skirt,
           (unsigned)(s_dco.filter_bw20 == f.bw20 && s_dco.filter_code == f.code &&
                      s_dco.filter_skirt == (int)f.skirt));
}

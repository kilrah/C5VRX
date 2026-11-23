/* C5VRX-5 by Twotoz and the C5VRX contributors. */
#include "experimental.h"
#include "receiver.h"
#include "phy.h"
#include <stdio.h>
#include <string.h>
#include "esp_system.h"

static experimental_blob_t s_exp = {.version = EXP_VERSION};
static volatile bool s_save_pending;
static uint32_t s_failed, s_revision;
static bool s_clock_d10;
bool experimental_clock_d10(void) { return s_clock_d10; }
uint32_t experimental_revision(void) { return s_revision; }

uint8_t experimental_get(unsigned item) { return item < EXP_ITEMS ? s_exp.value[item] : 0; }
unsigned experimental_log_interval(void)
{
    const uint8_t v = experimental_get(EXP_LOG_CADENCE);
    return v == 2 ? 0 : v == 1 ? 5 : 1;
}
static bool save(void)
{
    experimental_blob_t old;
    if (blob_load("exp", &old, sizeof(old)) && !memcmp(&old, &s_exp, sizeof(old))) return true;
    return blob_store("exp", &s_exp, sizeof(s_exp));
}
void experimental_load(void)
{
    experimental_blob_t b = {.version = EXP_VERSION};
    /* Items are appended, never reordered: an older, shorter blob keeps its
     * experiments and the newer items start on their baseline (before LOW
     * DC: one byte shorter; before LEVEL TOP: two; before STRONG LANE: three;
     * before AUTO OUTPUT: four; before Clock Pin D10: five). */
    for (unsigned missing = 0; missing <= 5u; ++missing) {
        memset(&b, 0, sizeof(b));
        if (blob_load("exp", &b, sizeof(b) - missing)) break;
        b = (experimental_blob_t){.version = EXP_VERSION};
    }
    /* Version 1 had AUTO OUTPUT as ON/OFF with ON the baseline: a stored 0
     * cannot tell the default from a choice, so it starts OFF, the new
     * baseline (an explicit OFF stays OFF). Saved by the next poll. */
    if (b.version == EXP_VERSION_V1) {
        b.version = EXP_VERSION;
        b.value[EXP_OUTPUT_AUTO] = 0;
        if (experimental_blob_valid(&b)) s_save_pending = true;
    }
    if (experimental_blob_valid(&b)) s_exp = b;
#ifdef HWX_NO_DAC
    s_exp.value[EXP_CLOCK_D10] = 0;   /* D10 is a hardware experiment pad */
#endif
    s_clock_d10 = experimental_get(EXP_CLOCK_D10) != 0;
}
bool experimental_phy_restore(void)
{
    bool ok = true;
    static const unsigned items[] = {EXP_IQ_CORR, EXP_LNA_BIAS, EXP_CURRENT, EXP_LOOP_CAP, EXP_SPUR0};
    for (unsigned i = 0; i < sizeof(items) / sizeof(items[0]); ++i) {
        const unsigned id = items[i];
        if (!experimental_phy_option(id, experimental_get(id))) { experimental_fault(id); ok = false; }
    }
    return ok;
}
void experimental_fault(unsigned item)
{
    const uint8_t e[] = {(uint8_t)item, experimental_get(item), 0, 0};
    ++s_revision;
    s_exp.value[item] = 0;
    s_failed |= 1u << item;
    s_save_pending = true;
    flightlog_event(FLOG_E_EXP, e, sizeof(e));
}
void experimental_poll(void)
{
    if (s_save_pending) { s_save_pending = false; if (!save()) s_save_pending = true; }
}
bool experimental_phy_failed(void) { return s_failed != 0; }
unsigned experimental_boot_lane(void)
{
    const unsigned v = experimental_get(EXP_IQ_LANE);
    return v == 1 ? RF_LANE_FINE : v == 2 ? RF_LANE_COARSE : RF_LANE_ULTRAFINE;
}
/* A lane STRONG LANE selected live (receiver.c) is no IQ LANE experiment and
 * needs no reboot; the transfer is still not the calibrated one. */
static bool lane_experiment(void) { return rf_lane() != RF_LANE_ULTRAFINE && !receiver_strong_lane(); }
bool experimental_reboot_needed(void)
{
    return (rf_lane() != experimental_boot_lane() && !receiver_strong_lane()) || s_clock_d10 != (experimental_get(EXP_CLOCK_D10) != 0);
}
bool experimental_set(unsigned item, uint8_t value)
{
    if (item >= EXP_ITEMS || !experimental_items[item].count || value >= experimental_items[item].count) return false;
#ifdef HWX_NO_DAC
    if (item == EXP_CLOCK_D10 && value) return false;
#endif
    const uint8_t old = item == EXP_AUTO_PROFILE ? profile_live_mode() : s_exp.value[item];
    if (old == value && s_exp.value[item] == value) { s_failed &= ~(1u << item); return true; }
    bool ok = true, locked = false;
    if (item == EXP_IQ_CORR || (item >= EXP_LNA_BIAS && item <= EXP_SPUR0)) {
        if (!phy_try_actuator(phy_generation())) return false;
        locked = true;
        phy_observation_changed();
        ok = experimental_phy_option(item, value);
    }
    if (!ok) experimental_fault(item);
    if (ok) {
        ++s_revision;
        s_exp.value[item] = value;
        s_failed &= ~(1u << item);
        if (item == EXP_AUTO_PROFILE) profile_set_live_mode(value != 0);
        ++s_control_epoch;   /* invalidate observations/history before another decision */
    }
    /* Publish the choice before releasing PHY ownership: an observer gain
     * write must never reapply the previous option in between. */
    if (locked) phy_end_actuator();
    if (ok) {
        receiver_dc_reset_all();
        if (!save()) printf("EXP persistence failed; value active for this boot\n");
    }
    const uint8_t e[] = {(uint8_t)item, old, ok ? value : 0, ok};
    flightlog_event(FLOG_E_EXP, e, sizeof(e));
    printf("EXP item=%s old=%u new=%u ok=%u\n", experimental_items[item].name, old, value, ok);
    return ok;
}
void experimental_reset(void)
{
    for (unsigned i = EXP_IQ_CORR; i < EXP_ITEMS; ++i) (void)experimental_set(i, 0);
}
void experimental_snapshot(uint8_t values[EXP_ITEMS])
{
    memcpy(values, s_exp.value, EXP_ITEMS);
    values[EXP_GAIN] = receiver_gain_mode();
    values[EXP_DFILT] = (uint8_t)phy_dfilt_mode();
    values[EXP_AFC] = !s_afc_enabled;
    values[EXP_DEMOD] = transport_demod();
    values[EXP_AUTO_PROFILE] = profile_live_mode();
}
uint32_t experimental_mask(void)
{
    uint8_t v[EXP_ITEMS];
    experimental_snapshot(v);
    uint32_t mask = 0;
    for (unsigned i = 0; i < EXP_ITEMS; ++i) {
        const unsigned baseline = i == EXP_GAIN ? GAIN_MODE_DEFAULT : i == EXP_DFILT ? PHY_DFILT_MODE_DEFAULT :
                                  i == EXP_DEMOD ? TRANSPORT_DEMOD_DEFAULT : 0;
        /* U80P64-R1 is the default decoder's own deviation fallback (control.c), not an experiment. */
        if (v[i] != baseline && !(i == EXP_DEMOD && v[i] == TRANSPORT_DEMOD_P64)) mask |= 1u << i;
    }
    if (lane_experiment()) mask |= 1u << EXP_IQ_LANE;
    if (s_clock_d10) mask |= 1u << EXP_CLOCK_D10;
    return mask;
}
void experimental_effective_snapshot(uint8_t values[EXP_ITEMS])
{
    experimental_snapshot(values);
    values[EXP_CLOCK_D10] = s_clock_d10;
    values[EXP_IQ_LANE] = !lane_experiment() ? 0 : rf_lane() == RF_LANE_FINE ? 1 : 2;
}
void experimental_log_state(void)
{
    ++s_revision;
    uint8_t p[2 + EXP_ITEMS] = {EXP_VERSION, EXP_ITEMS};
    experimental_effective_snapshot(p + 2);
    flightlog_record(FLOG_T_EXP, p, sizeof(p));
}

bool experimental_transfer_active(void)
{
    return experimental_get(EXP_IQ_CORR) || experimental_get(EXP_LNA_BIAS) || experimental_get(EXP_CURRENT) ||
           experimental_get(EXP_LOOP_CAP) || experimental_get(EXP_SPUR0) || rf_lane() != RF_LANE_ULTRAFINE;
}

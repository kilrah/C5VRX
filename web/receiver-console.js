// Passive UI state only. USB/status never controls the realtime video path.
// Parses the C5VRX-5 console (main/console.c, control.c, transport.c,
// profile.c) into the values the web receiver menu shows.

export const BAND_NAMES = {
  R: 'RACEBAND R', A: 'BOSCAM A', B: 'BOSCAM B', E: 'BOSCAM E', F: 'FATSHARK F', L: 'LOWBAND L'
};
// Index 2 remains readable for devices running older firmware.
export const KP_MODES = ['OFF', 'AUTO kp', 'AUTO kp+LP (legacy firmware)'];
// Keys the firmware routes through control.c command(): ignored while a
// calibration/lane-test status screen owns the picture.
export const MENU_COMMAND_KEYS = new Set(['o', 'e', 'v', 'f', 'n', 'x', 'c', 'b', 's', 'g', 'A', 'U', 'G', 'Q', 'Z', 'm']);
// Decoders the current firmware cycles with m (main/receiver.h order).
export const DECODERS = ['U80P64-R1', 'U85Q', 'U50P64-R1', 'D32-R2'];

const FIELDS = /(?:^|\s)(\w+)=([^\s]+)/g;
const fields = (line, target = {}) => {
  for (const match of line.matchAll(FIELDS)) target[match[1]] = match[2];
  return target;
};

export class ReceiverConsole {
  constructor() { this.reset(); }

  reset() {
    this.pending = '';
    this.version = '';
    this.current = false;
    this.calibrationDump = false;
    this.calibrationHealth = "unknown";
    this.directMenu = false;
    this.gainControl = false;
    this.dacTest = false;
    this.decoderControl = false;
    this.filterControl = false;
    this.control = {};
    this.demod = null;
    this.dac = {};
    this.output = {};
    this.gain = {};
    this.heartbeat = {};
    this.profile = {};
    this.scan = { active: false, candidates: 0, result: '', channel: '' };
    this.statusAt = 0;
    this.beatAt = 0;     // last STATUS beat: the firmware reports by itself
    this.busy = false;
    this.busyReported = false;
    this.refused = '';
  }

  // Returns the events in this chunk that make a fresh status report useful:
  // 'ready' (boot line), 'status' (CONTROL received), 'scan-done',
  // 'calibration-done' and 'changed' (a command acknowledged a change).
  push(chunk, now = Date.now()) {
    const events = new Set();
    const lines = (this.pending + chunk).split('\n');
    this.pending = lines.pop().slice(-4096);
    for (const raw of lines) this.line(raw.replace(/\r/g, ''), now, events);
    return [...events];
  }

  line(line, now, events) {
    if (line.startsWith('CAL_HEALTH ')) this.calibrationHealth = fields(line).state || 'unknown';
    if (line.startsWith('CAL_NEEDED ')) this.calibrationHealth = 'bad';
    if (line.startsWith('FIRMWARE ')) this.calibrationDump = fields(line).cal_dump === '1';
    const version = line.match(/^C5VRX5 (?:version=(\S+)|(\S+) ready:)/);
    if (version) {
      this.current = true;
      this.version = version[1] || version[2];
      // fw=<version>+<commit>: PR channels reuse one version string.
      const fw = line.match(/\bfw=(\S+)/);
      if (fw) this.version = fw[1];
      if (version[2]) {
        // Saved settings are restored by firmware; wait for fresh reports.
        this.control = {};
        this.demod = null;
        this.output = {};
        this.gain = {};
        this.directMenu = this.gainControl = this.dacTest = this.decoderControl = this.filterControl = false;
        this.dac = {};
        this.scan = { active: false, candidates: 0, result: '', channel: '' };
        this.refused = '';
        const readyDemod = line.match(/ready: [^,]+, ([^,(]+)/);
        if (readyDemod) this.control.demod = readyDemod[1].trim();
        events.add('ready');
      }
    }
    if (line.startsWith('C5VRX-5 keys:')) this.current = true;
    // The direct-control line moved over firmware versions: v and e shared a
    // line first, now e has its own line after v/f.
    if (/^\s+v video standard AUTO\/NTSC\/PAL, e close menu/.test(line) || /^\s+e close menu \(web menu/.test(line)) this.directMenu = true;
    if (!this.current) return;
    if (/^\s+G RX gain /.test(line)) this.gainControl = true;
    if (/^\s+Z DAC self-test /.test(line)) this.dacTest = true;
    if (/^\s+m decoder /.test(line)) this.decoderControl = true;
    if (/^\s+v video standard [^,]+, f analog filter /.test(line)) this.filterControl = true;

    if (line.startsWith('C5VRX5 version=')) {
      const identity = fields(line);
      if (identity.demod) this.control.demod = identity.demod;
    }
    let switched;
    if ((switched = line.match(/^DEMOD (?:switch to=\S+ result=ok active=|fallback to=)(\S+)/))) {
      this.control.demod = switched[1];
      if (this.demod) this.demod = { ...this.demod, selected: switched[1], verified: switched[1] };
      events.add('changed');
    } else if (line.startsWith('DEMOD switch ')) {
      events.add('changed');
    } else if (line.startsWith('DEMOD ')) {
      const identity = fields(line);
      this.demod = identity;
      if (identity.selected) this.control.demod = identity.selected;
    } else if (line.startsWith('STATUS ')) {
      // Status beat (control.c control_print_beat, every 2 s).
      const beat = fields(line);
      if (beat.output) this.output.level = beat.output;
      if (beat.live !== undefined) this.profile.live = beat.live === '1';
      delete beat.output;
      delete beat.live;
      Object.assign(this.control, beat);
      if (beat.busy !== undefined) this.setBusy(beat.busy === '1', events);
      this.statusAt = this.beatAt = now;
      events.add('status');
    } else if (line.startsWith('CONTROL ')) {
      fields(line, this.control);
      if (this.control.busy !== undefined) this.setBusy(this.control.busy === '1', events);
      this.statusAt = now;
      events.add('status');
    } else if (line.startsWith('OUTPUT level=')) {
      fields(line, this.output);
    } else if (line.startsWith('DACTEST step=')) {
      this.dac = fields(line);
      events.add('changed');
    } else if (line.startsWith('DACTEST pattern')) {
      this.dac = { step: 'PATTERN' };
      events.add('changed');
    } else if (line.startsWith('GAIN ')) {
      fields(line, this.gain);
      if (this.gain.mode) events.add('changed');
    } else if (line.startsWith('PROFILE monitor ')) {
      this.profile.live = fields(line).mode === 'live_switch';
    } else if (line.startsWith('PROFILE active=')) {
      this.profile.active = fields(line).active;
    } else if (line.startsWith('PROFILE select=')) {
      const select = line.match(/^PROFILE select=(.+?) active=(.+?) pending=/);
      if (select) {
        this.profile.select = select[1];
        this.profile.active = select[2];
        delete this.control.mode;
      }
      events.add('changed');
    } else if (line.startsWith('HB ')) {
      const hb = fields(line);
      const wasStartup = this.heartbeat.startup === '1';
      this.heartbeat = hb;
      if (hb.menu !== undefined && this.control.channel) this.control.menu = hb.menu;
      if (wasStartup && hb.startup === '0') events.add('calibration-done');
      if (hb.busy !== undefined) this.setBusy(hb.busy === '1', events);
    } else {
      this.acknowledgement(line, events);
    }
  }

  setBusy(busy, events) {
    if (this.busy && !busy) events.add('calibration-done');
    this.busy = busy;
    this.busyReported = true;
    if (!busy) this.refused = '';
  }

  // Older firmware has no busy field: its startup flag is the best hint.
  receiverBusy() {
    return this.busy || (!this.busyReported && this.heartbeat.startup === '1');
  }

  acknowledgement(line, events) {
    let match;
    if ((match = line.match(/^COMMAND (\S) refused=busy/))) {
      this.busy = true;
      this.refused = match[1];
      events.add('refused');
    } else if ((match = line.match(/^\[CHANNEL\] (\S+) (\d+) MHz/))) {
      Object.assign(this.control, { channel: match[1], freq: match[2], afc_locked: '0', signal: undefined, detected: undefined });
      events.add('changed');
    } else if ((match = line.match(/^OUTPUT manual (\S+) result=applied/))) {
      this.output.level = match[1];
      events.add('changed');
    } else if ((match = line.match(/^KP ladder (OFF|AUTO kp \+ LP|AUTO kp)/))) {
      this.control.kp_auto = match[1] === 'OFF' ? '0' : match[1] === 'AUTO kp' ? '1' : '2';
      events.add('changed');
    } else if ((match = line.match(/^PROFILE live switch (ON|OFF)/))) {
      this.profile.live = match[1] === 'ON';
    } else if (line.startsWith('[MENU] closed')) {
      this.control.menu = '0';
    } else if ((match = line.match(/^\[VIDEO\] locked (\S+) (\d+) MHz (PAL|NTSC)/))) {
      Object.assign(this.control, { channel: match[1], freq: match[2], detected: match[3], afc_locked: '1' });
    } else if (line.startsWith('SCAN candidate ')) {
      if (!this.scan.active) this.scan = { active: true, candidates: 0, result: '', channel: '' };
      this.scan.candidates += 1;
    } else if ((match = line.match(/^SCAN result=(\S+) channel=(\S+)/))) {
      this.scan = { active: false, candidates: this.scan.candidates, result: match[1], channel: match[2] };
      events.add('scan-done');
    } else if (line.startsWith('SCAN refused')) {
      this.scan = { active: false, candidates: 0, result: 'refused', channel: '' };
    }
  }

  // Firmware with the STATUS beat keeps the menu current by itself.
  reportsByItself(now = Date.now()) {
    return this.beatAt !== 0 && now - this.beatAt < 6000;
  }

  // The web menu sent s: SCAN candidate lines only name channels with video.
  startScan() {
    if (this.current) this.scan = { active: true, candidates: 0, result: '', channel: '' };
  }

  // The menu shows what the firmware last reported, nothing inferred.
  view() {
    const c = this.control;
    const band = c.channel ? c.channel.replace(/[0-9]+$/, '') : '';
    const signal = Number.parseInt(c.signal, 10);
    const kp = Number.parseInt(c.kp_auto, 10);
    const scan = this.scan;
    return {
      channel: c.channel || '',
      freq: c.freq || '',
      band: band ? BAND_NAMES[band] || band : '',
      signal: Number.isFinite(signal) ? Math.max(0, Math.min(100, signal)) : null,
      // VIDEO/SEARCH: confirmed sync (STATUS video=); older firmware only
      // reports the AFC lock, which stays 0 over a good unlocked picture.
      locked: c.video !== undefined ? c.video === '1' : c.afc_locked === undefined ? null : c.afc_locked === '1',
      standard: c.standard || '',
      detected: c.detected === undefined ? '' : c.detected === 'none' ? 'NO VIDEO' : c.detected,
      output: this.output.level || '',
      menu: c.menu === undefined ? null : c.menu === '1',
      calibrating: this.heartbeat.startup === '1',
      busy: this.receiverBusy(),
      refused: this.refused,
      stage: this.heartbeat.stage || '',
      gain: this.gain.gain || this.heartbeat.gain || '',
      gainMode: this.gain.mode || '',
      kpMode: Number.isInteger(kp) ? KP_MODES[kp] || '' : '',
      demod: this.demod ? (this.demod.verified && this.demod.verified !== 'unknown'
        ? this.demod.verified : `${this.demod.selected || c.demod || 'UNKNOWN'} (unverified)`) : c.demod || '',
      profile: c.mode || this.profile.active || '',
      filter: this.profile.select || '',
      liveProfile: this.profile.live,
      dacStep: this.dac.step || '',
      dacExpect: this.dac.expect_mv === undefined ? null : Number.parseInt(this.dac.expect_mv, 10),
      scan: scan.active ? `SCANNING · ${scan.candidates} WITH VIDEO`
        : scan.result === 'found' ? `FOUND ${scan.channel}`
          : scan.result === 'none_kept' ? `NO VIDEO FOUND · KEPT ${scan.channel}`
            : scan.result === 'refused' ? 'SCAN REFUSED' : 'READY',
      statusAt: this.statusAt
    };
  }

  supports(key) {
    if (key === 'C' || key === 'B') return this.current && this.calibrationDump;
    if (key === 'G') return this.current && this.gainControl;
    if (key === 'Z') return this.current && this.dacTest;
    if (key === 'm') return this.current && this.decoderControl;
    if (key === 'f') return this.current && this.filterControl;
    if (['K', 'k', 'j', 'P', 'O', 'H', 'U'].includes(key)) return false;
    return ['T', 'h', '?'].includes(key) || (this.current && (!['v', 'e'].includes(key) || this.directMenu));
  }
}

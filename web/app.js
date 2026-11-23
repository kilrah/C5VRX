import { ESPLoader, Transport } from './esptool.js';
import { CalibrationCapture } from './calibration-report.js';
import { SerialTerminal } from './serial-terminal.js';
import { ReceiverConsole, MENU_COMMAND_KEYS } from './receiver-console.js';
import { FlightLogCapture, decodeImage, summarize, secondsCsv } from './flight-log.js';

const $ = id => document.getElementById(id);

// ---- Flasher -------------------------------------------------------------
const btnConnect = $('btnConnect');
const btnConnectText = $('btnConnectText');
const btnFlash = $('btnFlash');
const unsupportedWarning = $('unsupportedWarning');

const tabGithub = $('tabGithub');
const tabAlpha = $('tabAlpha');
const tabLocal = $('tabLocal');
const tabHwx = $('tabHwx');
const paneAlpha = $('paneAlpha');
const paneGithub = $('paneGithub');
const paneLocal = $('paneLocal');
const paneHwx = $('paneHwx');

const selectAlphaBuild = $('selectAlphaBuild');
const selectAlphaPackageType = $('selectAlphaPackageType');
const alphaBuildDetails = $('alphaBuildDetails');

const selectHwxBuild = $('selectHwxBuild');
const selectHwxPackageType = $('selectHwxPackageType');
const hwxBuildDetails = $('hwxBuildDetails');

const selectRelease = $('selectRelease');
const selectPackageType = $('selectPackageType');
const releaseDetails = $('releaseDetails');

const dropzone = $('dropzone');
const inputLocalFile = $('inputLocalFile');
const localFileDetails = $('localFileDetails');
const inputFlashOffset = $('inputFlashOffset');

const selectBaud = $('selectBaud');
const chkEraseAll = $('chkEraseAll');
const statStatus = $('statStatus');
const progressContainer = $('progressContainer');
const progressBar = $('progressBar');
const progressStatusText = $('progressStatusText');
const progressPercentText = $('progressPercentText');
const consoleOutput = $('consoleOutput');

let activeSource = 'alpha'; // 'alpha' (C5VRX-5) | 'local' | 'hwx' (hardware experiments) | 'github' (legacy builds)
let port = null;
let transport = null;
let esploader = null;
let isConnected = false;
let isConnecting = false;
let isFlashing = false;
let githubReleases = [];
let githubAlphaBuilds = [];
let githubHwxBuilds = [];
let localFileBinary = null;

// Shown only when neither the Pages index nor the GitHub API answers.
const FALLBACK_RELEASES = [
  {
    tag_name: 'v3.0.0-rc1',
    name: 'C5VRX-3 v3.0.0-rc1: Seamless 16K Phase5 Production Receiver',
    published_at: '2026-09-18T17:01:49Z',
    prerelease: true,
    assets: [
      { name: 'c5vrx3.bin', size: 1082064, browser_download_url: 'https://github.com/Twotoz/C5VRX/releases/download/v3.0.0-rc1/c5vrx3.bin' },
      { name: 'bootloader.bin', size: 23232, browser_download_url: 'https://github.com/Twotoz/C5VRX/releases/download/v3.0.0-rc1/bootloader.bin' },
      { name: 'partition-table.bin', size: 3072, browser_download_url: 'https://github.com/Twotoz/C5VRX/releases/download/v3.0.0-rc1/partition-table.bin' },
    ]
  }
];
const VERSION_TAG_PATTERN = /^v\d+\.\d+\.\d+(?:-[0-9A-Za-z.-]+)?$/;
const ALPHA_TAG_PATTERN = /^c5vrx5-(?:pr-[0-9]+|v5\.[0-9]+\.[0-9]+)$/;
// Hardware experiments: immutable versions per experiment from main
// (c5vrx5-hwx-<id>-v5.0.N) and one channel per experiment and PR.
const HWX_TAG_PATTERN = /^c5vrx5-hwx-([a-z0-9_]+)-(?:v(5\.[0-9]+\.[0-9]+)|pr-([0-9]+))$/;

function log(msg, type = 'info') {
  if (type === 'error') $('flashLog').open = true;
  consoleOutput.textContent += `[${new Date().toLocaleTimeString()}] ${msg}\n`;
  consoleOutput.scrollTop = consoleOutput.scrollHeight;
}

// esptool-js writes its own progress through this terminal.
const terminal = {
  clean() { consoleOutput.textContent = ''; },
  writeLine(data) { this.write(`${data}\n`); },
  write(data) {
    consoleOutput.textContent += data;
    consoleOutput.scrollTop = consoleOutput.scrollHeight;
  }
};

function checkSerialSupport() {
  if ('serial' in navigator) return true;
  unsupportedWarning.hidden = false;
  log('Web Serial is not available in this browser. Use Chrome or Edge on a desktop to connect.');
  return false;
}

function activateSource(source) {
  if (isFlashing) return;
  activeSource = source;
  for (const [button, pane, name] of [[tabGithub, paneGithub, 'github'], [tabAlpha, paneAlpha, 'alpha'], [tabLocal, paneLocal, 'local'], [tabHwx, paneHwx, 'hwx']]) {
    button.setAttribute('aria-pressed', String(source === name));
    button.classList.toggle('active', source === name);
    pane.classList.toggle('active', source === name);
  }
  updateFlashButtonState();
}

tabGithub.addEventListener('click', () => activateSource('github'));
tabAlpha.addEventListener('click', () => activateSource('alpha'));
tabLocal.addEventListener('click', () => activateSource('local'));
tabHwx.addEventListener('click', () => activateSource('hwx'));
selectHwxBuild.addEventListener('change', () => onHwxBuildSelected());
selectAlphaBuild.addEventListener('change', () => onAlphaBuildSelected());
selectRelease.addEventListener('change', () => onReleaseSelected(parseInt(selectRelease.value, 10)));

// Local file: the Browse button sits inside the drop zone and bubbles here.
dropzone.addEventListener('click', (event) => {
  // The file input click bubbles too; never open the picker recursively.
  if (!isFlashing && event.target !== inputLocalFile) inputLocalFile.click();
});
dropzone.addEventListener('dragover', (event) => {
  event.preventDefault();
  dropzone.classList.add('dragover');
});
dropzone.addEventListener('dragleave', () => dropzone.classList.remove('dragover'));
dropzone.addEventListener('drop', (event) => {
  event.preventDefault();
  dropzone.classList.remove('dragover');
  if (!isFlashing && event.dataTransfer.files.length > 0) handleLocalFile(event.dataTransfer.files[0]);
});
inputLocalFile.addEventListener('change', (event) => {
  if (event.target.files.length > 0) handleLocalFile(event.target.files[0]);
  inputLocalFile.value = '';   // choosing the same file again still fires change
});

function handleLocalFile(file) {
  const name = file.name.toLowerCase();
  if (!name.endsWith('.bin')) {
    alert('Please select a compiled firmware binary (.bin file).');
    return;
  }
  $('localFileName').textContent = file.name;
  $('localFileSize').textContent = `${(file.size / 1024).toFixed(1)} KB (${file.size.toLocaleString()} bytes)`;
  inputFlashOffset.value = name.includes('merged') ? '0x0'
    : name.includes('bootloader') ? '0x2000'
      : name.includes('partition') ? '0x8000' : '0x10000';

  const reader = new FileReader();
  reader.onload = (event) => {
    localFileBinary = event.target.result;
    localFileDetails.hidden = false;
    log(`Loaded local file: ${file.name} (${file.size} bytes)`);
    updateFlashButtonState();
  };
  reader.onerror = () => log(`Could not read ${file.name}.`, 'error');
  reader.readAsArrayBuffer(file);
}

function parseFlashOffset(text) {
  const value = text.trim();
  if (!/^(?:0x)?[0-9a-f]+$/i.test(value)) throw new Error(`Invalid flash offset "${text}"; use hex such as 0x0 or 0x10000`);
  const offset = parseInt(value, 16);
  if (offset % 0x1000) throw new Error(`Flash offset ${value} is not on a 4 KB sector boundary`);
  return offset;
}

// Fetch the firmware index mirrored into the GitHub Pages artifact.
// Production flashing must stay same-origin: browsers cannot reliably fetch
// GitHub Release storage redirects due to CORS.
// quiet: background refresh. No placeholders or log lines, and the lists
// are only rebuilt (keeping the selected build) when a build was added or
// removed.
async function fetchReleases({ quiet = false } = {}) {
  if (!quiet) {
    selectRelease.innerHTML = '<option value="">Fetching firmware index…</option>';
    selectAlphaBuild.innerHTML = '<option value="">Fetching C5VRX-5 builds…</option>';
    selectHwxBuild.innerHTML = '<option value="">Fetching hardware experiments…</option>';
  }
  const say = quiet ? () => {} : log;

  let data = null;
  try {
    const mirrorUrl = new URL('firmware/releases.json', document.baseURI);
    mirrorUrl.searchParams.set('t', Date.now().toString());
    const mirrorRes = await fetch(mirrorUrl.href, { cache: 'no-store' });
    if (!mirrorRes.ok) throw new Error(`Pages firmware index HTTP ${mirrorRes.status}`);
    data = await mirrorRes.json();
    if (!Array.isArray(data)) throw new Error('Pages firmware index is not an array');
    say(`Loaded the firmware index (${data.length} build(s)).`);
  } catch (mirrorError) {
    if (quiet) return;   // a background refresh keeps what is shown
    // Metadata fallback only. Binary flashing still prefers same-origin assets;
    // this path mainly keeps local development usable for release browsing.
    say(`Pages firmware index unavailable (${mirrorError.message}); using GitHub release metadata.`);
    try {
      const res = await fetch('https://api.github.com/repos/Twotoz/C5VRX/releases?per_page=100', {
        headers: { Accept: 'application/vnd.github.v3+json' },
        cache: 'no-store'
      });
      if (!res.ok) throw new Error(`GitHub API HTTP ${res.status}`);
      data = await res.json();
      if (!Array.isArray(data)) throw new Error('GitHub releases response is not an array');
    } catch (apiError) {
      say(`Could not fetch release metadata (${apiError.message}). Showing the cached legacy release only.`);
      data = FALLBACK_RELEASES;
    }
  }

  const byDate = (a, b) => new Date(b.published_at || 0) - new Date(a.published_at || 0);
  let releases = data.filter(rel => VERSION_TAG_PATTERN.test(rel.tag_name || '')).sort(byDate);
  if (releases.length === 0) releases = FALLBACK_RELEASES;
  // C5VRX-5: versioned main builds first (newest first), then unmerged PR builds.
  const isPr = rel => Number(rel.tag_name.startsWith('c5vrx5-pr-'));
  const alpha = data.filter(rel => ALPHA_TAG_PATTERN.test(rel.tag_name || ''))
    .sort((a, b) => (isPr(a) - isPr(b)) || byDate(a, b));
  // Hardware experiments: by experiment, its versions (newest first) before PR channels.
  const hwxKey = rel => rel.tag_name.match(HWX_TAG_PATTERN);
  const versionParts = v => (v || '').split('.').map(Number);
  const byVersion = (a, b) => {
    const [x, y] = [versionParts(hwxKey(a)[2]), versionParts(hwxKey(b)[2])];
    for (let i = 0; i < 3; ++i) if (x[i] !== y[i]) return (y[i] || 0) - (x[i] || 0);
    return 0;
  };
  const hwx = data.filter(rel => HWX_TAG_PATTERN.test(rel.tag_name || ''))
    .sort((a, b) => hwxKey(a)[1].localeCompare(hwxKey(b)[1]) ||
      (Number(Boolean(hwxKey(a)[3])) - Number(Boolean(hwxKey(b)[3]))) ||
      (hwxKey(a)[3] ? byDate(a, b) : byVersion(a, b)));
  if (isFlashing) return;   // never swap the lists under a running flash
  const signature = list => list.map(rel => `${rel.tag_name}@${rel.published_at || ''}`).join(',');
  const releasesChanged = signature(releases) !== signature(githubReleases);
  const alphaChanged = signature(alpha) !== signature(githubAlphaBuilds);
  const hwxChanged = signature(hwx) !== signature(githubHwxBuilds);
  if (quiet && !releasesChanged && !alphaChanged && !hwxChanged) return;
  const keepRelease = githubReleases[Number(selectRelease.value)]?.tag_name;
  const keepAlpha = githubAlphaBuilds[Number(selectAlphaBuild.value)]?.tag_name;
  const keepHwx = githubHwxBuilds[Number(selectHwxBuild.value)]?.tag_name;
  githubReleases = releases;
  githubAlphaBuilds = alpha;
  githubHwxBuilds = hwx;
  say(`Available: ${githubAlphaBuilds.length} C5VRX-5 build(s), ${githubHwxBuilds.length} hardware experiment build(s), ${githubReleases.length} legacy release(s).`);

  if (!quiet || releasesChanged) populateReleaseDropdown(quiet ? keepRelease : undefined);
  if (!quiet || alphaChanged) populateAlphaBuildDropdown(quiet ? keepAlpha : undefined);
  if (!quiet || hwxChanged) populateHwxBuildDropdown(quiet ? keepHwx : undefined);
  if (quiet) log(`Firmware list updated: ${githubAlphaBuilds.length} C5VRX-5 build(s).`);
  updateFlashButtonState();
}

// The firmware lists follow new and removed builds by themselves.
const RELEASE_REFRESH_MS = 60000;
setInterval(() => { if (document.visibilityState !== 'hidden') fetchReleases({ quiet: true }); }, RELEASE_REFRESH_MS);
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'visible') fetchReleases({ quiet: true });
});

function populateAlphaBuildDropdown(keepTag) {
  selectAlphaBuild.replaceChildren();
  githubAlphaBuilds.forEach((rel, index) => {
    const opt = document.createElement('option');
    opt.value = index;
    opt.textContent = rel.tag_name.startsWith('c5vrx5-v')
      ? `C5VRX-5 ${rel.tag_name.slice('c5vrx5-'.length)}${index === 0 ? ' (Latest)' : ''}`
      : `C5VRX-5 — PR #${rel.tag_name.split('-').pop()} / unmerged`;
    selectAlphaBuild.appendChild(opt);
  });
  if (githubAlphaBuilds.length) {
    const kept = githubAlphaBuilds.findIndex(rel => rel.tag_name === keepTag);
    selectAlphaBuild.value = String(kept >= 0 ? kept : 0);
    onAlphaBuildSelected();
  } else {
    selectAlphaBuild.innerHTML = '<option value="">No C5VRX-5 builds published yet</option>';
    alphaBuildDetails.hidden = true;
  }
}

function populateHwxBuildDropdown(keepTag) {
  selectHwxBuild.replaceChildren();
  // One group per experiment; its newest version is marked Latest.
  let group = null;
  let latestShown = false;
  githubHwxBuilds.forEach((rel, index) => {
    const [, id, version, pr] = rel.tag_name.match(HWX_TAG_PATTERN);
    if (!group || group.label !== id) {
      group = document.createElement('optgroup');
      group.label = id;
      selectHwxBuild.appendChild(group);
      latestShown = false;
    }
    const opt = document.createElement('option');
    opt.value = index;
    opt.textContent = pr ? `${id} — PR #${pr} / unmerged`
      : `${id} v${version}${latestShown ? '' : ' (Latest)'}`;
    if (!pr) latestShown = true;
    group.appendChild(opt);
  });
  if (githubHwxBuilds.length) {
    const kept = githubHwxBuilds.findIndex(rel => rel.tag_name === keepTag);
    selectHwxBuild.value = String(kept >= 0 ? kept : 0);
    onHwxBuildSelected();
  } else {
    selectHwxBuild.innerHTML = '<option value="">No hardware experiments published yet</option>';
    hwxBuildDetails.hidden = true;
  }
}

function onHwxBuildSelected() {
  const rel = githubHwxBuilds[Number(selectHwxBuild.value)];
  if (!rel) return;
  showDetails(hwxBuildDetails, ['infoHwxBuildName', 'infoHwxBuildDate', 'infoHwxBuildTag', 'infoHwxBuildAssets'], rel);
  updateFlashButtonState();
}

function showDetails(box, ids, rel) {
  box.hidden = false;
  const values = [
    rel.name || rel.tag_name,
    rel.published_at ? new Date(rel.published_at).toLocaleDateString() : 'N/A',
    rel.tag_name,
    (rel.assets || []).map(a => a.name).join(', ') || 'No binary assets attached'
  ];
  ids.forEach((id, i) => { $(id).textContent = values[i]; });
}

function onAlphaBuildSelected() {
  const rel = githubAlphaBuilds[Number(selectAlphaBuild.value)];
  if (!rel) return;
  showDetails(alphaBuildDetails, ['infoAlphaBuildName', 'infoAlphaBuildDate', 'infoAlphaBuildTag', 'infoAlphaBuildAssets'], rel);
  updateFlashButtonState();
}

function populateReleaseDropdown(keepTag) {
  selectRelease.replaceChildren();
  githubReleases.forEach((rel, index) => {
    const opt = document.createElement('option');
    opt.value = index;
    const tag = rel.tag_name || rel.name;
    opt.textContent = `${tag}${rel.prerelease ? ' [Pre-release]' : ''}${index === 0 ? ' (Latest)' : ''}`;
    selectRelease.appendChild(opt);
  });
  const kept = Math.max(0, githubReleases.findIndex(rel => (rel.tag_name || rel.name) === keepTag));
  selectRelease.value = String(kept);
  onReleaseSelected(kept);
}

function onReleaseSelected(index) {
  const rel = githubReleases[index];
  if (!rel) return;
  showDetails(releaseDetails, ['infoReleaseName', 'infoReleaseDate', 'infoReleaseTag', 'infoReleaseAssets'], rel);
  updateFlashButtonState();
}

function updateFlashButtonState() {
  if (!isConnected || isFlashing) {
    btnFlash.disabled = true;
  } else if (activeSource === 'github') {
    btnFlash.disabled = !githubReleases[parseInt(selectRelease.value, 10)];
  } else if (activeSource === 'alpha') {
    btnFlash.disabled = !githubAlphaBuilds[parseInt(selectAlphaBuild.value, 10)];
  } else if (activeSource === 'hwx') {
    btnFlash.disabled = !githubHwxBuilds[parseInt(selectHwxBuild.value, 10)];
  } else {
    btnFlash.disabled = localFileBinary === null;
  }
}

function getSelectedRemoteBuild() {
  if (activeSource === 'github') {
    return { release: githubReleases[parseInt(selectRelease.value, 10)], packageType: selectPackageType.value };
  }
  if (activeSource === 'alpha') {
    return { release: githubAlphaBuilds[parseInt(selectAlphaBuild.value, 10)], packageType: selectAlphaPackageType.value };
  }
  if (activeSource === 'hwx') {
    return { release: githubHwxBuilds[parseInt(selectHwxBuild.value, 10)], packageType: selectHwxPackageType.value };
  }
  return { release: null, packageType: null };
}

function findApplicationAsset(assets) {
  // GitHub returns release assets in upload order. Never use the first .bin:
  // bootloader.bin is commonly uploaded before the application image.
  return assets.find(asset => /^c5vrx(?:[345])?\.bin$/i.test(asset.name || '')) ||
    assets.find(asset => {
      const name = (asset.name || '').toLowerCase();
      return name.endsWith('.bin') && !name.includes('bootloader') && !name.includes('partition') && !name.includes('merged');
    });
}

btnConnect.addEventListener('click', async () => {
  if (isFlashing || isConnecting) return;
  if (isConnected) await disconnectDevice();
  else await connectDevice();
});

// The USB reset never reaches the ROM loader on a running C5VRX-5: it resets
// the CPU but not the modem, whose dump writer keeps the loader from
// answering, and the board stays hung until it is power-cycled (main/rf.c,
// rf_reboot_to_download). The console key ` stops the writer, sets the
// LP_AON force-download boot and restarts cleanly, so the key goes first;
// a board already in the loader ignores the byte. tools/enter_download.py
// does the same.
const DOWNLOAD_KEY = 0x60;
const DOWNLOAD_ACK = 'download_boot=requested';
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
// After flashing, the RTS reset brings an ESP32-C5 back up in the loader
// (board 2026-10-25), and esptool.js has no C5 watchdog reset. These are
// esptool's own LP watchdog writes for the C5 (esp32c3.py watchdog_reset,
// esp32c6.py addresses): unlock, timeout, enable with a system reset.
const LP_WDT_CONFIG0_REG = 0x600b1c00;
const LP_WDT_CONFIG1_REG = 0x600b1c04;
const LP_WDT_WPROTECT_REG = 0x600b1c18;
const LP_WDT_WKEY = 0x50d83aa1;
const LP_WDT_RESET_CONFIG = 0xd0000102;

// Sends the key and says what came back: 'acknowledged', 'port closed' (the
// restart took the port), 'other output' or 'no reply'. Bounded: a console
// that does not read never blocks the write for more than a second.
async function sendDownloadKey(selectedPort) {
  await selectedPort.open({ baudRate: 115200 });
  // DTR low with RTS high is the USB-JTAG chip reset. Chrome may raise both
  // on open and drop them on close; dropped DTR-first while the receiver
  // still runs, that is the reset that hangs it. Lower them now, RTS first,
  // so closing changes nothing.
  try {
    await selectedPort.setSignals({ requestToSend: false });
    await selectedPort.setSignals({ dataTerminalReady: false });
  } catch { /* not supported: carry on */ }
  const reader = selectedPort.readable.getReader();
  const writer = selectedPort.writable.getWriter();
  let seen = '', outcome = '', cancelling = false;
  const decoder = new TextDecoder();
  // The read only ends by itself when the restart takes the port; ending it
  // here (cancel) says nothing about the board.
  const reading = (async () => {
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done) { if (!cancelling) outcome = 'port closed'; return; }
        seen = (seen + decoder.decode(value, { stream: true })).slice(-4096);
        if (seen.includes(DOWNLOAD_ACK)) { outcome = 'acknowledged'; return; }
      }
    } catch {
      if (!cancelling) outcome = 'port closed';
    }
  })();
  try {
    const writing = writer.write(new Uint8Array([DOWNLOAD_KEY])).catch(() => {});
    await Promise.race([writing, wait(1000)]);
    await Promise.race([reading, wait(1000)]);
    // The firmware restarts ~50 ms after its reply: leave the port alone
    // until it has.
    if (outcome === 'acknowledged') await wait(500);
  } finally {
    cancelling = true;
    // Bounded: a board that stopped reading never completes a pending write.
    try { await Promise.race([writer.abort(), wait(500)]); } catch { /* the device may be gone */ }
    try { await Promise.race([reader.cancel(), wait(500)]); } catch { /* the device may be gone */ }
    try { reader.releaseLock(); } catch { /* still reading */ }
    try { writer.releaseLock(); } catch { /* already released */ }
    // A port that vanished with the restart may never finish closing.
    try { await Promise.race([selectedPort.close(), wait(1000)]); } catch { /* the device may be gone */ }
  }
  return outcome || (seen ? 'other output' : 'no reply');
}

// If the restart drops the board off USB, Chrome hands it back as a new
// SerialPort (a 'connect' event; the permission is kept). USB vendor/product
// IDs identify a model, not an individual board: only reuse the selected
// SerialPort. A replacement object needs an explicit selection by the user.
function watchReconnect(selectedPort) {
  const watch = { vanished: false, back: null };
  const onDisconnect = event => {
    if (event.target === selectedPort) { watch.vanished = true; watch.back = null; }
  };
  const onConnect = event => {
    if (watch.vanished && event.target === selectedPort) watch.back = selectedPort;
  };
  navigator.serial.addEventListener('disconnect', onDisconnect);
  navigator.serial.addEventListener('connect', onConnect);
  watch.stop = () => {
    navigator.serial.removeEventListener('disconnect', onDisconnect);
    navigator.serial.removeEventListener('connect', onConnect);
  };
  return watch;
}

async function portAfterRestart(previous, watch) {
  for (let t = 0; t < 80 && watch.vanished && !watch.back; ++t) await wait(100);
  watch.stop();
  const ports = await navigator.serial.getPorts();
  if (ports.includes(previous)) return previous;
  throw new Error('The selected receiver is no longer available. Click Connect device and select the receiver again; another serial port cannot be identified safely.');
}

async function handshake(selectedPort, mode) {
  // esptool.js reopens the port (baud change) and closes it at the end.
  // Lowering DTR and RTS (RTS first) after every open keeps those closes
  // from passing through the chip reset; its USB reset sets them itself.
  class QuietTransport extends Transport {
    async connect(...args) {
      await super.connect(...args);
      try {
        await this.device.setSignals({ requestToSend: false });
        await this.device.setSignals({ dataTerminalReady: false });
      } catch { /* not supported: carry on */ }
    }
  }
  transport = new QuietTransport(selectedPort);
  esploader = new ESPLoader({
    transport,
    baudrate: parseInt(selectBaud.value, 10) || 460800,
    terminal,
    romBaudrate: 115200,
    debugLogging: false
  });
  await esploader.main(mode);
}

async function connectDevice() {
  if (isConnecting || isConnected || isFlashing || !['disconnected', 'connected'].includes(serialConsole.state) || !checkSerialSupport()) return;
  isConnecting = true;
  syncDeviceControls();

  try {
    await serialConsole.disconnect();
    log('Opening the serial port selector…');
    port = await navigator.serial.requestPort();
    // esptool.js clears the log when the loader object is created: steps go
    // to the status line and the route is logged once connected.
    statStatus.textContent = 'Connecting… sending the download key';
    const watch = watchReconnect(port);
    let keyReply;
    try { keyReply = await sendDownloadKey(port); } catch (err) { keyReply = `not sent: ${err.message || err}`; }
    // A receiver that took the key restarts into the loader, which answers
    // without any reset (board 2026-10-25: 3 of 3). The USB reset must not
    // follow it: that hung the board. Without C5VRX firmware, or with BOOT
    // held at power-up, nothing answers the key and the USB reset works.
    const restarted = keyReply === 'acknowledged' || keyReply === 'port closed';
    statStatus.textContent = `Connecting… waiting for the restart (download key: ${keyReply})`;
    await wait(2000);
    const previous = port;
    port = await portAfterRestart(previous, watch);
    keyReply += `; port ${port !== previous ? 'reconnected' : watch.vanished ? 'lost, reusing it' : 'kept'}`;
    statStatus.textContent = `Connecting… bootloader handshake (download key: ${keyReply})`;
    let route = 'download key';
    try {
      await handshake(port, 'no_reset');
    } catch (err) {
      if (restarted) throw new Error(`The receiver restarted into the bootloader but did not answer (${err.message || err}). Unplug it, plug it back in and connect again.`);
      try { await transport.disconnect(); } catch { /* already closed */ }
      statStatus.textContent = `Connecting… USB reset into the bootloader (download key: ${keyReply})`;
      await handshake(port, 'usb_reset');
      route = 'USB reset';
    }
    log(`Bootloader reached by ${route} (download key: ${keyReply}).`);

    isConnected = true;
    btnConnectText.textContent = 'Disconnect';
    const chipName = esploader.chip ? esploader.chip.CHIP_NAME : 'ESP32-C5';
    statStatus.textContent = `${chipName} connected · ready to flash`;
    log(`Connected to ${chipName}.`);
  } catch (err) {
    if (err?.name === 'NotFoundError') {
      log('No port selected.');
    } else {
      log(`Connection failed: ${err.message || err}`, 'error');
      alert(`Failed to connect: ${err.message || err}\n\nIf the port timed out, hold BOOT on the XIAO, tap RESET, release BOOT and try again.`);
    }
    await disconnectDevice();
  } finally {
    isConnecting = false;
    syncDeviceControls();
    updateFlashButtonState();
  }
}

async function disconnectDevice(message = 'No device connected') {
  const wasOpen = Boolean(transport);
  try {
    if (transport) await transport.disconnect();
  } catch {
    // The port may already be gone after a reset.
  }
  port = null;
  transport = null;
  esploader = null;
  isConnected = false;
  btnConnectText.textContent = 'Connect device';
  statStatus.textContent = message;
  syncDeviceControls();
  updateFlashButtonState();
  if (wasOpen) log('Device disconnected.');
}

async function collectRemoteImages(selected) {
  const rel = selected.release;
  if (!rel) throw new Error('No release selected');
  const assets = rel.assets || [];
  log(`Fetching binaries for ${rel.tag_name} (${selected.packageType})…`);

  if (selected.packageType !== 'merged') {
    const app = findApplicationAsset(assets);
    if (!app) throw new Error('Could not find application firmware binary in release assets');
    return [{ data: new Uint8Array(await fetchReleaseAsset(app)), address: 0x10000 }];
  }
  const mergedAsset = assets.find(a => a.name.includes('merged'));
  if (mergedAsset) return [{ data: new Uint8Array(await fetchReleaseAsset(mergedAsset)), address: 0x0 }];

  // Standard three-part layout.
  const bootloader = assets.find(a => a.name.includes('bootloader'));
  const ptable = assets.find(a => a.name.includes('partition'));
  const app = findApplicationAsset(assets);
  const missing = [!bootloader && 'bootloader', !ptable && 'partition table', !app && 'application firmware'].filter(Boolean);
  if (missing.length > 0) throw new Error(`Incomplete full firmware package: missing ${missing.join(', ')}`);
  const images = [];
  for (const [asset, address] of [[bootloader, 0x2000], [ptable, 0x8000], [app, 0x10000]]) {
    images.push({ data: new Uint8Array(await fetchReleaseAsset(asset)), address });
  }
  return images;
}

function setProgress(percent, text) {
  progressBar.style.width = `${percent}%`;
  progressPercentText.textContent = `${percent}%`;
  if (text) progressStatusText.textContent = text;
}

btnFlash.addEventListener('click', async () => {
  if (!isConnected || !esploader || isFlashing || serialConsole.state !== 'disconnected') return;
  const selected = getSelectedRemoteBuild();
  if (activeSource === 'alpha' && /^c5vrx5-pr-/.test(selected.release?.tag_name || '') &&
      !window.confirm('Flash an unmerged C5VRX-5 PR build? It may fail to boot or produce broken video.')) return;
  if (activeSource === 'hwx' && !window.confirm(
    `Flash hardware experiment ${selected.release?.tag_name || ''}? It is a bench build without DAC video output ` +
    'and drives pins for external hardware (see its hwx.h). Flash a normal C5VRX-5 build to go back.')) return;

  isFlashing = true;
  syncDeviceControls();
  updateFlashButtonState();
  $('flashLog').open = true;
  progressContainer.hidden = false;
  setProgress(0, 'Preparing firmware images…');

  try {
    let fileArray;
    if (activeSource === 'local') {
      if (!localFileBinary) throw new Error('No local binary selected');
      fileArray = [{ data: new Uint8Array(localFileBinary), address: parseFlashOffset(inputFlashOffset.value) }];
    } else {
      fileArray = await collectRemoteImages(selected);
    }

    const totalBytes = fileArray.reduce((acc, f) => acc + f.data.length, 0);
    log(`Writing ${fileArray.length} image(s), ${totalBytes.toLocaleString()} bytes.`);
    progressStatusText.textContent = 'Flashing…';

    await esploader.writeFlash({
      fileArray,
      flashSize: '8MB',
      flashMode: 'dio',
      flashFreq: '80m',
      eraseAll: chkEraseAll.checked,
      // ESP32-C5 native USB-Serial/JTAG can fail mid-write in the compressed path (status 201,0).
      compress: false,
      reportProgress: (fileIndex, written, total) => {
        setProgress(Math.floor((written / total) * 100),
          `Writing image ${fileIndex + 1}/${fileArray.length} (${(written / 1024).toFixed(0)} / ${(total / 1024).toFixed(0)} KB)`);
      }
    });

    setProgress(100, 'Done. Restarting the receiver…');
    log('Flash written and verified. Resetting into the new firmware…');
    let resetting = false;
    try {
      await esploader.writeReg(LP_WDT_WPROTECT_REG, LP_WDT_WKEY);
      await esploader.writeReg(LP_WDT_CONFIG1_REG, 2000);
      resetting = true;   // the chip resets on this write and may not answer it
      await esploader.writeReg(LP_WDT_CONFIG0_REG, LP_WDT_RESET_CONFIG);
    } catch {
      // No reply from a chip that is already resetting.
    }
    // Let the watchdog fire before the port closes: closing it changes the
    // USB control lines, which reset the chip back into the loader first
    // (board 2026-10-25; esptool also waits after this write).
    if (resetting) await wait(1000);
    if (!resetting) {
      try {
        await esploader.after('hard_reset');
      } catch {
        // The reset pulse can close the USB-CDC port.
      }
    }
    progressStatusText.textContent = 'Done. Open Terminal to watch the receiver boot.';
    log('Flashing completed.');
    // The ROM loader is gone after the reset: release the port for the
    // terminal while the controls are still locked.
    await disconnectDevice('Flashed · the receiver is restarting');
  } catch (err) {
    log(`Flashing failed: ${err.message || err}`, 'error');
    alert(`Flashing error: ${err.message || err}`);
    progressStatusText.textContent = 'Flash failed';
  } finally {
    isFlashing = false;
    syncDeviceControls();
    updateFlashButtonState();
  }
});

async function fetchReleaseAsset(asset) {
  if (!asset) throw new Error('Missing firmware asset metadata');
  log(`Downloading ${asset.name}…`);

  if (asset.local_url) {
    const localRes = await fetch(new URL(asset.local_url, document.baseURI).href, { cache: 'no-store' });
    if (!localRes.ok) throw new Error(`Pages firmware mirror HTTP ${localRes.status} for ${asset.name}`);
    const buffer = await localRes.arrayBuffer();
    if (asset.size && buffer.byteLength !== asset.size) {
      throw new Error(`Firmware mirror size mismatch for ${asset.name}: expected ${asset.size}, got ${buffer.byteLength}`);
    }
    return buffer;
  }

  // Development fallback only. Production Pages manifests always attach a
  // local_url. GitHub's release storage redirect may still be blocked by CORS.
  if (asset.url) {
    try {
      const apiRes = await fetch(asset.url, {
        headers: { Accept: 'application/octet-stream', 'X-GitHub-Api-Version': '2022-11-28' },
        redirect: 'follow',
        cache: 'no-store'
      });
      if (!apiRes.ok) throw new Error(`GitHub asset API HTTP ${apiRes.status}`);
      return await apiRes.arrayBuffer();
    } catch (apiError) {
      log(`GitHub asset API failed (${apiError.message}); trying the browser download URL…`);
    }
  }
  if (asset.browser_download_url) {
    try {
      const directRes = await fetch(asset.browser_download_url, { redirect: 'follow', cache: 'no-store' });
      if (!directRes.ok) throw new Error(`GitHub download HTTP ${directRes.status}`);
      return await directRes.arrayBuffer();
    } catch (directError) {
      throw new Error('This build is not mirrored on GitHub Pages and GitHub blocked the cross-origin download. ' +
        `Refresh the flasher after the Pages sync completes. Details: ${directError.message}`);
    }
  }
  throw new Error(`No downloadable URL available for ${asset.name || 'firmware asset'}`);
}

$('btnClearConsole').addEventListener('click', () => {
  terminal.clean();
  log('Console cleared.');
});

// ---- Receiver terminal ---------------------------------------------------
// Firmware and terminal own separate transports and never open concurrently.
const serialOutput = $('output');
const commandInput = $('commandInput');
const btnTerminalConnect = $('btnTerminalConnect');
const terminalBaud = $('terminalBaud');
const terminalStatus = $('terminalStatus');
const shortcutButtons = [...document.querySelectorAll('[data-command]')];
const MAX_LOG_CHARS = 120000;
// Keys whose effect the menu should show: one status report shortly after.
// Scan and calibration report when they finish instead.
const REFRESH_AFTER = new Set(['b', 'c', 'v', 'o', 'e', 'x', 'g', 'K', 'k', 'j', 'P', 'A', 'G', 'U']);
const REFRESH_DELAY_MS = 700;
const receiverConsole = new ReceiverConsole();
const calibrationCapture = new CalibrationCapture();
let calibrationTimer = null;
let calibrationRequest = null;
let commandBusy = false;
let refreshTimer = null;
let logPending = '';
let logTimer = null;
let logLinePartial = '';   // unfinished line, held until STATUS lines can be told apart

function setText(element, text) {
  if (element.textContent !== text) element.textContent = text;
}

function receiverValues(v) {
  if (!v) return {};
  return {
    channel: v.channel,
    freq: v.freq && `${v.freq} MHz`,
    band: v.band,
    signal: v.signal === null ? '' : `${v.signal}/100`,
    lock: v.locked === null ? '' : v.locked ? 'VIDEO' : 'SEARCH',
    standard: v.standard,
    detected: v.detected,
    output: v.output,
    gainMode: v.gainMode,
    gain: v.gain ? `G${v.gain}` : '',
    menu: v.menu === null ? '' : v.menu ? 'OPEN' : 'CLOSED · LIVE VIDEO',
    calibrate: receiverConsole.calibrationHealth === 'bad' ? 'CALIBRATION RECOMMENDED' : v.busy ? 'RUNNING · VIDEO STOPPED' : v.stage || v.statusAt ? 'READY' : '',
    scan: v.scan,
    demod: v.demod,
    liveProfile: v.liveProfile === undefined ? '' : v.liveProfile ? 'ON' : 'OFF · REPORT ONLY',
    profile: v.profile,
    filter: v.filter,
    dac: !v.dacStep ? '' : v.dacStep === 'PATTERN' ? 'TEST PATTERN ON SCREEN'
      : `${v.dacStep}${v.dacStep === 'NONE' || v.dacStep === 'ALL' ? '' : ' ONLY'} · EXPECT ${v.dacExpect} mV`
  };
}

function renderReceiver() {
  const connected = serialConsole.state === 'connected';
  const v = connected && receiverConsole.current ? receiverConsole.view() : null;
  setText($('terminalFirmware'), !connected
    ? 'Connect a C5VRX-5 receiver over USB to use its menu.'
    : v
      ? `C5VRX-5 ${receiverConsole.version} · ${v.statusAt ? `status from ${new Date(v.statusAt).toLocaleTimeString()}` : 'waiting for a status report'}`
      : 'Identifying the firmware… the menu needs C5VRX-5 firmware.');

  const notice = $('receiverNotice');
  notice.hidden = !v?.busy;
  if (v?.busy) {
    setText(notice, `${v.refused ? `Key ${v.refused} was ignored: ` : ''}the receiver is calibrating (or running a lane test), ` +
      'so it ignores menu commands until that finishes. The menu buttons come back by themselves.');
  }

  const values = receiverValues(v);
  for (const element of document.querySelectorAll('[data-value]')) {
    const empty = element.closest?.('.receiver-header') ? '' : '—';
    setText(element, values[element.dataset.value] || empty);
  }
  // Same bar count as the on-screen menu (menu.c).
  const signal = v?.signal ?? 0;
  const bars = signal <= 0 ? 0 : signal >= 100 ? 5 : Math.floor((signal + 19) / 20);
  for (const element of document.querySelectorAll('[data-bars]')) element.setAttribute('data-bars', String(bars));
  for (const element of document.querySelectorAll('[data-meter]')) element.style.width = `${signal}%`;
}

function flushLog() {
  clearTimeout(logTimer);
  logTimer = null;
  if (!logPending) return;
  serialOutput.textContent = (serialOutput.textContent + logPending).slice(-MAX_LOG_CHARS);
  logPending = '';
  if ($('autoScroll').checked) serialOutput.scrollTop = serialOutput.scrollHeight;
}

// The 2-s STATUS beat feeds the menu, not the log. A line that may still
// become a STATUS line is held until it does or cannot.
function visibleLog(text) {
  const parts = (logLinePartial + text).split('\n');
  const tail = parts.pop();
  const isStatus = line => { const l = line.replace(/\r$/, ''); return l.startsWith('STATUS ') || l.startsWith('FLOG_D '); };
  let out = parts.filter(line => !isStatus(line)).map(line => `${line}\n`).join('');
  if (tail.length <= 4096 && (tail.startsWith('STATUS ') || 'STATUS '.startsWith(tail))) {
    logLinePartial = tail;
  } else {
    out += tail;
    logLinePartial = '';
  }
  return out;
}

function appendSerial(text) {
  flightLogFeed(text);
  calibrationReportFeed(text);
  // Batch the log text: FIELD_DIAG and IQ dumps arrive in many small reads.
  logPending = (logPending + visibleLog(text)).slice(-MAX_LOG_CHARS);
  if (!logTimer) logTimer = setTimeout(flushLog, 50);
  for (const event of receiverConsole.push(text)) {
    if (event === 'ready') scheduleRefresh(1500);
    else if ((event === 'scan-done' || event === 'calibration-done') && !receiverConsole.reportsByItself()) scheduleRefresh(400);
  }
  renderReceiver();
  syncDeviceControls();
}

function scheduleRefresh(delay = REFRESH_DELAY_MS) {
  clearTimeout(refreshTimer);
  refreshTimer = setTimeout(async () => {
    refreshTimer = null;
    if (serialConsole.state !== 'connected') return;
    if (commandBusy) { scheduleRefresh(200); return; }
    commandBusy = true;
    try { await serialConsole.send('T'); } finally { commandBusy = false; syncDeviceControls(); }
  }, delay);
}

const serialConsole = new SerialTerminal({
  serial: navigator.serial,
  onData: appendSerial,
  onState: (state) => {
    const labels = { connected: 'Connected', connecting: 'Connecting…', disconnecting: 'Disconnecting…', disconnected: 'No device connected' };
    terminalStatus.textContent = labels[state];
    terminalStatus.setAttribute('data-state', state);
    if (state === 'connecting') {
      flushLog();
      logLinePartial = '';
      serialOutput.textContent = '';
      receiverConsole.reset();
    }
    if (state !== 'connected') {
      if (calibrationCapture.active) {
        calibrationCapture.fail('Receiver disconnected. Reconnect and download again.');
        clearTimeout(calibrationTimer);
        $('calibrationReportStatus').textContent = calibrationCapture.error;
      }
      clearTimeout(refreshTimer);
      refreshTimer = null;
    }
    // onState can run during connect, after construction has completed.
    renderReceiver();
    syncDeviceControls();
  },
  onError: (error) => appendSerial(`\n[Web terminal] ${error.message || error}\n`)
});

// Older firmware has no STATUS beat: ask for a report every 10 s while the
// terminal is idle, so the menu never needs a refresh button.
const LEGACY_POLL_MS = 10000;
setInterval(() => {
  if (serialConsole.state !== 'connected' || !receiverConsole.current || receiverConsole.reportsByItself()) return;
  if (!refreshTimer && Date.now() - receiverConsole.statusAt >= LEGACY_POLL_MS) scheduleRefresh(0);
}, 1000);

function syncDeviceControls() {
  const supported = 'serial' in navigator;
  for (const control of [tabGithub, tabAlpha, tabLocal, tabHwx, selectAlphaBuild, selectAlphaPackageType, selectHwxBuild, selectHwxPackageType, selectRelease, selectPackageType, selectBaud, chkEraseAll, inputLocalFile, inputFlashOffset, $('btnBrowseFile')]) {
    control.disabled = isFlashing;
  }
  const terminalConnected = serialConsole.state === 'connected';
  const terminalIdle = serialConsole.state === 'disconnected';
  const portBusy = !supported || isConnecting || isFlashing || (!terminalIdle && !terminalConnected);
  btnConnect.disabled = portBusy;
  btnConnect.title = terminalConnected ? 'Releases the terminal and connects the flasher' : '';
  btnTerminalConnect.disabled = portBusy;
  syncFlightLogControls(terminalConnected);
  $('btnCalibrationReport').disabled = !terminalConnected || commandBusy || calibrationCapture.active ||
    receiverConsole.receiverBusy() || !receiverConsole.supports('C') || flightLog.active;
  btnTerminalConnect.textContent = terminalConnected ? 'Disconnect' : serialConsole.state === 'connecting' ? 'Connecting…' : 'Connect receiver';
  btnTerminalConnect.title = isConnected ? 'Releases the flasher and connects the terminal' : '';
  terminalBaud.disabled = !terminalIdle;
  commandInput.disabled = !terminalConnected || commandBusy;
  $('btnSendCommand').disabled = !terminalConnected || commandBusy;
  const receiverBusy = receiverConsole.receiverBusy();
  for (const button of shortcutButtons) {
    const key = button.dataset.command;
    button.disabled = !terminalConnected || commandBusy || !receiverConsole.supports(key) || (receiverBusy && MENU_COMMAND_KEYS.has(key));
    button.title = receiverBusy && MENU_COMMAND_KEYS.has(key) ? 'The receiver ignores this until its calibration finishes'
      : button.dataset.hint || '';
  }
}

btnTerminalConnect.addEventListener('click', async () => {
  if (isConnecting || isFlashing) return;
  if (serialConsole.state === 'connected') {
    await serialConsole.disconnect();
  } else if (serialConsole.state === 'disconnected' && checkSerialSupport()) {
    // Reserve the UI while the previous flasher transport releases the port.
    isConnecting = true;
    syncDeviceControls();
    try {
      if (isConnected) await disconnectDevice();
      await serialConsole.connect(Number(terminalBaud.value));
      if (serialConsole.state === 'connected') await serialConsole.send('hT');
    } finally {
      isConnecting = false;
      syncDeviceControls();
    }
  }
});

async function sendTerminalCommand(command) {
  if (commandBusy || serialConsole.state !== 'connected') return false;
  const button = shortcutButtons.find(b => b.dataset.command === command);
  if (command.length !== 1 || (!button && command !== '?') || !receiverConsole.supports(command)) {
    appendSerial('\n[Web terminal] Unsupported key. Use h for help; the menu needs C5VRX-5 firmware.\n');
    return false;
  }
  commandBusy = true;
  syncDeviceControls();
  let sent = false;
  try {
    sent = await serialConsole.send(command);
  } finally {
    commandBusy = false;
    syncDeviceControls();
  }
  if (sent && command === 's') receiverConsole.startScan();
  // Firmware with the STATUS beat shows the effect within 2 s by itself.
  // GAIN is reported by T, not the short STATUS beat.
  if (sent && (command === 'G' || (REFRESH_AFTER.has(command) && !receiverConsole.reportsByItself()))) scheduleRefresh();
  if (sent) renderReceiver();
  return sent;
}

for (const button of shortcutButtons) {
  button.addEventListener('click', () => sendTerminalCommand(button.dataset.command));
}

$('commandForm').addEventListener('submit', async (event) => {
  event.preventDefault();
  const command = commandInput.value;
  if (!command) return;
  if (await sendTerminalCommand(command)) commandInput.value = '';
});
$('clear').addEventListener('click', () => {
  logPending = '';
  serialOutput.textContent = '';
});
$('downloadLog').addEventListener('click', () => {
  flushLog();
  const url = URL.createObjectURL(new Blob([serialOutput.textContent], { type: 'text/plain;charset=utf-8' }));
  const link = document.createElement('a');
  link.href = url;
  link.download = `c5vrx-${new Date().toISOString().replace(/[:.]/g, '-')}.log`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

// ---- Flight log (web/flight-log.js, docs/FLIGHT_LOG.md) -------------------
const flightLog = new FlightLogCapture();
let flightLogPartial = '';
let flightLogResult = null;
let flightLogSelected = null;
const flightLogStatus = $('flightLogStatus');

function flightLogFeed(text) {
  const parts = (flightLogPartial + text).split('\n');
  flightLogPartial = parts.pop().slice(-1100);
  let touched = false;
  for (const line of parts) touched = flightLog.push(line.replace(/\r$/, '')) || touched;
  if (!touched) return;
  if (flightLog.error) {
    flightLogStatus.textContent = flightLog.error;
  } else if (flightLog.active) {
    flightLogStatus.textContent = `Reading flight log… ${Math.round(flightLog.progress * 100)}%`;
  } else if (flightLog.needsPass && serialConsole.state === 'connected') {
    // Some lines arrived damaged (other console output): read again, keep what arrived intact.
    flightLog.nextPass();
    flightLogStatus.textContent = `Reading flight log… pass ${flightLog.passes} for ${flightLog.damaged} damaged line(s)`;
    serialConsole.send('W');
  } else if (flightLog.done && !flightLogResult) {
    flightLogResult = decodeImage(flightLog.image);
    renderFlightLog();
    const c = flightLog.completeness();
    if (c.blocks < Number(flightLog.info.blocks || 0) || c.snapshots < Number(flightLog.info.snapshots || 0))
      flightLogStatus.textContent += ` Some parts stayed damaged after ${flightLog.passes} passes; read again for the rest.`;
    if (flightLog.dumpMs !== null) flightLogStatus.textContent += ` Read in ${(flightLog.dumpMs / 1000).toFixed(1)} s.`;
    if (flightLog.slowPasses)
      flightLogStatus.textContent += ' It used the slower USB path: the receiver was short of memory, usually because its on-screen menu was open. Close the menu for a faster read.';
  }
  syncFlightLogControls(serialConsole.state === 'connected');
}

function syncFlightLogControls(connected) {
  $('btnReadFlightLog').disabled = !connected || commandBusy || flightLog.active;
  $('btnEraseFlightLog').disabled = !connected || commandBusy || flightLog.active;
  $('btnDownloadFlightLog').disabled = !flightLog.done;
  $('btnDownloadFlightCsv').disabled = !flightLogResult?.sessions.length;
}

function download(name, data, type) {
  const url = URL.createObjectURL(new Blob([data], { type }));
  const link = document.createElement('a');
  link.href = url;
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

const fmt = (v, digits = 1, unit = '') => (v === null || v === undefined || !Number.isFinite(v) ? '—' : `${v.toFixed(digits)}${unit}`);

function renderFlightLog() {
  const { sessions, snapshots } = flightLogResult;
  const table = $('flightLogSessions');
  const body = table.querySelector('tbody');
  body.textContent = '';
  for (const s of [...sessions].reverse()) {
    const sum = summarize(s);
    const row = document.createElement('tr');
    const minutes = sum.durationS >= 60 ? `${Math.floor(sum.durationS / 60)} min ${sum.durationS % 60} s` : `${sum.durationS} s`;
    const cells = [`#${sum.session}${sum.freq ? ` · ${sum.freq} MHz` : ''}`, minutes, `${sum.pictureS} s`,
      sum.snrMedian === null ? '—' : `${fmt(sum.snrMedian)} / ${fmt(sum.snrWorst)} dB`,
      sum.levelMedian === null ? '—' : `${fmt(sum.levelMedian, 0)} / ${fmt(sum.levelWorstWithPicture, 0)} dBm`,
      sum.outputDemod === null ? '—' : `${sum.outputDemod}${sum.outputLevel === null ? '' : ` · L${sum.outputLevel}`}`,
      sum.nativeHLockMedian === null ? '—' : `${fmt(sum.nativeHLockMedian, 0)} / ${fmt(sum.outputCoverageMedian, 0)} %`,
      sum.repairedTips === null ? '—' : `${sum.repairedTips} / ${sum.longestHUnlockMs} ms`, String(sum.dropouts),
      fmt(sum.dcWorst, 2, ' cells'), fmt(sum.tempMax, 0, ' °C'), fmt(sum.deviation, 2),
      `${sum.states.TRACK} / ${sum.states.EDGE} / ${sum.states.RECOVER} s`, sum.topReason || '—', sum.resetReason, sum.firmware];
    for (const text of cells) { const td = document.createElement('td'); td.textContent = text; row.append(td); }
    row.tabIndex = 0;
    row.addEventListener('click', () => selectFlightSession(s, row));
    row.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); selectFlightSession(s, row); } });
    body.append(row);
  }
  table.hidden = !sessions.length;
  const seconds = sessions.reduce((n, s) => n + s.seconds.length, 0);
  flightLogStatus.textContent = sessions.length
    ? `${sessions.length} session(s), ${Math.round(seconds / 60)} min recorded, ${snapshots.length} IQ snapshot(s). Click a session for its chart.`
    : 'The flight log is empty.';
  if (sessions.length) selectFlightSession(sessions[sessions.length - 1], body.firstElementChild);
}

function selectFlightSession(session, row) {
  flightLogSelected = session;
  for (const r of $('flightLogSessions').querySelectorAll('tbody tr')) r.classList.toggle('selected', r === row);
  drawFlightChart(session);
  syncFlightLogControls(serialConsole.state === 'connected');
}

function drawFlightChart(session) {
  const canvas = $('flightLogChart');
  const sec = session.seconds;
  canvas.hidden = !sec.length;
  $('flightLogLegend').hidden = !sec.length;
  if (!sec.length) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height, pad = 28;
  const css = getComputedStyle(document.documentElement);
  const color = (name, fallback) => css.getPropertyValue(name).trim() || fallback;
  ctx.clearRect(0, 0, w, h);
  const t0 = sec[0].t, t1 = sec[sec.length - 1].t || t0 + 1;
  const x = (t) => pad + ((t - t0) / Math.max(1, t1 - t0)) * (w - 2 * pad);
  const y = (v) => h - pad - (Math.max(0, Math.min(25, v)) / 25) * (h - 2 * pad);   /* 0..25 on one axis */
  ctx.strokeStyle = color('--border', '#888');
  ctx.globalAlpha = 0.4;
  for (let v = 0; v <= 25; v += 5) { ctx.beginPath(); ctx.moveTo(pad, y(v)); ctx.lineTo(w - pad, y(v)); ctx.stroke(); }
  ctx.globalAlpha = 1;
  ctx.fillStyle = color('--text-muted', '#888');
  ctx.font = '11px system-ui, sans-serif';
  for (let v = 0; v <= 25; v += 5) ctx.fillText(String(v), 4, y(v) + 4);
  ctx.fillText(`${Math.round((t1 - t0) / 60)} min`, w - pad - 30, h - 8);
  ctx.fillStyle = 'rgba(220, 60, 60, 0.25)';
  for (const s of sec) if (s.dropout) ctx.fillRect(x(s.t) - 1, pad, 3, h - 2 * pad);
  const series = [
    ['#2f8fdf', (s) => (s.lines > 0 ? s.snr : null)],
    ['#3aa76d', (s) => s.gain / 4],
    ['#e08a1e', (s) => s.dc * 5],
    ['#9a6ad6', (s) => (s.temp === null ? null : s.temp / 4)],
    ['#c8463c', (s) => (s.level === null ? null : (s.level + 100) / 2)],
    ['#00a6a6', (s) => (s.nativeHLockPct === null ? null : s.nativeHLockPct / 4)]
  ];
  for (const [stroke, value] of series) {
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let pen = false;
    for (const s of sec) {
      const v = value(s);
      if (v === null) { pen = false; continue; }
      if (pen) ctx.lineTo(x(s.t), y(v)); else ctx.moveTo(x(s.t), y(v));
      pen = true;
    }
    ctx.stroke();
  }
}

$('btnReadFlightLog').addEventListener('click', async () => {
  if (serialConsole.state !== 'connected' || commandBusy) return;
  flightLog.start();
  flightLogResult = null;
  flightLogStatus.textContent = 'Reading flight log…';
  syncFlightLogControls(true);
  commandBusy = true;
  try { await serialConsole.send('W'); } finally { commandBusy = false; syncDeviceControls(); }
});
$('btnEraseFlightLog').addEventListener('click', async () => {
  if (serialConsole.state !== 'connected' || commandBusy) return;
  if (!window.confirm('Erase the whole flight log on the receiver? Download it first if you want to keep it.')) return;
  commandBusy = true;
  try { await serialConsole.send('X'); await new Promise((r) => setTimeout(r, 300)); await serialConsole.send('X'); }
  finally { commandBusy = false; syncDeviceControls(); }
  flightLog.reset();
  flightLogResult = null;
  $('flightLogSessions').hidden = true;
  $('flightLogChart').hidden = true;
  $('flightLogLegend').hidden = true;
  flightLogStatus.textContent = 'Erasing the flight log (about 20 s)…';
});
$('btnDownloadFlightLog').addEventListener('click', () => {
  if (flightLog.done) download(`c5vrx-flight-${new Date().toISOString().replace(/[:.]/g, '-')}.c5log`, flightLog.image, 'application/octet-stream');
});
$('btnDownloadFlightCsv').addEventListener('click', () => {
  const s = flightLogSelected ?? flightLogResult?.sessions.at(-1);
  if (s) download(`c5vrx-session-${s.session}.csv`, secondsCsv(s), 'text/csv;charset=utf-8');
});

// Receiver menu pages: browsing never sends anything.
const menuTabs = [...document.querySelectorAll('[data-menu]')];
function selectMenu(tab, focus = false) {
  for (const item of menuTabs) {
    const active = item === tab;
    item.setAttribute('aria-selected', String(active));
    item.tabIndex = active ? 0 : -1;
    $(`pane-${item.dataset.menu}`).hidden = !active;
  }
  if (focus) tab.focus();
}
const MENU_KEYS = { ArrowDown: 1, ArrowRight: 1, ArrowUp: -1, ArrowLeft: -1 };
for (const tab of menuTabs) {
  tab.addEventListener('click', () => selectMenu(tab));
  tab.addEventListener('keydown', (event) => {
    const index = menuTabs.indexOf(tab);
    const target = event.key in MENU_KEYS ? (index + MENU_KEYS[event.key] + menuTabs.length) % menuTabs.length
      : event.key === 'Home' ? 0 : event.key === 'End' ? menuTabs.length - 1 : -1;
    if (target < 0) return;
    event.preventDefault();
    selectMenu(menuTabs[target], true);
  });
}

// Firmware / Terminal views; #terminal links straight to the terminal.
function showView(terminalView) {
  $('firmwareView').hidden = terminalView;
  $('terminalView').hidden = !terminalView;
  for (const [id, active] of [['firmwareTab', !terminalView], ['terminalTab', terminalView]]) {
    $(id).classList.toggle('active', active);
    $(id).setAttribute('aria-pressed', String(active));
  }
  if (globalThis.history?.replaceState) history.replaceState(null, '', terminalView ? '#terminal' : location.pathname + location.search);
}
$('firmwareTab').addEventListener('click', () => showView(false));
$('terminalTab').addEventListener('click', () => showView(true));

navigator.serial?.addEventListener('disconnect', async (event) => {
  if (event.target === serialConsole.port) await serialConsole.disconnect();
  if (event.target === port && !isFlashing && !isConnecting) await disconnectDevice();
});

// Keep docs visible on desktop and collapsible on smaller screens.
const docsBreakpoint = window.matchMedia('(min-width:801px)');
const docsNav = $('docsNav');
docsNav.open = docsBreakpoint.matches;
docsBreakpoint.addEventListener('change', (event) => { docsNav.open = event.matches; });

document.addEventListener('DOMContentLoaded', () => {
  log('C5VRX web flasher ready.');
  checkSerialSupport();
  if (globalThis.location?.hash === '#terminal') showView(true);
  renderReceiver();
  syncDeviceControls();
  fetchReleases();
});

// A dedicated read-only report, independent of the terminal log size limit.
function calibrationReportFeed(text) {
  if (!calibrationCapture.push(text)) return;
  if (calibrationCapture.error) {
    clearTimeout(calibrationTimer);
    $('calibrationReportStatus').textContent = calibrationCapture.error;
  } else if (calibrationCapture.done) {
    clearTimeout(calibrationTimer);
    const report = calibrationCapture.report(calibrationRequest || {});
    const device = report.records.find(r => r.type === 'device');
    const suffix = report.capturedAt.replace(/[:.]/g, '-');
    download(`c5vrx-calibration-${device?.board_id || 'board'}-${suffix}.json`, JSON.stringify(report, null, 2), 'application/json');
    $('calibrationReportStatus').textContent = 'Report downloaded. Send this JSON file to Leon. No calibration was started.';
  }
}

$('btnCalibrationReport').addEventListener('click', async () => {
  if (serialConsole.state !== 'connected' || commandBusy || calibrationCapture.active ||
      receiverConsole.receiverBusy() || flightLog.active || !receiverConsole.supports('C')) return;
  calibrationRequest = { boardLabel: $('calibrationBoard').value, vtx: $('calibrationVtx').value || 'unknown', notes: $('calibrationNotes').value };
  calibrationCapture.start();
  $('calibrationReportStatus').textContent = 'Reading calibration report…';
  syncDeviceControls();
  calibrationTimer = setTimeout(() => {
    if (!calibrationCapture.active) return;
    calibrationCapture.fail('Report timed out. Check the connection and firmware, then try again.');
    $('calibrationReportStatus').textContent = calibrationCapture.error;
    syncDeviceControls();
  }, 30000);
  if (!await serialConsole.send('C')) {
    clearTimeout(calibrationTimer);
    calibrationCapture.fail('Could not request the report. Reconnect and try again.');
    $('calibrationReportStatus').textContent = calibrationCapture.error;
    syncDeviceControls();
  }
});

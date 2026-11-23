import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, writeFileSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { spawnSync } from 'node:child_process';

// Exercise the production packager with real jq and mocked GitHub responses.
function prepare({ empty = false, retry = false, failDownload = false } = {}) {
  const dir = mkdtempSync(join(tmpdir(), 'c5vrx-pages-'));
  const release = { tag_name: 'c5vrx5-v5.0.387', draft: false,
    prerelease: false, published_at: '2026-10-22T12:00:00Z',
    assets: ['c5vrx5.bin', 'c5vrx5_merged.bin'].map(name => ({ name })) };
  const hwx = { ...release, tag_name: 'c5vrx5-hwx-fpga_iq-pr-12', prerelease: true,
    published_at: '2026-10-21T12:00:00Z' };
  // Eleven versions of one experiment: the mirror keeps the newest ten.
  const versions = Array.from({ length: 11 }, (_, i) => ({ ...release, prerelease: true,
    tag_name: `c5vrx5-hwx-fpga_iq-v5.0.${100 + i}`, published_at: `2026-10-${10 + i}T12:00:00Z` }));
  const unversioned = { ...release, tag_name: 'c5vrx5-hwx-fpga_iq', prerelease: true };
  // Two separate page arrays: gh --paginate output, without --slurp.
  writeFileSync(join(dir, 'pages.json'),
    `${JSON.stringify([release, unversioned])}\n${JSON.stringify([hwx, ...versions])}\n`);
  writeFileSync(join(dir, 'gh'), `#!/usr/bin/env bash
set -eu
if [[ "$1" == api ]]; then
  [[ " $* " != *" --slurp "* ]]
  if [[ "$EMPTY" == 1 ]] || { [[ "$RETRY" == 1 ]] && [[ ! -f "$MOCK_DIR/retried" ]]; }; then
    touch "$MOCK_DIR/retried"
    echo '[]'
  else
    cat "$MOCK_DIR/pages.json"
  fi
elif [[ "$1 $2" == 'release download' ]]; then
  [[ "$FAIL_DOWNLOAD" != 1 ]] || exit 1
  while [[ "$1" != --dir ]]; do shift; done
  printf firmware > "$2/c5vrx5.bin"
  printf firmware > "$2/c5vrx5_merged.bin"
else
  exit 2
fi
`, { mode: 0o755 });
  try {
    const result = spawnSync('bash', ['tools/prepare_pages_site.sh', join(dir, 'site')], {
      cwd: new URL('../', import.meta.url), encoding: 'utf8',
      env: { ...process.env, PATH: `${dir}:${process.env.PATH}`, GH_TOKEN: 'mock',
        GITHUB_REPOSITORY: 'Twotoz/C5VRX', MOCK_DIR: dir,
        EMPTY: empty ? '1' : '0', RETRY: retry ? '1' : '0',
        FAIL_DOWNLOAD: failDownload ? '1' : '0' }
    });
    if (result.status === 0) {
      const index = JSON.parse(readFileSync(join(dir, 'site/firmware/releases.json')));
      assert.equal(index[0].tag_name, release.tag_name);
      assert.equal(index[0].assets[0].local_url, `firmware/${release.tag_name}/c5vrx5.bin`);
      assert.deepEqual(index.map(rel => rel.tag_name), [release.tag_name,
        ...versions.slice(1).reverse().map(rel => rel.tag_name), hwx.tag_name]);
    }
    return result;
  } finally { rmSync(dir, { recursive: true, force: true }); }
}

test('Pages packager combines paginated release arrays and mirrors main firmware', () => {
  const result = prepare();
  assert.equal(result.status, 0, result.stderr);
});
test('Pages packager retries a transient empty listing', () => {
  const result = prepare({ retry: true });
  assert.equal(result.status, 0, result.stderr);
  assert.match(result.stderr, /retrying/);
});
test('Pages packager refuses to deploy an empty release listing', () => {
  const result = prepare({ empty: true });
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /refusing an empty firmware deployment/);
});
test('Pages packager refuses to deploy when latest main firmware download fails', () => {
  const result = prepare({ failDownload: true });
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /refusing deployment/);
});

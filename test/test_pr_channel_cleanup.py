#!/usr/bin/env python3
"""C5VRX by Twotoz/contributors: execute the cleanup shell with a mocked gh."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = (ROOT / '.github/workflows/pr-channel-cleanup.yml').read_text()
MOCK = r'''#!/usr/bin/env python3
import json, os, re, sys
from pathlib import Path
args=sys.argv[1:]
root=Path(os.environ['MOCK_ROOT'])
data=json.loads((root/'data.json').read_text())
with (root/'calls.jsonl').open('a') as f: f.write(json.dumps(args)+'\n')
if args[:2]==['api','--paginate']:
    if data.get('list_error'): sys.exit(1)
    assert '--jq' in args and 'select(test(' in args[-1]
    for tag in data['tags']:
        if re.fullmatch(r'c5vrx5-(hwx-[a-z0-9_]+-)?pr-[0-9]+',tag): print(tag)
elif args[:2]==['pr','list']:
    assert '--state' in args and args[args.index('--state')+1]=='open'
    if data.get('pr_list_error'): sys.exit(1)
    for pr,state in data['states'].items():
        if state=='open': print(pr)
elif args[:2]==['release','delete']:
    assert args[-2:]==['--yes','--cleanup-tag']
    if data.get('delete_error') is True or args[2] in (data.get('delete_error') or []): sys.exit(1)
else:
    raise AssertionError(args)
'''


class CleanupTest(unittest.TestCase):
    def execute(self, tags=(), states=None, **extra):
        lines = WORKFLOW.split('        run: |\n', 1)[1].splitlines()
        script = []
        for line in lines:
            if line and not line.startswith('          '):
                break
            script.append(line[10:])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'gh').write_text(MOCK)
            (root / 'gh').chmod(0o755)
            (root / 'data.json').write_text(json.dumps(dict(tags=tags, states=states or {}, **extra)))
            output = root / 'output'
            output.touch()
            result = subprocess.run(['bash', '-c', '\n'.join(script)], env={
                **os.environ, 'PATH': tmp + os.pathsep + os.environ['PATH'],
                'MOCK_ROOT': tmp, 'GITHUB_REPOSITORY': 'Twotoz/C5VRX',
                'GITHUB_OUTPUT': str(output)}, capture_output=True, text=True)
            calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()]
            deletes = [args[2] for args in calls if args[:2] == ['release', 'delete']]
            return result.returncode, deletes, output.read_text()

    def test_only_closed_pr_channels_are_removed(self):
        rc, deleted, output = self.execute(
            ['c5vrx5-v5.0.573', 'v3.1.0', 'c5vrx5-pr-299', 'c5vrx5-pr-276'],
            {'299': 'open', '276': 'closed'})
        self.assertEqual((rc, deleted, output), (0, ['c5vrx5-pr-276'], 'removed=true\n'))

    def test_hardware_experiment_pr_channels_are_removed(self):
        rc, deleted, output = self.execute(
            ['c5vrx5-hwx-fpga_iq', 'c5vrx5-hwx-fpga_iq-pr-299', 'c5vrx5-hwx-fpga_iq-pr-276'],
            {'299': 'open', '276': 'closed'})
        self.assertEqual((rc, deleted, output), (0, ['c5vrx5-hwx-fpga_iq-pr-276'], 'removed=true\n'))

    def test_empty_list_is_success(self):
        self.assertEqual(self.execute(), (0, [], ''))

    def test_listing_failure_is_not_hidden(self):
        rc, deleted, output = self.execute(list_error=True)
        self.assertNotEqual(rc, 0)
        self.assertEqual((deleted, output), ([], ''))

    def test_failed_open_pr_listing_never_deletes(self):
        rc, deleted, output = self.execute(['c5vrx5-pr-276'], {'276': 'closed'}, pr_list_error=True)
        self.assertNotEqual(rc, 0)
        self.assertEqual((deleted, output), ([], ''))

    def test_channel_of_a_pr_the_api_cannot_find_is_removed(self):
        # A per-PR lookup used to abort the sweep on HTTP 404 (every run from
        # 2026-11-13 on); a PR that is not in the open listing is not open.
        rc, deleted, output = self.execute(['c5vrx5-pr-276', 'c5vrx5-pr-296'], {'296': 'open'})
        self.assertEqual((rc, deleted, output), (0, ['c5vrx5-pr-276'], 'removed=true\n'))

    def test_partial_failure_preserves_refresh_signal(self):
        rc, deleted, output = self.execute(
            ['c5vrx5-pr-276', 'c5vrx5-pr-273'], {}, delete_error=['c5vrx5-pr-273'])
        self.assertNotEqual(rc, 0)
        self.assertEqual((deleted, output), (['c5vrx5-pr-276', 'c5vrx5-pr-273'], 'removed=true\n'))
        self.assertIn("always() && steps.cleanup.outputs.removed == 'true'", WORKFLOW)

    def test_delete_failure_does_not_claim_success(self):
        rc, _, output = self.execute(['c5vrx5-pr-276'], {'276': 'closed'}, delete_error=True)
        self.assertNotEqual(rc, 0)
        self.assertEqual(output, '')

    def test_permission_and_trusted_workflow(self):
        self.assertIn('  pull-requests: read', WORKFLOW)
        self.assertNotIn('actions/checkout', WORKFLOW)
        self.assertNotIn('gh pr view', WORKFLOW)
        self.assertIn('--ref main', WORKFLOW)


if __name__ == '__main__':
    unittest.main()

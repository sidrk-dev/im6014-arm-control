import test from 'node:test';
import assert from 'node:assert/strict';
import { applyConnectionSettings, validateConnectionSettings, draftNumber } from '../arm_web/presentation.js';

const settings = { kp: 30, kd: 3, max_rpm: 5, effort_limit_nm: 4,
  known_angle_deg: 0, direction: 1, lower_deg: -90, upper_deg: 90 };

test('Connect applies all selected settings and fresh reference declarations without arming', async () => {
  const calls = [];
  const entries = ['J1', 'J2', 'J3'].map(joint => ({ joint, settings: { ...settings } }));
  await applyConnectionSettings(async (name, values) => { calls.push({ name, values }); }, entries, true);
  assert.deepEqual(calls.map(call => call.name), ['ack', ...entries.flatMap(() => ['configure', 'check', 'calibrate', 'check'])]);
  assert.deepEqual(calls.filter(call => call.name === 'calibrate').map(call => call.values.joint), ['J1', 'J2', 'J3']);
  assert.equal(calls.some(call => call.name === 'arm'), false);
});

test('Invalid pose declarations and physical application limits reject before any action', async () => {
  for (const change of [{ known_angle_deg: 100 }, { upper_deg: -90 }, { max_rpm: 101 },
    { effort_limit_nm: 16 }, { kp: NaN }, { kp: 0 }, { lower_deg: -91 }]) {
    let calls = 0;
    await assert.rejects(applyConnectionSettings(async () => { calls++; }, [{ joint: 'J1', settings: { ...settings, ...change } }], true));
    assert.equal(calls, 0);
  }
  assert.doesNotThrow(() => validateConnectionSettings([{ joint: 'J1', settings: { ...settings, max_rpm: 100, effort_limit_nm: 15 } }]));
});

test('A setup failure stops the sequence before subsequent calibration or arming', async () => {
  const calls = [];
  await assert.rejects(applyConnectionSettings(async name => {
    calls.push(name);
    if (name === 'configure') throw new Error('J1 configuration rejected');
  }, [{ joint: 'J1', settings }], false), /J1 configuration rejected/);
  assert.deepEqual(calls, ['configure']);
});

 test('Blank drafts stay invalid for every joint instead of becoming a zero declaration', async () => {
  for (const joint of ['J1', 'J2', 'J3']) {
    for (const key of Object.keys(settings).filter(key => key !== 'direction')) {
      let calls = 0;
      const entries = ['J1', 'J2', 'J3'].map(name => ({ joint: name, settings: { ...settings, ...(name === joint ? { [key]: draftNumber('') } : {}) } }));
      await assert.rejects(applyConnectionSettings(async () => { calls++; }, entries, false));
      assert.equal(calls, 0);
    }
  }
  assert.equal(draftNumber('45'), 45);
  assert.equal(Number.isNaN(draftNumber('  ')), true);
});

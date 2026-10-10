/** Behavior tests for the pure input reducer and dashboard request queue. */

import assert from 'node:assert/strict';
import {
  createInputState,
  reduceKeyEvent,
  reduceGamepad,
  reduceArmState,
  clearStopLatch,
  validateGamepad,
  InputClient
} from '../arm_web/input_reducer.js';

function gamepad(overrides = {}) {
  return {
    index: 0,
    id: 'Xbox Controller',
    mapping: 'standard',
    connected: true,
    axes: [0, 0, 0, 0],
    buttons: Array.from({ length: 6 }, () => ({ pressed: false, value: 0 })),
    ...overrides
  };
}

function testKeyboardAndPostArmGate() {
  const state = createInputState();
  reduceKeyEvent(state, { type: 'keydown', code: 'KeyD' });
  reduceKeyEvent(state, { type: 'keydown', code: 'Space' });
  assert.deepEqual(state.keyboardAxes, [1, 0, 0]);
  assert.equal(state.keyboardDeadman, true);

  reduceArmState(state, true);
  assert.equal(state.effectiveDeadman, false, 'held controls cannot energize on Arm');
  reduceKeyEvent(state, { type: 'keyup', code: 'KeyD' });
  reduceKeyEvent(state, { type: 'keyup', code: 'Space' });
  assert.equal(state.neutralLatched, true, 'fresh neutral after Arm opens the gate');
  reduceKeyEvent(state, { type: 'keydown', code: 'KeyD' });
  assert.equal(state.effectiveDeadman, true);
  assert.deepEqual(state.effectiveAxes, [1, 0, 0]);
  reduceKeyEvent(state, { type: 'keyup', code: 'KeyD' });
  assert.deepEqual(state.effectiveAxes, [0, 0, 0]);

  // Keyup clears a previously held jog key even after focus enters an editor.
  reduceKeyEvent(state, { type: 'keyup', code: 'KeyD', isEditing: true });
  assert.deepEqual(state.keyboardAxes, [0, 0, 0]);
  reduceKeyEvent(state, { type: 'keydown', code: 'Escape', isEditing: true });
  assert.equal(state.stopLatched, true);
  assert.deepEqual(state.effectiveAxes, [0, 0, 0]);
  clearStopLatch(state);
  assert.equal(state.stopLatched, false);
}

function testGamepadSelectionValidationAndStop() {
  const state = createInputState();
  assert.equal(validateGamepad(gamepad()).valid, true);
  reduceGamepad(state, gamepad());
  assert.equal(state.stopLatched, true, 'new controller identity requires Stop/Check recovery');
  assert.equal(state.gamepadConnected, true);
  clearStopLatch(state);

  // Stick jitter alone does not take control from the keyboard.
  reduceGamepad(state, gamepad({ axes: [0.5, 0, 0, 0] }));
  assert.equal(state.activeDevice, 'keyboard');
  assert.equal(state.stopLatched, false);

  const buttons = gamepad().buttons;
  buttons[5] = { pressed: true, value: 1 };
  reduceGamepad(state, gamepad({ axes: [0.6, 0.4, 0, 0.8], buttons }));
  assert.equal(state.activeDevice, 'gamepad', 'RB explicitly selects the gamepad');
  assert.equal(state.gamepadDeadman, true);
  assert.ok(Math.abs(state.gamepadAxes[0] - ((0.6 - 0.12) / 0.88)) < 1e-9);
  assert.ok(Math.abs(state.gamepadAxes[1] - (-(0.4 - 0.12) / 0.88)) < 1e-9);
  assert.ok(Math.abs(state.gamepadAxes[2] - (-(0.8 - 0.12) / 0.88)) < 1e-9);

  const stopButtons = gamepad().buttons;
  stopButtons[1] = { pressed: true, value: 1 };
  reduceGamepad(state, gamepad({ buttons: stopButtons }));
  assert.equal(state.stopLatched, true, 'B is an immediate Stop');
  clearStopLatch(state);

  reduceGamepad(state, null);
  assert.equal(state.stopLatched, true, 'disconnect latches Stop');
  clearStopLatch(state);
  reduceGamepad(state, gamepad({ id: 'Replacement controller' }));
  assert.equal(state.stopLatched, true, 'identity replacement latches Stop');
  clearStopLatch(state);

  assert.equal(validateGamepad(gamepad({ mapping: '' })).valid, false);
  assert.equal(validateGamepad(gamepad({ mapping: 'standard' })).valid, true);
  assert.equal(validateGamepad(gamepad({ connected: false })).valid, false);
  assert.equal(validateGamepad(gamepad({ axes: [0, NaN, 0, 0] })).valid, false);
  assert.equal(validateGamepad(gamepad({ index: -1 })).valid, false);
}

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

function response(data, ok = true, status = ok ? 200 : 400) {
  return { ok, status, json: async () => data };
}

function makeClient(fetchImpl) {
  globalThis.fetch = fetchImpl;
  const client = new InputClient('http://127.0.0.1:8768');
  client.setSession('test-session');
  return client;
}

async function testNewestInputWinsWhileTicketIsIssued() {
  const ticketReply = deferred();
  const calls = [];
  const client = makeClient(async (url, options) => {
    calls.push({ url, options, body: JSON.parse(options.body) });
    if (url.endsWith('/api/ticket')) return ticketReply.promise;
    if (url.endsWith('/api/input')) return response({ ok: true });
    if (url.endsWith('/api/stop')) return response({ ok: true });
    throw new Error(`Unexpected request ${url}`);
  });

  const first = client.submitJog(true, [0.1, 0, 0], 'keyboard');
  await Promise.resolve();
  await client.submitJog(true, [0.7, -0.2, 0.3], 'keyboard');
  ticketReply.resolve(response({ ticket: 'fresh-ticket' }));
  assert.equal(await first, true);

  const submitted = calls.find(call => call.url.endsWith('/api/input')).body;
  assert.equal(submitted.ticket, 'fresh-ticket');
  assert.deepEqual(submitted.axes, [0.7, -0.2, 0.3]);
  assert.equal(submitted.seq, 0);
  assert.equal(client.ordinaryInFlight, false);
}

async function testStopDuringDelayedTicketInvalidatesRunWithoutUnlockingEarly() {
  const ticketReply = deferred();
  const calls = [];
  const client = makeClient(async (url, options) => {
    calls.push({ url, options });
    if (url.endsWith('/api/ticket')) return ticketReply.promise;
    if (url.endsWith('/api/stop')) return response({ ok: true });
    throw new Error(`Unexpected request ${url}`);
  });

  const input = client.submitJog(true, [1, 0, 0]);
  await Promise.resolve();
  const ticketCall = calls.find(call => call.url.endsWith('/api/ticket'));
  const stop = client.triggerStop('race_test');
  assert.equal(ticketCall.options.signal.aborted, true);
  assert.equal(client.ordinaryInFlight, true, 'old promise owns the ordinary lock until it settles');
  await stop;

  ticketReply.resolve(response({ ticket: 'stale-ticket' }));
  await assert.rejects(input, /invalidated/i);
  assert.equal(calls.some(call => call.url.endsWith('/api/input')), false);
  assert.equal(client.ordinaryInFlight, false);
  assert.equal(client.stopLatched, true);
}

async function testTicketedActionAndStopRace() {
  const calls = [];
  const client = makeClient(async (url, options) => {
    const body = JSON.parse(options.body);
    calls.push({ url, options, body });
    if (url.endsWith('/api/ticket')) return response({ ticket: 'action-ticket' });
    if (url.endsWith('/api/action')) return response({ operation: 'operation-1' });
    if (url.endsWith('/api/receipt')) return response({ status: 'complete' });
    if (url.endsWith('/api/stop')) return response({ ok: true });
    throw new Error(`Unexpected request ${url}`);
  });
  await client.triggerStop('before_check');
  const receipt = await client.requestAction('check');
  assert.equal(receipt.status, 'complete');
  const action = calls.find(call => call.url.endsWith('/api/action'));
  assert.equal(action.body.ticket, 'action-ticket');
  assert.equal(client.stopLatched, false, 'successful Check clears the client stop latch');

  const delayedTicket = deferred();
  const raceCalls = [];
  const racingClient = makeClient(async (url, options) => {
    raceCalls.push({ url, options });
    if (url.endsWith('/api/ticket')) return delayedTicket.promise;
    if (url.endsWith('/api/stop')) return response({ ok: true });
    throw new Error(`Unexpected request ${url}`);
  });
  const pendingAction = racingClient.requestAction('arm', {});
  await Promise.resolve();
  const stop = racingClient.triggerStop('during_action_ticket');
  await stop;
  delayedTicket.resolve(response({ ticket: 'stale-action-ticket' }));
  await assert.rejects(pendingAction, /invalidated/i);
  assert.equal(raceCalls.some(call => call.url.endsWith('/api/action')), false);
}

async function testRejectedTicketTriggersIndependentStop() {
  const calls = [];
  const client = makeClient(async (url, options) => {
    calls.push(url);
    if (url.endsWith('/api/ticket')) return response({ reason: 'ticket denied' }, false);
    if (url.endsWith('/api/stop')) return response({ ok: true });
    throw new Error(`Unexpected request ${url}`);
  });
  await assert.rejects(client.submitJog(true, [0.3, 0, 0]), /ticket denied/);
  assert.equal(calls.some(url => url.endsWith('/api/stop')), true);
  assert.equal(client.stopLatched, true);
}

await testNewestInputWinsWhileTicketIsIssued();
await testStopDuringDelayedTicketInvalidatesRunWithoutUnlockingEarly();
await testTicketedActionAndStopRace();
await testRejectedTicketTriggersIndependentStop();
testKeyboardAndPostArmGate();
testGamepadSelectionValidationAndStop();
console.log('Input reducer and request lifecycle behavior tests passed.');

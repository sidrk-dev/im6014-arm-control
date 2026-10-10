/** Pure input reducer and bounded HTTP client for the arm dashboard. */

const ZERO_AXES = () => [0, 0, 0];
const JOG_KEYS = new Set(['KeyA', 'KeyD', 'KeyW', 'KeyS', 'KeyQ', 'KeyE', 'Space']);

export function createInputState() {
  return {
    armed: false,
    neutralLatched: false,
    motionEnergized: false,
    stopLatched: false,
    stopRequested: false,
    stopReason: null,
    keysDown: new Set(),
    isEditing: false,
    keyboardAxes: ZERO_AXES(),
    keyboardDeadman: false,
    gamepadAxes: ZERO_AXES(),
    gamepadDeadman: false,
    lastGamepadIdentity: null,
    gamepadConnected: false,
    activeDevice: 'keyboard',
    effectiveAxes: ZERO_AXES(),
    effectiveDeadman: false,
    isEnergized: false
  };
}

function clearHeldInput(state) {
  state.keysDown.clear();
  state.keyboardAxes = ZERO_AXES();
  state.keyboardDeadman = false;
  state.gamepadAxes = ZERO_AXES();
  state.gamepadDeadman = false;
  state.motionEnergized = false;
  state.isEnergized = false;
}

export function updateEffectiveState(state) {
  if (state.stopLatched || state.stopRequested || !state.armed) {
    state.effectiveAxes = ZERO_AXES();
    state.effectiveDeadman = false;
    if (!state.armed || state.stopLatched) {
      state.neutralLatched = false;
      state.motionEnergized = false;
      state.isEnergized = false;
    }
    return state;
  }

  const gamepadActive = state.activeDevice === 'gamepad' && state.gamepadConnected;
  const rawDeadman = gamepadActive ? state.gamepadDeadman : state.keyboardDeadman;
  const rawAxes = gamepadActive ? state.gamepadAxes : state.keyboardAxes;
  const hasAxisInput = rawAxes.some(axis => axis !== 0);
  const isNeutral = !rawDeadman && !hasAxisInput;

  if (!state.neutralLatched) {
    if (!isNeutral) {
      state.effectiveAxes = ZERO_AXES();
      state.effectiveDeadman = false;
      state.motionEnergized = false;
      state.isEnergized = false;
      return state;
    }
    state.neutralLatched = true;
  }

  const isEnergized = state.neutralLatched && (hasAxisInput || rawDeadman);
  state.motionEnergized = isEnergized;
  state.isEnergized = isEnergized;
  state.effectiveDeadman = isEnergized;
  state.effectiveAxes = isEnergized ? [...rawAxes] : ZERO_AXES();
  return state;
}

function updateKeyboardAxes(state) {
  state.keyboardAxes = [
    Number(state.keysDown.has('KeyD')) - Number(state.keysDown.has('KeyA')),
    Number(state.keysDown.has('KeyW')) - Number(state.keysDown.has('KeyS')),
    Number(state.keysDown.has('KeyE')) - Number(state.keysDown.has('KeyQ'))
  ];
  state.keyboardDeadman = state.keysDown.has('Space');
}

export function latchStop(state, reason = 'operator_stop') {
  const alreadyLatched = state.stopLatched;
  state.stopLatched = true;
  state.stopRequested = !alreadyLatched;
  if (!alreadyLatched) state.stopReason = String(reason);
  state.armed = false;
  state.neutralLatched = false;
  clearHeldInput(state);
  return updateEffectiveState(state);
}

export function clearStopRequest(state) {
  state.stopRequested = false;
  return updateEffectiveState(state);
}

/** Call only after a successful operator Check lifecycle action. */
export function clearStopLatch(state) {
  state.stopLatched = false;
  state.stopRequested = false;
  state.stopReason = null;
  state.armed = false;
  state.neutralLatched = false;
  clearHeldInput(state);
  return updateEffectiveState(state);
}

export function reduceKeyEvent(state, event) {
  const { type, code, isEditing = false } = event;
  state.isEditing = Boolean(isEditing);

  if (type === 'keydown' && code === 'Escape') {
    return latchStop(state, 'keyboard_escape');
  }


  if (type === 'keyup') {
    state.keysDown.delete(code);
    updateKeyboardAxes(state);
    return updateEffectiveState(state);
  }
  if (type !== 'keydown' || isEditing || !JOG_KEYS.has(code)) return state;

  state.keysDown.add(code);
  state.activeDevice = 'keyboard';
  updateKeyboardAxes(state);
  return updateEffectiveState(state);
}

export function validateGamepad(pad) {
  if (!pad || typeof pad !== 'object') return { valid: false, reason: 'gamepad_invalid' };
  if (pad.connected !== true) return { valid: false, reason: 'gamepad_disconnected' };
  if (pad.mapping !== 'standard') return { valid: false, reason: 'unsupported_mapping' };
  if (!Number.isInteger(pad.index) || pad.index < 0) return { valid: false, reason: 'gamepad_invalid_index' };
  if (typeof pad.id !== 'string' || !pad.id.trim()) return { valid: false, reason: 'gamepad_invalid_id' };
  if (!Array.isArray(pad.axes) || pad.axes.length < 4) return { valid: false, reason: 'unsupported_axes_count' };
  if (!pad.axes.every(axis => Number.isFinite(axis) && axis >= -1 && axis <= 1)) {
    return { valid: false, reason: 'invalid_axis' };
  }
  if (!Array.isArray(pad.buttons) || pad.buttons.length < 6) {
    return { valid: false, reason: 'unsupported_buttons_count' };
  }
  if (!pad.buttons.every(button => button && typeof button === 'object'
    && typeof button.pressed === 'boolean'
    && Number.isFinite(button.value) && button.value >= 0 && button.value <= 1)) {
    return { valid: false, reason: 'invalid_button' };
  }
  return { valid: true };
}

function applyDeadzone(axis, deadzone = 0.12) {
  const magnitude = Math.abs(axis);
  if (magnitude <= deadzone) return 0;
  return Math.sign(axis) * (magnitude - deadzone) / (1 - deadzone);
}

export function reduceGamepad(state, pad) {
  if (!pad) {
    const wasConnected = state.gamepadConnected;
    state.gamepadConnected = false;
    state.lastGamepadIdentity = null;
    state.gamepadAxes = ZERO_AXES();
    state.gamepadDeadman = false;
    return wasConnected ? latchStop(state, 'gamepad_disconnected') : updateEffectiveState(state);
  }

  const validation = validateGamepad(pad);
  if (!validation.valid) {
    state.gamepadConnected = false;
    state.lastGamepadIdentity = null;
    state.gamepadAxes = ZERO_AXES();
    state.gamepadDeadman = false;
    return latchStop(state, validation.reason);
  }

  const identity = `${pad.index}:${pad.id}`;
  if (!state.gamepadConnected || state.lastGamepadIdentity !== identity) {
    state.gamepadConnected = true;
    state.lastGamepadIdentity = identity;
    state.gamepadAxes = ZERO_AXES();
    state.gamepadDeadman = false;
    return latchStop(state, 'gamepad_connected_or_replaced');
  }

  if (pad.buttons[1].pressed) return latchStop(state, 'gamepad_b_stop');

  state.gamepadDeadman = Boolean(pad.buttons[5]?.pressed);
  state.gamepadAxes = [
    applyDeadzone(pad.axes[0]),
    applyDeadzone(-pad.axes[1]),
    applyDeadzone(-pad.axes[3])
  ];

  if (state.gamepadDeadman) state.activeDevice = 'gamepad';
  return updateEffectiveState(state);
}

export function reduceBlur(state) {
  clearHeldInput(state);
  state.activeDevice = 'keyboard';
  return updateEffectiveState(state);
}

export function reduceVisibilityChange(state, isHidden) {
  return isHidden ? reduceBlur(state) : state;
}

export function reduceArmState(state, isArmed) {
  if (state.stopLatched) {
    state.armed = false;
    return updateEffectiveState(state);
  }
  const next = isArmed === true;
  state.armed = next;
  if (!next) {
    state.neutralLatched = false;
    state.motionEnergized = false;
  } else {
    const gamepadActive = state.activeDevice === 'gamepad' && state.gamepadConnected;
    const rawDeadman = gamepadActive ? state.gamepadDeadman : state.keyboardDeadman;
    const rawAxes = gamepadActive ? state.gamepadAxes : state.keyboardAxes;
    state.neutralLatched = !rawDeadman && rawAxes.every(axis => axis === 0);
    state.motionEnergized = false;
  }
  return updateEffectiveState(state);
}

function currentOrigin(apiBase) {
  if (apiBase) {
    try {
      return new URL(apiBase).origin;
    } catch {
      return '';
    }
  }
  return globalThis.window?.location?.origin || globalThis.location?.origin || '';
}

function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

/** Serializes normal ticket/action/input traffic; Stop has its own HTTP path. */
export class InputClient {
  constructor(apiBase = '') {
    this.apiBase = apiBase;
    this.session = null;
    this.queueGeneration = 0;
    this.seq = 0;
    this.ordinaryRun = null;
    this.pendingInput = null;
    this.stopLatched = false;
    this.motionSuppressed = false;
  }

  get ordinaryInFlight() {
    return this.ordinaryRun !== null;
  }

  setSession(session) {
    if (typeof session !== 'string' || !session) throw new Error('Invalid session');
    if (this.session && this.session !== session) {
      this.queueGeneration++;
      this.pendingInput = null;
      this.ordinaryRun?.controller?.abort();
      this.stopLatched = true;
      this.motionSuppressed = true;
    }
    this.session = session;
    this.seq = 0;
  }

  _assertCurrent(run) {
    if (this.ordinaryRun !== run || run.generation !== this.queueGeneration) {
      throw new Error('Request invalidated by Stop or session change');
    }
  }

  async _post(path, payload, timeoutMs, run = null, keepalive = false) {
    const origin = currentOrigin(this.apiBase);
    if (!origin) throw new Error('Dashboard origin is unavailable');
    const controller = new AbortController();
    if (run) {
      this._assertCurrent(run);
      run.controller = controller;
    }
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const response = await fetch(`${this.apiBase}${path}`, {
        method: 'POST',
        // The browser supplies the forbidden Origin header from this page's
        // origin; script code must not try to override it.
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
        signal: controller.signal,
        ...(keepalive ? { keepalive: true } : {})
      });
      if (run) this._assertCurrent(run);
      let data;
      try {
        data = await response.json();
      } catch {
        throw new Error(`Invalid JSON response from ${path}`);
      }
      if (run) this._assertCurrent(run);
      if (!response.ok) throw new Error(data?.reason || `HTTP ${response.status}`);
      return data;
    } finally {
      clearTimeout(timer);
      if (run && run.controller === controller) run.controller = null;
    }
  }

  async triggerStop(reason = 'operator_stop', { keepalive = false } = {}) {
    this.queueGeneration++;
    this.pendingInput = null;
    this.stopLatched = true;
    this.motionSuppressed = true;
    this.ordinaryRun?.controller?.abort();
    if (!this.session) return { ok: false, reason: 'No active session' };
    try {
      return await this._post('/api/stop', {
        session: this.session,
        reason: String(reason).slice(0, 96)
      }, 500, null, keepalive);
    } catch (error) {
      return { ok: false, error: error.message };
    }
  }

  _startRun(kind) {
    if (this.ordinaryRun) throw new Error('Another ordinary request is in flight');
    const run = { kind, generation: this.queueGeneration, controller: null };
    this.ordinaryRun = run;
    return run;
  }

  _finishRun(run) {
    if (this.ordinaryRun !== run) return;
    this.ordinaryRun = null;
    if (!this.stopLatched && !this.motionSuppressed && this.pendingInput
      && this.pendingInput.generation === this.queueGeneration) {
      queueMicrotask(() => this._drainInput());
    }
  }

  submitJog(deadman, axes, device = 'keyboard') {
    if (typeof deadman !== 'boolean' || !Array.isArray(axes) || axes.length !== 3
      || !axes.every(axis => Number.isFinite(axis) && axis >= -1 && axis <= 1)
      || typeof device !== 'string' || !device || device.length > 64) {
      return Promise.reject(new Error('Invalid jog intent'));
    }
    if (!this.session || this.stopLatched || this.motionSuppressed) return Promise.resolve(false);
    this.pendingInput = {
      deadman,
      axes: [...axes],
      device,
      generation: this.queueGeneration
    };
    return this._drainInput() || Promise.resolve(false);
  }

  _drainInput() {
    if (this.ordinaryRun || !this.pendingInput || this.stopLatched || this.motionSuppressed) return null;
    const run = this._startRun('input');
    return this._sendInput(run);
  }

  async _sendInput(run) {
    try {
      const ticketData = await this._post('/api/ticket', { session: this.session }, 1000, run);
      this._assertCurrent(run);
      if (typeof ticketData.ticket !== 'string' || !ticketData.ticket) {
        throw new Error('Ticket response did not contain a ticket');
      }

      const intent = this.pendingInput?.generation === run.generation ? this.pendingInput : null;
      this.pendingInput = null;
      if (!intent || this.stopLatched || this.motionSuppressed) return false;
      const seq = this.seq++;
      await this._post('/api/input', {
        session: this.session,
        ticket: ticketData.ticket,
        seq,
        deadman: intent.deadman,
        axes: intent.axes,
        device: intent.device
      }, 1000, run);
      this._assertCurrent(run);
      return true;
    } catch (error) {
      if (this.ordinaryRun === run && run.generation === this.queueGeneration) {
        await this.triggerStop('input_ticket_or_submit_failed');
      }
      throw error;
    } finally {
      this._finishRun(run);
    }
  }

  async requestAction(action, values = {}) {
    if (!this.session) throw new Error('No active session');
    if (this.ordinaryRun) throw new Error('Another ordinary request is in flight');
    if (typeof action !== 'string' || !action) throw new Error('Invalid action');
    if (action === 'release') {
      this.motionSuppressed = true;
      this.pendingInput = null;
    }

    const run = this._startRun('action');
    try {
      const ticketData = await this._post('/api/ticket', { session: this.session }, 1500, run);
      this._assertCurrent(run);
      if (typeof ticketData.ticket !== 'string' || !ticketData.ticket) {
        throw new Error('Ticket response did not contain a ticket');
      }
      const actionData = await this._post('/api/action', {
        session: this.session,
        action,
        values: values || {},
        ticket: ticketData.ticket
      }, 1500, run);
      this._assertCurrent(run);
      if (typeof actionData.operation !== 'string' || !actionData.operation) {
        throw new Error('Action response did not contain an operation');
      }

      const deadline = Date.now() + 3000;
      let receipt;
      do {
        this._assertCurrent(run);
        receipt = await this._post('/api/receipt', {
          session: this.session,
          operation: actionData.operation
        }, 500, run);
        this._assertCurrent(run);
        if (receipt.status !== 'pending') break;
        if (Date.now() >= deadline) throw new Error('Action receipt timed out');
        await delay(Math.min(100, Math.max(0, deadline - Date.now())));
      } while (Date.now() < deadline);
      if (!receipt || receipt.status === 'pending') throw new Error('Action receipt timed out');

      if (receipt.status === 'complete' && action === 'check') this.stopLatched = false;
      if (receipt.status === 'complete' && action === 'arm') this.motionSuppressed = false;
      return receipt;
    } catch (error) {
      if (this.ordinaryRun === run && run.generation === this.queueGeneration) {
        await this.triggerStop('action_ticket_or_receipt_failed');
      }
      throw error;
    } finally {
      this._finishRun(run);
    }
  }
}

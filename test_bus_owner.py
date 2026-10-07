"""Offline cross-process serial ownership checks using Linux pseudo-terminals."""
import os
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import im6014_port
from im6014_port import (ManagedPort, PortBusyError, PortCloseError, PortConfig,
                         PortOwnershipError, _validate_private_directory,
                         open_managed_port)


class PseudoTerminal:
    def __enter__(self):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        return self

    def __exit__(self, exc_type, exc, traceback):
        os.close(self.master)
        os.close(self.slave)


class FakeSerial:
    """Serial-shaped fake that opens only a supplied PTY for fstat checks."""

    def __init__(self, *, fail_dtr=False, fail_open=False, close_mode='normal',
                 open_path=None, after_open=None, **kwargs):
        self.port = None
        self.baudrate = kwargs['baudrate']
        self.timeout = kwargs['timeout']
        self.write_timeout = kwargs['write_timeout']
        self.exclusive = kwargs['exclusive']
        self.fail_dtr = fail_dtr
        self.fail_open = fail_open
        self.close_mode = close_mode
        self.open_path = open_path
        self.after_open = after_open
        self.fd = None
        self.is_open = False
        self.open_calls = 0
        self.close_calls = 0
        self._dtr = None
        self._rts = None

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        if self.fail_dtr:
            raise OSError('injected DTR setter failure')
        self._dtr = value

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = value

    def open(self):
        self.open_calls += 1
        path = self.open_path or self.port
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.is_open = True
        if self.after_open:
            self.after_open()
        if self.fail_open:
            raise OSError('injected PTY open failure')

    def fileno(self):
        return self.fd

    def close(self):
        self.close_calls += 1
        if self.close_mode == 'raise':
            raise OSError('injected close failure')
        if self.close_mode == 'leave_open':
            return
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.is_open = False

    def simulate_closed(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.is_open = False


def factory_for(created, **options):
    def factory(**kwargs):
        bus = FakeSerial(**options, **kwargs)
        created.append(bus)
        return bus
    return factory


class C01BusOwnership(unittest.TestCase):
    def config(self, path):
        return PortConfig(path, 6_000_000, expected_serial=None, expected_channel=None)

    def test_same_process_and_symlink_alias_reject_before_factory(self):
        with PseudoTerminal() as device, tempfile.TemporaryDirectory() as temp:
            alias = str(Path(temp) / 'by-id-alias')
            os.symlink(device.path, alias)
            first, second = [], []
            owner = open_managed_port(self.config(device.path), factory_for(first))
            try:
                self.assertIsInstance(owner, ManagedPort)
                self.assertTrue(owner.verified)
                with self.assertRaises(PortBusyError):
                    open_managed_port(self.config(alias), factory_for(second))
                self.assertEqual(len(first), 1)
                self.assertEqual(second, [])
            finally:
                self.assertTrue(owner.close())

    def test_subprocess_cannot_bypass_owner_with_alias(self):
        with PseudoTerminal() as device, tempfile.TemporaryDirectory() as temp:
            alias = str(Path(temp) / 'by-id-alias')
            os.symlink(device.path, alias)
            owner = open_managed_port(self.config(device.path), factory_for([]))
            code = '''
import os, sys
from im6014_port import PortBusyError, PortConfig, open_managed_port
class Factory:
    def __call__(self, **kwargs):
        print("FACTORY_CALLED", flush=True)
        raise AssertionError("factory must not run")
try:
    open_managed_port(PortConfig(sys.argv[1], 6000000, None, None), Factory())
except PortBusyError:
    print("BUSY", flush=True)
'''
            env = dict(os.environ)
            env['PYTHONPATH'] = str(Path(__file__).resolve().parent) + os.pathsep + env.get('PYTHONPATH', '')
            try:
                result = subprocess.run([sys.executable, '-c', code, alias], env=env,
                                        capture_output=True, text=True, timeout=5, check=True)
                self.assertEqual(result.stdout.strip(), 'BUSY')
            finally:
                self.assertTrue(owner.close())

    def test_killed_owner_releases_kernel_lock(self):
        with PseudoTerminal() as device:
            code = '''
import os, signal, sys
from im6014_port import PortConfig, open_managed_port
class Fake:
    def __init__(self, **kwargs): self.port = None; self.is_open = False; self.fd = None
    def open(self): self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK); self.is_open = True
    def fileno(self): return self.fd
    def close(self):
        if self.fd is not None: os.close(self.fd); self.fd = None
        self.is_open = False
owner = open_managed_port(PortConfig(sys.argv[1], 6000000, None, None), Fake)
print("READY", flush=True)
signal.pause()
'''
            env = dict(os.environ)
            env['PYTHONPATH'] = str(Path(__file__).resolve().parent) + os.pathsep + env.get('PYTHONPATH', '')
            child = subprocess.Popen([sys.executable, '-c', code, device.path], env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                ready, _, _ = select.select([child.stdout], [], [], 5)
                self.assertTrue(ready, 'child did not report ownership acquisition')
                self.assertEqual(child.stdout.readline().strip(), 'READY')
                child.kill()
                child.wait(timeout=5)
                owner = open_managed_port(self.config(device.path), factory_for([]))
                self.assertTrue(owner.verified)
                self.assertTrue(owner.close())
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=5)
                child.stdout.close()
                child.stderr.close()

    def test_constructor_setter_and_open_failures_release_after_cleanup(self):
        with PseudoTerminal() as device:
            with self.assertRaisesRegex(OSError, 'constructor'):
                open_managed_port(self.config(device.path),
                                  lambda **kwargs: (_ for _ in ()).throw(OSError('constructor failure')))

            setter_buses = []
            with self.assertRaisesRegex(OSError, 'DTR'):
                open_managed_port(self.config(device.path), factory_for(setter_buses, fail_dtr=True))
            self.assertEqual(setter_buses[0].open_calls, 0)
            self.assertEqual(setter_buses[0].close_calls, 1)
            self.assertFalse(setter_buses[0].is_open)

            open_buses = []
            with self.assertRaisesRegex(OSError, 'PTY open'):
                open_managed_port(self.config(device.path), factory_for(open_buses, fail_open=True))
            self.assertEqual(open_buses[0].open_calls, 1)
            self.assertEqual(open_buses[0].close_calls, 1)
            self.assertFalse(open_buses[0].is_open)

            owner = open_managed_port(self.config(device.path), factory_for([]))
            self.assertTrue(owner.close())

    def test_unconfirmed_close_keeps_lease_until_retry_and_is_idempotent(self):
        with PseudoTerminal() as device:
            created = []
            owner = open_managed_port(self.config(device.path),
                                      factory_for(created, close_mode='raise'))
            with self.assertRaises(PortBusyError):
                open_managed_port(self.config(device.path), factory_for([]))
            self.assertFalse(owner.close())
            self.assertEqual(created[0].close_calls, 1)
            created[0].simulate_closed()
            created[0].close_mode = 'normal'
            self.assertTrue(owner.close())
            self.assertTrue(owner.close())
            self.assertEqual(created[0].close_calls, 2)
            replacement = open_managed_port(self.config(device.path), factory_for([]))
            self.assertTrue(replacement.close())

    def test_returned_open_bus_that_stays_open_retains_lease(self):
        with PseudoTerminal() as device:
            created = []
            owner = open_managed_port(self.config(device.path),
                                      factory_for(created, close_mode='leave_open'))
            self.assertFalse(owner.close())
            with self.assertRaises(PortBusyError):
                open_managed_port(self.config(device.path), factory_for([]))
            created[0].close_mode = 'normal'
            self.assertTrue(owner.close())
            self.assertTrue(owner.close())

    def test_alias_replacement_during_open_is_rejected_and_closed(self):
        with PseudoTerminal() as first, PseudoTerminal() as second, tempfile.TemporaryDirectory() as temp:
            alias = str(Path(temp) / 'changing-alias')
            os.symlink(first.path, alias)
            buses = []

            def replace_alias():
                os.unlink(alias)
                os.symlink(second.path, alias)

            factory = factory_for(buses, open_path=first.path, after_open=replace_alias)
            with self.assertRaises(PortOwnershipError):
                open_managed_port(self.config(alias), factory)
            self.assertEqual(buses[0].close_calls, 1)
            self.assertFalse(buses[0].is_open)
            replacement = open_managed_port(self.config(first.path), factory_for([]))
            self.assertTrue(replacement.close())

    def test_namespace_rejects_symlinks_and_insecure_modes(self):
        with tempfile.TemporaryDirectory() as temp:
            private = Path(temp) / 'private'
            private.mkdir(mode=0o700)
            alias = Path(temp) / 'alias'
            os.symlink(private, alias)
            with self.assertRaises(PortOwnershipError):
                _validate_private_directory(str(alias))
            os.chmod(private, 0o755)
            with self.assertRaises(PortOwnershipError):
                _validate_private_directory(str(private))

    def test_lockfile_symlink_is_rejected_before_factory(self):
        with PseudoTerminal() as device, tempfile.TemporaryDirectory() as temp:
            namespace = Path(temp) / 'namespace'
            namespace.mkdir(mode=0o700)
            device_number = os.stat(device.path).st_rdev
            lockfile = namespace / f'{os.major(device_number)}-{os.minor(device_number)}.lock'
            target = Path(temp) / 'target'
            target.write_text('not a lock')
            os.symlink(target, lockfile)
            created = []
            with patch.object(im6014_port, '_LOCK_DIRECTORY', str(namespace)):
                with self.assertRaises(PortOwnershipError):
                    open_managed_port(self.config(device.path), factory_for(created))
            self.assertEqual(created, [])

    def test_direct_open_runs_passive_identity_inspection_before_factory(self):
        events = []
        with PseudoTerminal() as device:
            def inspect(config):
                events.append('inspect')

            def factory(**kwargs):
                events.append('factory')
                bus = FakeSerial(**kwargs)
                return bus

            with patch('im6014_port.inspect_port', side_effect=inspect):
                owner = open_managed_port(self.config(device.path), factory)
                self.assertEqual(events[:2], ['inspect', 'factory'])
                self.assertTrue(owner.close())

        created = []
        wrong_identity = PortConfig(
            '/dev/serial/by-id/usb-FTDI_other-if01-port0', 6_000_000)
        with self.assertRaises(ValueError):
            open_managed_port(wrong_identity, factory_for(created))
        self.assertEqual(created, [])

    def test_context_exit_reports_unconfirmed_close_and_preserves_body_error(self):
        with PseudoTerminal() as device:
            created = []
            owner = open_managed_port(self.config(device.path),
                                      factory_for(created, close_mode='leave_open'))
            with self.assertRaises(PortCloseError) as raised:
                with owner:
                    pass
            self.assertIs(raised.exception.managed_port, owner)
            created[0].simulate_closed()
            created[0].close_mode = 'normal'
            self.assertTrue(raised.exception.managed_port.close())

            owner = open_managed_port(self.config(device.path),
                                      factory_for(created, close_mode='leave_open'))
            original = ValueError('body failure')
            with self.assertRaises(ValueError) as raised_body:
                with owner:
                    raise original
            self.assertIs(raised_body.exception, original)
            self.assertIs(original.managed_port, owner)
            created[-1].simulate_closed()
            created[-1].close_mode = 'normal'
            self.assertTrue(original.managed_port.close())

    def test_gui_disconnect_reports_unconfirmed_close_and_close_retries_owner(self):
        from motor_session import MotorSession

        with PseudoTerminal() as device:
            created = []
            owner = open_managed_port(self.config(device.path),
                                      factory_for(created, close_mode='leave_open'))
            session = MotorSession()
            session.port_owner = owner
            session.bus = owner.bus
            session._set(connected=True, state='Connected')
            try:
                session.request('disconnect')
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    snapshot = session.snapshot()
                    if snapshot['state'] == 'Fault — serial close unconfirmed':
                        break
                    time.sleep(.01)
                self.assertEqual(session.snapshot()['state'], 'Fault — serial close unconfirmed')
                self.assertIs(session.port_owner, owner)
                self.assertIsNone(session.bus)
                self.assertFalse(session.close())
                created[0].simulate_closed()
                created[0].close_mode = 'normal'
                self.assertTrue(session.close())
                self.assertIsNone(session.port_owner)
            finally:
                created[0].simulate_closed()
                created[0].close_mode = 'normal'
                session.close()

    def test_tmpdir_does_not_change_lock_namespace(self):
        with PseudoTerminal() as device, tempfile.TemporaryDirectory() as temp:
            created, second = [], []
            owner = open_managed_port(self.config(device.path), factory_for(created))
            try:
                with patch.dict(os.environ, {'TMPDIR': temp, 'XDG_RUNTIME_DIR': temp}):
                    with self.assertRaises(PortBusyError):
                        open_managed_port(self.config(device.path), factory_for(second))
                self.assertEqual(len(created), 1)
                self.assertEqual(second, [])
            finally:
                self.assertTrue(owner.close())


if __name__ == '__main__':
    unittest.main()

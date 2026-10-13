from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import os
import pty
import unittest
from unittest.mock import Mock, patch

import serial
from im6014_port import (ADAPTER_PROFILES, DEFAULT_ADAPTER_PROFILE, PortConfig,
                         PortRecord, default_port, enumerate_ports, inspect_port,
                         open_managed_port)
from im6014_transport import PosixDeadlineSerial


class U2D2PortConfig(unittest.TestCase):
    def test_managed_pyserial_open_uses_deadline_adapter(self):
        master, slave = pty.openpty()
        try:
            owner = open_managed_port(
                PortConfig(os.ttyname(slave), 6_000_000, 'custom'), serial.Serial)
            try:
                self.assertIsInstance(owner.bus, PosixDeadlineSerial)
                self.assertIsInstance(owner.bus._serial, serial.Serial)
                self.assertTrue(owner.bus.is_open)
            finally:
                self.assertTrue(owner.close())
        finally:
            os.close(master)
            os.close(slave)

    def test_supported_baud_is_an_exact_integer(self):
        for baud in (4_000_000.0, 6_000_000.0, True, False, 5_000_000):
            with self.subTest(baud=baud), self.assertRaisesRegex(ValueError, 'Baud'):
                PortConfig('/dev/ttyUSB3', baud)

    def test_profile_choices_have_one_u2d2_default(self):
        self.assertEqual(DEFAULT_ADAPTER_PROFILE, 'u2d2')
        self.assertEqual(tuple(profile.id for profile in ADAPTER_PROFILES),
                         ('u2d2', 'legacy-ft4232h', 'custom'))

    def test_legacy_by_id_hint_reports_identity_as_incomplete(self):
        path = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0'
        report = inspect_port(PortConfig(path, 6_000_000, 'legacy-ft4232h'), ports=())
        self.assertIn('FTC2RQ7H', report)
        self.assertIn('interface 3', report)
        self.assertIn('incomplete', report)
        self.assertIn('not fully verified', report)

    def test_wrong_legacy_by_id_channel_rejects(self):
        config = PortConfig(
            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if01-port0',
            6_000_000, 'legacy-ft4232h')
        with self.assertRaisesRegex(ValueError, r'expected legacy FT4232H .* observed .*interface 1'):
            inspect_port(config, ports=())

    def test_wrong_legacy_serial_interface_and_vid_pid_reject(self):
        config = PortConfig('/dev/ttyUSB8', 6_000_000, 'legacy-ft4232h')
        with self.assertRaisesRegex(ValueError, 'OTHER123.*interface 1'):
            inspect_port(config, metadata=dict(serial_number='OTHER123', interface='1',
                                               vid='0403', pid='6011'))
        with self.assertRaisesRegex(ValueError, 'USB VID 1A86'):
            inspect_port(config, metadata=dict(serial_number='FTC2RQ7H', channel=3,
                                               vid=0x1A86, pid=0x6011))
        with self.assertRaisesRegex(ValueError, 'USB PID 6014'):
            inspect_port(config, metadata=dict(serial_number='FTC2RQ7H', channel=3,
                                               vid=0x0403, pid=0x6014))

    def test_u2d2_accepts_integer_and_hex_metadata_without_a_known_serial(self):
        for vid, pid, interface in ((0x0403, 0x6014, 0), ('0403', '0x6014', '0')):
            report = inspect_port(PortConfig('/dev/ttyUSB8', 6_000_000),
                                  metadata=dict(vid=vid, pid=pid, interface=interface))
            self.assertIn('FT232H-class U2D2 candidate', report)
            self.assertIn('does not prove ROBOTIS provenance', report)

    def test_u2d2_known_contradictions_reject_before_serial_constructor(self):
        config = PortConfig('/dev/ttyUSB8', 6_000_000)
        factory = Mock()
        for metadata in (
                dict(vid=0x0403, pid=0x6014, interface=1),
                dict(vid=0x0403, pid=0x6011, interface=0),
                dict(vid=0x1A86, pid=0x6014, interface=0)):
            with self.subTest(metadata=metadata), patch(
                    'im6014_port.inspect_port', side_effect=lambda cfg, data=metadata: inspect_port(cfg, metadata=data)):
                with self.assertRaises(ValueError):
                    open_managed_port(config, factory)
        factory.assert_not_called()

    def test_bool_metadata_and_product_text_do_not_identify_candidate(self):
        for metadata in (
                dict(vid=True, pid=0x6014, interface=0, product='FT232H'),
                dict(vid=None, pid=None, interface=0, manufacturer='FTDI', product='FT232H')):
            report = inspect_port(PortConfig('/dev/ttyUSB8', 6_000_000), metadata=metadata)
            self.assertIn('metadata is incomplete', report)

    def test_custom_profile_does_not_assert_identity_but_rejects_known_unwired_legacy_channel(self):
        config = PortConfig('/dev/pts/42', 6_000_000, 'custom')
        report = inspect_port(config, ports=())
        self.assertIn('Custom port route', report)
        self.assertIn('no adapter identity is asserted', report)
        self.assertIn('identity is not verified', report)
        with self.assertRaisesRegex(ValueError, 'legacy FT4232H interface 2 is not wired'):
            inspect_port(config, metadata=dict(serial='FTC2RQ7H', interface=2))

    def test_unknown_u2d2_metadata_stays_unverified_without_opening(self):
        config = PortConfig('/not-real', 6_000_000)
        with patch('serial.Serial') as constructor:
            report = inspect_port(config, ports=())
        self.assertIn('metadata is incomplete', report)
        self.assertIn('not verified', report)
        constructor.assert_not_called()

    def test_enumeration_uses_only_a_present_alias_for_the_observed_device(self):
        with TemporaryDirectory() as directory:
            by_id = Path(directory, 'by-id')
            by_id.mkdir()
            device = Path(directory, 'ttyUSB0')
            other = Path(directory, 'ttyUSB1')
            device.touch()
            other.touch()
            alias = by_id / 'usb-FTDI_FT232H_SERIAL-if00-port0'
            alias.symlink_to(device)
            wrong_alias = by_id / 'usb-FTDI_FT232H_OTHER-if00-port0'
            wrong_alias.symlink_to(other)
            port = SimpleNamespace(device=str(device), serial_number=None, interface=0,
                                   vid='0403', pid='6014', description='USB adapter')
            with patch('serial.Serial') as constructor:
                records = enumerate_ports(ports=[port], by_id_directory=str(by_id))
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0].path, str(alias))
            self.assertEqual(records[0].kind, 'ft232h-candidate')
            constructor.assert_not_called()

            alias.unlink()
            wrong_alias.unlink()
            records = enumerate_ports(ports=[port], by_id_directory=str(by_id))
            self.assertEqual(records[0].path, str(device))
            self.assertNotIn('SERIAL', records[0].path)

    def test_robotis_alias_supplies_interface_but_never_invents_usb_ids(self):
        with TemporaryDirectory() as directory:
            device = Path(directory, 'ttyUSB0')
            device.touch()
            alias = Path(directory, 'usb-ROBOTIS_U2D2_TEST-if00-port0')
            alias.symlink_to(device)
            port = SimpleNamespace(device=str(device), vid=0x0403, pid=0x6014)
            record = enumerate_ports(ports=[port], by_id_directory=directory)[0]
            self.assertEqual(record.path, str(alias))
            self.assertEqual(record.kind, 'ft232h-candidate')
            port.vid = port.pid = None
            self.assertEqual(enumerate_ports(ports=[port], by_id_directory=directory)[0].kind, 'unknown')

    def test_long_by_id_alias_falls_back_to_complete_device_path(self):
        with TemporaryDirectory() as directory:
            by_id = Path(directory, 'by-id')
            by_id.mkdir()
            device = Path(directory, 'ttyUSB0')
            device.touch()
            alias = by_id / ('usb-FTDI_' + 'S' * 220 + '-if00-port0')
            alias.symlink_to(device)
            self.assertGreater(len(str(alias)), 240)
            port = SimpleNamespace(device=str(device), vid=0x0403, pid=0x6014, interface=0)
            record = enumerate_ports(ports=[port], by_id_directory=str(by_id))[0]
            self.assertEqual(record.path, str(device))
            self.assertTrue(Path(record.path).exists())

    def test_enumeration_classifies_only_complete_candidate_metadata(self):
        ports = [
            SimpleNamespace(device='/dev/ttyUSB0', serial_number='unknown', interface=0,
                            vid='0403', pid='6014', description='FT232H'),
            SimpleNamespace(device='/dev/ttyUSB1', serial_number='unknown', interface=0,
                            vid=True, pid=0x6014, description='FT232H'),
            SimpleNamespace(device='/dev/ttyUSB2', serial_number='unknown', interface=0,
                            vid=None, pid=None, manufacturer='FTDI', product='FT232H'),
            SimpleNamespace(device='/dev/ttyS0', serial_number=None, interface=None),
        ]
        records = enumerate_ports(ports=ports, by_id_directory='/no-such-by-id')
        self.assertEqual([record.kind for record in records],
                         ['ft232h-candidate', 'unknown', 'unknown'])

    def test_default_requires_one_candidate_for_the_selected_profile(self):
        candidate = PortRecord('/dev/ttyUSB0', '/stable/u2d2', 'U2D2', 'FT232H', 'ft232h-candidate')
        another = PortRecord('/dev/ttyUSB1', '/stable/other', 'Other', 'FT232H', 'ft232h-candidate')
        legacy = PortRecord('/dev/ttyUSB3', '/stable/legacy', 'Legacy', 'FT4232H', 'legacy-ft4232h')
        self.assertEqual(default_port((), 'u2d2'), '')
        self.assertEqual(default_port((candidate,), 'u2d2'), '/stable/u2d2')
        self.assertEqual(default_port((candidate, another), 'u2d2'), '')
        self.assertEqual(default_port((legacy,), 'legacy-ft4232h'), '/stable/legacy')
        self.assertEqual(default_port((legacy,), 'custom'), '')

    def test_enumeration_is_bounded_and_filters_builtin_uart(self):
        ports = [SimpleNamespace(device=f'/dev/ttyS{index}', description='UART') for index in range(40)]
        ports.extend(SimpleNamespace(device=f'/dev/ttyUSB{index:02}', description='USB') for index in range(40))
        records = enumerate_ports(ports=ports, by_id_directory='/no-such-by-id')
        self.assertEqual(len(records), 32)
        self.assertTrue(all(not record.device.startswith('/dev/ttyS') for record in records))

    def test_enumerated_metadata_overrides_conflicting_alias_hint(self):
        path = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0'
        config = PortConfig(path, 6_000_000, 'legacy-ft4232h')
        port = SimpleNamespace(device='/dev/ttyUSB3', serial_number='OTHER',
                               location='2-1:1.2', manufacturer='FTDI', vid=0x0403, pid=0x6011)
        with patch('im6014_port._same_path', return_value=True):
            with self.assertRaisesRegex(ValueError, 'OTHER.*interface 2'):
                inspect_port(config, ports=[port])

    def test_by_id_hint_completes_partial_os_metadata(self):
        path = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0'
        config = PortConfig(path, 6_000_000, 'legacy-ft4232h')
        port = SimpleNamespace(device='/dev/ttyUSB3', serial_number='FTC2RQ7H',
                               location=None, manufacturer='FTDI', vid=0x0403)
        with patch('im6014_port._same_path', return_value=True):
            report = inspect_port(config, ports=[port])
        self.assertIn('OS enumeration metadata and by-id basename', report)
        self.assertIn('interface 3', report)

    def test_wrong_cli_channel_fails_before_log_file_or_serial_constructor(self):
        from spin_5rpm import main

        with TemporaryDirectory() as directory:
            log_path = Path(directory, 'must-not-be-created.json')
            argv = ['spin_5rpm.py', '--port',
                    '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if01-port0',
                    '--adapter-profile', 'legacy-ft4232h', '--log', str(log_path)]
            with patch('spin_5rpm.serial.Serial') as constructor, patch('sys.argv', argv):
                with self.assertRaises(SystemExit) as error:
                    main()
            self.assertEqual(error.exception.code, 2)
            self.assertFalse(log_path.exists())
            constructor.assert_not_called()

    def test_wrong_gui_channel_fails_before_serial_constructor(self):
        from motor_session import MotorSession

        session = object.__new__(MotorSession)
        session.bus = None
        values = dict(port='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if01-port0',
                      id=0, baud=6_000_000, demo=False, adapter_profile='legacy-ft4232h')
        with patch('motor_session.serial.Serial') as constructor:
            with self.assertRaisesRegex(ValueError, r'expected legacy FT4232H'):
                session._handle('connect', values)
        constructor.assert_not_called()

    def test_numeric_string_channel_metadata_matches_or_rejects(self):
        config = PortConfig('/dev/unit-test-fixture', 6_000_000, 'legacy-ft4232h')
        for channel in ('3', ' 3 ', '03'):
            report = inspect_port(config, metadata=dict(
                serial_number='FTC2RQ7H', channel=channel, vid=0x0403), ports=())
            self.assertIn('metadata is unknown or incomplete', report)
        with self.assertRaises(ValueError):
            inspect_port(config, metadata=dict(
                serial_number='FTC2RQ7H', channel='1', vid=0x0403), ports=())


if __name__ == '__main__':
    unittest.main()

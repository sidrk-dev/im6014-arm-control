from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from im6014_port import PortConfig, inspect_port


class B13PortConfig(unittest.TestCase):
    def test_supported_baud_is_an_exact_integer(self):
        for baud in (4_000_000.0, 6_000_000.0, True, False, 5_000_000):
            with self.subTest(baud=baud), self.assertRaisesRegex(ValueError, 'Baud'):
                PortConfig('/dev/ttyUSB3', baud)

    def test_by_id_basename_reports_matching_serial_and_interface(self):
        config = PortConfig('/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0', 6_000_000)
        report = inspect_port(config, ports=())
        self.assertIn('FTC2RQ7H', report)
        self.assertIn('interface 3', report)
        self.assertIn('Motor communication is not verified', report)

    def test_wrong_by_id_channel_names_expected_and_observed(self):
        config = PortConfig('/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if01-port0', 6_000_000)
        with self.assertRaisesRegex(ValueError, r'expected FTDI serial FTC2RQ7H, interface 3; observed .*interface 1'):
            inspect_port(config, ports=())

    def test_wrong_enumerated_serial_and_channel_are_reported(self):
        config = PortConfig('/dev/ttyUSB8', 4_000_000)
        metadata = SimpleNamespace(serial_number='OTHER123', location='2-1:1.1',
                                   manufacturer='FTDI', product='FT4232H', vid=0x0403)
        with self.assertRaisesRegex(ValueError, r'expected FTDI serial FTC2RQ7H, interface 3; observed .*OTHER123.*interface 1'):
            inspect_port(config, metadata=metadata)

    def test_non_ftdi_vid_rejects_even_with_matching_serial_and_channel(self):
        config = PortConfig('/dev/ttyUSB8', 6_000_000)
        for manufacturer in (None, 'FTDI'):
            with self.subTest(manufacturer=manufacturer), self.assertRaisesRegex(
                    ValueError, 'observed non-FTDI USB VID'):
                inspect_port(config, metadata=dict(serial_number='FTC2RQ7H',
                    channel=3, manufacturer=manufacturer, vid=0x1A86))

    def test_enumerated_metadata_for_symlink_alias_overrides_path_hint(self):
        with TemporaryDirectory() as directory:
            target = Path(directory, 'ttyUSB3')
            alias = Path(directory, 'usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0')
            target.touch()
            alias.symlink_to(target)
            config = PortConfig(str(alias), 6_000_000)
            port = SimpleNamespace(device=str(target), serial_number='WRONG',
                                   location='2-1:1.2', manufacturer='FTDI', vid=0x0403)
            with self.assertRaisesRegex(ValueError, r'observed .*WRONG.*interface 2'):
                inspect_port(config, ports=[port])

    def test_by_id_hint_completes_partial_os_metadata(self):
        path = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0'
        config = PortConfig(path, 6_000_000)
        port = SimpleNamespace(device='/dev/ttyUSB3', serial_number='FTC2RQ7H',
                               location=None, manufacturer='FTDI', vid=0x0403)
        with patch('im6014_port._same_path', return_value=True):
            report = inspect_port(config, ports=[port])
        self.assertIn('OS enumeration metadata and by-id basename', report)
        self.assertIn('interface 3', report)

    def test_explicit_alternate_route_does_not_claim_ftdi_identity(self):
        config = PortConfig('/dev/pts/42', 6_000_000,
                            expected_serial=None, expected_channel=None)
        report = inspect_port(config, ports=())
        self.assertIn('Alternate port route', report)
        self.assertIn('no FTDI serial/channel is asserted', report)
        self.assertIn('metadata: unknown', report)
        self.assertIn('identity is not verified', report)

    def test_unknown_default_path_stays_unknown_and_does_not_open(self):
        config = PortConfig('/not-real', 6_000_000)
        with patch('serial.Serial') as constructor:
            report = inspect_port(config, ports=())
        self.assertIn('metadata unknown', report)
        self.assertIn('identity is not verified', report)
        constructor.assert_not_called()

    def test_wrong_cli_channel_fails_before_log_file_or_serial_constructor(self):
        from spin_5rpm import main

        with TemporaryDirectory() as directory:
            log_path = Path(directory, 'must-not-be-created.json')
            argv = ['spin_5rpm.py', '--port',
                    '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if01-port0',
                    '--log', str(log_path)]
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
                      id=0, baud=6_000_000, demo=False)
        with patch('motor_session.serial.Serial') as constructor:
            with self.assertRaisesRegex(ValueError, r'expected FTDI serial FTC2RQ7H, interface 3'):
                session._handle('connect', values)
        constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()

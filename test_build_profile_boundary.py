from copy import deepcopy
from dataclasses import replace
import json
import unittest

import build_profile as bp
from test_build_profile import profile_dict


class D01BoundaryTests(unittest.TestCase):
    def parse(self, record):
        return bp.parse_json(json.dumps(record))

    def test_repeated_json_keys_rejected_at_every_depth(self):
        text = json.dumps(profile_dict())
        substitutions = (
            ('"revision": "71a0a5f"', '"revision": "first", "revision": "71a0a5f"'),
            ('"motor_id": 1', '"motor_id": 0, "motor_id": 1'),
            ('"direction": 1', '"direction": -1, "direction": 1'),
        )
        for old, new in substitutions:
            with self.subTest(field=old):
                self.assertIn(old, text)
                with self.assertRaisesRegex(ValueError, 'repeats field'):
                    bp.parse_json(text.replace(old, new))

    def test_padded_identifiers_cannot_hide_duplicate_bus_slot(self):
        original = profile_dict()
        for field in ('name', 'bus', 'actuator_identity_source', 'mapping_source'):
            with self.subTest(field=field):
                record = deepcopy(original)
                record['joints'][0][field] = ' padded '
                with self.assertRaises(ValueError):
                    self.parse(record)
        for field in ('robot', 'build', 'revision'):
            with self.subTest(field=field):
                record = deepcopy(original)
                record[field] = ' padded '
                with self.assertRaises(ValueError):
                    self.parse(record)

    def test_unknown_backend_and_non_string_identity_rejected(self):
        for field, values in (
            ('backend', ('unknown', '', True, 1, [], {})),
            ('revision', (None, True, 1, [], {})),
        ):
            for value in values:
                with self.subTest(field=field, value=value):
                    record = profile_dict()
                    record[field] = value
                    with self.assertRaises(ValueError):
                        self.parse(record)

    def test_all_provenance_fields_are_required(self):
        for field in ('actuator_identity', 'actuator_identity_source', 'mapping_source', 'mapping_status'):
            with self.subTest(field=field):
                record = profile_dict()
                del record['joints'][0][field]
                with self.assertRaisesRegex(ValueError, 'missing required'):
                    self.parse(record)

    def test_unknown_actuator_identity_and_declared_mapping_do_not_gain_authority(self):
        profile = self.parse(profile_dict())
        joint = profile.joints[0]
        self.assertIsNone(joint.actuator_identity)
        self.assertEqual(joint.mapping_status, 'declared')
        self.assertFalse(hasattr(profile, 'permits'))
        self.assertFalse(hasattr(profile, 'allowed_actions'))
        self.assertFalse(hasattr(joint, 'calibrated'))
        self.assertEqual(bp.parse_json(profile.to_json()), profile)

    def test_actuator_identity_is_explicit_and_source_is_preserved(self):
        record = profile_dict()
        record['joints'][0].update(actuator_identity='synthetic-actuator-17', actuator_identity_source='synthetic nameplate fixture')
        joint = self.parse(record).joints[0]
        self.assertEqual(joint.actuator_identity, 'synthetic-actuator-17')
        self.assertEqual(joint.actuator_identity_source, 'synthetic nameplate fixture')
        for value in ('', ' padded ', True, 1, [], {}):
            with self.subTest(value=value):
                record['joints'][0]['actuator_identity'] = value
                with self.assertRaises(ValueError):
                    self.parse(record)

    def test_mapping_provenance_is_required_in_direct_constructor(self):
        joint = self.parse(profile_dict()).joints[0]
        for changes in ({'mapping_source': ''}, {'mapping_source': ' padded '}, {'mapping_status': 'unknown'}, {'mapping_status': True}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(joint, **changes)

    def test_measurements_retain_units_source_and_verification(self):
        record = profile_dict()
        record['joints'][0]['measurements'] = {
            'mass': {'status': 'unknown', 'units': 'kg', 'source': 'not measured'},
            'offset': {'status': 'declared', 'units': 'rad', 'source': 'synthetic design', 'value': 0.25},
            'length': {'status': 'measured', 'units': 'm', 'source': 'synthetic caliper fixture', 'value': 0.3},
        }
        profile = self.parse(record)
        again = bp.parse_json(profile.to_json())
        self.assertEqual(profile, again)
        for name, fields in record['joints'][0]['measurements'].items():
            measurement = again.joints[0].measurement(name)
            self.assertEqual(measurement.to_dict(), fields)
        self.assertIsNone(again.joints[0].measurement('mass').value)

    def test_measurement_provenance_cannot_be_dropped_or_nonfinite(self):
        record = profile_dict()
        original = {'status': 'measured', 'units': 'kg', 'source': 'synthetic fixture', 'value': 0.3}
        for field in ('status', 'units', 'source', 'value'):
            with self.subTest(missing=field):
                entry = dict(original)
                del entry[field]
                record['joints'][0]['measurements'] = {'mass': entry}
                with self.assertRaises(ValueError):
                    self.parse(record)
        for field, values in (
            ('source', ('', ' padded ', True)),
            ('units', ('', ' padded ', None)),
            ('value', (True, float('nan'), float('inf'), 10**400, '0.3')),
        ):
            for value in values:
                with self.subTest(field=field, value=repr(value)):
                    record['joints'][0]['measurements'] = {'mass': dict(original, **{field:value})}
                    with self.assertRaises(ValueError):
                        bp.from_dict(record)

    def test_missing_declared_value_names_actual_status_and_optional_null_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'a declared measurement requires a value'):
            bp.Measurement('mass', 'declared', 'kg', 'synthetic fixture')
        for field in ('position_limits', 'measurements'):
            with self.subTest(field=field):
                record = profile_dict()
                record['joints'][0][field] = None
                with self.assertRaises(ValueError):
                    self.parse(record)

    def test_parser_returns_detached_immutable_values_and_preserves_joint_order(self):
        record = profile_dict()
        second = deepcopy(record['joints'][0])
        second.update(name='j2', motor_id=2)
        record['joints'].append(second)
        profile = bp.from_dict(record)
        record['joints'][0]['mapping']['reduction'] = 999
        serialized = profile.to_dict()
        serialized['joints'][1]['mapping']['direction'] = -1
        self.assertEqual(profile.joint_names(), ('j1', 'j2'))
        self.assertEqual(profile.joints[0].mapping.reduction, 6)
        self.assertEqual(profile.joints[1].mapping.direction, 1)
        with self.assertRaises(ValueError):
            replace(profile, joints=list(profile.joints))
        with self.assertRaises(ValueError):
            replace(profile, backend='other')


if __name__ == '__main__':
    unittest.main()

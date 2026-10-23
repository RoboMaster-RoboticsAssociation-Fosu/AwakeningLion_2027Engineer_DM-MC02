"""Independent wire vectors and malformed-stream regressions for the Pi client."""
from pathlib import Path
import math
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import arm_usb


ANGLES = [0.0, 1.0, -1.0, 2.0, -2.0, 0.5, -0.5, 12.5]
COMMAND_VECTOR = bytes.fromhex(
    'a55a01102000cdab000000000000803f000080bf00000040000000c0'
    '0000003f000000bf00004841f267'
)
FEEDBACK_VECTOR = bytes.fromhex(
    'a55a011161003412000000000000803f000080bf00000040000000c00000003f000000bf000048410000000000000000000000000000000000000000000000000000000000000000000000000000803f000080bf00000040000000c00000003f000000bf00004841ff4758'
)


class UsbProtocolTests(unittest.TestCase):
    """The serial hardware is not replaced: these tests exercise pure wire codecs only."""

    def test_crc_and_command_golden_vector(self):
        self.assertEqual(arm_usb.crc16(b'123456789'), 0x6F91)
        self.assertEqual(arm_usb.encode_command(ANGLES, 0xABCD), COMMAND_VECTOR)
        self.assertEqual(len(COMMAND_VECTOR), 42)

    def test_reject_invalid_command(self):
        for value in [float('nan'), float('inf'), -float('inf'), 1e100]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                arm_usb.encode_command([value] + [0.0] * 7, 1)
        for values in [[], [0.0] * 7, [0.0] * 9]:
            with self.subTest(count=len(values)), self.assertRaises(ValueError):
                arm_usb.encode_command(values, 1)
        for sequence in [-1, 65536]:
            with self.assertRaises(ValueError):
                arm_usb.encode_command(ANGLES, sequence)

    def test_every_fragment_boundary(self):
        for boundary in range(len(FEEDBACK_VECTOR) + 1):
            parser = arm_usb.StreamParser()
            frames = parser.feed(FEEDBACK_VECTOR[:boundary])
            frames += parser.feed(FEEDBACK_VECTOR[boundary:])
            self.assertEqual(len(frames), 1)
            message_type, sequence, payload = frames[0]
            self.assertEqual((message_type, sequence), (0x11, 0x1234))
            feedback = arm_usb.decode_feedback(payload)
            self.assertEqual(feedback['angle_rad'], ANGLES)
            self.assertEqual(feedback['velocity_rad_s'], [0.0] * 8)
            self.assertEqual(feedback['load_torque_nm'], ANGLES)
            self.assertEqual(feedback['load_valid_mask'], 0xFF)

    def test_bytewise_noise_bad_crc_and_concatenation(self):
        corrupted = bytearray(FEEDBACK_VECTOR)
        corrupted[20] ^= 1
        stream = b'noise\xa5\x00' + corrupted + COMMAND_VECTOR + FEEDBACK_VECTOR
        parser = arm_usb.StreamParser()
        frames = []
        for byte in stream:
            frames.extend(parser.feed(bytes([byte])))
        self.assertEqual([(message, sequence) for message, sequence, _ in frames],
                         [(0x10, 0xABCD), (0x11, 0x1234)])
        self.assertFalse(parser.buffer)

    def test_bad_version_and_length_recover(self):
        bad_version = bytearray(COMMAND_VECTOR)
        bad_version[2] = 2
        oversized = arm_usb.HEADER.pack(arm_usb.SOF, 1, 0x11, 257, 0)
        frames = arm_usb.StreamParser().feed(bad_version + oversized + FEEDBACK_VECTOR)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][1], 0x1234)

    def test_unavailable_load_is_not_a_zero_measurement(self):
        payload = arm_usb.FEEDBACK.pack(*([math.nan] * 24), 0)
        feedback = arm_usb.decode_feedback(payload)
        self.assertEqual(feedback['angle_rad'], [None] * 8)
        self.assertEqual(feedback['load_torque_nm'], [None] * 8)
        self.assertEqual(feedback['load_valid_mask'], 0)
        for mask in [1, 0x80, 0xFF]:
            with self.assertRaises(ValueError):
                arm_usb.decode_feedback(arm_usb.FEEDBACK.pack(*([math.nan] * 24), mask))
        with self.assertRaises(ValueError):
            arm_usb.decode_feedback(payload[:-1])


if __name__ == '__main__':
    unittest.main()

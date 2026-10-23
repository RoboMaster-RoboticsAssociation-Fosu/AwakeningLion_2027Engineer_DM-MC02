#!/usr/bin/env python3
"""Pi-side USB CDC framing and communication check; uses only the Python standard library.

Joint order: yaw, pitch1, roll1, pitch2, roll2, pitch3, roll3, grip.
The current H7 communication receiver stores targets without commanding motors.
"""

import argparse
import glob
import json
import math
import os
import secrets
import select
import struct
import time

HEADER = struct.Struct('<2sBBHH')
COMMAND = struct.Struct('<8f')
FEEDBACK = struct.Struct('<24fB')
SOF = b'\xa5\x5a'
VERSION = 1
COMMAND_TYPE = 0x10
FEEDBACK_TYPE = 0x11
MAX_PAYLOAD = 256


def crc16(data):
    """Match firmware CRC: reflected polynomial 0x8408, init 0xFFFF, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def encode_command(angles, sequence):
    """Encode exactly eight finite binary32 targets; reject malformed commands."""
    if len(angles) != 8 or not all(math.isfinite(value) for value in angles):
        raise ValueError('Exactly eight finite angles in rad are required')
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError('Sequence must fit uint16')
    try:
        payload = COMMAND.pack(*angles)
    except (OverflowError, struct.error) as error:
        raise ValueError('Angles must fit binary32') from error
    frame = HEADER.pack(SOF, VERSION, COMMAND_TYPE, len(payload), sequence) + payload
    return frame + struct.pack('<H', crc16(frame))


class StreamParser:
    """Incrementally parse fragmented/concatenated frames and resynchronize after errors."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        """Return valid (type, sequence, payload) tuples; retain an incomplete tail."""
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] != SOF[0] or (
                len(self.buffer) >= 2 and self.buffer[1] != SOF[1]
            ) or (len(self.buffer) >= 3 and self.buffer[2] != VERSION):
                del self.buffer[0]
                continue
            if len(self.buffer) < HEADER.size:
                break
            _, _, message_type, length, sequence = HEADER.unpack_from(self.buffer)
            if length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            total = HEADER.size + length + 2
            if len(self.buffer) < total:
                break
            if crc16(self.buffer[:total - 2]) != struct.unpack_from('<H', self.buffer, total - 2)[0]:
                del self.buffer[0]
                continue
            frames.append((message_type, sequence, bytes(self.buffer[HEADER.size:total - 2])))
            del self.buffer[:total]
        return frames


def decode_feedback(payload):
    """Decode actual feedback; unavailable observations become JSON null, not zero."""
    if len(payload) != FEEDBACK.size:
        raise ValueError('Feedback payload must be 97 bytes')
    values = FEEDBACK.unpack(payload)
    valid_mask = values[24]
    for joint in range(8):
        if valid_mask & (1 << joint) and not math.isfinite(values[16 + joint]):
            raise ValueError('A valid load torque must be finite')
    return {
        'angle_rad': [value if math.isfinite(value) else None for value in values[:8]],
        'velocity_rad_s': [value if math.isfinite(value) else None for value in values[8:16]],
        'load_torque_nm': [values[16 + joint] if valid_mask & (1 << joint) else None
                           for joint in range(8)],
        'load_valid_mask': valid_mask,
    }


def open_port(path):
    """Configure a Linux USB CDC terminal for raw, nonblocking binary traffic."""
    # termios is only needed when opening the Pi's port; pure codecs also run on Windows.
    import termios

    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attributes = termios.tcgetattr(fd)
        attributes[0] = attributes[1] = attributes[3] = 0
        attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attributes[4] = attributes[5] = termios.B115200
        attributes[6][termios.VMIN] = attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        termios.tcflush(fd, termios.TCIFLUSH)
    except BaseException:
        os.close(fd)
        raise
    return fd


def write_all(fd, data):
    """Handle partial nonblocking writes; fail instead of waiting indefinitely."""
    remaining = memoryview(data)
    deadline = time.monotonic() + 1.0
    while remaining:
        timeout = deadline - time.monotonic()
        if timeout <= 0 or not select.select([], [fd], [], timeout)[1]:
            raise TimeoutError('USB write timed out')
        try:
            written = os.write(fd, remaining)
        except BlockingIOError:
            continue
        if written == 0:
            raise OSError('USB write returned zero bytes')
        remaining = remaining[written:]


def main():
    """Monitor telemetry, optionally send targets, and verify sequence acknowledgements."""
    arguments = argparse.ArgumentParser(description=__doc__)
    arguments.add_argument('--port', help='Stable /dev/serial/by-id/... path; auto-select only one STM32')
    arguments.add_argument('--angles', type=float, nargs=8, metavar='RAD',
                           help='Send eight target angles; omitted means receive only')
    arguments.add_argument('--duration', type=float, default=5.0, help='Check duration in seconds')
    arguments.add_argument('--continuous', action='store_true',
                           help='Keep the port open until stopped; intended for the boot service')
    arguments.add_argument('--rate', type=float, default=100.0, help='Target send rate in Hz')
    args = arguments.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        arguments.error('--duration must be finite and positive')
    if not math.isfinite(args.rate) or not 0 < args.rate <= 1000:
        arguments.error('--rate must be in (0, 1000] Hz')
    port = args.port
    if port is None:
        ports = glob.glob('/dev/serial/by-id/*STM32*')
        if len(ports) != 1:
            arguments.error('Expected exactly one STM32 USB CDC port; specify --port')
        port = ports[0]
    sequence = secrets.randbelow(0xFFFF) + 1
    if args.angles is not None:
        try:
            encode_command(args.angles, sequence)
        except ValueError as error:
            arguments.error(str(error))

    fd = open_port(port)
    parser = StreamParser()
    sent_sequences = set()
    sent_count = received_count = acknowledged_count = 0
    last_feedback = None
    start = time.monotonic()
    deadline = None if args.continuous else start + args.duration
    next_send = next_print = start
    if args.continuous:
        print(json.dumps({'event': 'usb_opened', 'port': port}), flush=True)
    try:
        while deadline is None or time.monotonic() < deadline:
            now = time.monotonic()
            if args.angles is not None and now >= next_send:
                write_all(fd, encode_command(args.angles, sequence))
                sent_sequences.add(sequence)
                sent_count += 1
                sequence = (sequence + 1) & 0xFFFF
                next_send = now + 1.0 / args.rate
            timeout = 0.05 if deadline is None else max(0.0, min(0.05, deadline - time.monotonic()))
            if args.angles is not None:
                timeout = min(timeout, max(0.0, next_send - time.monotonic()))
            if select.select([fd], [], [], timeout)[0]:
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    raise OSError('USB device disconnected')
                for message_type, ack_sequence, payload in parser.feed(data):
                    if message_type != FEEDBACK_TYPE:
                        continue
                    last_feedback = decode_feedback(payload)
                    last_feedback['command_sequence'] = ack_sequence
                    received_count += 1
                    if ack_sequence in sent_sequences:
                        acknowledged_count += 1
            if last_feedback is not None and time.monotonic() >= next_print:
                # Continuous operation reports progress every five seconds to keep the journal bounded.
                report = {**last_feedback, 'received_feedback': received_count} if args.continuous else last_feedback
                print(json.dumps(report, allow_nan=False), flush=True)
                next_print = time.monotonic() + (5.0 if args.continuous else 0.5)
    finally:
        os.close(fd)
    summary = {
        'port': port, 'sent_commands': sent_count, 'received_feedback': received_count,
        'acknowledged_feedback': acknowledged_count,
        'elapsed_s': round(time.monotonic() - start, 3),
    }
    print(json.dumps(summary), flush=True)
    return 0 if received_count and (args.angles is None or acknowledged_count) else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error

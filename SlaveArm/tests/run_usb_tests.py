"""Strict C99 USB regression with real production bodies and pure Python client tests.

The native hardware boundary is defined in test_arm_usb.c. Production includes
are omitted only in ignored generated excerpts; actual firmware is built with Keil.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import sys


def main():
    """Build fresh GCC/Clang binaries and stop immediately on any failed step."""
    board = Path(__file__).resolve().parents[1]
    root = board.parent
    output = board / 'build/arm_usb/tests'
    output.mkdir(parents=True, exist_ok=True)
    for relative, name in [('User/APP/arm_usb_system.c', 'arm_usb_system_production.inc'),
                           ('User/Bsp/bsp_usb.c', 'bsp_usb_production.inc')]:
        source = (board / relative).read_text(encoding='utf-8')
        source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.MULTILINE)
        (output / name).write_text(source, encoding='utf-8')

    flags = ['-std=c99', '-O1', '-Wall', '-Wextra', '-Wconversion', '-Wshadow',
             '-Wundef', '-Werror', '-pedantic']
    for relative in ['SlaveArm/User/APP', 'SlaveArm/User/Bsp', 'Shared/Protocol/ArmUsb',
                     'Shared/Protocol/BoardLink', 'Shared/Algorithm/CRC']:
        flags += ['-I', str(root / relative)]
    flags += ['-I', str(output)]
    sources = [str(board / 'tests/test_arm_usb.c'),
               str(root / 'Shared/Protocol/ArmUsb/arm_usb_protocol.c'),
               str(root / 'Shared/Protocol/BoardLink/board_link_protocol.c'),
               str(root / 'Shared/Algorithm/CRC/CRC.c')]
    gcc, clang = shutil.which('gcc'), shutil.which('clang')
    if not gcc or not clang:
        raise RuntimeError('Both GCC and Clang are required')
    clang_flags = []
    if os.name == 'nt':
        gnu_root = Path(gcc).resolve().parent.parent
        libgcc = Path(subprocess.check_output([gcc, '-print-libgcc-file-name'], text=True).strip())
        clang_flags = ['--target=x86_64-w64-windows-gnu', '--sysroot=' + str(gnu_root),
                       '-B' + str(gnu_root / 'bin'), '-L' + str(libgcc.parent)]
    for name, compiler, platform in [('gcc', gcc, []), ('clang', clang, clang_flags)]:
        binary = output / ('test_usb_' + name + ('.exe' if os.name == 'nt' else ''))
        print(name + ': real USB production bodies and codecs', flush=True)
        subprocess.run([compiler, *platform, *flags, *sources, '-lm', '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(board / 'tests'),
                    '-p', 'test_arm_usb_client.py', '-v'], check=True)


if __name__ == '__main__':
    main()

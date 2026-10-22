"""Compile the real QD4310 driver against board HAL headers and run host regressions.

Only test_qd4310.c replaces hardware transmission. No firmware project includes it.
Each compiler gets a fresh binary path; a failed compile never runs an old binary.
"""

from pathlib import Path
import os
import shutil
import subprocess


def main():
    """Run GCC/Clang C99 tests and C++03 header checks with strict warnings."""
    module = Path(__file__).resolve().parent.parent
    root = module.parents[2]
    output = root / "Chassis/build/qd4310"
    output.mkdir(parents=True, exist_ok=True)
    gcc = shutil.which("gcc")
    clang = shutil.which("clang")
    if not gcc or not clang:
        raise RuntimeError("Both GCC and Clang must be available on PATH")

    common = [
        "-Wall", "-Wextra", "-Wconversion", "-Wshadow", "-Wundef", "-Werror", "-pedantic",
        "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-I", str(module),
    ]
    # Treat vendor headers as system headers; all driver and test warnings remain errors.
    for include in (
        "SlaveArm/Core/Inc",
        "Shared/Drivers/STM32H7xx_HAL_Driver/Inc",
        "Shared/Drivers/CMSIS/Include",
        "Shared/Drivers/CMSIS/Device/ST/STM32H7xx/Include",
    ):
        common.extend(["-isystem", str(root / include)])

    clang_platform = []
    if os.name == "nt":
        # Native Windows Clang uses the existing MinGW runtime for headers and linking.
        gnu_root = Path(gcc).resolve().parent.parent
        gnu_machine = subprocess.check_output([gcc, "-dumpmachine"], text=True).strip()
        gnu_include = gnu_root / gnu_machine / "include"
        if not (gnu_include / "math.h").is_file():
            gnu_include = gnu_root / "include"
        libgcc = subprocess.check_output([gcc, "-print-libgcc-file-name"], text=True).strip()
        clang_platform = [
            "--target=x86_64-w64-windows-gnu", f"--sysroot={gnu_root}",
            f"-B{gnu_root / 'bin'}", f"-L{Path(libgcc).parent}",
        ]

    consumer = output / "header_consumer.cpp"
    consumer.write_text(
        '#include "QD4310_Motor.h"\n'
        'QD4310_Motor_Status_e header_consumer(FDCAN_HandleTypeDef *bus, '
        'const QD4310_Motor_Info_t *motor)\n{\n'
        '    return QD4310_Motor_Control(bus, motor, QD4310_MOTOR_MODE_CURRENT, 0.0f, 0);\n'
        '}\n'
        'QD4310_Motor_Status_e torque_header_consumer(FDCAN_HandleTypeDef *bus, '
        'const QD4310_Motor_Info_t *motor)\n{\n'
        '    return QD4310_Motor_ControlTorque(bus, motor, 0.027f, 0);\n'
        '}\n',
        encoding="utf-8",
    )
    for name, compiler, platform in (("gcc", gcc, []), ("clang", clang, clang_platform)):
        binary = output / (f"test_{name}.exe" if os.name == "nt" else f"test_{name}")
        # Delete only this runner's exact stale binary, not directories or unrelated output.
        binary.unlink(missing_ok=True)
        print(f"{name}: strict C99 protocol tests", flush=True)
        subprocess.run(
            [compiler, *platform, "-std=c99", *common, str(module / "QD4310_Motor.c"),
             str(module / "tests/test_qd4310.c"), "-lm", "-o", str(binary)], check=True,
        )
        subprocess.run([str(binary)], check=True)
        cpp = shutil.which("g++" if name == "gcc" else "clang++")
        if not cpp:
            raise RuntimeError(f"C++ compiler missing for {name}")
        # CMSIS converts pointers to uint32_t; C++ requires a 32-bit pointer model.
        # This check does not link or run, so the installed 64-bit MinGW headers suffice.
        cpp_platform = [flag for flag in platform if not flag.startswith("-L")]
        if os.name == "nt":
            cpp_platform = [flag.replace("x86_64-w64", "i686-w64") for flag in cpp_platform]
            if name == "clang":
                cpp_platform.extend(["-isystem", str(gnu_include)])
        if name == "gcc":
            cpp_platform.append("-m32")
        subprocess.run(
            [cpp, *cpp_platform, "-std=c++03", "-fno-exceptions", "-fno-rtti", *common,
             "-fsyntax-only", str(consumer)], check=True,
        )
        print(f"{name}: C++03 header passed", flush=True)


if __name__ == "__main__":
    main()

"""Host verification of the real chassis controller, CAN dispatcher and drivers.

The hardware boundary is supplied by test_motor_control.c only. Build products
stay in ignored build/chassis_motor_test; this does not flash or operate motors.
"""
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


def main():
    """Compile both host toolchains and execute only newly linked binaries."""
    chassis = Path(__file__).resolve().parents[1]
    shared = chassis.parent / "Shared"
    output = chassis / "build/chassis_motor_test"
    output.mkdir(parents=True, exist_ok=True)
    options = ET.parse(chassis / "MDK-ARM/CtrlBoard-H7_IMU.uvprojx").find(
        ".//TargetArmAds/Cads/VariousControls"
    )
    flags = ["-std=c99", "-O1", "-g", "-ffunction-sections", "-fdata-sections",
             "-fno-asynchronous-unwind-tables", "-fno-unwind-tables",
             "-Wno-attributes", "-Wno-int-to-pointer-cast", "-Wno-pointer-to-int-cast",
             "-D__ARM_FP=4"]
    flags.extend("-D" + item for item in options.findtext("Define").split(","))
    for include in options.findtext("IncludePath").split(";"):
        flags.extend(["-isystem", str((chassis / "MDK-ARM" / include).resolve())])
    warnings = ["-Wall", "-Wextra", "-Wshadow", "-Wundef", "-Werror", "-pedantic"]
    gcc = shutil.which("gcc")
    clang = shutil.which("clang")
    if not gcc or not clang:
        raise RuntimeError("GCC and Clang are both required for this check")
    gcc_root = Path(gcc).resolve().parent.parent
    libgcc = Path(subprocess.check_output([gcc, "-print-libgcc-file-name"], text=True).strip())
    clang_platform = ["--target=x86_64-w64-windows-gnu", "--sysroot=" + str(gcc_root),
                      "-B" + str(gcc_root / "bin"), "-L" + str(libgcc.parent)]
    sources = [
        chassis / "tests/test_motor_control.c",
        chassis / "User/Bsp/bsp_can.c",
        shared / "Devices/DJI_Motor/DJI_Motor.c",
        shared / "Devices/DM_Motor/DM_Motor.c",
        shared / "Algorithm/PID/pid.c",
    ]
    for label, compiler, platform in (("gcc", gcc, []), ("clang", clang, clang_platform)):
        print(label + ": controller, CAN callbacks, real drivers and PID", flush=True)
        compile_platform = [arg for arg in platform if not arg.startswith("-L")]
        # Check the controller as its own translation unit as well as executing
        # its static tick in the host harness below.
        subprocess.run([compiler, *compile_platform, *flags, *warnings, "-Wconversion",
                        "-fsyntax-only", str(chassis / "User/APP/chassis_task.c")], check=True)
        objects = []
        for index, source in enumerate(sources):
            obj = output / (label + "_" + source.stem + ".o")
            # Existing vendor decoders keep their warning baseline. The changed
            # controller, actual dispatcher and test harness also check conversions.
            strict = ["-Wconversion"] if index < 2 else []
            subprocess.run([compiler, *compile_platform, *flags, *warnings, *strict,
                            "-c", str(source), "-o", str(obj)], check=True)
            objects.append(str(obj))
        binary = output / ("test_motor_control_" + label + ".exe")
        subprocess.run([compiler, *platform, *objects, "-Wl,--gc-sections", "-lm",
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()

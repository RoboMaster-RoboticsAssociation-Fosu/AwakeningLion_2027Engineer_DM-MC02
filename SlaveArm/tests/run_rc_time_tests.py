"""Test the actual offline-check bodies without emulating the UART/DMA startup.

Production excerpts are copied verbatim to ignored build output. Both complete
firmware projects are also built separately to validate their real integration.
"""
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


def main():
    """Strict-check and execute the timing regression with GCC and Clang."""
    board = Path(__file__).resolve().parents[1]
    shared = board.parent / "Shared"
    output = board / "build/feedback_time"
    output.mkdir(parents=True, exist_ok=True)
    source = (board / "User/APP/rc_system.c").read_text(encoding="utf-8")
    bodies = []
    for name in ("RC_UpdateOfflineState", "RC_ResetEdgeState"):
        definition = re.search(r"^static (?:void|uint32_t) " + name +
                               r"\([^\n]*\)\n\{", source, re.MULTILINE)
        if definition is None:
            raise RuntimeError("Production function not found: " + name)
        end = source.index("\n}\n", definition.start()) + 3
        bodies.append(source[definition.start():end])
    timeout = re.search(r"^#define RC_OFFLINE_TIMEOUT_MS\s+\d+U", source, re.MULTILINE)
    if timeout is None:
        raise RuntimeError("Production timeout not found")
    (output / "rc_time_production.inc").write_text(
        "/* Generated verbatim from rc_system.c; never included by firmware. */\n" +
        timeout.group() + "\n\n" + "\n".join(bodies), encoding="utf-8"
    )
    options = ET.parse(board / "MDK-ARM/CtrlBoard-H7_IMU.uvprojx").find(
        ".//TargetArmAds/Cads/VariousControls"
    )
    flags = ["-std=c99", "-O1", "-g", "-ffunction-sections", "-fdata-sections",
             "-fno-asynchronous-unwind-tables", "-fno-unwind-tables", "-D__ARM_FP=4",
             "-Wno-attributes", "-Wno-int-to-pointer-cast", "-Wno-pointer-to-int-cast",
             "-I", str(output)]
    flags.extend("-D" + item for item in options.findtext("Define").split(","))
    for include in options.findtext("IncludePath").split(";"):
        flags.extend(["-isystem", str((board / "MDK-ARM" / include).resolve())])
    warnings = ["-Wall", "-Wextra", "-Wshadow", "-Wundef", "-Werror", "-pedantic"]
    gcc, clang = shutil.which("gcc"), shutil.which("clang")
    if not gcc or not clang:
        raise RuntimeError("GCC and Clang are both required")
    gcc_root = Path(gcc).resolve().parent.parent
    libgcc = Path(subprocess.check_output([gcc, "-print-libgcc-file-name"], text=True).strip())
    platform = ["--target=x86_64-w64-windows-gnu", "--sysroot=" + str(gcc_root),
                "-B" + str(gcc_root / "bin"), "-L" + str(libgcc.parent)]
    for label, compiler, target in (("gcc", gcc, []), ("clang", clang, platform)):
        print(label + ": actual RC offline check and Remote_Control driver", flush=True)
        compile_target = [item for item in target if not item.startswith("-L")]
        test_object = output / ("test_" + label + ".o")
        driver_object = output / ("remote_" + label + ".o")
        subprocess.run([compiler, *compile_target, *flags, *warnings, "-Wconversion",
                        "-c", str(board / "tests/test_rc_feedback_time.c"),
                        "-o", str(test_object)], check=True)
        # The unchanged legacy parser names parameters after its global instance
        # and retains an unused declaration and signed deadzone comparison.
        # Keep the changed check and
        # harness strict while preserving this driver's existing warning baseline.
        driver_warnings = [item for item in warnings if item != "-Wshadow"]
        subprocess.run([compiler, *compile_target, *flags, *driver_warnings,
                        "-Wno-unused-function", "-Wno-sign-compare",
                        "-c", str(shared / "Devices/Remote_Control/Remote_Control.c"),
                        "-o", str(driver_object)], check=True)
        executable = output / ("test_rc_feedback_time_" + label + ".exe")
        subprocess.run([compiler, *target, str(test_object), str(driver_object),
                        "-Wl,--gc-sections", "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()

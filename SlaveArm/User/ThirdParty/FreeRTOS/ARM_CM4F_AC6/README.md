# FreeRTOS ARM Compiler 6 port

This directory contains the ARMCLANG-compatible Cortex-M4F/M7 FreeRTOS V10.6.2
port from STM32CubeH7 V1.13.0. It is shared by the EIDE and Keil AC6 targets so
the kernel and portable layer stay on the same release.

STM32CubeMX generates `Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F`
from its MDK-ARM 5 template. Those generated files use ARMCC5-only syntax and
must not be selected by an ARM Compiler 6 build. CubeMX runs
`tools/cubemx/post_generate.cmd` after generation to remove that RVDS source and
include path from the Keil project.

On a newly cloned Windows workspace, select that batch file once in
`Project Manager > Code Generator > User Actions > After Code Generation`.

# FreeRTOS ARM Compiler 6 port

This directory contains the ARMCLANG-compatible Cortex-M4F/M7 FreeRTOS port
used by the EIDE and Keil AC6 targets.

STM32CubeMX generates `Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F`
from its MDK-ARM 5 template. Those generated files use ARMCC5-only syntax and
must not be selected by an ARM Compiler 6 build.

Keep project source and include paths pointed at this directory after importing
or recreating an IDE project.

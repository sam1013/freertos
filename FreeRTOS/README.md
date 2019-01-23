# FreeRTOS port to RISC-V privileged spec 1.10

This is based on https://github.com/cjlano/freertos.git which is based on https://github.com/illustris/FreeRTOS-RISCV.

The fixes allow this code to run nicely in a Rocket RISC-V processor with local interrupt controller (Clint) using preemption.

This port supports TIMBER-V.

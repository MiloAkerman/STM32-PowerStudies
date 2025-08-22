# STM32 Help, Tips, and Advice

When working with STM32, particularly the H7 series, you might encounter several issues or concepts that seem unintuitive. As a team that has spent collective weeks working with them, we have written this document in the hopes it might help you debug issues particular to this project.

## Warnings / Advice

- **AVOID CUBEMX AS MUCH AS POSSIBLE**. For the sake of clarity, and because it was unavoidable on occasion, we have written a lot of code in sections outside of the designated user sections. CubeMX will delete this code if you regenerate. In general, CubeMX will mess with the filesystem and code in ways too intrusive for a mid-to-late-stage project. Avoid it.
    - Should you accidentally or purposefully regenerate code, you can always return to the previous state by means of `git checkout` on specific files or folders.

## Common Bugs
- Generic HAL issues
    - As far as we have found, the most stable configuration for HAL files is to include them twice: once in the root project directory, and once in the core directory. Modify this structure at your own risk.
- `_close/_fstat/_getpid/[...] is not implemented and will always fail`
    - You are missing `syscalls.c` under `Core/Src/`. For some reason, CubeMX likes to delete this file on occasion. Copy it over from another project.
- `arm_math.h: No such file or directory`
    - You are most likely missing Drivers/CMSIS/DSP in your imports `CM7 > Properties > C/C++ Build > Settings > MCU/MPU GCC Compiler > Include Paths`
        - In general, most `no such file or directory` issues result from incorrect include paths
- `Undefined reference to PDM_Filter_[...]`
    - You might be missing `:libPDMFilter_CM7_GCC_wc32.a` under `CM7 > Properties > C/C++ Build > Settings > MCU/MPU G++ Linker > Libraries > Libraries (-l)`. Alternatively, you might be missing the library search path (directly under).
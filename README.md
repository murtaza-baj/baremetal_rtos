# rtos\_baremetal

**Cooperative + preemptive RTOS framework in C** — implementing core real-time concepts (task scheduling, context switching, semaphores, IPC, and timing) for resource-constrained microcontrollers (TM4C123GXL).

---

## Project overview

This repository contains a compact, RTOS written in C for the **EK-TM4C123GXL (TM4C123GH6PM)** development board. It demonstrates both cooperative and preemptive multitasking and includes kernel services implemented via SVC, context switching via PendSV, a 1 ms SysTick, and simple semaphore-based synchronization.

The codebase is organized and modularized for clarity so it can be used for learning, experimentation, or as a starting point for embedded projects.

---

## Quick description

> Cooperative and preemptive RTOS framework in C implementing essential real-time concepts — task scheduling, context switching, semaphore synchronization, IPC primitives, and timing — targeted at TM4C-based embedded systems.

---

## Highlights

* Preemptive and cooperative scheduling modes
* Priority-based scheduler with an 8-level skip bias
* SVC-based kernel services: yield, sleep, wait/post semaphores, kill, ps, ipcs
* PendSV context switching with stack seeding for new threads
* Priority inheritance for semaphore holders
* Simple UART shell for runtime inspection and control (ps, ipcs, pid, kill, rtos mode switches)
* Per-task CPU% accounting using a wide timer

---

## Repository layout

```
rtos_baremetal/
├─ include/
│  └─ rtos.h                  # shared types, macros, prototypes, externs
├─ src/
│  ├─ 01_rtos.c               # kernel + ISRs + globals
│  ├─ hw.c                    # hw init, readPbs, stack/register wrappers, timers
│  ├─ tasks.c                 # tasks, shell, UART helpers
│  └─ main.c                  # main()
├─ startup/
│  └─ 01_tm4c123gh6pm_startup_ccs.c  # startup / vector table
└─ README.md
```

---

## Supported UART shell commands

* `ps` — print process list and CPU% accounting
* `ipcs` — print semaphore table and wait queues
* `pid of <name>` — print pid of process `<name>`
* `kill <pid>` — destroy task with numeric pid
* `reboot` — software reset
* `rtos preempt|coop` — switch RTOS between preemptive and cooperative modes
* `sched priority|rr` — switch scheduler bias on/off
* `pi on|off` — toggle priority inheritance
* `<process_name> &` — launch a stored directory process by name in background

---

## Hardware assumptions / notes

* System clock: **40 MHz**. SysTick configured for a 1 ms tick.
* Board: **EK-TM4C123GXL (TM4C123GH6PM)**.
* LEDs and push-buttons follow the original assignment wiring. Bit-banded references are used for direct GPIO access.
* Wide Timer (WTIMER5A) is used for per-task timing / profiling.

If you retarget to different pins or boards, update GPIO initializations and bit-band macros in `hw.c`.

---

## Building the Project in Code Composer Studio (CCS)

Follow these steps to import, build, and flash the project with CCS (tested workflow):

1. **Clone the repository**

   ```bash
   git clone https://github.com/<your-username>/<your-repo>.git
   cd <your-repo>
   ```

2. **Open Code Composer Studio**

   * Launch CCS and select a workspace.

3. **Import the project**

   * File → Import → Code Composer Studio → CCS Projects.
   * Click **Browse** and select the cloned repository folder.
   * The project should be discovered automatically (if not, choose **General → Existing Projects into Workspace** and select the `src`/`startup` files).
   * Ensure the project checkbox is checked and click **Finish**.

4. **Verify project properties**

   * Right-click the project → Properties.
   * Under **General → Device**, ensure the target device is **TM4C123GH6PM** (or the equivalent name shown by CCS for the TM4C123GXL).
   * Under **Build → MSP430/ARM Compiler → Include Options**, make sure `include/` is listed. If not, add the `include` directory to the compiler include path.

5. **Build**

   * Click the **Build** (hammer) icon. Resolve any missing include paths by adding `include/` in the Project Properties → Build → Compiler → Include Options.

6. **Connect the hardware**

   * Attach the EK-TM4C123GXL board via USB (or your TI/debug probe of choice). Attach the LED hardware board as well.

7. **Flash and run**

   * Click the **Debug** (green bug) icon. CCS will build (if required), load the program to the target, and start a debug session.
   * Use the **Resume** button to run or single-step. The UART banner is printed on successful startup.

8. **Open serial terminal**

   * Configure a serial terminal (115200 8N1) on the COM port associated with the board's UART0.
   * Interact with the shell commands.

**Troubleshooting tips**

* If the startup file or vector table symbol is not linked, ensure `01_tm4c123gh6pm_startup_ccs.c` is included in the project sources.
* If `getR0`/`getR1`/`getR2` SVC helper functions behave incorrectly, confirm your toolchain’s calling convention or provide assembler wrappers as needed (the sample uses simple C/asm wrappers).
* If you change the clock configuration, update the SysTick reload constants accordingly.

---

## Design notes

* Scheduler: circular scan with per-priority `skip` mechanism to bias higher-priority tasks.
* Context switching: PendSV ISR saves/restores task context; new tasks are seeded with a fake exception frame so they start correctly.
* IPC: fixed-size weighted semaphores with simple queue arrays (`MAX_QUEUE_SIZE`).
* Priority inheritance: when enabled, a semaphore holder’s `currentPriority` can be temporarily raised to the waiting task’s priority.

---

## Extending the project

Ideas for improvements and experiments:

* Replace fixed-size stacks/TCBs with dynamic allocation.
* Implement message queues or mailboxes with blocking/timed operations.
* Swap the scheduling policy to a priority queue or EDF for latency-sensitive tasks.
* Add unit tests for scheduler logic and semaphore edge cases.

---

## Acknowledgments

Special thanks to Professor **Jason Losh** for creating the basic framework used by this project. Portions of that original work are included under the MIT License.

---

## License

This project is licensed under the MIT License.  
© 2025 Murtaza Baj. Portions of the framework © Jason Losh, used under the MIT License.

---

## Author

**Murtaza Baj**

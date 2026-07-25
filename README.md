\# 8086 Smart Elevator Controller



A 4-floor elevator controller built entirely from discrete 8086-era hardware and hand-written 8086 assembly, simulated in \*\*Proteus\*\*. No microcontroller, no high-level firmware — just a bare 8086 in minimum mode, memory-mapped I/O, and real peripheral chips doing the work.



\---



\## 🖼️ Simulation



| Circuit running in Proteus |

|---|

| !\[Simulation 1](docs/simulation-1.png) |

| !\[Simulation 2](docs/simulation-2.png) |



\---



\## 🧠 How it works



The controller runs a \*\*SCAN (elevator) scheduling algorithm\*\*: it keeps travelling in its current direction as long as any hall or cabin request lies ahead, only reversing direction once nothing remains ahead — the classic anti-oscillation strategy used by real elevators.



\*\*Finite-state machine:\*\* `IDLE → UP / DOWN → DOOR → IDLE`, with a latched `EMERGENCY` state for power-outage / free-fall conditions.



\*\*Key subsystems, all tied together in the main loop:\*\*

\- Hall + cabin call buttons → request queues

\- SCAN scheduler → decides direction and next stop

\- Stepper-motor driver → physically "moves" the cabin between floors

\- Dual 7-segment displays → current floor / target floor

\- Character LCD → status messages ("MOVING UP", "DOOR OPEN", "EMERGENCY", ...)

\- Emergency sensors (simulated power-cut and free-fall switches), polled from every execution point in the program so an emergency is caught within one motor step or button cycle



Emergency handling is done via \*\*polling\*\*, not a hardware interrupt controller — a deliberate design decision documented in the source, after `CALL`/`RET` and interrupt linkage were found to be unreliable on this particular Proteus 8086 setup (see \[Hardware notes](#-hardware-notes--proteus-quirks) below). All subroutine linkage in the program is done manually with return-id variables and a direct-jump ladder instead of `CALL`/`RET`.



\---



\## 🔩 Hardware Architecture



\- \*\*Intel 8086\*\* microprocessor, minimum mode

\- \*\*74LS373\*\* address latches (demultiplexing the multiplexed AD bus)

\- \*\*74LS138\*\* decoders for I/O and memory chip-select

\- Even/odd banked memory: \*\*62256\*\* SRAM + \*\*27C256\*\* EPROM (16-bit bus split into even/odd byte banks)

\- \*\*Two 8255A PPIs\*\* — one drives the 7-segment displays, LCD, and motor; the other reads hall/cabin buttons and the emergency sensors

\- \*\*ULN2003A\*\* Darlington array driving a 4-phase \*\*stepper motor\*\* (cabin motion)

\- Dual \*\*7-segment displays\*\* — current floor and target floor

\- \*\*16x2 character LCD\*\* — status/message output

\- Hall call buttons (floors 1–4) and in-cabin call buttons (cabin 0–3)

\- Simulated \*\*power-cut\*\* and \*\*free-fall\*\* sensor switches, plus an emergency-lock indicator LED



\---



\## 🛠️ Hardware notes \& Proteus quirks



This project involved extensive low-level debugging of Proteus-specific simulator behavior that doesn't necessarily reflect real 8086 hardware, including:

\- A stale prefetch-queue write bug, worked around with a fixed pattern: \*\*a three-instruction dummy read before every `OUT` / ALU-result write\*\*

\- LCD initialization quirks specific to the simulated LM016L module

\- Manual subroutine linkage (return-id + jump ladder) used throughout, after `CALL`/`RET` proved unreliable in this simulated environment



\---



\## 📁 Repository Contents



```

ELEVATOR.pdsprj              Proteus project (schematic + simulation)

elevator\_layer8\_swint.asm    Main 8086 assembly source (heavily commented)

elevator\_layer8\_swint\_even.bin   Even-byte ROM image

elevator\_layer8\_swint\_odd.bin    Odd-byte ROM image

docs/                        Simulation screenshots

```



\## ▶️ Running it



1\. Open `ELEVATOR.pdsprj` in Proteus.

2\. Run the simulation — the pre-built `even`/`odd` ROM binaries are already wired into the memory chips.

3\. To modify the assembly and rebuild: assemble `elevator\_layer8\_swint.asm`, then split the resulting binary into even/odd byte banks to match the 16-bit-bus / 8-bit-memory-chip layout before loading it back into Proteus.



\---



\## 📄 License



This project was built as a university coursework / portfolio project.


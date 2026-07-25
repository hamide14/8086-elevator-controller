; ================================================================
;  SMART ELEVATOR CONTROLLER  -  8086 / 2x 8255 / Proteus
;  LAYER 8 : L7 + REAL INTERRUPTS via software INT (40h emergency,
;            41h power) - IVT/ISR/IRET, validated on this board, no 8259
; ----------------------------------------------------------------
;  Ties together every proven subsystem:
;    buttons (L3) -> request queues -> SCAN scheduler ->
;    stepper motion (L2) -> 7-seg current+target (L1) ->
;    LCD status messages (L4) -> door service.
;
;  FSM STATES (RAM 1006h):  0=IDLE  1=UP  2=DOWN  3=DOOR  4=EMERGENCY
;  DIRECTION (RAM 1004h):   0=idle  1=up   2=down
;
;  SCAN ALGORITHM (anti-oscillation):
;    * keep travelling in the current direction while any request
;      lies ahead in that direction (hall OR cabin);
;    * only reverse when nothing remains ahead;
;    * stop + open door at every requested floor reached;
;    * idle (motor holds) when no requests remain.
;
;  BUTTON MAP  : PC0-3 = HALL floors 0-3 , PC4-7 = CABIN floors 0-3
;  7-SEG       : PA1 = current floor , PB1 = target (next stop)
;  MOTOR       : PC2 , UP = 01,02,04,08 (phase index in BP)
;                DOWN = decrement phase -> 08,04,02,01
;                STEPS_PER_FLOOR = 10  (immediate at MU_STEP/MD_STEP)
;
;  NO CALL / RET / indirect jump. Manual linkage = per-routine
;  return-id variable + direct-jump ladder.
; ----------------------------------------------------------------
;  RAM MAP (DS=0000)
;    0500h DUMMY      bus dummy-read (=AAAAh)
;    1000h CUR_FLOOR  word (high byte kept 0)
;    1002h TGT_FLOOR  (informational)
;    1004h DIRECTION  0/1/2
;    1006h STATE      0/1/2/3/4 (init FFh to force first paint)
;    1008h HALL_REQ   word  bit0-3
;    100Ah CAB_REQ    word  bit0-3
;    100Ch PREV_KEYS  last PC1 sample
;    1010h RET_CMD    LCD_CMD  return id
;    1012h RET_DELAY  DELAY    return id
;    1014h RET_FLOOR  SHOW_FLOOR return id
;    1016h RET_SHOW   SHOW_MSG return id
;    1018h RET_RD     RD_BUTTONS return id
;                       1=main, 2=door dwell, 3=motor step UP, 4=motor step DOWN
;    101Ah RET_DWELL  DOOR_DWELL return id (1=after ARRIVED, 2=after door)
;    101Ch PWR_FLAG   0=normal, 1=currently in simulated power outage
;    101Dh FALL_FLAG  0=normal, 1=latched EMERGENCY (never cleared - reset only)
;    BP (register)    motor phase index 0-3 (persistent)
;
;  rev2: buttons are polled CONTINUOUSLY during the door dwell
;  (DOOR_DWELL), so a call pressed while the door is open is never
;  missed.
;
;  rev3: buttons are ALSO polled once per motor step, inside
;  MU_STEP/MD_STEP (between the OUT and the inter-step delay).
;  rev2 alone still missed a full press+release that happened
;  entirely while the cabin was travelling between floors (the
;  10-step / ~1s motion had NO button sampling at all) - that gap
;  is what rev3 closes.
;
;  LAYER 6 (this revision): power-outage + free-fall handling, per
;  the project proposal.
;
;  HARDWARE ADDED: two switches (10k pull-up each, active-low, same
;  style as the request buttons) on 8255 #2's Port C UPPER nibble:
;    PC4 (bit 0x10) = POWER_OUTAGE sensor
;    PC5 (bit 0x20) = FREE_FALL sensor  (simulated accelerometer)
;  CTRL2 (000Eh) changed from 80h (all out) to 88h, which makes
;  Port C upper nibble INPUT while PA/PB/PC-lower (motor phases)
;  stay OUTPUT (8255 mode 0 supports split PC direction).
;
;  DESIGN DECISION - polling instead of a real 8259 interrupt:
;  the proposal asks for hardware interrupts (8259) to prioritise
;  these events. INT/IRET push/pop CS:IP the same way CALL/RET do,
;  and CALL/RET were already proven unreliable on this Proteus
;  8086 setup (see project summary report) - so real INT was judged
;  too risky to introduce untested at this stage. Instead, both
;  sensors are checked inside RD_BUTTONS (CHECK_EMERGENCY), which
;  already runs from every execution point in the program (MAIN,
;  DOOR_DWELL, MU_STEP, MD_STEP) - so an emergency is caught within
;  one motor step or one dwell tick from anywhere, without touching
;  CALL/RET/INT. Functionally equivalent prioritisation, without
;  the unproven mechanism. If real 8259 wiring is wanted later, it
;  should be tested in isolation (a tiny INT/IRET-only program)
;  before being merged into this codebase.
;
;  POWER_OUTAGE behaviour: motor freezes exactly where it is (BP,
;  DI, CUR_FLOOR, HALL_REQ, CAB_REQ are all untouched while paused,
;  since nothing modifies them during the wait), LCD shows "POWER
;  OUTAGE", and execution resumes exactly where it paused once the
;  switch clears - this satisfies "save and restore" for free,
;  because nothing was ever actually lost. On restore, the correct
;  LCD message (MOVING UP/DOWN, ARRIVED, DOOR OPEN, or IDLE) is
;  re-painted immediately from STATE (and RET_DWELL for the
;  ARRIVED-vs-DOOR-OPEN distinction), so the display does not stay
;  stuck on "POWER OUTAGE" after the switch clears.
;
;  FREE_FALL behaviour: stepper is de-energised immediately
;  (protected write of 00h to the motor port = emergency brake),
;  STATE is set to 4 (EMERGENCY) and FALL_FLAG (101Dh) is latched
;  for anyone inspecting RAM, LCD shows "EMERGENCY STOP", a
;  physical DOOR_LOCK indicator (PB2 bit3, 000Ah) is driven high as
;  the final write that port ever receives, and the program halts
;  permanently (FALL_HANG) - matching real emergency-stop behaviour,
;  this requires a hardware reset, not an automatic recovery.
;  PB2 bit3 is safe to claim: LCD_CMD/LCD_DATA only ever write
;  00h/01h/04h/05h to that port, so bit3 is always 0 until this.
; ================================================================

ORG     0000h
START:
    CLI
    MOV     AX, 0000h
    MOV     DS, AX
    MOV     ES, AX
    MOV     SS, AX
    MOV     SP, 7000h
    MOV     WORD PTR [0500h], 0AAAAh

    ; ---------- 8255 #1 : PA out, PB out, PC in (89h) ----------
    MOV     AL, 89h
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    MOV     DX, 0006h
    OUT     DX, AL
    ; ---------- 8255 #2 : PA/PB/PCL out, PCU in (88h) ----------
    ; PC4 = POWER_OUTAGE sensor switch, PC5 = FREE_FALL sensor switch
    ; (both active-low with 10k pull-ups, same style as the buttons)
    MOV     AL, 88h
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    MOV     DX, 000Eh
    OUT     DX, AL

    ; ---------- rev7: restore state from SIMULATED NVRAM (0600h) if a ----
    ; ---------- prior power failure saved one (signature 5A5Ah@0600h) ---
    MOV     AX, [0600h]
    CMP     AX, 5A5Ah
    JNE     SEED_DEFAULTS
    ; ----- valid snapshot found -> restore (protected stores) -----
    MOV     AX, [0602h]                   ; saved CUR_FLOOR
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [1000h], AX
    MOV     AX, [0604h]                   ; saved DIRECTION
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [1004h], AX
    MOV     AX, [0608h]                   ; saved HALL_REQ
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [1008h], AX
    MOV     AX, [060Ah]                   ; saved CAB_REQ
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [100Ah], AX
    MOV     AX, [060Ch]                   ; saved motor phase
    MOV     BP, AX
    MOV     BYTE PTR [1002h], 00h         ; TGT_FLOOR
    MOV     BYTE PTR [1006h], 0FFh        ; STATE = force repaint from restored dir/reqs
    MOV     BYTE PTR [100Ch], 0FFh        ; PREV_KEYS = released
    MOV     BYTE PTR [101Ch], 00h         ; PWR_FLAG = normal
    ; invalidate snapshot so we restore only ONCE (protected)
    MOV     AX, 0000h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0600h], AX
    JMP     SEED_DONE
SEED_DEFAULTS:
    ; ---------- seed state (cold boot / no saved snapshot) ----------
    MOV     WORD PTR [1000h], 0000h       ; CUR_FLOOR = 0
    MOV     BYTE PTR [1002h], 00h         ; TGT_FLOOR
    MOV     BYTE PTR [1004h], 00h         ; DIRECTION = idle
    MOV     BYTE PTR [1006h], 0FFh        ; STATE = unknown (force paint)
    MOV     WORD PTR [1008h], 0000h       ; HALL_REQ
    MOV     WORD PTR [100Ah], 0000h       ; CAB_REQ
    MOV     BYTE PTR [100Ch], 0FFh        ; PREV_KEYS = released
    MOV     BYTE PTR [101Ch], 00h         ; PWR_FLAG = normal
    MOV     BP, 0000h                     ; motor phase index
SEED_DONE:

    ; ---------- rev8: install SOFTWARE-INTERRUPT vectors (no 8259) ----------
    ; INT 40h -> EMERGENCY ISR (free-fall / fire / e-stop)  vector @ 0100h
    ; INT 41h -> POWER-OUTAGE ISR                            vector @ 0104h
    ; A software INT needs no IF/STI and no hardware handshake - proven
    ; to work on this board by software_int_test (RAM 1000h=22,1001h=AA).
    MOV     AX, OFFSET FALL_EVENT
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0100h], AX
    MOV     AX, 0F000h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0102h], AX
    MOV     AX, OFFSET PWR_EVENT
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0104h], AX
    MOV     AX, 0F000h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0106h], AX

    ; ================= LCD INITIALISATION (proven) =================
    MOV     BYTE PTR [1012h], 01h
    JMP     DELAY
D_R1:
    MOV     AL, 38h
    MOV     BYTE PTR [1010h], 01h
    JMP     LCD_CMD
INIT_R1:
    MOV     BYTE PTR [1012h], 02h
    JMP     DELAY
D_R2:
    MOV     AL, 0Ch
    MOV     BYTE PTR [1010h], 02h
    JMP     LCD_CMD
INIT_R2:
    MOV     BYTE PTR [1012h], 03h
    JMP     DELAY
D_R3:
    MOV     AL, 01h
    MOV     BYTE PTR [1010h], 03h
    JMP     LCD_CMD
INIT_R3:
    MOV     BYTE PTR [1012h], 04h
    JMP     DELAY
D_R4:
    MOV     AL, 06h
    MOV     BYTE PTR [1010h], 04h
    JMP     LCD_CMD
INIT_R4:
    MOV     BYTE PTR [1012h], 05h
    JMP     DELAY
D_R5:
    ; init done -> run

; ================================================================
;  MAIN LOOP
; ================================================================
MAIN:
    ; (A) sample buttons -> update HALL_REQ / CAB_REQ
    MOV     BYTE PTR [1018h], 01h         ; RET_RD = 1 (return to MA_AFTER_READ)
    JMP     RD_BUTTONS
MA_AFTER_READ:
    ; (B) REQ = HALL | CAB  (bits 0-3) -> BL
    MOV     AL, [1008h]
    OR      AL, [100Ah]
    AND     AL, 0Fh
    MOV     BL, AL                        ; BL = REQ mask
    ; (C) CF -> CL
    MOV     CL, [1000h]
    ; (D) current-floor bit mask -> DL
    CMP     CL, 00h
    JNE     Q1
    MOV     DL, 01h
    JMP     QDONE
Q1:
    CMP     CL, 01h
    JNE     Q2
    MOV     DL, 02h
    JMP     QDONE
Q2:
    CMP     CL, 02h
    JNE     Q3
    MOV     DL, 04h
    JMP     QDONE
Q3:
    MOV     DL, 08h
QDONE:
    ; (E) request at current floor?  -> SERVICE
    MOV     AL, BL
    AND     AL, DL
    CMP     AL, 00h
    JNE     SERVICE
    ; (F) no requests at all?  -> IDLE
    CMP     BL, 00h
    JNE     DECIDE
    JMP     GO_IDLE

; SCAN direction decision +++++++++++++++++++++++++++++++++++++++++++++++++++++
DECIDE:
    ;masks 
    CMP     CL, 00h
    JNE     DC1
    MOV     DH, 0Eh
    MOV     DL, 00h
    JMP     DMASK
DC1:
    CMP     CL, 01h
    JNE     DC2
    MOV     DH, 0Ch
    MOV     DL, 01h
    JMP     DMASK
DC2:
    CMP     CL, 02h
    JNE     DC3
    MOV     DH, 08h
    MOV     DL, 03h
    JMP     DMASK
DC3:
    MOV     DH, 00h
    MOV     DL, 07h
DMASK:
    ; DH = REQ_ABOVE , DL = REQ_BELOW
    MOV     AL, BL
    AND     AL, DH
    MOV     DH, AL
    MOV     AL, BL
    AND     AL, DL
    MOV     DL, AL
    ; branch on current DIRECTION
    MOV     CH, [1004h]
    CMP     CH, 01h
    JE      DIR_UP
    CMP     CH, 02h
    JE      DIR_DN
    ; idle: choose toward nearest request (prefer up if any above)
    CMP     DH, 00h
    JNE     GOTO_UP
    JMP     GOTO_DN
DIR_UP:
    CMP     DH, 00h
    JNE     GOTO_UP
    JMP     GOTO_DN                       
DIR_DN:
    CMP     DL, 00h
    JNE     GOTO_DN
    JMP     GOTO_UP                       

GOTO_UP:
    MOV     AL, [1006h]
    CMP     AL, 01h
    JE      MOVE_UP1                     
    MOV     BYTE PTR [1004h], 01h
    MOV     BYTE PTR [1006h], 01h
    MOV     BX, OFFSET STR_MUP
    MOV     BYTE PTR [1016h], 01h
    JMP     SHOW_MSG
GU_R1:
    JMP     MOVE_UP1

GOTO_DN:
    MOV     AL, [1006h]
    CMP     AL, 02h
    JE      MOVE_DN1
    MOV     BYTE PTR [1004h], 02h
    MOV     BYTE PTR [1006h], 02h
    MOV     BX, OFFSET STR_MDN
    MOV     BYTE PTR [1016h], 02h
    JMP     SHOW_MSG
GD_R1:
    JMP     MOVE_DN1

; ---------------- move one floor UP ----------------
MOVE_UP1:
    ; target = nearest requested floor above CF  -> PB1
    MOV     CL, [1000h]
    MOV     AL, [1008h]
    OR      AL, [100Ah]
    AND     AL, 0Fh
    MOV     BL, AL
    MOV     CH, 0FFh
    TEST    BL, 08h
    JZ      UT_A
    MOV     CH, 03h
UT_A:
    TEST    BL, 04h
    JZ      UT_B
    CMP     CL, 02h
    JAE     UT_B
    MOV     CH, 02h
UT_B:
    TEST    BL, 02h
    JZ      UT_C
    CMP     CL, 01h
    JAE     UT_C
    MOV     CH, 01h
UT_C:
    CMP     CH, 0FFh
    JNE     UT_SHOW
    MOV     CH, CL
UT_SHOW:
    MOV     AL, CH
    MOV     DX, 0002h                     ; PB1
    MOV     BYTE PTR [1014h], 01h
    JMP     SHOW_FLOOR
MU_R2:
    ; --- 10 steps up ---
    MOV     DI, 000Ah                     ; STEPS_PER_FLOOR
MU_STEP:
    CMP     BP, 0000h
    JNE     MUP1
    MOV     AL, 01h
    JMP     MUPSET
MUP1:
    CMP     BP, 0001h
    JNE     MUP2
    MOV     AL, 02h
    JMP     MUPSET
MUP2:
    CMP     BP, 0002h
    JNE     MUP3
    MOV     AL, 04h
    JMP     MUPSET
MUP3:
    MOV     AL, 08h
MUPSET:
    MOV     DX, 000Ch
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    ; poll buttons during motor step (fixes concurrent-request miss)
    MOV     BYTE PTR [1018h], 03h         ; RET_RD = 3 -> MU_STEP_CONT
    JMP     RD_BUTTONS
MU_STEP_CONT:
    PUSH    CX
    MOV     CX, 1800h
MU_SD:
    LOOP    MU_SD
    POP     CX
    INC     BP
    CMP     BP, 0004h
    JNE     MU_PHOK
    MOV     BP, 0000h
MU_PHOK:
    DEC     DI
    JNZ     MU_STEP
    ; CF = CF + 1 (protected store)
    MOV     AX, [1000h]
    INC     AX
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [1000h], AX
    ; PA1 = CF
    MOV     AL, [1000h]
    MOV     DX, 0000h
    MOV     BYTE PTR [1014h], 02h
    JMP     SHOW_FLOOR
MU_R3:
    JMP     MAIN

; ---------------- move one floor DOWN ----------------
MOVE_DN1:
    ; target = nearest requested floor below CF -> PB1
    MOV     CL, [1000h]
    MOV     AL, [1008h]
    OR      AL, [100Ah]
    AND     AL, 0Fh
    MOV     BL, AL
    MOV     CH, 0FFh
    TEST    BL, 01h
    JZ      DT_A
    MOV     CH, 00h
DT_A:
    TEST    BL, 02h
    JZ      DT_B
    CMP     CL, 01h
    JBE     DT_B
    MOV     CH, 01h
DT_B:
    TEST    BL, 04h
    JZ      DT_C
    CMP     CL, 02h
    JBE     DT_C
    MOV     CH, 02h
DT_C:
    CMP     CH, 0FFh
    JNE     DT_SHOW
    MOV     CH, CL
DT_SHOW:
    MOV     AL, CH
    MOV     DX, 0002h                     ; PB1
    MOV     BYTE PTR [1014h], 03h
    JMP     SHOW_FLOOR
MD_R2:
    MOV     DI, 000Ah                     ; STEPS_PER_FLOOR
MD_STEP:
    CMP     BP, 0000h
    JNE     MDP1
    MOV     AL, 01h
    JMP     MDPSET
MDP1:
    CMP     BP, 0001h
    JNE     MDP2
    MOV     AL, 02h
    JMP     MDPSET
MDP2:
    CMP     BP, 0002h
    JNE     MDP3
    MOV     AL, 04h
    JMP     MDPSET
MDP3:
    MOV     AL, 08h
MDPSET:
    MOV     DX, 000Ch
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    ; poll buttons during motor step (fixes concurrent-request miss)
    MOV     BYTE PTR [1018h], 04h         ; RET_RD = 4 -> MD_STEP_CONT
    JMP     RD_BUTTONS
MD_STEP_CONT:
    PUSH    CX
    MOV     CX, 1800h
MD_SD:
    LOOP    MD_SD
    POP     CX
    ; advance phase DOWN (wrap 0 -> 3)
    CMP     BP, 0000h
    JNE     MD_DEC
    MOV     BP, 0004h
MD_DEC:
    DEC     BP
    DEC     DI
    JNZ     MD_STEP
    ; CF = CF - 1 (protected store)
    MOV     AX, [1000h]
    DEC     AX
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [1000h], AX
    MOV     AL, [1000h]
    MOV     DX, 0000h
    MOV     BYTE PTR [1014h], 04h
    JMP     SHOW_FLOOR
MD_R3:
    JMP     MAIN

; ---------------- service (door) at current floor ----------------
SERVICE:
    ; ---- rev7: refresh PB1 (target 7-seg) to the NEXT destination -----
    ; Fixes cosmetic stale-target bug when a stop is added mid-route.
    ; Inputs from MAIN: CL = current floor, BL = REQ mask (bits 0-3).
    ; Pure display refresh: does NOT touch HALL/CAB/DIRECTION/motor; every
    ; register it uses is reloaded by the ARRIVED code that follows SV_TGT.
    CMP     CL, 00h
    JNE     STM1
    MOV     AH, 01h
    JMP     STMSK
STM1:
    CMP     CL, 01h
    JNE     STM2
    MOV     AH, 02h
    JMP     STMSK
STM2:
    CMP     CL, 02h
    JNE     STM3
    MOV     AH, 04h
    JMP     STMSK
STM3:
    MOV     AH, 08h
STMSK:
    NOT     AH                            ; AH = ~current-floor bit
    MOV     AL, BL
    AND     AL, AH                        ; AL = remaining requests (excl current)
    MOV     DH, AL
    CMP     DH, 00h
    JE      ST_CUR                        ; nothing else pending -> show current
    MOV     CH, [1004h]                   ; DIRECTION (1=up,2=down,0=idle)
    CMP     CH, 02h
    JE      ST_DN
; ----- prefer UP: nearest above, else nearest below -----
ST_UP:
    TEST    DH, 02h
    JZ      SU2
    CMP     CL, 01h
    JAE     SU2
    JMP     ST_F1
SU2:
    TEST    DH, 04h
    JZ      SU3
    CMP     CL, 02h
    JAE     SU3
    JMP     ST_F2
SU3:
    TEST    DH, 08h
    JZ      SU_BELOW
    CMP     CL, 03h
    JAE     SU_BELOW
    JMP     ST_F3
SU_BELOW:
    TEST    DH, 04h
    JZ      SUB1
    CMP     CL, 02h
    JBE     SUB1
    JMP     ST_F2
SUB1:
    TEST    DH, 02h
    JZ      SUB0
    CMP     CL, 01h
    JBE     SUB0
    JMP     ST_F1
SUB0:
    TEST    DH, 01h
    JZ      ST_CUR
    JMP     ST_F0
; ----- prefer DOWN: nearest below, else nearest above -----
ST_DN:
    TEST    DH, 04h
    JZ      SD1
    CMP     CL, 02h
    JBE     SD1
    JMP     ST_F2
SD1:
    TEST    DH, 02h
    JZ      SD0
    CMP     CL, 01h
    JBE     SD0
    JMP     ST_F1
SD0:
    TEST    DH, 01h
    JZ      SD_ABOVE
    JMP     ST_F0
SD_ABOVE:
    TEST    DH, 02h
    JZ      SDA2
    CMP     CL, 01h
    JAE     SDA2
    JMP     ST_F1
SDA2:
    TEST    DH, 04h
    JZ      SDA3
    CMP     CL, 02h
    JAE     SDA3
    JMP     ST_F2
SDA3:
    TEST    DH, 08h
    JZ      ST_CUR
    CMP     CL, 03h
    JAE     ST_CUR
    JMP     ST_F3
; ----- resolve chosen floor -> AL, then paint PB1 -----
ST_F0:
    MOV     AL, 00h
    JMP     ST_OUT
ST_F1:
    MOV     AL, 01h
    JMP     ST_OUT
ST_F2:
    MOV     AL, 02h
    JMP     ST_OUT
ST_F3:
    MOV     AL, 03h
    JMP     ST_OUT
ST_CUR:
    MOV     AL, CL
ST_OUT:
    MOV     DX, 0002h                     ; PB1 (target 7-seg)
    MOV     BYTE PTR [1014h], 07h
    JMP     SHOW_FLOOR
SV_TGT:
    MOV     BYTE PTR [1006h], 03h         ; STATE = DOOR
    ; LCD "ARRIVED"
    MOV     BX, OFFSET STR_ARR
    MOV     BYTE PTR [1016h], 03h
    JMP     SHOW_MSG
SV_R1:
    ; short "arrived" dwell - keeps polling buttons
    MOV     DI, 0030h
    MOV     BYTE PTR [101Ah], 01h         ; RET_DWELL = 1
    JMP     DOOR_DWELL
SV_AFTER_ARR:
    ; LCD "DOOR OPEN"
    MOV     BX, OFFSET STR_DOOR
    MOV     BYTE PTR [1016h], 04h
    JMP     SHOW_MSG
SV_R2:
    ; door-open dwell - keeps polling buttons
    MOV     DI, 0090h
    MOV     BYTE PTR [101Ah], 02h         ; RET_DWELL = 2
    JMP     DOOR_DWELL
SV_AFTER_DOOR:
    ; clear this floor's request bit from HALL and CAB
    MOV     CL, [1000h]
    CMP     CL, 00h
    JNE     CM1
    MOV     DL, 01h
    JMP     CMDONE
CM1:
    CMP     CL, 01h
    JNE     CM2
    MOV     DL, 02h
    JMP     CMDONE
CM2:
    CMP     CL, 02h
    JNE     CM3
    MOV     DL, 04h
    JMP     CMDONE
CM3:
    MOV     DL, 08h
CMDONE:
    MOV     AL, DL
    NOT     AL
    MOV     DL, AL                        ; DL = ~mask
    ; HALL_REQ &= ~mask
    MOV     AL, [1008h]
    AND     AL, DL
    MOV     AH, 00h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [1008h], AX
    ; CAB_REQ &= ~mask
    MOV     AL, [100Ah]
    AND     AL, DL
    MOV     AH, 00h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [100Ah], AX
    JMP     MAIN

; ---------------- idle ----------------
GO_IDLE:
    MOV     AL, [1006h]
    CMP     AL, 00h
    JE      GI_POLL                       ; already idle -> just poll
    MOV     BYTE PTR [1004h], 00h
    MOV     BYTE PTR [1006h], 00h
    MOV     BX, OFFSET STR_IDLE
    MOV     BYTE PTR [1016h], 05h
    JMP     SHOW_MSG
GI_R1:
    MOV     AL, [1000h]                   ; PA1 = current floor
    MOV     DX, 0000h
    MOV     BYTE PTR [1014h], 05h
    JMP     SHOW_FLOOR
GI_R1B:
    MOV     AL, [1000h]                   ; PB1 = current floor (no target)
    MOV     DX, 0002h
    MOV     BYTE PTR [1014h], 06h
    JMP     SHOW_FLOOR
GI_R2B:
GI_POLL:
    JMP     MAIN

; ================================================================
;  RD_BUTTONS : sample PC1, release-edge detect, OR into requests
;  4 callers, dispatched via RET_RD (1018h):
;    1=MAIN, 2=DOOR_DWELL, 3=MU_STEP (motor up), 4=MD_STEP (motor down)
; ================================================================
RD_BUTTONS:
    MOV     DX, 0004h
    IN      AL, DX                        ; AL = now (R4: IN safe)
    MOV     BL, [100Ch]                   ; prev
    MOV     BH, AL                        ; now (for prev update)
    XOR     BL, AL                        ; changed
    AND     BL, BH                        ; release edges (0->1) -> BL
    ; prev = now  (protected)
    MOV     AL, BH
    MOV     AH, 00h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [100Ch], AX
    ; HALL_REQ |= edges[0:3]
    MOV     AL, BL
    AND     AL, 0Fh
    MOV     DL, [1008h]
    OR      AL, DL
    MOV     AH, 00h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [1008h], AX
    ; CAB_REQ |= edges[4:7] >> 4
    MOV     AL, BL
    AND     AL, 0F0h
    MOV     CL, 04h
    SHR     AL, CL
    MOV     DL, [100Ah]
    OR      AL, DL
    MOV     AH, 00h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [100Ah], AX
    JMP     CHECK_EMERGENCY

; ================================================================
;  EMERGENCY SENSORS (8255 #2, Port C upper nibble, 000Ch)
;  PC4 = POWER_OUTAGE (active-low)   PC5 = FREE_FALL (active-low)
;  Checked here because RD_BUTTONS is already called from every
;  execution point (MAIN, DOOR_DWELL, MU_STEP, MD_STEP) - so this
;  reacts within one motor step / one dwell tick from anywhere in
;  the program, without needing real hardware interrupts.
; ================================================================
CHECK_EMERGENCY:
    MOV     DX, 000Ch
    IN      AL, DX
    TEST    AL, 20h                       ; FREE_FALL bit (active-low)
    JZ      CE_FALL
    TEST    AL, 10h                       ; POWER_OUTAGE bit (active-low)
    JZ      CE_PWR
    ; --- rev8 OPTIONAL (enable AFTER wiring PC6/PC7 with 10k pull-ups) ---
    ; TEST    AL, 80h                     ; FIRE sensor (PC7)
    ; JZ      CE_FALL
    ; TEST    AL, 40h                     ; EMERGENCY-STOP button (PC6)
    ; JZ      CE_FALL
RD_DISPATCH:
    CMP     BYTE PTR [1018h], 01h
    JE      MA_AFTER_READ
    CMP     BYTE PTR [1018h], 02h
    JE      DD_CONT
    CMP     BYTE PTR [1018h], 03h
    JE      MU_STEP_CONT
    JMP     MD_STEP_CONT

; ---- rev8: raise emergency / power events as SOFTWARE INTERRUPTS ----
CE_FALL:
    INT     40h                           ; -> FALL_EVENT ISR (does NOT return)
    JMP     RD_DISPATCH                   ; (never reached; emergency hangs)
CE_PWR:
    INT     41h                           ; -> PWR_EVENT ISR (returns when power back)
    JMP     RD_DISPATCH                   ; resume exactly where we were

; ---------------- power outage ISR (INT 41h): pause + resume --------
PWR_EVENT:
    PUSH    AX
    PUSH    BX
    PUSH    CX
    PUSH    DX
    PUSH    SI
    CMP     BYTE PTR [101Ch], 01h
    JE      PWR_WAIT                      ; message already shown
    MOV     BYTE PTR [101Ch], 01h
    ; ---- rev7: snapshot live state into simulated NVRAM (protected) ----
    MOV     AX, [1000h]
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0602h], AX
    MOV     AL, [1004h]
    MOV     AH, 00h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0604h], AX
    MOV     AX, [1008h]
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0608h], AX
    MOV     AX, [100Ah]
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [060Ah], AX
    MOV     AX, BP
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [060Ch], AX
    ; write signature LAST so a half-written snapshot never looks valid
    MOV     AX, 5A5Ah
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0600h], AX
    MOV     BX, OFFSET STR_PWR
    MOV     BYTE PTR [1016h], 06h
    JMP     SHOW_MSG
PWR_R1:
PWR_WAIT:
    MOV     DX, 000Ch
    IN      AL, DX
    TEST    AL, 10h
    JZ      PWR_WAIT                      ; still out -> keep waiting
    MOV     BYTE PTR [101Ch], 00h         ; power restored
    ; rev7: normal in-place resume -> invalidate NVRAM snapshot (protected)
    MOV     AX, 0000h
    MOV     SI, AX
    MOV     AX, [0500h]
    PUSH    SI
    POP     AX
    MOV     [0600h], AX
    ; restore the correct LCD message for the current STATE, then resume
    MOV     AL, [1006h]
    CMP     AL, 01h
    JE      PWR_RS_UP
    CMP     AL, 02h
    JE      PWR_RS_DN
    CMP     AL, 03h
    JE      PWR_RS_DOOR
    MOV     BX, OFFSET STR_IDLE
    JMP     PWR_RESTORE
PWR_RS_UP:
    MOV     BX, OFFSET STR_MUP
    JMP     PWR_RESTORE
PWR_RS_DN:
    MOV     BX, OFFSET STR_MDN
    JMP     PWR_RESTORE
PWR_RS_DOOR:
    CMP     BYTE PTR [101Ah], 01h         ; which door-phase were we in?
    JE      PWR_RS_ARR
    MOV     BX, OFFSET STR_DOOR
    JMP     PWR_RESTORE
PWR_RS_ARR:
    MOV     BX, OFFSET STR_ARR
PWR_RESTORE:
    MOV     BYTE PTR [1016h], 08h
    JMP     SHOW_MSG
PWR_R2:
    POP     SI
    POP     DX
    POP     CX
    POP     BX
    POP     AX
    IRET                                  ; return from software interrupt -> resume

; ---------------- free fall: emergency brake, latch forever ------
FALL_EVENT:
    MOV     BYTE PTR [1006h], 04h         ; STATE = 4 (EMERGENCY)
    MOV     BYTE PTR [101Dh], 01h         ; FALL_FLAG = 1 (latched)
    ; de-energize the stepper immediately (protected write)
    MOV     AL, 00h
    MOV     DX, 000Ch
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    MOV     BX, OFFSET STR_FALL
    MOV     BYTE PTR [1016h], 07h
    JMP     SHOW_MSG
FALL_R1:
    ; DOOR_LOCK indicator: PB2 bit3 (0x08) is never set by LCD_CMD/
    ; LCD_DATA (they only ever write 00h/01h/04h/05h), so this is
    ; safe to claim. This is the LAST write PB2 will ever see (we
    ; hang right after), so it latches high forever - a real,
    ; physical "doors locked" signal, not just an LCD message.
    MOV     AL, 09h                        ; RS=1,E=0 (idle) | DOOR_LOCK=1
    MOV     DX, 000Ah
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
FALL_HANG:
    JMP     FALL_HANG                     ; requires hardware reset

; ================================================================
;  DOOR_DWELL : hold the door open while CONTINUOUSLY polling the
;  buttons, so a request pressed/released during the door dwell is
;  never missed. DI = number of poll ticks. RET_DWELL (101Ah) picks
;  the return point.
; ================================================================
DOOR_DWELL:
DD_LOOP:
    MOV     BYTE PTR [1018h], 02h         ; RD_BUTTONS returns to DD_CONT
    JMP     RD_BUTTONS
DD_CONT:
    PUSH    CX
    MOV     CX, 0400h                     ; proven SHORT_DELAY tick (~poll interval)
DD_SD:
    LOOP    DD_SD
    POP     CX
    DEC     DI
    JNZ     DD_LOOP
    CMP     BYTE PTR [101Ah], 01h
    JE      SV_AFTER_ARR
    JMP     SV_AFTER_DOOR

; ================================================================
;  SHOW_FLOOR : AL=floor 0..3 , DX=port , RET_FLOOR=id
; ================================================================
SHOW_FLOOR:
    CMP     AL, 00h
    JNE     SF1
    MOV     AL, 3Fh
    JMP     SFOUT
SF1:
    CMP     AL, 01h
    JNE     SF2
    MOV     AL, 06h
    JMP     SFOUT
SF2:
    CMP     AL, 02h
    JNE     SF3
    MOV     AL, 5Bh
    JMP     SFOUT
SF3:
    MOV     AL, 4Fh
SFOUT:
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    CMP     BYTE PTR [1014h], 01h
    JE      MU_R2
    CMP     BYTE PTR [1014h], 02h
    JE      MU_R3
    CMP     BYTE PTR [1014h], 03h
    JE      MD_R2
    CMP     BYTE PTR [1014h], 04h
    JE      MD_R3
    CMP     BYTE PTR [1014h], 05h
    JE      GI_R1B
    CMP     BYTE PTR [1014h], 07h
    JE      SV_TGT
    JMP     GI_R2B

; ================================================================
;  SHOW_MSG : home cursor, print 16-char ROM string at BX
;  RET_SHOW = id
; ================================================================
SHOW_MSG:
    MOV     AL, 80h                       ; DDRAM addr 0 (line 1 start)
    MOV     BYTE PTR [1010h], 05h
    JMP     LCD_CMD
SM_RET_HOME:
    PUSH    CX
    MOV     CX, 0400h
SM_SD:
    LOOP    SM_SD
    POP     CX
    JMP     PRINT_MSG
SM_RET_PRINT:
    CMP     BYTE PTR [1016h], 01h
    JE      GU_R1
    CMP     BYTE PTR [1016h], 02h
    JE      GD_R1
    CMP     BYTE PTR [1016h], 03h
    JE      SV_R1
    CMP     BYTE PTR [1016h], 04h
    JE      SV_R2
    CMP     BYTE PTR [1016h], 05h
    JE      GI_R1
    CMP     BYTE PTR [1016h], 06h
    JE      PWR_R1
    CMP     BYTE PTR [1016h], 07h
    JE      FALL_R1
    JMP     PWR_R2

; ---------------- PRINT_MSG : BX -> null-terminated ROM string ----
PRINT_MSG:
PM_LOOP:
    MOV     AL, CS:[BX]
    CMP     AL, 00h
    JE      PM_DONE
    JMP     LCD_DATA
PM_RET_DATA:
    PUSH    CX
    MOV     CX, 0400h
PM_SD:
    LOOP    PM_SD
    POP     CX
    INC     BX
    JMP     PM_LOOP
PM_DONE:
    JMP     SM_RET_PRINT

; ---------------- DELAY : proven ~6-7 s ; RET_DELAY=id ----------
DELAY:
    PUSH    BX
    PUSH    DX
    MOV     BX, 0002h
DLY_O:
    MOV     DX, 0FFFFh
DLY_I:
    DEC     DX
    JNZ     DLY_I
    DEC     BX
    JNZ     DLY_O
    POP     DX
    POP     BX
    CMP     BYTE PTR [1012h], 01h
    JE      D_R1
    CMP     BYTE PTR [1012h], 02h
    JE      D_R2
    CMP     BYTE PTR [1012h], 03h
    JE      D_R3
    CMP     BYTE PTR [1012h], 04h
    JE      D_R4
    JMP     D_R5

; ---------------- LCD_CMD : AL=cmd ; RET_CMD=id ----------------
LCD_CMD:
    MOV     DX, 0008h
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    MOV     AL, 04h
    MOV     DX, 000Ah
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    MOV     AL, 00h
    MOV     DX, 000Ah
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    CMP     BYTE PTR [1010h], 01h
    JE      INIT_R1
    CMP     BYTE PTR [1010h], 02h
    JE      INIT_R2
    CMP     BYTE PTR [1010h], 03h
    JE      INIT_R3
    CMP     BYTE PTR [1010h], 04h
    JE      INIT_R4
    JMP     SM_RET_HOME

; ---------------- LCD_DATA : AL=char ; single caller ----------
LCD_DATA:
    MOV     DX, 0008h
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    MOV     AL, 05h
    MOV     DX, 000Ah
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    MOV     AL, 01h
    MOV     DX, 000Ah
    PUSH    AX
    MOV     AX, [0500h]
    POP     AX
    OUT     DX, AL
    JMP     PM_RET_DATA

; ================================================================
;  ROM STRINGS (space-padded to 16 chars, null-terminated)
; ================================================================
STR_MUP:    DB 'MOVING UP       ', 0
STR_MDN:    DB 'MOVING DOWN     ', 0
STR_ARR:    DB 'ARRIVED         ', 0
STR_DOOR:   DB 'DOOR OPEN       ', 0
STR_IDLE:   DB 'IDLE            ', 0
STR_PWR:    DB 'POWER OUTAGE    ', 0
STR_FALL:   DB 'EMERGENCY STOP  ', 0

; ---------- Reset vector (physical FFF0h) ----------
ORG     0FFF0h
    DB      0EAh, 00h, 00h, 00h, 0F0h

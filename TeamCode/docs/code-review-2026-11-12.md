# Code review, 2026-11-12 (written overnight for the coach)

> **Note, 2026-11-14:** the `[CONFIRM]` markers this review talks about were taken out of the docs and code on 2026-11-14. Decisions are now GitHub Issues with a `[#nn]` marker, and small robot checks are `// TODO` comments in the code (#88 lists them). Where this review says `[CONFIRM]` below, it is describing how things were on 2026-11-12.

*You asked four things: (1) why the drive drops from field-centric to robot-centric and how to protect against it, (2) what is unnecessarily complex, (3) what questions to ask about code to make it human-maintainable, and (4) code suggestions on a separate branch. This is the written answer. The code is on the branch `claude/stoic-pasteur-ggp2jy-code-suggestions`. Nothing here has been run on a robot.*

> **Updates after your replies (same day), newest first:**
> - **Latest:** you said the hub is on the **left wall** (bird's-eye view), so the logo faces `RIGHT`: set (`6ab0134`). You also asked for the TeleOp to be renamed **"TeleOp - Alpha 14"**: done (`6a9938c`), and now in `main` (merged as `5c3e46d`) together with the changes that make it Alpha 14. All three Autos' `preselectTeleOp` were changed to match and a script checked the four names are identical. 43 laptop checks pass. I read "left" as the **robot's** left with the intake at the top of the picture; if you meant the viewer's left, the logo is `LEFT`.
> - **Hub:** you said the Control Hub stands on its end with its **USB ports at the ceiling** and its **logo facing the middle of the robot**. USB is `UP` in the code. The logo direction depends on **which wall** the hub is bolted to (left wall: `RIGHT`, right wall: `LEFT`, back: `FORWARD`, front: `BACKWARD`). *(Superseded: you then said left wall, so it is `RIGHT`.)*
> - **A trap this exposed (and fixed):** my first default (logo `UP`) is physically impossible with USB `UP`. The real SDK **throws** on that pair, which would have **crashed the TeleOp at INIT**. `ImuSubsystem` now catches it, logs it, shows why on the screen, and falls back to robot-centric. Laptop checks guard it, and a mutation test (the bad setting put back) turns one red.
> - **Pollen:** the manual (section 9.8) says POLLEN are about 2.8 in **Gopher ResisDent polyethylene** balls, so my static-electricity hypothesis is **stronger** than I first rated it. I was also wrong to say the manual did not give the material, and to say "nectar": your robot only takes in pollen.
> - **Merged into `main` on 2026-11-12:** the suggestions branch went in as `5c3e46d` and this document's corrections as `f547949`, so `main` now has both the code and the docs. (These notes were written before either merge.)

## 0. The 60-second version

1. **I would not assume "micro-brown-out" yet.** Your own troubleshooting table already says a fresh battery did not stop the drops. Nothing in the code recorded the battery voltage, so nobody can say whether it dipped. That is fixed on the branch (section 4).
2. **The drive was one bad sample away from giving up for good.** Any single Pinpoint blink, restart, garbled reading, or a long loop during a fast spin latched the drive to robot-centric until a driver pressed a button (section 1.2). Whatever the root cause, that design turns a one-loop event into a lasting failure.
3. **The fix that works whatever the cause: take the drive's heading from the Control Hub's built-in IMU, not the Pinpoint.** The IMU is inside the hub with no cable to wiggle. It also let me delete about 120 lines of "what if the Pinpoint blinked" logic from the TeleOp (about 70 lines were added back for the IMU wiring and logging). **It needs a 5-minute test (`ImuAlpha`) before it goes near a match,** because it has never run on the robot (the hub orientation is now set from your answers: USB `UP`, logo `RIGHT` for the left wall).
4. **One hub's power wire is already a suspect in your own notes** (the "Battery too low" message from Expansion Hub 2 while the Control Hub was fine). The new battery readout shows both hubs side by side, so a bad wire or crimp shows up as one hub always reading lower.
5. **`BiobuzzTeleOp.loop()` is 512 lines; the next longest method in the repo is 208.** Half of every file is comments. The same logic is copy-pasted between the TeleOp and two test OpModes. Section 2 has the full list, ranked.
6. **The "checked on a laptop (41 scenarios)" tests in your notes were never saved,** so nobody can re-run them. The branch adds a `laptop-checks/` folder that can be re-run in about 7 seconds.
7. **Twelve small commits on the suggestions branch; most can be taken or dropped on their own** (table in section 4). Several have simple on/off switches.

**What is verified and what is not**

| Verified | Not verified |
|---|---|
| All team code except the three Pedro files compiles against the **real FTC SDK 12.0.0 jars** (not stand-ins) | A Gradle build in Android Studio |
| 39 laptop checks pass (battery-dip detector 9, drive acceleration limit 23, IMU wrapper 7) | Anything on a robot: IMU orientation, how the acceleration limit feels, loop-time change, voltage numbers |
| I read how the SDK's Pinpoint driver decides "READY" by decompiling the 12.0.0 jar | The three Pedro-dependent files (`ShootAndParkAuto`, `LeaveAndParkAuto`, `PedroForwardBackAlpha`): I did not touch them and could not compile them here |
| | The whole `BiobuzzTeleOp.loop()` against stand-ins (earlier sessions did this; the harness was not saved, so I could not repeat it) |

---

## 1. The field-centric drop-outs

### 1.1 First, which kind of drop was it? (read this off the screen)

On `main` and on the session branch, the **DRIVE MODE** line says why. Each wording points to a different cause:

| DRIVE MODE line says | What actually happened | What it points to |
|---|---|---|
| `ROBOT-CENTRIC (BACK on either gamepad switches to FIELD-CENTRIC)`, **no "because"** | Someone pressed **BACK**. It is the toggle on **both** gamepads. | A button press, not hardware. Easy to brush on a Logitech pad. |
| `...because the Pinpoint was not READY for more than 0.25 seconds` | The Pinpoint stopped reporting READY for over a quarter second. | Read **Last drop**: `FAULT_X/Y_POD...` = a pod cable, `FAULT_BAD_READ` = the I2C cable, `NOT_READY > CALIBRATING` = it restarted (power or reset). |
| `...came back from a glitch with its heading N degrees off` | A short gap, and then the heading was more than 20 degrees different. | A restart, **or a fast spin during the gap** (the code's own comment admits this). |
| `...the Pinpoint heading jumped N degrees` | The heading moved more than 45 degrees between two loops. | A restart, a garbled reading, **or one long loop while the robot was spinning** (see 1.2). Check "Loop time worst". |

**If you only do one thing this week:** the next time it happens, write down which line it was and what **Last drop** says. That narrows the cause more than any theory.

### 1.2 What I found

**In your code (`BiobuzzTeleOp.checkPinpointHealth()` on the session branch):**

- **The latch is permanent and trips on one sample.** Three different events set `fieldCentricBlocked = true`, and only a button press clears it. A single bad loop is enough.
- **The 45-degree jump test ignores how long the loop took.** It is written as "45 degrees in 20 ms is impossible", but it compares two *loops*, not two *moments*. A robot can spin a few hundred degrees per second (ours is not measured yet, see #88: spin at full stick and read the Pinpoint **Turn deg/s** line), so a single 100-150 ms loop (the Limelight, a garbage-collection pause, a telemetry burst) can be a perfectly real 45-degree turn. Nothing shows the worst loop to the drive logic, only to the screen.
- **The 20-degree "resume" check is tighter than reality.** A 0.25 s gap at a few hundred degrees per second is 50 to 100 degrees of real turning. The code comment already says a fast spin "can trip this too".
- **No bulk reads, so loops were longer than they needed to be.** `grep LynxModule` finds nothing in team code. The TeleOp makes four separate trips to the hub every loop (intake, flywheel, hood wheel, turret).
- **Nothing recorded the battery.** Your troubleshooting table says it directly: "the TeleOp no longer shows hub voltages." Nothing wrote anything to a log either, so a blink that lasts a moment is gone before anyone reads the screen.

**In the SDK driver (decompiled `GoBildaPinpointDriver`, FTC SDK 12.0.0; I found no documentation, so treat the thresholds as read from the code):**

- The driver sets `FAULT_BAD_READ` only when a reading is **not a number or jumps by an enormous amount** (the driver's constants are 5000, 120, 10000 and 120, in its own units; I found no documentation for them, so I would not lean on the exact numbers). Its default error check is `LOCAL_TEST`. A garbled reading that is still a plausible number goes straight through to your code.
- The driver has a CRC mode (`ErrorDetectionType.CRC`) that checks its own reads, but only on one of its two read paths (the one for newer Pinpoint firmware), and I found **no public way to switch it on** from our code. Worth asking goBILDA whether your Pinpoint's firmware is current. I could not verify which firmware version enables it.
- On the driver's standard read path, the status is the first 4 bytes of the same 40-byte I2C transfer as the position, so I found no separate "the read failed" signal.

**In your notes:**

- Alpha 11's note: `DriveAlpha` **still dropped with a fresh battery**, "so a weak battery is not the whole story". I agree, and the troubleshooting table (row 53) already tells you to check cables first.
- Issue #54 has no comments, so no drop-log result has been recorded anywhere yet.
- Row 61 of the troubleshooting table: "Battery too low to run motors and servos" from **Expansion Hub 2** while the Control Hub was fine, in the 10-26 robot log. That is a local wiring problem (one hub's power path), and the old fix was "check that hub's power wire and connector". That is real evidence, and it is the kind of thing a battery readout per hub will show immediately.

### 1.3 My honest pushback on "micro-brown-out"

A brown-out fits some of what you saw (a weak battery made the Pinpoint blink in test 1; the Limelight restarts "after a bump or a brownout"). It does **not** yet fit others (a fresh battery still dropped; no voltage was ever recorded). Treat these as **hypotheses to test, not findings**:

| Hypothesis | Why plausible | How the branch / a practice session tests it |
|---|---|---|
| A loose or intermittent connector (Pinpoint I2C or a pod cable; a hub power wire) | Vibration and flywheel shock; row 61; `FAULT_` statuses would show | **Last drop** story and the log; wiggle test (1.5) |
| Static electricity (a flywheel throwing balls is a classic generator) | Random, brief, no pattern with battery | Blinks that happen right after shots, with normal voltage. *The manual (9.8) says POLLEN are 2.8 in **polyethylene** balls, a plastic that builds static easily, thrown by a flywheel at over a thousand RPM: a stronger suspect than I first rated it.* |
| Real voltage sag from motors starting together (4 drive motors, flywheel spin-up, Limelight on the hub's USB) | Row 62; and your tall robot is also at risk of tipping | **POWER** lines and `BATTERY DIP` log lines; hub-versus-hub readings |
| Software false alarm (a long loop during a spin trips the 45-degree test) | Section 1.2 | **Loop time worst** on the screen next to the drop; `ImuAlpha`'s disagreement log |
| An accidental BACK press | BACK is on both gamepads | The "no because" wording in 1.1 |

### 1.4 What I changed (all on the suggestions branch; section 4 has the details)

| Change | What it does for this problem |
|---|---|
| **`ImuSubsystem` + the TeleOp's drive uses it** | The drive no longer depends on the Pinpoint at all, so **no Pinpoint blink, restart, spike, or loop stall can drop it out of field-centric**, whatever the cause. A missing IMU falls back to robot-centric, with a clear message. The Pinpoint still feeds the turret's hive memory, and a blink there only costs the turret its memory until the camera sees the hive again. |
| **`PowerMonitorSubsystem` + `BiobuzzLog`** | Puts the hub voltages back on the screen (both hubs), counts dips, and writes every dip, Pinpoint drop and glitch to the Robot Controller log with the battery state. After a match: `adb logcat -d \| grep BIOBUZZ`. |
| **`ImuAlpha`** | A 6-step orientation test **and** a "witness": it compares the Pinpoint's heading to the IMU's and logs every moment one moved and the other did not. Leave it running on the robot in practice: it tells you which sensor blinked, when, and what the battery was doing. |
| **Drive acceleration limit** | Four motors told to go from stopped to full in one instant pull a burst of current together. Each wheel's power now rises over 0.2 s; stopping is never limited. A cheap way to reduce sag, and to reduce tipping. |
| **Bulk reads (AUTO)** | Should shorten the loop (measure it, do not assume). Use the existing **Loop time** line, and set `USE_BULK_READS = false` to compare. |

### 1.5 Hardware checklist (cheap, by inspection; the code cannot fix these)

*Before rewiring anything, check the wire-gauge table in the Competition Manual (search "AWG") and the electrical rules.*

- **Both hubs' power wires and connectors.** Push each in; look for a crimp that has pulled out; look for a green or dull contact. Compare the two hubs' voltages on the new **POWER** lines: they should be within about a tenth of a volt of each other. A hub that always reads lower is your answer.
- **The main power switch and the battery connector.** These are the most common resistance points on FTC robots, and they sag under load.
- **Battery:** fresh, charged, and the same one for the whole test. Note its resting voltage (your checklist already says to). A battery near the end of its life reads fine at rest and sags hard under load.
- **The Pinpoint's I2C cable and both pod cables.** Push each in at **both ends**, look for a cable that rubs, is pinched, or is under strain, and add strain relief with a zip tie near the connector. Your troubleshooting row already says this. Route I2C cables away from motor wires.
- **The Limelight's USB cable and where its power comes from.** It draws noticeable current from the Control Hub. If you suspect it, run a practice session with it unplugged and compare the **POWER** lines (the vision code reports "camera not connected" instead of crashing). Check the Competition Manual's rules before changing how it is powered.
- **Load balance.** Check which motors are on which hub. All four drive motors together on one hub is the worst case for that hub's voltage.
- **Static:** if blinks cluster right after shots, keep the Pinpoint, the hubs, and their cables away from the flywheel's air stream, keep metal parts that touch the balls grounded to the frame (this is common FTC advice and I have not tested it on our robot), and ask other teams what they use.

### 1.6 A practice-session protocol (30 minutes)

1. Deploy the suggestions branch. Run **IMU - Alpha** and do its six steps. (Fix `LOGO_FACING_DIRECTION` / `USB_FACING_DIRECTION` if step 2 or 3 fails.)
2. Leave **IMU - Alpha** running. Drive or push the robot around for 10 minutes, bump walls, spin fast.
3. Run the TeleOp for 10 minutes with real shooting. Watch **POWER** (both hubs, lowest voltage, dips), **Loop time worst**, **Pinpoint glitches**.
4. Read the log: `adb logcat -d | grep BIOBUZZ` (or download the Robot Controller log from the Control Hub).
5. Decide:

| What the log shows | Most likely | Do |
|---|---|---|
| `BATTERY DIP` lines, and a Pinpoint drop in the same second | Real voltage sag | Hardware checklist 1.5 first (battery, connectors, one hub lower). If it is the load: lower `MAX_POWER_CHANGE_PER_SECOND`, stage the flywheel spin-up away from hard driving. |
| Pinpoint drops with `FAULT_X/Y_POD` or `FAULT_BAD_READ`, and **no** dip | A cable or connector | Reseat and add strain relief; check cable routing; check the Pinpoint's firmware. |
| `NOT_READY > CALIBRATING` and a big heading move, no dip | The Pinpoint restarted, but the battery was fine: reset or static | Look at static; look at the Pinpoint's power wiring. |
| `HEADINGS DISAGREED` with the Pinpoint status READY the whole time | A garbled reading passed the driver's weak check | Firmware update; consider reading only the heading (it is a shorter transfer). |
| Nothing at all, but you saw ROBOT-CENTRIC | A BACK press | Move or remove BACK on one pad. |

### 1.7 What I would **not** do

- **Don't raise `PINPOINT_GLITCH_TOLERANCE_SECONDS` or the 20 and 45 degree limits to "make it stop".** That hides the symptom and makes a real restart slower to notice.
- **Don't hold the last heading forever through a long gap.** It drives the wrong way if the Pinpoint restarted. (The IMU avoids this entirely.)
- **Don't add more latches.** Section 3, question 14.

---

## 2. Is anything unnecessarily complex?

Measured on the session branch (`TeamCode/src`, Pedro's own tuner code excluded): **4,974 lines of code and 4,622 lines of comments** (48% comments), 310 methods, 12 of them over 60 lines.

### 2.1 Findings, ranked by how much they cost you

| # | Finding | Evidence | Verdict | Action | Done on the branch? |
|---|---|---|---|---|---|
| 1 | **`BiobuzzTeleOp.loop()` is 512 lines** | Next longest method: 208. Inside it: the shot-target logic (~70 lines), the "is the turret ready" logic (~30), an 8-way gate `else if` chain (~55), the intake (~35), and **~150 lines of telemetry**. | **Simplify.** It breaks your own "one idea per method" rule. | Split into the methods in 2.2, **one OpMode per session**, with the laptop checks around it. | **No** (too risky to do blind; plan in 2.2) |
| 2 | **Field-centric safety machinery in the TeleOp** (`checkPinpointHealth`, the glitch hold, the resume check, `fieldCentricBlocked`, the reason string, the toggle's resume path) | About 120 lines, all there because the drive used the Pinpoint's heading | **Unnecessary once the drive uses the IMU.** | Delete for the drive; keep a small version for the turret's memory | **Yes** (commit `5061fa2`) |
| 3 | **The TeleOp's header was ~300 lines, half of it release notes** ("NEW IN ALPHA 13 ... 8") | 151 lines of history above the button map | **Simplify.** History belongs in a change log. | Moved word for word to `biobuzz-teleop-change-log.md` | **Yes** (`51ccbaa`) |
| 4 | **Copy-pasted logic across files** | `BiobuzzTeleOp` and `FiringSequenceAlpha` share the volley logic (10 overlapping 8-line blocks); `BiobuzzTeleOp` and `DriveAlpha` share the Pinpoint health check (7); `LeaveAndParkAuto` and `ShootAndParkAuto` (12); `ShooterRPMAlpha` and `ShooterRatioAlpha` (8) | **A bug fixed in one copy stays in the other.** Your rule says "don't copy a number into two files"; logic is the same problem. | Retire test OpModes the TeleOp has replaced (`FiringSequenceAlpha`, `DriveAlpha`) once the TeleOp's version is confirmed; share the Autos' common park code | No (needs your decision) |
| 5 | **Two "ready to shoot" checks** | `ShooterSubsystem.isAtSpeed()` (both wheels) and `isFlywheelAtSpeed()` (flywheel only). The TeleOp uses the second; `ShootAndParkAuto`, `ShotTableAlpha` and `ShooterRatioAlpha` still use the first; the TeleOp still shows both on screen. | **Two answers to one question** is how "READY flickers" bugs happen. | Decide issue #62 (is flywheel-only enough?), then delete the other path and the "OK to shoot?" line | No (**depends on an open decision**: #62) |
| 6 | **A method documented as a trap is still called** | `TurretSubsystem.isOnTarget()`: your notes say "do not gate anything on it while auto-align is running", yet `ShootAndParkAuto` lines 352 and 496 do | **A known hazard left in place.** Your notes say it can stall the Auto and skip the shot. | Switch the Auto to the camera's `tx` check the TeleOp uses (issue #63, session 3), or rename it so the trap is in the name | No (an open decision, now #89) |
| 7 | **The same button, two gamepads, different meanings** | START = alliance on both pads. BACK = drive toggle on both pads. **LOGO = SOLO MODE on gamepad 1 but DIAL/TABLE on gamepad 2.** | **Hard for drivers under pressure, and easy to confuse.** The ambiguous-BACK case is also one of the five ways field-centric turns off (1.1). | Put each function on **one** gamepad. | No (needs your decision; it changes the controls sheet) |
| 8 | **Features built for problems not yet seen on a robot** | Hive-memory heading follow (`HiveBearingMemory`, stale rules, feed-forward, out-of-range hold) and the two-cell lock (`LOCK_RELEASE_SECONDS`, `CELL_SWITCH_MARGIN_DEGREES`) were "checked on a laptop, not on the robot". The memory follow already snapped the turret around once (Alpha 10). | **Unverified complexity.** It may be right, but it is the part most likely to surprise you at an event. | Keep, but add one on/off constant for each so a bad robot test can disable it in one line | No |
| 9 | **Gate control has three sources and eight branches** | Sequence, manual X/B, Y-lift (operator or SOLO driver), release handling. Plus an intake rule that reverses on its own while the gate closes. | **Essential behavior, tangled expression.** | Move the arbitration into one method that returns what the gate should do and why (`decideGateMove()`) | No (part of #1) |
| 10 | **`ARCHITECTURE.md` is 102 KB, with single bullets of 6,803, 4,148 and 3,250 characters** | Those bullets are release notes written as one paragraph. Over 30 `[CONFIRM]` markers. | **A student cannot find the rule.** It is the "source of truth" your protocol tells every session to read first. | Split each subsystem's "what it is" (5-8 lines, current) from its "what changed" (the change log). Give every `[CONFIRM]` an owner and a date. | Partly (marked the superseded paragraph) |
| 11 | **Telemetry duplicates state the code already knows** | About 150 lines at the end of `loop()` re-describe values computed above, with `String.format` and per-case wording | **Useful, but it grows with every feature.** | Each subsystem already has `addTelemetry()`; move the TeleOp's own lines into `showXxxTelemetry()` methods | No (part of #1) |
| 12 | **No bulk reads** | Section 1.2 | **A missing optimization, not complexity**, but it is the cheapest speed-up | AUTO bulk reads | **Yes** (`488ee97`) |

**Things that look complex but earn their place:** `PinpointSubsystem`'s drop log (it is the right instrument, it just did not write anywhere); `VisionSubsystem`'s pipeline recovery (it came from a real Limelight restart); the firing sequence's pause-when-not-aimed (a real review request); the shot-freeze during a volley (it fixes a real "READY flickers" problem). The *behavior* is justified. The *size of the places it is written* is the problem.

### 2.2 A plan for `loop()` (for a future session; I did not do it blind)

Keep your four-step order (read, decide, command, telemetry). Make each step a short method, and turn the ~25 locals that cross sections into fields named for what they mean:

```java
@Override
public void loop() {
    timeTheLoop();                 // step 0
    readSensors();                 // step 1: intake, vision, pinpoint, imu, power, health check
    readGamepads();                // step 2: sets fields: driveForward, driveStrafe, driveTurn, fireHeld ...
    chooseShotTarget();            // dial or table, dead band, freeze during a volley (about 70 lines now)
    driveTheRobot();               // field- or robot-centric
    runShooterWheels();
    aimTheTurret();
    runFiringSequence();           // turretReady, readyToStart, firingSequence.update(...)
    moveTheGate();                 // decideGateMove(): one method that returns the move and the reason
    runTheIntake();
    showTelemetry();               // showDriveTelemetry(), showShooterTelemetry(), ...
}
```

Rules for the session that does it: **one OpMode only** (your protocol); a **laptop check first** for the volley and the gate (the harness is in `main`, in `TeamCode/laptop-checks/`); extract **without changing behavior** and diff the telemetry before and after; and do the gate method last, because it is the one with eight branches.

---

## 3. Questions to ask about code to make it human-maintainable and readable

The goal is "a student who did not write this can change it safely". Ask these in a code review, out loud. Each group has the question, then why it matters. After the lists, I score this repo honestly.

### A. Can I read it cold?
1. **Can someone who did not write this class say what it is *for* in one sentence from its header?** (If the header needs a scroll, it is a change log, not a header.)
2. **If I want to change a number, which one line do I edit, and is it the only place?** (Constants at the top; no copy in a second file.)
3. **Does each name say what it means in the game, not how it works?** (`isClearingChute()` yes; `wasStartPressed` next to `wasDriverStartPressed`, no.)
4. **Does each method do one thing, and does the name say which?** A method over ~40 lines is a method with two jobs.
5. **Could I trace one button from the gamepad to the motor in under a minute?** Count the hops.

### B. Can I change it safely?
6. **If I change this, what else is affected?** Can I find out with a search, or do I have to remember?
7. **Is each decision made in exactly one place?** ("Who controls the gate right now?" "Is the shooter ready?") Two answers to one question will eventually disagree.
8. **Can I run it without the robot, and was the last check saved so I can run it again?** A test nobody can re-run is a rumor.
9. **When I change X, which document must change, and is that written down?** Docs drift when nothing says they must follow.
10. **Can I undo my change in one step?** (Small commits; a named on/off constant such as `USE_BULK_READS`.)

### C. What happens when it breaks?
11. **For each thing that can fail (sensor gone, cable out, camera lost), what does the code do, and does the screen say so in words?**
12. **Does it write down *why* it did something surprising, or only show the latest state?** A screen is erased every loop; a log line is not.
13. **What would I look at first at 2 a.m. in the pit, and is it at the top of the screen?**
14. **Can one blip make a permanent decision?** A decision that cannot undo itself needs more evidence than one sample. *(This is exactly what the field-centric latch did.)* Prefer "wait until it persists for N loops" to "latch on the first".
15. **Does a "safety" rule have a second way to be wrong?** A jump limit that ignores loop time is a guess about time that the code never measured.

### D. Is it the right size and shape?
16. **Is any file or method bigger than a student can hold in their head?** (Rough limits: ~40 lines per method, ~400 per file.)
17. **Are comments saying *why* (keep) or narrating history ("NEW IN ALPHA 13") or restating the line (delete)?** History goes in a change log. Pick three comments at random: are they still true?
18. **Is any logic copy-pasted between files?** What happens when you fix a bug in only one copy?

### E. Is it worth its weight?
19. **Have we seen the problem this feature solves on the real robot?** A feature that was only checked on a laptop is a bet. Say so in the code.
20. **What is the simplest version that would work, and what exactly would we lose?**
21. **If it broke the day before competition, could I turn it off in one line?** If not, add the switch before you need it.

### F. For the team
22. **Can each student name the one subsystem they own and explain its states out loud?** Your mentor checklist's last item ("a student could explain every line") is the real test.
23. **Does every `[CONFIRM]` have an owner and a date?** There are over 30 in `ARCHITECTURE.md`. Unowned ones become permanent.
24. **Did the person who reviewed it change something, or only say "looks good"?**

### How this repo scores (honestly)

| Strong | Weak |
|---|---|
| Constants at the top with a comment saying how to change them (Q2). Names mean something in the game (Q3). One subsystem per mechanism, plain classes (Q7, mostly). `[CONFIRM]` markers that admit what is unknown (Q19). Comments explain *why* (Q17, in the good cases). A strict, written protocol (Q9). | A 512-line `loop()` (Q4, Q16). Release notes inside headers and doc bullets (Q17, finding 10). Copy-pasted logic (Q18). Two ready checks (Q7). A permanent latch from one sample (Q14, Q15). Tests that were run but not saved (Q8; fixed on the branch). Voltage and blinks not recorded (Q12). Several `[CONFIRM]`s with no owner (Q23). |

### A 10-minute review card for students (print this)

1. Read the class header out loud. Does it say what the class is *for*?
2. Pick the biggest method. Count its lines. Over 40? Say what its two jobs are.
3. Find a number. Is it a named constant, in one place, with a comment saying how to change it?
4. Find one `if` that decides something permanently. What if it was wrong once?
5. Find one thing that can fail. What does the screen say when it does?
6. Pick three comments. Are they still true? Do they say *why*?
7. Search for a piece of logic you suspect is copied. Is it?
8. Say how you would test this without the robot, and how you would turn it off in one line.

---

## 4. The code-suggestions branch

Branch: `claude/stoic-pasteur-ggp2jy-code-suggestions` (12 commits on top of the session branch). **It was merged into `main` on 2026-11-12 as `5c3e46d`, a merge commit, so `git revert -m 1 5c3e46d` undoes all of it.** I did not open a pull request: the merge was done directly, on your instruction. The table below describes the commits as they were when it was written; their hashes are still valid in `main`'s history.

| Commit | What | Risk | Depends on | On/off | How to test on the robot |
|---|---|---|---|---|---|
| `24d71d6` | `PowerMonitorSubsystem` (hub voltage, current, dip count, dip log) + `BiobuzzLog`; Pinpoint drops and glitches also go to the log; `blockFieldCentric()` helper | **None** (read only) | none | `READ_MOTOR_CURRENT` | The **POWER** block shows two hubs at about 13 V in INIT. Check **Loop time** did not get much longer; if it did, set `SAMPLE_INTERVAL_SECONDS` to 0.2 or `READ_MOTOR_CURRENT` to false. |
| `488ee97` | AUTO bulk reads | **Low** | none | `USE_BULK_READS` | Compare the **Loop time** average with it true and false. |
| `dbd2f45` | Drive acceleration limit (0 to full in 0.2 s) and `getTurnPower()` from the real wheel powers | **Low-medium: it changes how the robot feels** | none | `LIMIT_ACCELERATION`, `MAX_POWER_CHANGE_PER_SECOND` | A driver tries it. **`EmergencyLeaveAuto`'s timed drive loses ~4% distance** (about 0.03 s of full power); re-measure it. |
| `51ccbaa` | Move the TeleOp's release notes into `biobuzz-teleop-change-log.md` | **None** (comments only, 2,177 words moved, 2,177 counted back) | none | n/a | n/a |
| `5061fa2` | **`ImuSubsystem` + `ImuAlpha`; the drive's heading comes from the IMU; the Pinpoint safety logic for the drive is deleted** | **Medium until `ImuAlpha` passes** | `24d71d6`, `51ccbaa` | none (`git revert`, or press BACK for robot-centric) | **Run `IMU - Alpha` first.** Then the TeleOp: a blink of the Pinpoint cable (unplug and replug it) must **not** drop the drive. |
| `f021e22` | `ARCHITECTURE.md` and change-log updates | None | all above | n/a | n/a |
| `a6780b4` | `TeamCode/laptop-checks/`: a script and 39 re-runnable checks | None (Gradle ignores the folder) | none | n/a | `cd TeamCode/laptop-checks && ./run-laptop-checks.sh` |
| `ab34dea` | Troubleshooting table and match-day checklist match the IMU drive and the POWER lines | None (docs) | `5061fa2` | n/a | n/a |
| `861e0c0` | `laptop-checks`: the download retries and says what to do on a rate limit (HTTP 429) | None | `a6780b4` | n/a | n/a |
| `50f877b` | `ImuSubsystem`: USB `UP` (your answer); logo `FORWARD` (guess, since replaced); a bad orientation pair is caught instead of crashing INIT; 3 new laptop checks | Low (it only affects the IMU) | `5061fa2` | n/a | `ImuAlpha` steps 2 and 3 |
| `6ab0134` | `ImuSubsystem`: the hub is on the **left wall**, so the logo is `RIGHT`; a laptop check for it (43 checks now) | Low | `50f877b` | n/a | `ImuAlpha` steps 2 and 3 |
| `6a9938c` | **Rename to "TeleOp - Alpha 14"** (OpMode name, the 3 Autos' `preselectTeleOp`, comments, live docs; all history left as written; the change log's newest section is now Alpha 14) | Low (a string), **but a mismatch would stop the Driver Station loading the TeleOp after Auto**: a script checked all four names match | none | n/a | Run an Auto and confirm the Driver Station offers "TeleOp - Alpha 14" |

*Two commit messages quote check counts that are slightly off from memory (the drive commit says 21 and the laptop-checks commit says 22 and 8). The script reports 39 passing: 9 + 23 + 7.*

**Things I changed beyond the commits above:** none on the session branch except this document.

### Not done on purpose (and why)

- **No split of `loop()`** (finding 1): 512 lines of behavior that no saved test covers. Plan in 2.2.
- **Nothing that depends on an open decision:** the one-ready-check decision (#62), the Auto's switch away from `isOnTarget()` (#63 session 3), the gate positions, hood wheel behavior, the flavors and `main` versus `master`. Your protocol says to stop and ask. They are in section 5.
- ~~No rename to "TeleOp - Alpha 14".~~ **Done at your request** (`6a9938c`), and now in `main` (merged as `5c3e46d`): the TeleOp, the three Autos' `preselectTeleOp` and the live docs all say "TeleOp - Alpha 14". (Before that merge, the review branch still said Alpha 13 because it did not have the changes that make it Alpha 14.)
- **`DriveAlpha` unchanged.** It still has the old Pinpoint-heading drive. Retire it or point it at `ImuSubsystem` after the TeleOp's IMU drive is confirmed.
- **Did not touch** `FtcRobotController/`, the Gradle files, or the Pedro files.
- **Did not add a dependency.** The laptop checks use only the FTC jars you already depend on, downloaded on a laptop and not into the build.

---

## 5. Decisions I made for you, and what I need from you

### Best-case defaults I used (change any of them)

| Decision | My default | Why |
|---|---|---|
| Hub orientation for the IMU | **USB UP** and **logo RIGHT** (from you: the hub is on the left wall, logo facing the middle) | Now set from your answers. It still needs `ImuAlpha` steps 2 and 3 on the robot, and "left" is read as the robot's own left. |
| Which gyro the drive uses | The Control Hub IMU; the Pinpoint stays for the turret | Removes the failure for the drive whatever the cause |
| Dip threshold | 10.0 V | A guess. Set it just below the lowest voltage a **normal** match shows. |
| Battery sample rate | 10 times a second, current included | Cheap; a dip shorter than 0.1 s can be missed, so try 0.05 if a blink happens with no dip logged |
| Acceleration limit | 5.0 per second (0.2 s to full) | Gentle enough to be unnoticed by most drivers; raise it if it feels heavy |
| Bulk reads | AUTO, not MANUAL | AUTO cannot hand back an old number. MANUAL is a few milliseconds faster but needs a clear call in every OpMode. |
| Two branches | Review on the session branch; code on a new branch off the same commit | So you can take the code without the docs, or the reverse |
| Commit trailer | `Co-Authored-By: Claude <noreply@anthropic.com>` with no model name | Keeps a model name out of repo history |
| Protocol | I broke "one subsystem class (plus its test OpMode) per session" (three new classes) | You asked for suggestions on a separate branch; each commit is small and mostly independent |

### Questions only you can answer

1. ~~Which wall is the Control Hub on?~~ **Answered:** the left wall, USB ports up, logo facing the middle, so the logo is `RIGHT`. `ImuAlpha` steps 2 and 3 will confirm it on the robot. Two readings I made, which `ImuAlpha` also checks: "ports" means the **USB ports** (if you meant the motor and sensor ports, the USB direction changes), and "left wall" means the **robot's** left (if the viewer's left, the logo is `LEFT`).
2. **When it dropped, what did the DRIVE MODE line say, and what did "Last drop" say?** (1.1) Does it happen only after shots, only when spinning, or after hitting something?
3. ~~What are the balls made of?~~ **Answered:** the robot only takes in **pollen**, which the manual says is polyethylene (see the update at the top).
4. **Is the Pinpoint's firmware current?** (See goBILDA's page.)
5. **Which gamepad should own START, BACK and LOGO?** (finding 7)
6. **Is flywheel-only "close enough" (#62)?** If yes, I can delete the second ready check next session.
7. **Do you want `FiringSequenceAlpha` and `DriveAlpha` retired** once the TeleOp covers them? (finding 4)
8. **Should hive memory and the cell lock get on/off switches?** (finding 8)

### Docs worth updating when you merge

- `biobuzz-troubleshooting.md` row 53 ("Drive keeps dropping from FIELD to ROBOT centric") and row 61 ("the TeleOp no longer shows hub voltages") described the old behavior before the merge. **They are updated in `main`** (`ab34dea`, merged as part of `5c3e46d`) to match the IMU drive and the POWER lines.
- `biobuzz-match-day-checklists.md`: the after-match line "Note the lowest hub voltages the screen showed" only works once the POWER lines exist. `ab34dea` extends it to the dip count and the log.

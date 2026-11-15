# BIOBUZZ OpMode Controls: Every TeleOp and Test OpMode

Print this and keep it at the driver station. It lists **what every button does in every OpMode**, using the Logitech controller names. The full TeleOp sheets (driver page and operator page) are in `biobuzz-teleop-controls.md`.

**Controller:** Logitech F310 / F710, switch on the back set to **X** (XInput), not D.
- Face buttons: **A, B, X, Y**. Bumpers: **LB / RB**. Triggers: **LT / RT**. Stick buttons: **LS / RS** (press the stick in).
- **Start** and **Back** are in the middle. (The old PS5 "Share" button is **Back**.)
- **Pairing:** **Start + A** pairs gamepad 1. **Start + B** pairs gamepad 2. Pairing while an OpMode is running also presses Start and A or B in that OpMode, so check the alliance and any saved rows afterward. (In the TeleOp, **Start on either gamepad** switches the alliance, and gamepad 2's B starts a FORCED SHOT (wheels, gate and intake), so pair before START.)

**Names on the Driver Station:** `TeleOp - Alpha 15`, `Pinpoint - Alpha`, `Firing Sequence - Alpha`, `Drive Alpha` (shown for now, for the field-centric test), `Flower Picker Testing - Alpha` (shown for now). **Hidden** (`@Disabled`, put `//` in front of that line and redeploy to bring them back): `Shooter RPM - Alpha 2`, `Shooter Ratio - Alpha`, `Shot Table - Alpha`, `Intake Testing - Alpha`, `Shooter Power - Alpha`, `Turret Testing`, `Vision - Alpha`. Their controls are still listed below.

**The same button does different jobs in different OpModes.** Check the OpMode name before you press anything.

---

## 1. TeleOp - Alpha 15 (the real TeleOp)

Two controllers. The turret **tracks the hive all the time** (ALWAYS-TRACK, always ON, no switch). The drive starts **FIELD-centric**: the left stick's "forward" is **away from the driver** (the screen shows **DRIVE MODE** under the alliance). Details are on the printable sheet, `biobuzz-teleop-controls.md`.

### Gamepad 1: DRIVER

| Control | What it does |
|---|---|
| **Left Stick** | Drive forward / backward and strafe left / right |
| **Right Stick** | Turn |
| **LT** (hold) | **Slow mode** |
| **RB** | **Intake** (pull balls in). **The intake is the driver's only.** It does nothing while the gate is closing (the intake reverses by itself then, see below) |
| **LB** | **Reverse the intake** (push balls out). Beats RB and a volley's feed. **The intake also reverses BY ITSELF for as long as the gate takes to close** (0.5 s, the same from OPEN or from LIFTED), after every volley, abort and operator X. The screen's intake line says "gate closing: REVERSE (automatic)" |
| **RT** (hold) | **Does not fire.** The operator's RT fires. **Only in SOLO MODE** (LOGO) does the driver's RT fire a volley too |
| **LOGO** (the center **guide** button) | **SOLO MODE** on / off, for a one-person match (starts OFF) |
| **Y** (HOLD) | **Only in SOLO MODE:** gate **LIFTED**, the same as the operator's Y (pushes the last ball in). Does nothing with SOLO MODE off. Driver X does nothing |
| **B** (HOLD) | **Only in SOLO MODE:** the **FORCED SHOT**, the same as the operator's B (issue #80). Does nothing with SOLO MODE off |
| **START** | **Alliance** BLUE / RED, the same as the operator's Start (starts **BLUE**, check it on the screen) |
| **BACK** | Drive **FIELD-centric / ROBOT-centric** toggle, the same as the operator's BACK |
| **D-pad Up** | **PRE-REV** on / off: the wheels spin without firing (starts OFF) |
| **A / B / X / Y** | Not used (the gate moved to the operator's X / B / Y) |
| **D-pad Left** | **Re-aim** (field-centric): the way the robot faces right now is "away from me". Does not touch the Pinpoint |

### Gamepad 2: OPERATOR

| Control | What it does |
|---|---|
| **X** | Gate **CLOSED** (holds the ball back). **Between volleys only** |
| **B** (HOLD) | **FORCED SHOT**: the wheels spin, the gate **OPENS** and the intake runs forward **right now**, without waiting for flywheel speed or turret aim. Works with RT held or not. Let go and the gate **CLOSES** (with the auto-reverse). If RT is still held, the volley starts over at "waiting". Y still lifts. For when the camera or the READY check is wrong |
| **Y** (HOLD) | Gate **LIFTED**, to push the very last ball into the flywheel (OPENED is between LIFTED and CLOSED). **While a volley feeds:** hold to lift, let go and the gate goes back to OPENED. **Between volleys:** let go and the gate closes (with the auto-reverse). Ignored while a volley waits. The firing sequence never lifts by itself. If X and Y are pressed at once, CLOSED wins. X does nothing during a volley or a forced shot |
| **A** (hold) | **Float** the turret so you can turn it by hand. Let go and it tracks the hive again |
| **D-pad Up / Down** | Flywheel target RPM **+ / - 100** (range 500 to 1450) |
| **D-pad Right / Left** | Hood / flywheel **ratio + / - 0.05** (range 1.00 to 3.00, starts 2.00) |
| **RT** (hold) | **FIRE a volley** at the dialed or table RPMs: it waits until the flywheel is at speed and the camera says the turret is aimed (within 2 degrees for 0.15 s), then opens the gate and **keeps feeding for as long as you hold** (no timer: the intake pauses by itself when the flywheel is not at speed or the turret is not aimed). **Let go to END it** (gate closes, intake stops). No one-volley limit. Unless PRE-REV is on, both wheels go to 0 power |
| **Start** | **Alliance** BLUE / RED (starts **BLUE**, check it on the screen). The driver's Start does the same |
| **Back** | Drive **FIELD-centric / ROBOT-centric** (starts FIELD). The driver's **BACK** does the same. (The physical **MODE** button cannot be read by the code: see the controls sheet. The INIT screen has a **BUTTON TEST** to check your controller.) |
| **LOGO** (the center **guide** button) | Flywheel speed source **TABLE / DIAL** (starts **TABLE**, the camera). (On the driver's controller the same button is the SOLO MODE toggle.) |
| **RB** | **Flower picker UP** (issue #75): lifts the flower bar. One move per press. If both bumpers go down on the same loop, UP wins |
| **LB** | **Flower picker DOWN**: lowers the flower bar, ready to pick a flower. One move per press. Works at any time, also during a volley |
| **Sticks, LT** | Not used. The turret has no manual control, the intake is the driver's, and the shooter wheels cannot be reversed from this OpMode |

**Starts at:** flywheel 1300 RPM, ratio 2.00, so the hood wheel is 2600 RPM. The screen shows loop time, the alliance, **PRE-REV**, the firing sequence step and what it is **Waiting for**, and **OK to shoot?** (both wheels, for information: the volley itself checks the flywheel only).

---

## 2. Shooter RPM - Alpha 2 (two controllers)

Speed-control version of the shooter test. **Hold** a button, the wheels run, let go and they coast. New in Alpha 2: the gate has **three positions**, and the intake runs from gamepad 2.

**Gamepad 1: shooter and gate**

| Control | What it does |
|---|---|
| **Y** (hold) | Flywheel **1400** RPM, hood wheel **2800** RPM |
| **X** (hold) | Flywheel **1300** RPM, hood wheel **2600** RPM. If Y and X are both held, Y wins |
| **RT** (hold) | **Encoder check:** both motors at FULL power, no speed control. Wheels clear. Expect about **6,000 RPM** hood wheel and **1,500 to 1,620 RPM** flywheel |
| **RB** (hold) | Reverse the **flywheel only** |
| **LB** (hold) | Reverse the **hood wheel only** |
| **D-pad Left** | Gate **CLOSED** (holds the balls back, so balls can be loaded into the shooter) |
| **D-pad Right** | Gate **OPENED** (lets a ball pass into a ready flywheel and shooter) |
| **LT** | Gate **LIFTED** (lifts to push the last ball into the shooter) |
| **D-pad Up / Down** | **Nudge the current gate position** + / - 0.01 (Up raises the number) |

**Gamepad 2: intake**

| Control | What it does |
|---|---|
| **RB** (hold) | **Intake** |
| **LB** (hold) | **Reverse** the intake. Beats RB |

Priority, highest first: bumpers, RT (encoder check), Y / X. The gate buttons are on the left side so your right thumb can keep holding Y or X. The gate starts CLOSED at INIT.

---

## 3. Shooter Power - Alpha (two controllers) (hidden)

Plain **power**, no speed control. Note: with the 1620 RPM flywheel motor, the same power is now a **much slower** flywheel, so these are not the old shots.

**Gamepad 1: shooter and gate**

| Control | What it does |
|---|---|
| **Y** (hold) | Flywheel **60%** and hood wheel **60%** power |
| **X** (hold) | Flywheel **55%** and hood wheel **55%** power. If both held, the higher power wins |
| **RB** (hold) | Reverse the **flywheel only** |
| **LB** (hold) | Reverse the **hood wheel only** |
| **LT** | **Toggle** the gate CLOSED / OPENED |
| **D-pad Up / Down** | **Nudge the gate** position (small step) |

**Gamepad 2: intake**

| Control | What it does |
|---|---|
| **RB** (hold) | **Intake** at the dial power |
| **LB** (hold) | **Reverse** the intake. Beats RB |
| **D-pad Up / Down** | Intake power dial **+ / - 5%** |

---

## 4. Shooter Ratio - Alpha (two controllers)

Finds the best **hood / flywheel ratio**. Shoot only when the screen says **At Speed? true**. Do not record a shot marked **CAPPED**.

**Gamepad 1: shooter, gate, dials**

| Control | What it does |
|---|---|
| **B / A** | Flywheel target RPM **up / down** by 100 (range 500 to 1450, starts 1300) |
| **D-pad Right / Left** | Ratio **up / down** by 0.05 |
| **Y** | Shooter **ON** (spins at the dialed speeds and keeps going) |
| **X** | Shooter **OFF** (wheels coast down, wait for them to stop) |
| **RB** (hold) | Reverse the **flywheel only** |
| **LB** (hold) | Reverse the **hood wheel only** |
| **LT** | **Toggle** the gate CLOSED / OPENED |
| **D-pad Up / Down** | **Nudge the gate** position |
| **Back** | Reset the **lowest RPM** readings |

**Gamepad 2: intake**

| Control | What it does |
|---|---|
| **RB** | **Intake** |
| **LB** | **Reverse** the intake |

Careful: **D-pad Right / Left** is the ratio here, and **D-pad Up / Down** is the gate nudge.

---

## 5. Shot Table - Alpha (gamepad 1 only)

Measures and checks the shot table. **No drive, no turret, no gate.** Move the robot **by hand**, load balls by hand. The intake feeds only when the screen says **READY**.

| Control | What it does |
|---|---|
| **RT** (hold) | Shoot at the **DIAL** speed |
| **LT** (hold) | Shoot at the **TABLE** speed (from the camera's `ty` right now) |
| **D-pad Up / Down** | Dial **+ / - 25 RPM** (range 500 to 1450, starts 1300) |
| **A** | **Save a row** (current `ty` + dial RPM) |
| **B** | **Clear** the saved rows |
| **LB** (hold) | Reverse **both wheels AND the intake**. Beats everything |
| **Start** | **Alliance** BLUE / RED (starts BLUE). Pairing with Start + A **also saves a row** |

Priority: LB, then RT, then LT. Copy the saved rows into `ShotTable` before you stop. Nothing is kept.

---

## 6. Turret Testing (gamepad 2 only) (hidden)

| Control | What it does |
|---|---|
| **Left Stick X** | Turn the turret by hand (push right, it turns right) |
| **D-pad Left / Right** | **Hold** to turn left / right |
| **D-pad Up / Down** | Turret power **+ / - 5%** (starts 100%) |
| **Y** | Preset: **center (0°)** |
| **X** | Preset: **far left (+90°)** |
| **B** | Preset: **far right (-90°)** |
| **A** | **Stop** (turret floats) |

Priority: stick, D-pad Left / Right, Y / X / B, then A. Letting go of the stick or D-pad holds the turret. Angle limits are **+/-120°**, measured from where the turret pointed when the Control Hub powered on. Power on with the turret **straight ahead**.

---

## 7. Vision - Alpha (hidden)

Step 1 is camera only (no controls, just INIT and watch). Step 2 aims the turret.

| Control | What it does |
|---|---|
| **Gamepad 1 Start** | **Alliance** BLUE / RED (starts BLUE) |
| **Gamepad 2 RB** (hold) | **Auto-align** the turret on the hive |
| **Gamepad 2 Left Stick X** | Turn the turret by hand. Beats auto-align |
| **Gamepad 2 A** | **Stop** the turret. Beats everything |

Letting go of RB or the stick holds the turret.

---

## 8. Hidden OpModes (`@Disabled`)

**Drive Alpha** (gamepad 1). **Currently SHOWN on the Driver Station** for the field-centric drive test (issue #54); it goes back to hidden when the testing is done. It starts **FIELD-centric**: "forward" is away from you. **Put the robot on the floor facing away from you before INIT**, and keep it still. The 7-step test order is in the OpMode's header.

| Control | What it does |
|---|---|
| **Back** | Switch **FIELD-centric / ROBOT-centric** |
| **D-pad Left** | **Re-aim**: the way the robot faces right now is "away from me". Does not touch the Pinpoint |
| **Left Stick** | Drive and strafe |
| **Right Stick** | Turn |
| **RT** (hold) | **Slow mode** (this OpMode only. The TeleOp uses **LT**) |
| **X / A / Y / B** (hold) | Wheel test, robot **on blocks**: **X** left front, **A** left rear, **Y** right front, **B** right rear. Each must push the robot forward |

**Intake Testing - Alpha** (gamepad 1)

| Control | What it does |
|---|---|
| **RB** (hold) | **Intake** at the dial power |
| **LB** (hold) | **Reverse**. Beats RB |
| **D-pad Up / Down** | Power dial **+ / - 5%** |

---

## 9. Pinpoint - Alpha (gamepad 1 only)

The robot does **not** drive. Push it around **by hand** and watch the odometry numbers. The test order is in the OpMode's header and in `pinpoint-setup-guide.md`.

| Control | What it does |
|---|---|
| **A** | Make the robot's current spot the new start (X = 0, Y = 0, heading = 0) |
| **B** | Recalibrate the gyro **and** zero the position. Hold the robot **still** |

---

## 10. Firing Sequence - Alpha (gamepad 1 only)

The robot does **not** drive. Move it **by hand** to a spot where the camera sees the hive's tags, and load the balls. The turret tracks the hive by itself. The 8-step test order is in the OpMode's header.

| Control | What it does |
|---|---|
| **Right Trigger** (hold) | **FIRE a volley.** Waits until the flywheel is at speed and the camera says the turret is aimed (within 2 degrees for 0.15 s; with no camera in DIAL mode the turret just has to be holding still, and in TABLE mode the camera must see the hive), then opens the gate and **keeps feeding while you hold** (no timer; the intake pauses when the flywheel is off speed or the turret is not aimed). **Let go** closes the gate (gently) and stops the intake |
| **D-pad Up** (tap) | **PRE-REV** on / off: the wheels spin without firing, so the spin-up wait is over before you fire. Shown big on the screen. Starts OFF |
| **Right Bumper** (hold) | **INTAKE** (pick up balls). The gate stays where it is. While a volley is feeding, the sequence runs the intake instead |
| **Left Bumper** (hold) | **REVERSE** both wheels and the intake (clear a jam). Beats everything. (A volley carries on feeding by itself when you let go) |
| **X / B / Y** | Gate by hand: **X** CLOSED (only while no volley is running), **B** (hold) FORCED SHOT (wheels on, gate OPENED, intake running, checks skipped; let go and it closes), **Y** (hold) LIFTED: while a volley feeds, hold it to push the last ball in and let go to go back to OPENED; between volleys let go to close |
| **Back** | Speed source **TABLE / DIAL**. TABLE (starts here): the table sets the flywheel speed, and a volley will **not** start with no camera target. DIAL: a fixed speed, no camera needed |
| **START** | Alliance BLUE / RED. Starts as BLUE. (Pairing the controller is Start + A, so check the alliance after pairing) |

---

## 11. Flower Picker Testing - Alpha (gamepad 1 only)

Moves the flower bar with `FlowerPickerSubsystem`, to check both servos and to find the UP and DOWN positions (issue #75). **Robot on blocks, hands and cables clear:** at INIT both servos jump to UP.

| Control | What it does |
|---|---|
| **RB** | Picker **UP** (one move per press) |
| **LB** | Picker **DOWN** (one move per press). If both bumpers go down on the same loop, UP wins |
| **D-pad Up / Down** | Nudge the **LEFT** servo **+ / - 0.01** (moves the current state's number: UP or DOWN) |
| **Y / A** | Nudge the **RIGHT** servo **+ / - 0.01** |

A nudge only lasts while the OpMode runs. When a position is right, type the four numbers from the screen into `FlowerPickerSubsystem` (`LEFT_UP_POSITION`, `LEFT_DOWN_POSITION`, `RIGHT_UP_POSITION`, `RIGHT_DOWN_POSITION`). The TeleOp uses the same bumpers on gamepad 2.

---

## Quick "which OpMode am I in" differences

| Job | TeleOp Alpha 15 | The test OpModes |
|---|---|---|
| Shoot | Operator **RT** (hold) = a whole volley (the driver's RT only in SOLO MODE) | **Y / X** hold (RPM and Power Alpha), **Y** on / **X** off (Ratio Alpha), **RT / LT** (Shot Table) |
| Gate | Automatic during a volley. Between volleys, operator **X** closed / **B** (hold) forced shot / **Y** (hold) lifted. During a feed, hold **Y** to lift | Shooter RPM Alpha 2: **D-pad Left** closed / **D-pad Right** opened / **LT** lifted. Power and Ratio Alpha: **LT** toggles closed / opened. Shot Table: none |
| Flywheel speed | Operator **D-pad Up / Down** | **B / A** (Ratio Alpha), **D-pad Up / Down** (Shot Table) |
| Alliance | **Start** on either gamepad | **Start** (Vision, Shot Table) |
| Slow mode | Driver **LT** | Driver **RT** (Drive Alpha) |
| Flower picker | Operator **RB** up / **LB** down | **RB** up / **LB** down on gamepad 1 (Flower Picker Alpha) |

If a button here doesn't match the robot, the code changed. Update this sheet.

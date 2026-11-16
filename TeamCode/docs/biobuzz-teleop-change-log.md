# TeleOp change log (`BiobuzzTeleOp`, newest first)

*This text used to be the top of `BiobuzzTeleOp.java`. It was moved here, unchanged, so the file's header can describe how the TeleOp works **right now**. Each entry is what the code did when that Alpha was written: later Alphas may have replaced it (for example, the gentle gate close in Alpha 13 was removed later). When something here and the code disagree, the code wins. The current button map is in `biobuzz-teleop-controls.md`.*

## ALPHA 15: THE FLOWER PICKER (issue #75)

*The screen name became "TeleOp - Alpha 15" (it was Alpha 14). The three Autos' `preselectTeleOp`, the comments and the docs that name it were changed to match, and the Alpha 14 entries below stay as they were written.*

```text
WHAT CHANGED
  1. THE FLOWER PICKER (FlowerPickerSubsystem): the OPERATOR's Right Bumper lifts the flower bar (UP) and
     the Left Bumper lowers it (DOWN). One move per press. If both bumpers go down on the same loop, UP
     wins. It works at any time, also during a volley, and does not touch the gate, the intake or the
     shooter. The two mirrored servos (left_flower_servo, right_flower_servo) start at UP at INIT.
  2. WHY THE OPERATOR AND THE BUMPERS (the coach's choice): the driver is busy driving and running the
     intake (the left thumb is on the stick and the D-pad, the index fingers are on the intake bumpers).
     The operator's D-pad Up / Down was turned down because it is the flywheel RPM dial. The operator's
     bumpers were free (they did nothing before this).
  3. NO SOLO MODE COPY: the driver's bumpers are the intake, so in a one-person match nobody can move
     the picker. A free driver button (A, X, D-pad Down or D-pad Right) would be needed if we want it.
  4. A robot config without the two servos stops this OpMode at INIT (the same as a missing
     shooter_servo). Both competition configs (11940, 22091) have them.
  5. The four servo positions were tuned on the robot with FlowerPickerAlpha. This TeleOp wiring has
     not been run on the robot yet.
```

## ALPHA 14: THE IMU DRIVE, A BATTERY BLACK BOX AND A LOG

*The screen name became "TeleOp - Alpha 14" (it was Alpha 13). The three Autos' `preselectTeleOp`, the comments and the docs that name it were changed to match, and the Alpha 13 entries in these notes stay as they were written.*

```text
WHAT CHANGED
  1. THE DRIVE'S HEADING IS THE CONTROL HUB'S IMU, NOT THE PINPOINT (ImuSubsystem). A Pinpoint blink or
     restart can no longer drop the drive out of field-centric. The safety rule, the glitch hold
     (HOLD_LAST_HEADING_DURING_GLITCH) and the resume check were deleted for the drive. The same
     checks still run in checkPinpointHealth(), but they only throw away the turret's hive memory.
     Field-centric now switches off only on BACK (either gamepad) or when there is no IMU.
     D-pad Left (re-aim) still only moves an offset. Test with ImuAlpha FIRST: the hub orientation
     is USB UP and logo RIGHT (the coach: the hub is on the robot's left wall, USB ports at the ceiling, logo facing the middle). Not yet run on the robot.
  2. A BATTERY BLACK BOX (PowerMonitorSubsystem) and a log (BiobuzzLog, tag BIOBUZZ): hub voltage and
     current on the screen, dips counted and logged, Pinpoint drops and glitches logged.
  3. BULK READS (USE_BULK_READS, AUTO): four hub trips per loop become about one per hub.
  4. A DRIVE ACCELERATION LIMIT in DriveSubsystem (0 to full in 0.2 s, stopping is never limited).
  5. This history moved out of the header into this file.
  6. FORCED SHOT (issue #80): the operator HOLDS B to shoot even when the READY checks say "not ready". The
     wheels spin, the gate OPENS and the intake runs forward at once; let go and the gate CLOSES (with
     the usual intake reverse), or the volley restarts at WAIT_FOR_READY if the trigger is still held.
     B was a one-press "gate OPENED" button that did nothing while a volley waited. The driver's B does
     the same, but only in SOLO MODE. FiringSequence has a new FORCED step and a 5-argument update().
     The Start + B pairing combo now holds a forced shot if it happens while the TeleOp runs. Not yet
     run on the robot.
  7. WALL SHOT (issue #9): the operator HOLDS the Left Trigger for a dumb shot from a known spot (up against
     the wall). The turret goes straight ahead (WALL_SHOT_TURRET_DEGREES, 0) and both wheels run at fixed
     speeds (WALL_SHOT_FLYWHEEL_RPM 1300, WALL_SHOT_HOOD_WHEEL_RPM 2600) from the moment of the press. After
     WALL_SHOT_GATE_DELAY_SECONDS (1.5) the gate OPENS and the intake runs, through the same forced-shot
     path as B (no camera, no READY checks, no flywheel-speed check). Let go and the gate CLOSES (with
     the usual intake reverse) and the wheels go to 0 power (unless PRE-REV is on). While it is held, B and
     the Right Trigger are ignored, and A still floats the turret. Operator only: the driver's Left Trigger
     is SLOW MODE. All four numbers are guesses with a TODO to verify on the robot. Nothing in
     FiringSequence or the subsystems changed. Not yet run on the robot.
```

## ALPHA 13: THE FIRING SEQUENCE (issue #63)

```text
NEW IN ALPHA 13: THE FIRING SEQUENCE (issue #63). The screen name became "TeleOp - Alpha 13" (it was
Alpha 12). The three Autos' preselectTeleOp use the new name, so the Driver Station still loads
this OpMode after Auto.
  Before, the driver held the Right Trigger to spin the wheels and the operator opened and closed
  the gate by hand, so a ball could go through before the wheels were at speed. Now FiringSequence
  (a plain class, see subsystems/FiringSequence.java) runs a whole VOLLEY by itself:
    HOLD the OPERATOR's Right Trigger (gamepad2) -> the wheels spin (if they are not already). The
    volley WAITS
    until the flywheel is at speed AND the camera says the turret is aimed (tx within 2 degrees for
    0.15 s) AND (in TABLE mode) the camera sees the hive right now. (With no camera in DIAL mode the
    turret just has to be holding still.) Then the gate OPENS and STAYS OPEN, and the intake feeds
    the balls for as long as the trigger is HELD (there is no timer). The intake runs only while the
    flywheel is at speed AND the turret is aimed, and it pauses by itself when either is lost (the
    screen says which). Let go of the trigger to END the volley: the gate closes and the
    intake stops (and reverses for the length of the close, see GATE CLOSING AUTO-REVERSE).
    There is no "one volley per press" limit: keep holding and whatever the intake brings in is
    shot. (There is no automatic LIFT step: its timing was off and it wedged the last ball. The
    operator lifts by hand with Y, below.)
  WHO FIRES: the OPERATOR's Right Trigger (gamepad2). The driver's Right Trigger does NOT fire, unless
    SOLO MODE is ON. SOLO MODE is for a match with ONE person driving: the driver presses gamepad1
    LOGO (the center "guide" button) to switch it ON / OFF (it starts OFF, and it works in INIT too).
    While it is ON the driver's Right Trigger fires too and the driver's Y (HOLD) lifts the gate
    (issue #78), and the screen says SOLO MODE ON. In a
    solo match nobody can use the rest of the operator's buttons (gate CLOSED / OPENED, the dials, DIAL / TABLE),
    so set the speed source with gamepad 2 (LOGO) first if you can. (The driver CAN set the
    alliance with START and switch field / robot centric with BACK, see below.) The LOGO button is
    known to reach the Robot Controller on our Logitech controllers.
  PRE-REV: the driver's D-pad Up switches the wheels ON / OFF while you drive to the shooting
    spot, so the 4 seconds of spin-up are not paid at every volley. It starts OFF, and the screen
    says PRE-REV ON. It does not touch the gate or the intake.
  The gate buttons (operator X / B / Y) are an OVERRIDE. Between volleys X, B and Y all work.
    While a volley WAITS (the gate is held CLOSED) they do nothing. While it FEEDS the gate is
    OPENED, and holding Y LIFTS it (see LIFTED below); X and B do nothing.
  The intake is the DRIVER's only (gamepad1 bumpers; the operator's bumpers do nothing). REVERSE (Left
    Bumper) beats the volley, which beats INTAKE (Right Bumper).
  GATE CLOSING AUTO-REVERSE: any time the gate closes from OPENED or from LIFTED,
    the intake runs in REVERSE for a short time (CLOSE_REVERSE_SECONDS in ShooterServoSubsystem, 0.5 s to start with, the same from both),
    so a ball in the gate's way is pushed back out instead of being pulled in against the closing
    gate. It happens after every volley, on an abort, and on the manual X button. It beats the
    driver's INTAKE button while the gate is closing. The screen says "gate closing: REVERSE
    (automatic)". An instant close (STOP) does not do it: STOP stops everything.
  The READY check uses the FLYWHEEL ONLY (ShooterSubsystem.isFlywheelAtSpeed()), like
    ShootAndParkAuto, and the screen shows the hood wheel's error. The "OK to shoot?" line on the
    screen is still the old both-wheels check, for information.
  The flywheel target and the ratio are FROZEN during the feed and the end of a volley.
    In the other steps the target only changes when the wanted RPM moves by more than
    RPM_CHANGE_DEADBAND, so a wobbling table answer cannot keep the "at speed" check from ever
    holding.
  The sequence's steps and its optional feed limit (not used here) live in FiringSequence, not here.
  NOT TESTED ON THE ROBOT YET: whether all 4 balls leave, and the timings. See issue #63.
```

## ALPHA 12: CONTROL CHANGES (issue #74)

```text
NEW IN ALPHA 12: CONTROL CHANGES (issue #74). The screen name became "TeleOp - Alpha 12" (it was
Alpha 11, and is Alpha 13 now).
  REMOVED from gamepad2: the turret stick, the turret presets (Y, X, B), the Right Stick Button
    ALWAYS-TRACK switch, the gate triggers, and the shooter-wheel reverse bumpers.
  REMOVED from gamepad1: the X / B / Y gate buttons (they moved to gamepad2).
  MOVED: the gate is now gamepad2 X = CLOSED, B = OPENED, Y = LIFTED.
  NEW on gamepad2: Right Bumper = INTAKE and Left Bumper = REVERSE, the same as gamepad1.
  SWAPPED on gamepad2: Start = alliance (was Back), Back = flywheel speed DIAL / TABLE (was Start).
  CHANGED: gamepad2 A is now HOLD to float the turret (so you can turn it by hand). Letting go
    goes back to tracking. (A used to switch ALWAYS-TRACK off, and nothing is left to turn it on.)
  NOTE: there is NO way to reverse the shooter wheels now, so a jam in the wheels cannot be
    cleared from the controllers. (The intake REVERSE still works.)
  The full button map for both gamepads is in TeamCode/docs/biobuzz-teleop-controls.md.
```

## ALPHA 11: FIELD-CENTRIC DRIVE (issue #54)

```text
NEW IN ALPHA 11: FIELD-CENTRIC DRIVE (issue #54)
  The drive now STARTS FIELD-CENTRIC: pushing the left stick forward moves the robot AWAY FROM THE
  DRIVER, whichever way the robot is turned. (Before, "forward" always meant the way the robot
  faces. That is still there: ROBOT-CENTRIC.) It uses DriveSubsystem.driveFieldCentric() and the
  Pinpoint's heading.
  - THE DRIVE TOGGLE (FIELD-CENTRIC <-> ROBOT-CENTRIC) is BACK on BOTH gamepads. The drive mode is
    on the screen next to the alliance line, big.
    THE PHYSICAL "MODE" BUTTON cannot be used: the FTC SDK's Gamepad class has no field for it, and
    Logitech says MODE is a hardware switch that swaps the D-pad and the left stick (its LED
    lights when swapped). The INIT screen has a "BUTTON TEST" that shows every button each gamepad
    reports. WARNING: do not press MODE on the driver's controller while driving. If its LED is
    on, the left stick reads as the D-pad and the robot will not drive. Press MODE again.
  - THE DRIVER's D-pad Left RE-AIMS: "the way the robot faces right now is away from me". It only
    changes which way forward means. The Pinpoint is NEVER reset, because the turret's hive
    memory depends on its heading and position.
  - WHICH WAY IS "AWAY FROM THE DRIVER" depends on the alliance. From the field picture the BLUE
    alliance area is on the RIGHT and the RED one on the LEFT, so the blue driver faces LEFT (field
    heading 180) and the red driver faces RIGHT (0). The Pinpoint's heading is zero wherever the
    robot sits at INIT, and our Autos never turn the robot, so after Auto it faces the way it
    started: BLUE facing DOWN (-90), RED facing UP (+90). The heading that goes into the drive is
    the Pinpoint's heading plus (the robot's field heading at INIT - the driver's forward field
    heading). Blue: -90 - 180 = +90. Red: +90 - 0 = +90. The same answer, because red is the blue
    plan turned 180 degrees. The four numbers are kept per alliance (see the constants), so a
    different start orientation later is a one-number change. The offset is set from the
    alliance (operator Start) in INIT and fixed at START: changing the alliance after START does
    NOT change which way the stick drives. D-pad Left replaces it with the re-aim.
  - ASSUMPTIONS: the blue drivers stand on the right and the red drivers on the left, and the robot
    is not moved between the end of Auto and INIT (the Pinpoint is zeroed at INIT). With no Auto
    before (practice), put the robot in the Auto start orientation, or press D-pad Left.
  - THE SAFETY RULE: if the Pinpoint is not READY for MORE than PINPOINT_GLITCH_TOLERANCE_SECONDS
    (0.25 s) after START, or its heading jumps (the same check that protects the hive memory),
    field-centric switches ITSELF OFF and the robot drives ROBOT-CENTRIC. The screen says why. It
    does NOT switch itself back on, because a restarted Pinpoint has a new zero. The drive toggle
    (BACK on either gamepad) or the driver's D-pad Left turns it on again.
    GLITCH DEBOUNCE (added after the first robot test, where a weak battery made the Pinpoint blink
    and field-centric kept switching off): a gap SHORTER than 0.25 s is a glitch, not a restart. The
    drive keeps the LAST good heading (HOLD_LAST_HEADING_DURING_GLITCH), the hive memory is kept, and
    when the Pinpoint is READY again its heading must be within RESUME_MAX_HEADING_CHANGE_DEGREES
    (20) of the last good one, or we assume it restarted and switch field-centric off after all.
    The screen counts the glitches ("Pinpoint glitches"). Many of them means a weak battery.
    THE CAMERA: VisionSubsystem now checks every 0.5 s that the Limelight is on the AprilTag
    pipeline, and switches it back if it restarted into another one ("Camera pipeline fixes").
  - The turret still follows the robot's spin the same way (the turn stick is the same in both
    modes), and the hive memory is not touched by any of this.
```

## ALPHA 10: SAFER PINPOINT HEADING FOLLOW

```text
NEW IN ALPHA 10: SAFER PINPOINT HEADING FOLLOW
  (after the first robot test, where the turret snapped around):
     - The remembered hive direction is THROWN AWAY whenever the Pinpoint is not READY, or its
       heading jumps by more than MAX_PINPOINT_HEADING_JUMP_DEGREES in one loop (a robot cannot
       turn that fast, so the Pinpoint must have restarted, for example after a power dip).
       Before, a restarted Pinpoint (heading back at 0) could send the turret to a wrong angle.
     - If the remembered hive is somewhere the turret cannot go (for example behind the robot),
       the turret now HOLDS where it is. Before, a small turn could flip the target between the
       two limits and swing the turret all the way across.
```

## ALPHA 9: THE PINPOINT HELPS THE TURRET (and our hive's two sides)

```text
NEW IN ALPHA 9: THE PINPOINT HELPS THE TURRET (and our hive's two sides)
  1. PINPOINT HEADING FOLLOW. While the camera sees the hive, HiveBearingMemory saves which
     direction the hive is in on the FIELD: robotHeading + turretAngle - tx. If the camera then
     loses the hive (the arm hides the tags, a ball blocks the view), ALWAYS-TRACK keeps the
     turret aimed at that remembered direction: turret angle = remembered direction - robotHeading.
     So the turret keeps pointing at the hive while the robot SPINS. It gives up (STALE, the
     turret holds still) when the Pinpoint says the robot has DRIVEN more than 12 inches since the
     camera last saw the hive, or after 5 seconds, because driving changes the real direction
     and heading alone cannot see that. When the camera sees the hive again it takes over.
     If the Pinpoint is missing or not READY, the turret just holds, like Alpha 8. It does not
     search. The screen line "Hive memory" says FOLLOWING, STALE or none.
  2. BOTH SIDES OF OUR HIVE. Each hive has two cells (FAR and AUDIENCE), and only the raised one
     scores. VisionSubsystem now LOCKS onto one cell, switches to our other cell only when it is
     clearly higher, and lets go of the lock after the cell has been out of view for half a
     second. After a hive tip you drive to the other side, the old cell leaves the view, and the
     cell you now see becomes the target by itself. The screen says "Aiming at BLUE FAR side"
     (or AUDIENCE). There is NO new button. The alliance button (START) also clears
     the memory and the lock, so we never aim at the other hive's direction.
  LIMITS: no search (if the camera has not seen the hive from the new side, turn the robot
  toward the hive until it does). And if only the LOWERED cell is in view, ty alone cannot say
  it is lowered, so the turret would still aim at it.
```

## ALPHA 8: ODOMETRY ON THE SCREEN

```text
From ALPHA 8: ODOMETRY ON THE SCREEN
  PinpointSubsystem reads the goBILDA Pinpoint odometry computer once per loop, and the Driver
  Station shows where the robot is: X (inches forward), Y (inches left), and heading (degrees,
  turning left is positive), plus the speeds and the Pinpoint's status. The numbers are shown
  during INIT too, so check the status says READY before START. The robot must be STILL during
  INIT, because the Pinpoint calibrates its gyro then, and X, Y and heading start at 0 where the
  robot sits at INIT. As of Alpha 9 the turret uses the
  heading and position (see above). If the config has no device named "pinpoint", the screen
  says so and everything else still works.
```

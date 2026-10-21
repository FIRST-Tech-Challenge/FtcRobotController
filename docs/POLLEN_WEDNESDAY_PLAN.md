> Historical preparation notes. For current build, controls and robot tests, use
> [POLLEN_AUTONOMOUS.md](POLLEN_AUTONOMOUS.md) and
> [POLLEN_SHOOT_FIRST.md](POLLEN_SHOOT_FIRST.md). Old patch commands below are superseded.

# Pollen autonomous: plan for Lev's review

Prepared Monday evening, October 19, 2026. Target: Wednesday practice, October 21.

**Priority: prove Option A, preserve that working version, then develop Option B.**
Wi-Fi deployment is working; USB troubleshooting is no longer on the critical path.
The bot currently runs V2. GitHub's `pollen-auto-prep` branch was checked at
`cd79e0a2de1a3d297cedc4c5a659e40c9f46b5df` (Lev's V2 commit).

This is a proposed operating and testing plan for Lev to review. The included
patch combines the existing V2-to-V3 starting-delay patch with this plan and the
patch helper. It is not a finished shooting route.
The additional B changes below remain to be implemented after agreeing on the route.

## 1. Decisions to discuss with Lev first

1. Which autonomous file and selection process actually worked last year?
   The repository has `Auto.java` and a `MecanumOpMode` that cycles options with
   the PS/Home button before starting. Are these the versions the kids used?
2. Did their successful routines use timed motion, encoders, an IMU, or Pedro?
   The old routines inspected here use `driveWithTime(...)`. If a different,
   proven controller exists for this exact drivetrain, show it before we add one.
3. Is the current robot mechanically identical to `BioBuzzStarterbotOpMode`?
   Confirm names, wheel directions, shooter, feeder, and launch direction.
4. Have they already found a reliable shooting location and launcher setting?
   An observed good shot is more useful than the initial range guess below.
5. Are two separate autonomous entries on the Driver Hub acceptable?
   Recommended: A = Park, B = Shoot then Park. No controller A/B buttons needed.
6. Do we have field foam tiles, pollen, and a real hive or accurately sized target
   at home? If not, Wednesday must include final traction and shooting validation.
7. Which alliance and likely partner route should we prepare B for first?
   A uses manual placement; B includes a turn and needs an explicit mirrored route.

**What we are retaining from the old code:** timed, ordered actions and speed-gated
shooting. **What differs:** the current pollen code uses separate autonomous entries,
StarterBot device names, and an iterative sequence that can also run without hardware.
The existing old Blue Wall route is forward, turn, shoot, turn, strafe. This is very
close conceptually to B. Its times and older launcher presets should not be copied
onto this robot without measurement. In `shootWithTime(3, 7000)`, 3 is a launcher
preset, not three projectiles.

## 2. What the kids will do

| Task | Option A | Option B |
| --- | --- | --- |
| Current Driver Hub entry | `Pollen ROBOT Park` | `Pollen ROBOT Shoot` |
| Proposed final names | `Pollen A - Leave + Park` | `Pollen B Red - Shoot + Park` / `Pollen B Blue - Shoot + Park`, once each is tested |
| Physical start | Either of two measured, manually angled placements adjacent to parking | One documented shooting-route start per tested alliance |
| After INIT | Confirm READY; set delay with Gamepad 1 D-pad up/down in V3 | Confirm B READY and correct alliance; selectable delay is a proposed later change |
| After START | Wait, drive straight, stop | Wait, approach, settle, shoot, turn, drive to park, stop |

The final student procedure is: choose the routine, position the robot using its
placement card, INIT, confirm route/delay/READY, then press the Driver Hub's START
at the match start. Use the 30-second autonomous timer. Delay selection happens
before the countdown; the code locks it on START. STOP remains available.

A never needs to know which of its two starting placements humans chose. The
same forward power/time applies to both, provided testing confirms the geometry.
Do not confuse those two physical placements with the A/B routine choice.

For now use the existing names. Proposed renaming should be a separate, reviewed
change after V3 is installed. Keep BENCH modes hidden in the final practice build.
Keep B disabled or explicitly experimental until its full route has passed.

## 3. Monday night: install V3 and establish the baseline

Allow about 45–75 minutes. Stop after a sound baseline if people are tired.

1. Photograph the robot and its wiring; record which Control Hub is actually on
   the robot. In Driver Hub configuration, record the active configuration name,
   motor/servo names, port numbers and hub serials. Preserve the working configuration.
   Do not replace it with the earlier empty bench configuration.
2. Put the existing working APK in a dated backup folder outside the checkout if
   you still have it. Record its commit/configuration. An APK alone does not capture
   the robot's configuration or physical launch-angle adjustment.
3. On home internet, inspect local changes and fast-forward the branch as described
   in section 9. Save intentional changes before applying a patch.
4. Apply `pollen-v3-wednesday.patch` using `pollen-patch.sh`. It checks patch
   applicability, saves a backup branch, runs the Java tests and builds the APK.
5. Deploy over the working hub Wi-Fi connection. Check the Driver Hub telemetry
   reports **pollen-prep-v3** after INIT. Building, pushing and deploying are three
   different actions; GitHub updates do not update the hub.
6. With the robot securely supported and wheels free, no pollen loaded, run A.
   Verify all four wheels would drive forward, then stop. INIT must not move anything.
   Fix mapping/direction errors before putting it on the floor. Matching names alone
   does not establish direction. Check the known working TeleOp as a cross-check.
7. Verify A's delay at 0 seconds and 3 seconds. A held D-pad button should count as
   one press; a new INIT returns to the default. Inputs after START must not alter
   the wait or route. Test STOP during the wait and during a short drive.
8. Optional: temporarily enable BENCH Park to inspect commands without movement.
   BENCH modes simulate hardware; they cannot verify wiring or physical stopping.
   Restore `@Disabled` and rebuild before the final practice release.

For moving tests, one person runs Driver Hub and STOP, the other observes from
outside the travel path with access to the robot power switch. Keep Abby and other
people out of the route. Remove power before handling mechanisms or clearing jams.
Use an appropriate enclosed catching area for any projectile test.

## 4. Tuesday first: make Option A repeatable

Allocate the first 60–90 minutes to A; do not start B tuning until A passes.

### Lay out and measure

Use actual field foam if available. Tape a mock loading zone, a perimeter line
and a fixed starting reference. A sturdy mock wall is useful for repeatable contact.
Use the actual robot outline, including protrusions, not an assumed 18-inch square.

The current TU05 manual describes a loading zone roughly 23 inches along the wall
by 11 inches inward, including its tape. A legal start touches the perimeter,
stays outside the loading zone and on its alliance side, avoids flowers, and has
four pollen preloads. At the end of AUTO, no wall contact earns LEAVE (3); partial
loading-zone overlap earns PARK (5). Verify with manual sections 9.3, 10.5.4–5 and G304.

The goal is a clear wall gap and substantial zone overlap, not a borderline tape
touch. Start by aiming for about 2 inches of wall clearance and 3 inches of overlap
past the entry boundary; these are our test margins, not rules. Adjust the start
angle and location if the actual footprint cannot provide both.

Begin with the manually angled straight-ahead concept already discussed. Around
10 degrees away from a path parallel to the wall is a starting experiment, not a
prescribed angle. Measure both placements separately. Use a placement card or
alignment reference so each child can reproduce the starting position and heading.

### Tune one variable at a time

The V3 starting values are **0.20 forward power for 0.75 seconds**. They are examples,
not a calibrated parking distance. In `PollenConfig.java`, tune only
`PARK_FORWARD_POWER` and `PARK_FORWARD_SECONDS` initially.

1. Use zero start delay. Run once in a clear area and measure actual travel, sideways
   drift and heading change. Confirm the path leaves the wall instead of scrubbing it.
2. Hold power constant and adjust duration in small steps, such as 0.1 seconds.
   For a rough first correction, new time = old time × desired travel / measured
   travel. Startup acceleration makes this approximate, especially for short moves.
3. If 0.20 stalls or behaves inconsistently, raise it cautiously and recalibrate
   time. Do not add voltage scaling, ramps or new navigation before checking wheel
   friction, battery condition and the driving surface.
4. Verify both physical starts with the same constants. If one fails, fix its
   placement geometry; do not silently make different code for that start.
5. Reproduce match loading and robot mass for final trials. Record battery voltage
   displayed by Driver Hub before each run. Use charged and normally used batteries,
   not a deliberately depleted pack.

### Acceptance gate for A

Run three consecutive trials for each combination of two starts and two battery
conditions: **12 trials total**. Record every trial, including failures. Each must:

- finish with the agreed wall-clearance and parking-overlap margins;
- stay within its planned corridor with no contact or intervention;
- finish stopped with zero requested output;
- show no unexpected drift or heading change that threatens the margins.

Separately repeat one full run with a 3-second delay, test STOP, and have Lev and
another intended driver independently select and start A. If robot geometry allows
only one placement to pass, release that one and document the other as unavailable.

If the home surface differs from field tiles, label the result **home verified**.
Do not claim field calibration. If both starts still cannot be reliable on matching
foam after a focused tuning session, assess encoder distance control or a proven
existing controller as a separate change. Keep a hard timeout; a wheel encoder is
not proof of ground travel and does not remove slip.

Save the measured constants and results, build and install that exact version, then
commit and push it. Create a local release tag such as
`pollen-a-home-verified-20261020` on the verified commit. Keep its APK and a photo
of each placement. Further work on B must preserve this recoverable A version.

## 5. Tuesday second: locate a good shot, then build B around it

### Initial guess, not a known shooting range

Start about **36 inches horizontally from the launcher exit to the center of the
upward-facing cell of our hive**, looking toward it from our own alliance half.
Favor the side of the hive nearer our loading zone. Try 30 and 42 inches as nearby
alternatives. These ranges are proposed experiments; no inspected source establishes
the optimum range for this assembled robot. Use an already proven team spot instead
if Lev has one.

Keep the launch angle fixed initially. Begin with the starter-code target of
1250 encoder ticks/second, which is not RPM. Record short/long/left/right misses.
Change range first; change velocity or mechanical angle separately if necessary.
The target cell is elevated and changes position when the hive tips: a floor bucket
does not validate the real shot. If there is no correct target at home, prepare
mechanism and sequence tests now and reserve shooting calibration for Wednesday.

At each promising position, fire three sets of four pollen. Record hits per set,
time to spin up, velocity recovery after shots, whether all four feed, and jams.
An initial selection gate is at least 10/12 hits with all sets clearing the feeder,
then several complete approach-to-shot runs. This is a team test target, not proof
of match reliability. Aim for a forgiving range and short total route, not the
highest score from one lucky stationary run.

Do not assume every preload makes an immediate autonomous score. Hive tipping is
the relevant immediate hive event; pollen remaining in the cell is scored at match
end. Coordinate which cell the partner targets and when it may tip. A fixed-aim B
cannot follow a cell that changes position; begin with an agreed target state.

### Choose a legal start from the measured shot

Once a shooting pose is selected, work backward toward the nearest *legal* wall
contact pose that permits a short, clear approach. The nearest point geometrically
can be inside the loading zone or place the robot across an obstacle. Slide along
the wall as needed, checking the full footprint and preloads.

Prefer starting already aimed for the shot if the approach permits it. Otherwise
use an explicit approach turn/strafe. A minimum-length path is not worth unreliable
alignment. Document one fixed start, shooting pose and parking destination first.

Requested B sequence:

1. Configured start delay.
2. Measured approach to shooting pose.
3. Stop and settle.
4. Spin launcher; wait for stable speed; feed with bounded recovery pauses.
5. Stop launcher and feeder.
6. Turn toward the parking destination; settle if measurement shows it is needed.
7. Drive the measured parking leg and hold stopped.

The current `shootRoute()` only approaches, shoots and reverses. It has **no turning
step**, and its times are placeholders. It does not implement the route above yet.
The existing `Step.drive(...)` supports a turn using its final argument, so a new
navigation framework is not required just to express this sequence. Turn duration
and power must be measured. Heading control with an IMU is a useful later step if
timed turns are unreliable and the team's working IMU setup is known.

Red/blue B routes need explicit validation. A mirrored route can reverse turn and
strafe signs when its poses are truly mirrored; merely moving the robot to the
other alliance does not mirror the Java route. Start with one tested B alliance,
then add the other if time permits. Do not add a start-position selector to A.

## 6. Required B code changes before driving its new route

Keep these in a small separate commit after A is protected:

1. **Fix the init fallback assumption.** V2/V3 automatically switch to `parkRoute()`
   if shooter mapping fails. That is unsafe once B starts somewhere different from
   A. For the proposed B, block initialization, zero outputs and display an instruction
   to select A and physically reposition. Alternatively, a separately tested
   B-start-to-park route could be provided later. Do not use A's route from B's start.
2. Add the measured turn and B parking leg. Name tuning constants clearly and keep
   A's constants independent. Require a valid worst-case time budget before starting.
3. Extend the INIT-only delay selector to B if partner coordination requires it.
   Calculate its maximum from B's longer worst case, not A's 20-second cap alone.
   Preserve locking on START and no driver-input steering after START.
4. Give each tested choice an unmistakable name and preselect the working TeleOp.
   Keep untested choices disabled in the final driver build.
5. Add focused tests for B's init-block behavior, route order, turn signs, delay
   budget and timeout continuation. Retain existing STOP and zero-output tests.

An ordinary low-speed/spinup timeout at the shooting pose should skip remaining
feeding and continue B's own parking leg. A runtime hardware exception should
stop everything. Do not attempt a guessed recovery route from an unknown pose.

The current shot logic requests two seconds of active feeding with up to six seconds
elapsed after feed begins, plus a separate four-second spinup limit. This is **not a
four-ball counter**. Measure the four-ball sequence before extending a window or
loosening the 1200-tick/s feed gate. Check the full worst-case route plus delay fits
the existing guard, with at least the sequencer's required one-second reserve.

## 7. Wednesday practice: validate, then release

**First 15–20 minutes: A on the actual field.** Use the real perimeter, zone and
surface. Repeat both starts, verify the final footprint and wall gap, and check the
partner's start, entire path, timings and final footprint. A staggered start does
not guarantee clearance if a partner occupies our destination or corridor.

Correct traction-dependent timing before spending time on B. Have the kids run the
full selection/INIT/delay/START process themselves. A should work even if B is not ready.

**Next 20–30 minutes: B only after A passes.** Validate the actual hive shot, settle
time, loaded feeder behavior, turning direction and final park. Test a complete
four-pollen run, not just stationary shooting. Demonstrate that a bounded shot
failure still takes B's parking leg. Do not unplug powered hardware to provoke it;
use simulation for dangerous failure cases and a controlled test build if needed.

Before calling B ready, require at least three consecutive complete runs on the
tested alliance, with shots meeting the agreed standard and successful parking.
Try a normally used battery as well as a fresh one. If it fails, leave B experimental
and use A at practice. Record exactly what remains to fix.

**Final 10 minutes:** restore hidden bench/test entries, deploy the final build to
the robot's actual hub, verify the active configuration and build identifier, run A
one more time, then commit/push the exact source. Capture a short video and release
note with commit, APK, motor configuration, battery range, placements and tested modes.

## 8. Working with Gemini in Android Studio

Yes: use Gemini for local, narrowly scoped edits and build-error diagnosis. Google
currently documents Android Studio benefits for Google One AI Pro/Ultra accounts;
sign in with the personal account that owns the plan. A separate API key is not
required for that subscription path. Gemini needs internet access; the hub Wi-Fi
normally does not provide it.

Use this rhythm:

1. Home internet: pull, edit, review the diff, test and build.
2. Hub Wi-Fi: connect ADB, deploy, test the robot and record results.
3. Home internet: commit/push changes and share the commit hash here.

Only one coding agent should edit a given change at a time. Treat GitHub as the
shared source of truth. I can inspect pushed commits when you give me the hash;
I cannot see unpushed Android Studio edits or monitor them automatically.

Suggested first Gemini task after A's baseline is installed:

> Inspect the pollen package and docs/POLLEN_WEDNESDAY_PLAN.md. Summarize the current
> flow and compare it with Auto.java and MecanumOpMode.java. Do not edit yet. Identify
> the exact changes needed for B to use a different starting pose without unsafe
> fallback to A, and list which constants need physical measurements. Keep the
> validated A route, motor mapping, launcher PIDF, and build dependencies unchanged.

After Lev agrees, give Gemini **one** implementation task at a time, e.g. blocking
B on shooter-init failure. Require a minimal diff, meaningful tests and an APK build.
Then push and ask for review before moving to the route change. Calibration edits
are another good local task once you provide actual measured values.

Do not ask an agent to "optimize autonomous" without those measurements; it cannot
infer the robot's traction, launch angle or shot dispersion from Java source.

## 9. Patch installation and Wi-Fi deployment

The kit contains `pollen-patch.sh`, `pollen-v3-wednesday.patch` and this plan. Unzip
outside the repository, for example into `~/Downloads/pollen-wednesday-kit/`.

In Terminal, go to your existing FtcRobotController checkout. While on home internet:

```bash
git branch --show-current
git status --short
git diff
```

If there are intentional local edits (such as enabling bench modes), review and
commit them first. `git add -p` lets you select tracked edits; add intended new files
by name. Do not blindly discard edits or stage unrelated files. Once the tree is clean:

```bash
git switch pollen-auto-prep
git pull --ff-only origin pollen-auto-prep
bash "$HOME/Downloads/pollen-wednesday-kit/pollen-patch.sh" apply \
  "$HOME/Downloads/pollen-wednesday-kit/pollen-v3-wednesday.patch"
```

The script requires the team origin and this branch, resolves Android Studio's JDK,
checks the patch, creates a local backup branch pointing to the clean pre-patch
commit, applies the patch, runs sequence tests, and builds `:TeamCode:assembleDebug`.
It does not stash, discard, commit, push or install. If checks/build fail, it stops;
applied changes remain visible for diagnosis. A network-dependent build failure
does not mean the patch should be applied again. Already-applied patches are detected.

If deliberate edits conflict with patch context, the script refuses to force it.
Share the error and diff so the change can be reconciled. Never use reset/clean to
get past the check without first preserving your work.

After any local/Gemini edit, run verification without applying a patch:

```bash
bash "$HOME/Downloads/pollen-wednesday-kit/pollen-patch.sh" verify
```

For a nonstandard Android Studio location, set `FTC_JAVA_HOME` to its JDK Home
directory. The script sets both `JAVA_HOME` and Gradle's `org.gradle.java.home`,
preserving the solution to your Java 11/17 issue. With cached dependencies, you can
use `FTC_OFFLINE=1` before the command while connected to hub Wi-Fi.

After a successful build, switch the Mac to the Control Hub Wi-Fi:

```bash
"$HOME/Library/Android/sdk/platform-tools/adb" connect 192.168.43.1:5555
"$HOME/Library/Android/sdk/platform-tools/adb" devices -l
```

Select that device in Android Studio and Run TeamCode. Or, to install the APK built
earlier without another Gradle invocation, first confirm this file exists:

```bash
ls -l TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
"$HOME/Library/Android/sdk/platform-tools/adb" -s 192.168.43.1:5555 install -r \
  TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
```

If your build uses another APK filename, use that actual output; do not select an
old APK. Stop the OpMode before deploying. `install -r` replaces the app without an
uninstall; do not uninstall to fix a signing error without preserving configuration.
After a Terminal-only install, reopen/restart the Robot Controller app if necessary
and wait for the Driver Hub to reconnect. Confirm the displayed build and configuration.

Back on home internet, review and publish only intended source changes:

```bash
git diff --stat
git diff
git status --short
git add -p
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenStartDelay.java
git add tools/pollen-tests/PollenStartDelayTest.java
git add tools/pollen-patch.sh docs/POLLEN_WEDNESDAY_PLAN.md
git diff --cached
git commit -m "Add selectable starting delay for pollen parking"
git push origin pollen-auto-prep
git rev-parse HEAD
```

Those explicit `git add` paths are the new V3 and workflow files; skip them for unrelated
later changes. Keep using the personal/team identity already configured locally.
If the push is rejected because someone else advanced the branch, fetch and review
the new work; do not force-push over teammates. Send the resulting commit hash here.

## 10. Test record to keep with the code

Use one line per attempt. Keep failed trials; they identify weak margins.

| Date/build/commit | Mode/start/alliance | Battery V | Surface/load | Delay | Power/time | Travel/drift/heading | Wall gap/zone overlap | Shots/hits/feed time | STOP/result |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | | | |

**Ready for Wednesday** means A's source, installed APK, setup card and home test
record agree; B's readiness is reported separately. **Field validated** additionally
requires the actual field checks above. If time is short, deliver reliable A and
a measured, clearly labeled B experiment.

## Sources

- Team branch: https://github.com/FtcBagelBits/FtcRobotController/tree/pollen-auto-prep
- FIRST BIOBUZZ Competition Manual, TU05, checked October 19:
  https://ftc-resources.firstinspires.org/ftc/game/manual
- FIRST Control Hub management (direct wireless ADB):
  https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/managing_control_hub/Managing-a-Control-Hub.html
- Google Android Studio Gemini tiers, checked October 19:
  https://developer.android.com/studio/gemini/feature-comparison

The 36-inch shot, suggested test counts and physical margins are our engineering
proposals. They are not published goBILDA calibrations or additional game rules.

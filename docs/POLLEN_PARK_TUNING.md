# Fast parking calibration with Gamepad 1

This patch targets `pollen-auto-prep` at `9453bb1` and adds controller tuning to
`Pollen ROBOT Park` (and BENCH Park if explicitly enabled). The build marker is
`pollen-prep-v3.1-tuning`. It preserves the updated team motor directions and
`PollenBot` TeleOp selection. Shooting hardware and routes are unchanged.

Install once. Further parking power/time changes do not need a build, deployment,
Wi-Fi connection to the laptop, or restarting the Robot Controller app.

## Controls after INIT, before START

| Gamepad 1 control | Action |
| --- | --- |
| D-pad RIGHT / LEFT | Drive duration + / - 0.05 seconds |
| R1 / L1 (right / left bumper) | Forward power + / - 0.05 |
| D-pad UP / DOWN | Starting delay + / - 0.5 seconds |
| Triangle | Reset all three settings to the defaults compiled in PollenConfig |

Each press changes one increment. Release before pressing again; holding a button
does not repeatedly change the value. Opposing buttons cancel that pair. There
are no motor commands caused by tuning buttons. START locks all settings; controller
inputs cannot change the running route. Driver Hub STOP ends the OpMode.

Drive power is bounded to 0.05–0.50 and duration to 0.05–5.00 seconds. Delay is
bounded by the route's remaining budget, up to 20 seconds. Increasing drive duration
can reduce an excessive delay to preserve the required reserve. Invalid code
defaults outside these ranges block INIT instead of silently choosing another move.

## Repeat a test

1. Select Pollen ROBOT Park and press INIT. Read power, drive seconds and delay.
2. Set delay to zero while calibrating travel. Adjust time first, keeping power fixed.
3. Place the robot at its measured start, clear the route, press START.
4. After DONE, press STOP. Reposition, select Park again if TeleOp was preselected,
   and press INIT. Change the next value and START again.
5. Photograph or record the settings and measured outcome for each trial.

Power and drive duration are remembered across STOP/INIT and switching OpModes
while the Robot Controller app process remains alive. Delay resets to its code
default each INIT. Bench settings are kept separately from robot settings.

**These are temporary session settings.** Restarting the app, rebooting or installing
an APK loses them. Triangle restores the code defaults. The screen shows the exact
`PARK_FORWARD_POWER` and `PARK_FORWARD_SECONDS` values to copy to PollenConfig once
calibration is complete. Make that final source edit, build, deploy and commit so
the measured values survive restart and are available to the team.

## Apply from your local checkout

Save intended calibration edits first; the patch helper requires a clean checkout.
Keep the downloaded patch outside the repository. For existing local edits to
PARK_FORWARD_POWER/SECONDS, retain your measured values when reviewing any conflict.

```bash
bash tools/pollen-patch.sh apply "$HOME/Downloads/pollen-park-tuning.patch"
```

That helper applies, tests and builds using Android Studio's JDK. Alternatively,
apply explicitly and use your existing ftcgradle function:

```bash
git apply --check "$HOME/Downloads/pollen-park-tuning.patch" &&
git apply "$HOME/Downloads/pollen-park-tuning.patch"

PATH="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin:$PATH" \
  bash tools/test-pollen-sequence.sh &&
ftcgradle :TeamCode:assembleDebug
```

Connect to hub Wi-Fi and deploy that successful build once. Confirm the new build
marker in telemetry. Test the first adjustment with wheels supported and no pollen.
Remember to stage the new PollenParkTuning.java, its test and this document when
committing; `git add -p` alone will not include new files.

If the Driver Hub's INIT button alone still takes many seconds without any new
installation, that is a separate initialization/connectivity problem. This patch
removes recurring build/deployment time; it does not promise to fix slow hardware
initialization or an actual hub reboot.

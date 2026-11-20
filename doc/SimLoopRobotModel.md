# BioBuzz SimLoop model and measurement provenance

Updated November 5, 2026. Source: `BioBuzzSimHardware`, `BioBuzzSimModel`, production subsystem constants.
This is an SDK/lifecycle and coordination model, not a calibrated digital twin. Configuration values
below are installed-code settings unless explicitly described as physical measurements. Do not tune
real mechanisms from simulated speed, current, distances or shot outcomes.

| Hardware / relationship | Fixture configuration | Evidence / limit |
| --- | --- | --- |
| Drive | leftfront/rightfront/leftback/rightback; physical mounting signs +,-,+,-; goBILDA 5000-0002-4008, 28 motor ticks/rev, 5800 RPM no-load at 12 V; 20:80 gears then coaxial 30:91 belt | Team-reported transmission, 12.133333 motor rev/wheel rev; manufacturer-tested shaft speed, not loaded robot measurement; mounting signs independent of SDK directions, updated from the team's November 5 all-directions-reversed report (previous -,+,-,+ was an assumption); physical shaft measurements pending |
| Wheels | 104-mm diameter goBILDA mecanums with 30A rollers; radius 52/25.4 in; motor ticks/in = 28*(80/20)*(91/30)/(pi*104/25.4) = 26.411232 | Team-reported nominal diameter and compound; loaded compression, wear and slip unmeasured |
| Chassis dynamics | 14-in track/wheelbase; lateral ratio .85, response 10/s | Synthetic chassis geometry, friction and acceleration; 30A hardness does not determine traction parameters |
| Intake | intake_motor; no intake encoder; turret encoder on its Expansion Hub port 1 input | Operator-confirmed wiring November 13; turret removes SDK motor-direction sign; never reset shared feedback; no ball-contact model |
| Transfer | transfer_motor_1 and _2, both FORWARD and mechanically linked on stage 1 (motor 2 moved off the removed back rollers 2026-11-18); motor 1's own encoder on port 2 | Stage 1 (both motors, one command) loads and feeds shots at full open-loop power; every scenario asserts the two motor powers never diverge; its encoder is telemetry only; fixture motor dynamics synthetic, no ball or linkage load |
| Queue sensor | distance_sensor; digital HIGH, LOW default | Operator or test controls signal; production owns 0.5-second HIGH confirmation, immediate intake stop, 0.5-second stage-1 run-on and immediate LOW recovery |
| Gate | gate_servo; REVERSE; PWM 500–2500 us, logical .5 closed/.2 open | Asserted physical .5/.8 and 1500/2100 us; .10-s competition gate delay (.20-s diagnostic/autonomous default) is code timing, not loaded clearance proof; direct Expansion Hub supply, no 6 V injection; electrical/loaded gate speed not modeled |
| Depollinator | depollinator; 30/355 home, 165/355 deployed; 500–2500 us | Position/PWM commands checked; linkage collision/travel not modeled |
| Turret | turret_servo_1/_2; 355-deg span, 1:1; actual encoder 8192 CPR *108/40 =61.44 ticks/deg | Independent physical yaw approaches PWM-derived target at synthetic 180 deg/s; jam freezes physical yaw |
| Hood | turret_pitch; home .5+16/355; 2:1 servo/hood | Commands bounded by production code; no hood feedback sensor or loaded model |
| Flywheel | turret_motor; goBILDA 5203 Yellow Jacket, 28 ticks/rev, 5400 observed motor RPM at 14 V; 1:1 output ratio | Team observation November 1 at 14 V in PIDF/bang-bang; upstream response rate remains synthetic, no ball impulse or calibrated drag |
| Battery | Control Hub FakeVoltageSensor =14 V | Synthetic constant; not a battery-sag/electrical/thermal model |
| Pinpoint | pinpoint; physical offsets +32,-74 mm, physical pod directions X FORWARD / Y REVERSED (hand-push measured 2026-11-13; earlier both REVERSED was an unmeasured assumption), 2000/(32*pi) ticks/mm | Chassis motion drives fake SDK I2C device; production config/reset/calibration/fault handling executes |
| Camera | limelight; pivot forward50.68184865mm; camera forward from pivot -99.53073974mm, height337mm (from `CAMERA_HEIGHT_MM`), pitch up55deg (from `CAMERA_PITCH_UP_DEG`), yaw0 | Matches documented team mount; fixture translates/rotates the camera with physical turret yaw |
| Camera optics/timing | 54x41-deg FOV, 2–200-in range, 30Hz, 15+20ms latency | Synthetic settings; real pipeline, exposure, intrinsics and latency need verification |
| Raised tags | nominal TU06 audience tags34–41, 3.25in, hive centers +/-12.75in, production-derived row/height, face tilted30deg | Nominal drawing map; not surveyed field coordinates; far/lowered states absent from this layout |

The drive plant integrates in +X forward at heading zero, +Y left, CCW. The fixture deliberately uses
that numeric frame as BioBuzz's field X/Y frame for its own tag layout; it does not import SimLoop's
season field objects or their 90-degree transform. Inches become mm for Pinpoint; camera field/layout
inputs remain inches. Sensor updates use the preceding plant step: the runner ticks fake devices once,
then advances the plant and supplies the next pose. No device is double-stepped.

The fake external encoder is driven from **physical yaw**, never the controller's desired angle. Flywheel
assertions inspect physical shaft velocity. Gate position is commanded position, not measured travel.
Neither estimated homing nor hood tracking should be described as physical feedback.

The drive reduction is applied once when converting physical motor ticks/s into
wheel distance. SDK motor types retain bare-shaft 28 ticks/rev, 5800 RPM and internal
gearing 1:1. The model independently specifies tooth counts and catalog speed rather
than importing production conversion constants. The generic intake/transfer fake
retains its existing synthetic 5550 RPM response and the flywheel retains its own
observation. See [drivetrain units and calibration](Drivetrain.md).

## Camera convention boundary

The corner-rehome scenario uses a separate layout with red far tags 30–33 and
blue audience tags 38–41 nominally raised. It injects independent 25/20-mm chassis
placement error relative to the corner seed and tests whether camera/prior fusion
reduces that error after the turret settles. The original audience-only fixture,
finite turret response, measured encoder path, camera latency and production
calibration flags are unchanged. Both raw-camera success and inverted-camera
rejection/fallback remain tested; this does not establish physical accuracy.

SimLoop alpha.2 uses printed face normal **-Z** (into the print); `TagObservation` math uses outward **+Z**.
A physical frame from this robot (2026-11-05, raised Red far tags 30-33) showed the installed Limelight
also reports +Z into the print: raw facing scores were -0.96 to -0.99. `LimelightObservation` now applies
Rx(pi) once at the SDK boundary, so the raw SimLoop camera is the default fixture and localizes.
The physical Limelight is mounted upside down, and its per-tag 3D poses stay in that inverted sensor frame
(the web UI orientation rotates only the stream; physical frame 2026-11-13). `TargetNormalCameraAdapter`
therefore always converts SimLoop's upright camera into that sensor frame: Rz(pi) about the optical axis
(right and down negated, camera-side rotation Rz(pi) * R), which `LimelightObservation` undoes once. SimLoop
draws stickers upright, as the robot sees them (tag +X camera-right). With `physicalTagConvention=false` it
also inverts the normal (target-side Rx(pi)), which every OpMode must reject. 2D tx/ty pass through unchanged;
no OpMode aims from them.
The adapter exposes only the camera fields consumed by current robot code.

**Still pending:** physical pose accuracy. The same frame measured tag height about 0.91 m against the
mapped 1.268 m, and single-tag plane angles of about 49-72 deg; see
[LimelightGoalStateDiagnostics.md](LimelightGoalStateDiagnostics.md). Do not remove the inverted-camera
regression or move the conversion out of `LimelightObservation`.

Production receipt age now uses SDK `getStaleness()` plus capture/target latency; it does not subtract a
fake device's elapsed timestamp from SimLoop's fixed 2026 wall-clock epoch. Both ordinary epoch-equivalence
and geometric acquisition regressions exercise that boundary.

## To calibrate this model

Record revision, hardware/config changes, battery voltage, measurement method and data files. Measure
drive dimensions/ticks-per-distance and response, turret rate/settling, flywheel loaded response, gate
clearance/feed timing, camera intrinsics/latency and known-pose localization. Replace only corresponding
synthetic parameters, then reproduce independent robot traces. Real shot rows/release timing remain
separate production measurements. A faster synthetic plant or perfect feedback is never a bug fix.

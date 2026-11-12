# Drivetrain hardware and encoder units

Updated November 5, 2026 from the team's transmission description and drive-direction report.
Each mecanum wheel is driven by a [goBILDA 5000 Series bare motor, SKU 5000-0002-4008](https://www.gobilda.com/5000-series-12vdc-motor-with-8mm-rex-pinion-shaft/).
The manufacturer lists 28 quadrature encoder ticks per motor revolution, 6000 RPM
theoretical no-load speed and 5800 RPM tested no-load speed at 12 V. The configured
drivetrain baseline uses 5800 RPM; this is catalog evidence, not loaded robot data.

The wheels are the team's **104-mm goBILDA mecanums with 30A rollers**. goBILDA's
[30A GripForce roller pack](https://www.gobilda.com/gripforce-mecanum-wheel-roller-pack-30a/)
is specified for its 104-mm wheels. The nominal radius is 52 mm; hardness alone
does not establish loaded compression, traction or strafe efficiency.

The motor's 20-tooth gear drives an 80-tooth gear. The 80-tooth gear shares a shaft
with a 30-tooth pulley, which drives a 91-tooth pulley sharing the wheel shaft.
Coaxial parts add no further reduction.

| Quantity | Calculation | Value |
| --- | --- | --- |
| Gear reduction | 80 / 20 | 4 motor rev / intermediate rev |
| Belt reduction | 91 / 30 | 3.033333 intermediate rev / wheel rev |
| Total reduction | (80 / 20) * (91 / 30) | 12.133333 motor rev / wheel rev |
| Wheel encoder scale | 28 * 12.133333 | 339.733333 motor ticks / wheel rev |
| Wheel no-load estimate at 12 V | 5800 / 12.133333 | 478.021978 RPM |
| Wheel theoretical no-load speed | 6000 / 12.133333 | 494.505495 RPM |
| Nominal wheel circumference | pi * 104 mm | 326.725636 mm |
| Nominal encoder distance scale | 339.733333 / 326.725636 | 1.039812 motor ticks/mm |
| Nominal unloaded rolling-speed estimate | 478.021978 / 60 * 326.725636 | 2603.033913 mm/s |

`MecanumDrivetrain` owns the tooth counts and derived reduction, wheel encoder scale,
wheel speed estimate and signed motor-ticks/s-to-wheel-RPM conversion. All four
SDK motor types are cloned and describe the bare shaft: **28 ticks/rev, 5800 RPM,
internal gearing 1:1**. External gearing is accounted for at the wheel conversion;
do not also set the SDK ticks/rev to 339.733333 while retaining bare-shaft RPM.
`getCurrentPosition()` and `getVelocity()` remain motor ticks and motor ticks/s.
Telemetry labels those units and separately reports estimated wheel RPM.

The shared subsystem applies to both competition alliance wrappers, the isolated
drivetrain test, `PedroDrive` and all autonomous callers. TeleOp uses
`RUN_USING_ENCODER`; Pedro selects `RUN_WITHOUT_ENCODER` during INIT. BRAKE, motor
aliases and normalized power mixing retain their existing settings. The November 5
report that forward/backward, left/right and both turns were reversed is corrected
by swapping all four SDK motor directions: **left FORWARD, right REVERSE**. The
shared `LEFT_MOTOR_DIRECTION` / `RIGHT_MOTOR_DIRECTION` constants apply in both
encoder-controlled TeleOp and open-loop Pedro/autonomous, including direct wheel
commands. Joysticks keep the usual negative-Y-to-forward mapping; positive wheel
power means wheel-forward after the motor-direction correction. Use
**Test: Drivetrain Only** and its RB + X/Y/A/B wheel
checks to verify actual forward/strafe/turn signs after mechanical work. Without RB,
hold Y/A/X/B for a repeatable 30% forward/backward/strafe-left/strafe-right drive
to compare loaded wheel velocities. The gear
mesh reverses shaft rotation; the installed orientation determines each wheel's sign.

`MecanumDrivetrain` also owns the nominal wheel diameter, circumference, motor
ticks/mm and unloaded rolling-speed estimate. Actual chassis travel and loaded
speed depend on roller compression and slip. Pinpoint uses its own
odometry pods; their resolution and offsets do not change with drive gearing.
Pedro's Java/editor velocities and braking remain explicit unmeasured examples,
with `TUNING_VERIFIED = false`. Measure loaded forward/strafe speed and braking on
this transmission with the [Pedro web tuner](Autonomous.md#pedro-web-tuner) before
replacing those examples. Shooter and intake gearing
are separate mechanisms.

November 2 transmission simulation decision: use the isolated drive test, both actual competition wrappers
and the actual Pedro diagnostic to check motor-feedback-to-chassis conversion and
STOP. Run Drivetrain, Competition and Sensors scenarios because the shared plant's
distance scale changes. See [verification evidence](TeleOpVerificationMatrix.md).
The model independently specifies the reported tooth counts and catalog shaft
speed and **104-mm nominal wheel diameter**; motor ticks/in now equals
`28 * (80/20) * (91/30) / (pi*104/25.4) = 26.411232`, replacing the old arbitrary 64
and the former synthetic 2-in radius.
Track, wheelbase, strafe efficiency and response remain synthetic. This checks
software units and integration, not real traction, loaded speed or path accuracy.

November 5 direction simulation decision: run Drivetrain, Competition and Sensors,
including all six stick directions in the isolated drive test, both actual alliance
wrappers and the Pedro diagnostic. The physical model's shaft-to-wheel mounting
signs are now +,-,+,- (FL, FR, BL, BR), replacing the earlier assumed -,+,-,+ from
before the team's reversal report. These are explicit model inputs independent
of the production direction constants, not measured shaft data. Reproduce backward
travel with the old SDK directions before applying the fix. See the
[verification matrix](TeleOpVerificationMatrix.md) for results and model limits.

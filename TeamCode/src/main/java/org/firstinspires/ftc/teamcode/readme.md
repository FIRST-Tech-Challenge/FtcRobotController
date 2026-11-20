## TeamCode Module — BioBuzz 10100 (`V1`)

November 5 drive-direction correction: `MecanumDrivetrain` sets left motors FORWARD
and right motors REVERSE for both alliance TeleOps, wheel tests and Pedro/autonomous.
This swaps all six motions to correct the reported reversed driving controls.

November 4 software integration: [final TeleOp audit](../../../../../../../../doc/TeleOpConsistencyChecklist.md) records collection while aiming, GP2 auto offsets, independent status lights and the retained current controls. Bench tuning/interlocks remain explicitly documented diagnostic policies.

Competition field rehome: GP2 D-pad Left sets the alliance-specific corner pose
opposite its pollen depot, once per press in INIT or ACTIVE. Seat the rear/side
against the walls, front inward: Blue audience-right at 180 degrees, Red far-left
at 0 degrees. It then aims the Limelight and fuses five fresh, consistent frames
with the corner reference; keep still until telemetry reports VISION REFINED or
CORNER ONLY (three-second timeout). With the measured 55-degree Limelight the
raised tags are below the image from the corner, so expect CORNER ONLY.
GP2 X remains turret-only home.
See [placement and coordinates](../../../../../../../../doc/CompetitionTeleOp.md).

Final Red/Blue collection overrides: either controller's RB forces intake and
stage-1 loading forward past full; LB reverses intake only and pauses loading;
opposing bumpers stop collection. Release restores automatic collection. Gate
and shooting/aiming/inhibit priorities are unchanged.

Final Red/Blue feeding (2026-11-19): CONTINUOUS only for automatic and manual
feed; the staggered mode and its GP2 B toggle were removed. Continuous starts the transfer (both linked
motors) with the gate command, with no gate delay, timed window or recovery pause. The shared robot-tuned
timing (`ShootingConfiguration`) is unchanged and still used by autonomous, transfer-only,
shot-calibration and the bench test.

The combined intake/index/shooter bench test uses the competition continuous feed
policy, except that every feed opens the gate first and starts the transfer (both
motors) 0.10 s after that command, and the flywheel has no idle (it runs only
while RT is held). See `doc/IntakeShooterTest.md`.

See the repository `doc/HardwareReviewImplementationStatus.md` for the October 23 implementation, controls and calibration boundaries. `Test: Limelight Goal State 3D` initializes only the camera. Calibration displays session averages without saving files. See the current competition policy in `doc/CompetitionTeleOp.md`.

Turret rotation, hood, and depollinator servos are **Axon MAX+**; the gate is a **goBILDA 2000 Series Dual Mode Super Speed (2000-0025-0004)** in positional mode. References to Axon MAX mean MAX+. The rated maximum speed at **6.0 V is 0.115 s/60°** (about **521.74°/s** at the shaft; **260.87°/s** at the hood through its 2:1 linkage), per the [manufacturer specification](https://docs.axon-robotics.com/archive/max). `AxonMaxPlus` centralizes this rating and the robot's programmed 355° / 500–2500 us settings. Travel estimates are not physical position feedback; verify programmed range and loaded timing on the robot. The gate uses 300° travel over 500–2500 us: closed `0.5`, open `0.2` (90° CW viewed from the output shaft with REVERSE direction). Its [manufacturer specification](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/) rates 0.043 s/60° at 6.0 V (0.0645 s for 90°); the staggered delay is 0.10 s in every OpMode (competition, autonomous, bench and diagnostics). The gate servo is wired directly to Expansion Hub servo port 3, bypassing the 6 V power injector. It receives immediate full-position targets without software ramping for maximum available speed. The 0.043 s/60-degree rating at 6 V is catalog reference only; actual direct-hub loaded speed is unmeasured.

Welcome! This module (`TeamCode`) holds our team's robot code built on **FTC Robot Controller SDK v12.0**.

**Autonomous:** All autonomous entry points use Pedro Pathing v3 and Ivy through Zenith. [BioBuzzRobot.java](BioBuzzRobot.java) registers existing mechanisms as named commands. [PollenAuto.java](PollenAuto.java) is **Test: Pollen Navigation (Pedro + Ivy)**, a separate Test-group experiment using the C270 and Pedro paths/holds. [PollenVisionTestOpMode.java](PollenVisionTestOpMode.java) tests only the camera; [PedroDriveTestOpMode.java](PedroDriveTestOpMode.java) tests only the drive/Pinpoint subsystem. Physical tuning/calibration remains required. Read [autonomous setup](../../../../../../../../doc/Autonomous.md) and [pollen calibration](../../../../../../../../doc/PollenAuto.md).

- **Architecture & Best Practices**: Read [AGENTS.md](../../../../../../../../AGENTS.md) for FTC software best practices, including how to organize code into **Subsystem classes**, individual **Test OpModes**, and **Combined Competition OpModes**.

### Confirmed Control Hub Connections

| Device | Hardware-map name | Control Hub connection | Encoder cable |
| --- | --- | --- | --- |
| Front-left drivetrain | `leftfront` | Motor port 0 | Connected |
| Front-right drivetrain | `rightfront` | Motor port 1 | Connected |
| Back-left drivetrain | `leftback` | Motor port 2 | Connected |
| Back-right drivetrain | `rightback` | Motor port 3 | Connected |
| Pinpoint odometry | `pinpoint` | I2C port 0 | Two odometry pods connect to Pinpoint |

These are wiring assignments for the active Robot Configuration; Java selects each device by its hardware-map name.

Servo power is supplied by the [goBILDA 6V Servo Power Injector 3125-0001-0001](https://www.gobilda.com/6v-servo-power-injector-6-channel-8-15v-input/). It regulates 8–15 V input to 6 V, with up to 24 A total shared across six channels; the Expansion Hub provides the control signals.

Both the Limelight (`limelight`) and USB webcam (`webcam_1`) connect to the **same powered USB hub**, which plugs into the Control Hub. Rename the webcam entry in the active Robot Configuration to `webcam_1`.

| Expansion Hub servo port | Configuration name | Mechanism |
| --- | --- | --- |
| 0 | `turret_pitch` | Hood/pitch |
| 1 | `turret_servo_1` | Turret rotation 1 (locked pair with port 2: same direction, PWM range and position; identical Axon programmer settings required) |
| 2 | `turret_servo_2` | Turret rotation 2 (locked pair with port 1) |
| 3 | `gate_servo` | goBILDA Super Speed gate, 150 degrees closed / 60 degrees open (90 degrees CW with REVERSE direction) |
| 4 | `depollinator` | Axon MAX+, 30 degrees home / 165 degrees deployed |
| 5 | `slide_servo` (**CR Servo** entry) | The single Axon MAX+ in continuous mode driving the side linear slide; the shot blocker is permanently attached (no claw). Back on this port since 2026-11-20 |

| Other port | Configuration name | Mechanism |
| --- | --- | --- |
| Control Hub analog 1 | `slide_encoder` (Analog Input) | Slide Axon MAX+ analog feedback; its signal wire is the **white** (n+1) wire of the 0-1 connector, so configure port **1**, not 0. Turns counted from INIT (slide must start fully down) |


### Independent status-light connections

| Control Hub servo port | Configuration name | Role |
| --- | --- | --- |
| 3 | `led_intake` | Collection/full status |
| 4 | `led_shooter` | Preparation/readiness/feeding/inhibit |
| 5 | `led_status` | Turret home and field-reference availability |

Control Hub servos 0-2 are unused (the slide moved to Expansion Hub servo 5 on 2026-11-20; the claw was removed). Use positional Servo entries for the lights. Missing
lights are reported without preventing TeleOp. See the final TeleOp audit for
colors and the isolated `Test: Status Lights` controls.

### Current Implemented Code

Choose **Mecanum TeleOp - Red** or **Mecanum TeleOp - Blue** in the Competition group. Pipeline 0 can detect all IDs 30–45; competition selects the nearest cell of its alliance from odometry; drivers decide when that cell is raised. Nominal per-tag positions are documented, with periodic camera relocalization enabled. Visibility does not select a different opening or establish a scoring state. See [configuration and controls](../../../../../../../../doc/SoftwareConfiguration.md) for calibration dependencies, pose/vision timing, current-pose aiming, shared firing and autonomous routes.

| Alliance | Far / scoring-table goal | Audience goal |
| --- | --- | --- |
| Red | 30–33 | 34–37 |
| Blue | 42–45 | 38–41 |

Tag assignments follow [FIRST's BIOBUZZ manual, AprilTags section](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20V1.htm). Shot profiles, individual tag geometry and camera/pivot/muzzle offsets require measured configuration; individual tag centers are not the cell opening center.

1. **Layer 1 — Subsystems**:
   - **[DepollinatorSubsystem.java](DepollinatorSubsystem.java)**: Axon MAX+ on Expansion Hub servo port 4, named `depollinator`. Uses the existing Axon MAX+ assumption of 355-degree programmed travel and 500–2500 us PWM, with increasing PWM set to CW (servo must not be programmed inverted), viewed from the output shaft. Home/up is 30 degrees (`0.08451`); deploy is 165 degrees (`0.46479`), a 135-degree CW move. Homes during INIT. GP1 right-stick click deploys and left-stick click homes; commands are edge-triggered and hold their position after release. Home wins simultaneous presses. The servo retains its last commanded position when the OpMode ends.
   - **[TransferSubsystem.java](TransferSubsystem.java)**: Single-stage transfer since 2026-11-18: `transfer_motor_1` (Expansion Hub motor port 2) and `transfer_motor_2` (Expansion Hub motor port 3) are mechanically linked and both drive stage 1; the back rollers motor 2 used to drive as stage 2 were removed. Both are `FORWARD` (motor 2 was `REVERSE` while it drove the back rollers; a wrong direction makes the linked motors fight). Both use `BRAKE` and `RUN_WITHOUT_ENCODER`, and always receive the same command at full power for loading and shots. Motor 1 (goBILDA 5203 6000 RPM Yellow Jacket) has its own motor encoder on port 2, read for speed telemetry only. There is no stall limit or safety cut-off (the `TransferStallGuard` was removed 2026-11-19 by request): stage 1 always gets full open-loop power while loading and feeding, even if jammed, so watch the current telemetry and clear jams by hand. Stage 1 loads with the gate closed and feeds forward at full power during shots after gate opening. The goBILDA 3124-0001-0001 laser distance sensor is configured as a digital input named `distance_sensor` on Expansion Hub digital port 0. After START, the motor runs at full open-loop power until the digital signal stays HIGH continuously for 0.5 seconds, confirming full. Brief HIGH pulses do not stop it; LOW resets pending confirmation. Once full is confirmed, stage 1 runs on for 0.5 seconds to seat the fourth ball, then the transfer holds stopped while HIGH remains confirmed; LOW immediately resumes loading automatically. Pause, feed, and shutdown cancel the run-on. Uses a non-blocking timer, reports raw state and confirmation time, and stops on OpMode shutdown. Sensor placement and its potentiometer threshold must be set to indicate four balls loaded; software does not count balls. No gamepad binding is needed.
   - **[IntakeSubsystem.java](IntakeSubsystem.java)**: 1620 RPM goBILDA YellowJacket on Expansion Hub motor port 1, named `intake_motor`, with external 10T motor / 8T intake gearing. Uses `RUN_WITHOUT_ENCODER` and `BRAKE`. The intake has no encoder and no jam "jiggle" recovery; its encoder port 1 input carries the turret Through-Bore encoder, so the intake never resets that channel. GP1 right bumper runs forward at full power (motor CCW viewed facing the shaft), left bumper reverses, and both held stops. Releasing the bumpers restores automatic forward intake with space, or a stopped intake while HIGH remains confirmed. Turret feedback is read through `intake_motor`'s encoder input on port 1.
   - **[GateSubsystem.java](GateSubsystem.java)**: `gate_servo` on Expansion Hub servo port 3. goBILDA 2000 Series Dual Mode Super Speed (2000-0025-0004) in positional mode, 300-degree travel with 500–2500 us PWM. Closed/home is centered at 150 degrees (`0.5`); open is 60 degrees (`0.2`), giving 90 degrees CW with REVERSE direction viewed from the output shaft. Closes during INIT and shutdown; automatic/manual feeding opens it after readiness or override.
   - **[MecanumDrivetrain.java](MecanumDrivetrain.java)**: Four **goBILDA 5000-0002-4008 bare motors** driving 20T:80T gears, then coaxial 30T:91T belt pulleys (**12.133333 motor rev/wheel rev**). SDK settings stay in motor units (`28 ticks/rev`, `5800 RPM` catalog-tested no-load at 12 V, internal gearing `1:1`, `RUN_USING_ENCODER`, `BRAKE`); 104-mm goBILDA mecanum wheels with 30A rollers have `339.733333 motor ticks/rev` and a `478.02 RPM` no-load estimate. Supports config names `leftfront`/`rightfront`/`leftback`/`rightback` (as well as `front_left_drive`, etc.). Shared by both alliance TeleOps and Pedro/autonomous. See [hardware and units](../../../../../../../../doc/Drivetrain.md).
   - **[PinpointOdometry.java](PinpointOdometry.java)**: Plain Java subsystem for the **goBILDA Pinpoint Odometry Computer** and two **4-Bar Odometry Pods** (`goBILDA_4_BAR_POD`, `19.8944 ticks/mm`, X pod `EncoderDirection.FORWARD` and Y pod `EncoderDirection.REVERSED`, hand-push measured 2026-11-13):
     - Forward/Backward (X) pod: `24.80 + 14.40/2 = +32.00 mm` left of center (`xOffset = +32.00 mm`), `72.20 mm` back of center.
     - Left/Right Strafe (Y) pod: `81.20 - 14.40/2 = 74.00 mm` behind center (`yOffset = -74.00 mm`), `33.80 mm` right of center.
   - **[TurretSubsystem.java](TurretSubsystem.java)**: Plain Java subsystem for the Turret & Limelight 3A assembly:
     - **Rotation Servos (`"turret_servo_1"`, `"turret_servo_2"`)**: 2x Axon MAX+ in Servo Mode (`500–2500 us`), Expansion Hub Servo Ports 1 & 2 (via goBILDA 6V Servo Power Injector), `1:1` with turret rotation. Home (`0.5`) = robot heading `90°` (right). Headings increase clockwise: front `0°`, right `90°`, back `180°`, left `270°`; the left `265°..275°` zone is excluded. Internal servo-relative yaw is counterclockwise, `-175°..+175°` from home.
     - **Through-Bore Encoder**: REV Through-Bore Encoder V1 (`8192 CPR`) on Expansion Hub Quadrature Port 1, read through `intake_motor` (`turret_encoder` is a standalone configuration alias). The `108T Turret : 40T Encoder` gearing gives `2.7x` faster rotation, `22,118.4 ticks/turret rev` and `61.44 ticks/deg`. Homing captures a software zero without resetting the shared channel; feedback supplies measured angle/rate and bounded encoder correction.
     - **Pitch Servo (`"turret_pitch"`)**: 1x Axon MAX+ in Servo Mode on Expansion Hub Servo Port 0, linked `2:1` with hood pitch. Home position `0.573239` (13 hood degrees below the original servo 0.5 reference; the measured physical +15° position) is the new +15° command reference. Increasing PWM lowers elevation to -15° over 30° of hood travel (60° shaft travel, servo position approximately `0.742254`).
     - **Shooter Motor (`"turret_motor"`)**: 6000 RPM goBILDA 5203 Yellow Jacket on Expansion Hub Motor & Quadrature Port 0 (`28.0 ticks/rev`, `6000.0 configured RPM`, `RUN_USING_ENCODER`, `FLOAT`, 1:1 motor-to-flywheel drive).
     - **Limelight 3A (`"limelight"`)**: Connected to Control Hub USB 3.0 via powered USB hub, lens axis angled `55°` up from horizontal toward the goal (front glass 35° from horizontal; lens axis is perpendicular to the glass), lens height `337 mm` (`13.268 in`), pipeline `0`.
2. **Layer 2 — Standalone Mechanism Test OpModes**:
   - **[DepollinatorTestOpMode.java](DepollinatorTestOpMode.java)** (`Test: Depollinator Only`, group `Test`): Tests only `depollinator` with GP1 right-stick click to deploy and left-stick click to home.
   - **[LinearSlideTestOpMode.java](LinearSlideTestOpMode.java)** (`Test: Linear Slide`, group `Test`): Tests only `slide_servo` and `slide_encoder`. Start with the slide fully down. Same controls as competition (GP2 right-stick click = extend, left-stick click = home), plus GP2 left-stick Y manual slide power (half scale, no limits) and GP2 Back re-zero. Use it to re-measure `LinearSlideController.MAX_TRAVEL_DEG`.
   - **[LinearSlideSubsystem.java](LinearSlideSubsystem.java)** / **[LinearSlideController.java](LinearSlideController.java)**: Side linear slide, one Axon MAX+ (continuous) on Expansion Hub servo port 5 with feedback on Control Hub analog 1, with the shot blocker permanently attached. Counts Axon analog turns from INIT and drives to fully down (0) or fully up with proportional power; stall, wrong-direction and timeout faults cut power (press again to retry). No power during INIT. The INIT angle is fully down; fully up is 700 degrees (about 1.94 turns) counterclockwise viewed from the servo face, with the analog angle falling (`SERVO_DIRECTION = FORWARD`, `ENCODER_INVERTED = true`). Every INIT reading re-zeros it, so the down position is the last angle read before START.
   - **[TransferTestOpMode.java](TransferTestOpMode.java)** (`Test: Transfer Only`, group `Test`): Tests stage 1 automatic loading (both linked transfer motors) and the digital full sensor; both transfer motors and gate must be configured. Hold RB to request the shared timed gate-open feed cycle; release closes the gate and returns to loading. LB pauses loading and inhibits feed with priority. The flywheel is not initialized; exercise feeding empty first. After START, stage 1 loads until continuous HIGH for 0.5 seconds confirms full. Stage 1 runs on for 0.5 seconds after full confirmation, then holds while HIGH remains confirmed; LOW immediately resumes loading. Reports raw digital state, confirmation time and commanded power; stops on shutdown.
   - **[IntakeTestOpMode.java](IntakeTestOpMode.java)** (`Test: Intake Only`, group `Test`): Tests only `intake_motor` using GP1 right/left bumpers, without requiring other mechanisms.
   - **[GateTestOpMode.java](GateTestOpMode.java)** (`Test: Gate Only`, group `Test`): Tests only `gate_servo`; hold GP1 left trigger to open to 60 degrees (`0.2`, 90 degrees CW with REVERSE direction from home), release to close to 150 degrees (`0.5`).
   - **[DrivetrainTestOpMode.java](DrivetrainTestOpMode.java)** (`@TeleOp(name = "Test: Drivetrain Only", group = "Test")`): Tests only `MecanumDrivetrain` (joystick driving + `RB + X/Y/A/B` single-wheel test + `X/Y/A/B` fixed 30% forward/back/strafe drive).
   - **[PinpointTestOpMode.java](PinpointTestOpMode.java)** (`@TeleOp(name = "Test: Pinpoint Odometry Only", group = "Test")`): Tests only `PinpointOdometry` (push robot by hand to inspect pose and raw pod counts).
   - **[ServoZeroCalibrationOpMode.java](ServoZeroCalibrationOpMode.java)** (`Calibrate: DISCONNECTED Servo RAW`, group `Calibration`): Select ONE mechanically disconnected servo with D-pad Left/Right. After START, both bumpers toggle arming; hold RT plus A/X/Y for midpoint/0/1. No command is sent at INIT or START. Installed turret/hood, gate and depollinator use their bounded subsystem tests; coupled turret servos are tested together.
   - **[TurretTestOpMode.java](TurretTestOpMode.java)** (`@TeleOp(name = "Test: Turret", group = "Test")`): Manual turret rotation, homing, Through-Bore encoder trim, `0°–30°` downward hood travel (+15° to -15° launch elevation), and flywheel testing. Limelight and odometry are not initialized; no automatic aiming is performed.
   - **[AutoAimTestOpMode.java](AutoAimTestOpMode.java)** (`@TeleOp(name = "Test: Auto Aim", group = "Test")`): Limelight + Pinpoint + turret automatic aiming using the competition `ShotCoordinator`. GP1 `RT` aims, `RT + RB` also spins the flywheel, `A` rehomes, `B`/`X` selects Red/Blue, and competition odometry selects the nearest alliance cell. RS-X manually turns the turret with RT released. Push the chassis by hand; no drivetrain, intake, transfer, or gate initialization and no feeding. Uses competition `TELEOP_ODOMETRY` localization and encoder correction. A mapped raised tag acquires the field reference; calibrated hood/RPM still require measured profile/timing. Uncalibrated defaults permit odometry yaw only and keep feeding inhibited.
3. **Layer 3 — Combined TeleOp**:
   - **Competition intake and full queue**: Intake runs forward on LOW and stops (never reverses) on confirmed full (0.5 s HIGH). Either controller's bumpers override; opposing directions stop. After confirmation, stage 1 runs on for 0.5 seconds to seat the fourth ball, then the transfer (both motors) holds stopped while HIGH remains confirmed; LOW resumes loading automatically. The combined bench test and autonomous use the same full-queue intake stop.
   - **Automatic shot feeding**: GP1 RT prepares yaw, hood and RPM for the nearest alliance cell using field odometry; RT+LT requests automatic shooting. Drivers manage hive state; camera visibility does not gate competition feeding. Homing, shot-profile calibration, pose quality, yaw error within 3 degrees, hood and RPM readiness still apply. Chassis speed, turret speed and acceleration do not gate competition feeding. GP2 B selects continuous (startup) or staggered for automatic and manual feeding. Staggered feeds last 0.30 s with 0.10 s closed-gate recovery and stage 1 starts at the gate-open midpoint. Manual feeding bypasses shot readiness in both modes.
   - **Combined TeleOp controls**: Both alliance TeleOps use the blue loading-corner starting pose documented in `doc/CompetitionTeleOp.md`. GP1 sticks drive; stick clicks deploy/home the depollinator. GP2 RS-X commands linear turret speed with a measured-angle hold on release. GP2 X rehomes yaw. GP2 D-pad up/down adjusts hood by 0.5 degrees; Y/A adjusts manual flywheel RPM by 50, starting at 3500 and capped at 6000. GP1 RT prepares odometry aim, hood and calibrated RPM; RT+LT requests automatic shooting, and releasing either trigger cancels it. Releasing RT restores manual RPM. GP1 A or GP2 RT manually feeds; GP2 LT inhibits feeding. GP2 B toggles continuous/staggered feeding (starts continuous); D-pad Left explicitly rehomes the field pose and Right clears auto offsets. Nearest-cell selection follows field half automatically, with 5 cm of midline hysteresis. Limelight supplies periodic checked pose corrections using the nominal drawing map and documented mount. No drift or camera-loss feeding timeout is assumed. The turret-only test remains unchanged.

---

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""

Autonomous uses Zenith's Ivy runtime and Pedro v3. Author routines in canonical `autos/` through Zenith, then run the team `npm run zenith:deploy` workflow. Each normal source routine generates one full-robot OpMode in **Generated**; legacy red/blue Route Required wrappers are disabled. `PollenAuto` retains the sole Test-group collection entry and isolated hardware profile. See `doc/ZenithAuthoringGuide.md`, `doc/ZenithIntegrationAudit.md`, and `doc/Autonomous.md` at the repository root for command coverage, evidence and calibration requirements.

## Desktop verification

Optional major-change SimLoop checks and human keyboard review run from the repository root. See
[usage and replay](../../../../../../../../simloop/README.md) and
[audit / model limits](../../../../../../../../doc/SimLoopIntegrationAudit.md).
The simulations exercise actual test and competition OpModes; physical calibration remains separate.


November 5 competition gate update: Both final Red/Blue TeleOps start CONTINUOUS: once a feed is authorized, the gate-open command and the full-power transfer command (both linked transfer motors) occur together, without a gate delay. Automatic readiness and GP2 LT inhibit still apply; manual feed bypasses readiness. STAGGERED opens the gate, starts stage 1 at 0.10 s, and closes the gate after the shared 0.20 s feed (0.30 s total open window). Stage 1 keeps running while the gate is closed through the recovery and any subsequent readiness wait. At the next opening the transfer stops, then repeats the shared 0.10 s delay. Release, inhibit, invalid automatic solution, tracking loss or STOP cancels transfer feeding; tracking-loss recovery does not continue closed-gate feed. Mode changes reset the sequence. Autonomous and diagnostic defaults are unchanged.

package org.firstinspires.ftc.teamcode.bot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Every setting on the robot, with a name and its normal value.
 *
 * <p>A {@code 0.05} typed straight into the code is a bug waiting to happen.
 * {@code trgBtnThresh} is something a student can find and change.
 *
 * <p><b>How an OpMode uses it.</b> Make one, change only what this OpMode needs to be
 * different, then build the {@link Bot} with it:
 *
 * <pre>
 *     BotProperties properties = new BotProperties();
 *     properties.driveDisabled = true;       // this OpMode only tests the launcher
 *     properties.launcherStartRpm = 2000;    // and starts it slower than normal
 *     Bot bot = new Bot(this, properties);
 * </pre>
 *
 * <p>A change made in an OpMode only counts in that OpMode. To change a setting for every
 * OpMode, change its normal value in this file.
 *
 * <p>The Safe Mode settings at the bottom are locked: the same in every OpMode.
 *
 * <p>This file took over from types/Constants.java (docs/decisions/0010). The names are the
 * same, so {@code Constants.driveSlow} in an older note means {@code driveSlow} here.
 */
public class BotProperties {

    // ---- Which parts of the robot this OpMode uses ----------------------------------------
    // Normally all of them. Switch a part off to test another part alone. A part that is
    // off is never set up, so it does not even have to be in the robot configuration.

    /**
     * true: no driver control of the wheels. The Bot never touches the drive motors or the
     * heading sensor. Autos set this too: in an auto, Pedro Pathing drives the wheels.
     */
    public boolean driveDisabled = false;
    /** true: the Bot never touches the launcher motor. */
    public boolean launcherDisabled = false;
    /** true: the Bot never looks for a camera. Aim assist is off. */
    public boolean visionDisabled = false;
    /** true: the Bot never touches the indicator light. */
    public boolean indicatorDisabled = false;
    /** true: the Bot never touches the LED strip. */
    public boolean stripDisabled = false;

    // ---- Drive ------------------------------------------------------------------------------

    /** true: forward on the stick is always away from the driver. false: the way the robot faces. */
    public boolean fieldCentricDrive = false;
    /** How far a trigger has to be pulled before it counts as pressed. */
    public double trgBtnThresh = 0.05;
    /** Top drive power while holding the right trigger. */
    public double driveBoosted = 1;
    /** Top drive power with no trigger held. */
    public double driveNormal = 1;
    /** Top drive power while holding the left trigger, for careful lining up. */
    public double driveSlow = 0.25;
    /** Mecanum wheels strafe a bit slower than they drive, so strafe gets a small boost. */
    public double driveStrafeGain = 1.1;
    /** Most a drive motor's power may change in one loop. 0.2 means 0 to full in 5 loops. */
    public double driveMotorSlewRate = 0.2;
    /** Loops a button must be held before it counts. Filters out electrical noise. */
    public int debounce = 3;

    // ---- Drive hardware names (must match the robot configuration on the Driver Station) --

    public String cfgDriveFrontLeft = "leftFront";
    public String cfgDriveFrontRight = "rightFront";
    public String cfgDriveBackLeft = "leftBack";
    public String cfgDriveBackRight = "rightBack";
    public String cfgPinpoint = "pinpoint";
    /** The IMU inside the Control Hub. Every Control Hub configuration has it by default. */
    public String cfgHubImu = "imu";

    // ---- goBILDA Pinpoint odometry (tuned on the Decode robot) -------------------------
    // These only affect position tracking, not heading. Field-centric drive needs heading
    // alone, so a mistake here will not change how the robot drives. From Decode
    // MecanumDrive.PARAMS.inPerTick and PinpointLocalizer.PARAMS (lines 19-20).

    /** goBILDA 4-bar pod: 19.894 ticks per mm, the pod type the Pinpoint Tuner was told (2026-10-17). */
    public double pinpointInPerTick = 1 / (19.89436789 * 25.4);
    public double pinpointParYTicks = -1209.212244933345;
    public double pinpointPerpXTicks = -2661.37617633336;

    // ---- Control Hub IMU mounting (docs/decisions/0005) ---------------------------------
    // Which way the REV logo and the USB ports point on the robot. Wrong values make
    // turning read as tilting. Decode's robot: logo LEFT, USB UP (Decode Imu_Hub line 28).
    // Check this robot: if turning left does not raise Yaw on the Driver Station, fix these.

    public RevHubOrientationOnRobot.LogoFacingDirection hubLogoFacing =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public RevHubOrientationOnRobot.UsbFacingDirection hubUsbFacing =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // ---- Odometry pods, read by Pedro Pathing (docs/decisions/0008) ---------------------
    // With no Pinpoint, the two odometry pods plug straight into the hub's encoder jacks.
    // An encoder jack belongs to a motor port, so the "name" of a pod is the name of the
    // motor on that port. CHECK THESE ON THE ROBOT: they are a guess until someone looks.

    /** Motor port whose encoder jack has the pod that rolls when the robot drives FORWARD. */
    public String cfgForwardPodPort = "leftFront";
    /** Motor port whose encoder jack has the pod that rolls when the robot drives SIDEWAYS. */
    public String cfgStrafePodPort = "rightBack";

    // ---- Nectar launcher (docs/decisions/0009-launcher-velocity-control.md) ---------------
    // The launcher holds a SPEED (RPM), not a power. Each loop it measures the wheel's
    // speed and picks the power that gets it to the target. Starting numbers are Decode's
    // (Decode subsystems/Launcher.java lines 36-48, types/Constants.java lines 17-18).

    /** The launcher motor. Must match the robot configuration on the Driver Station. */
    public String cfgLaunchNectar = "nectarLauncher";
    /**
     * Encoder ticks for one turn of the launcher motor's shaft. 28 = a motor with the
     * encoder on the bare motor shaft (no gearbox), like Decode's. CHECK THIS: a geared
     * motor has a different number, and every RPM on the screen will be wrong.
     */
    public double launcherTicksPerRev = 28;
    /** Target speed when the OpMode starts. */
    public double launcherStartRpm = 1000;
    /** How much one shoulder-button press changes the target speed. */
    public double launcherRpmStep = 50;
    /** The target speed never goes above this. */
    public double launcherMaxRpm = 6000;
    /** How often the launcher measures its speed and picks a new power, in milliseconds. */
    public double launcherUpdateMs = 50;
    /** "At speed" means the measured RPM is within this much of the target. */
    public double launcherAtSpeedRpm = 50;

    // Speed control gains. Tune on the robot: kF first, then kP, then kI (see 0009).
    /** Feed forward: power per RPM of target. Does most of the work. 0.000242 x 2475 = 0.60. */
    public double launcherKF = 0.000242;
    /** Proportional: extra power per RPM the wheel is too slow. */
    public double launcherKP = 0.003;
    /**
     * Integral: extra power for being too slow for a long time. Starts at 0: Decode's was
     * 0.0001, but FTCLib capped it so hard it never added more than 0.0001 power. Raise it
     * only if the wheel settles a little below the target and stays there.
     */
    public double launcherKI = 0;

    // ---- Vision: AprilTags and aim assist (docs/decisions/0011) ----------------------------
    // The Bot looks for a Limelight first, then a webcam. With neither, aim assist is off
    // and everything else still works.

    /** The Limelight 3A, if the robot has one. Configure it as a "Limelight 3A". */
    public String cfgLimelight = "limelight";
    /** The USB webcam, if the robot has one instead. Configure it as a "Webcam". */
    public String cfgWebcam = "Webcam 1";
    /** How many times a second the Limelight is asked for a new result. */
    public int limelightPollRateHz = 100;
    /** Which of the Limelight's pipelines (0 to 9) is set up for AprilTags. */
    public int limelightAprilTagPipeline = 0;
    /** A tag last seen longer ago than this is treated as gone. Keeps the robot from chasing a ghost. */
    public double visionMaxAgeMs = 250;

    // Aim assist: hold the left bumper and the robot turns to face the nearest tag.
    /**
     * Turn power per degree the tag is off center. 0.02 x 20 degrees = 0.4 power.
     * CHECK ON THE ROBOT: if the robot turns away from the tag, make this negative.
     */
    public double aimKP = 0.02;
    /** Aim assist never turns harder than this, either way. */
    public double aimMaxRotate = 0.6;
    /** Within this many degrees of center, aim assist stops turning. Stops the wobble. */
    public double aimDoneDegrees = 1.0;

    // ---- Indicator light (docs/decisions/0015) ---------------------------------------------
    // A goBILDA RGB indicator light on a servo port. Each color is a servo position, from
    // goBILDA's table: 0 off, 0.28 red, 0.333 orange, 0.388 yellow, 0.5 green, 0.611 blue,
    // 0.722 violet, 1 white.

    /** The light. Configure it as a "Servo" on the servo port it is plugged into. */
    public String cfgIndicator = "led";
    /** Three or more tags in view. */
    public double indicatorGreen = 0.5;
    /** Two tags in view. One tag in view blinks this color. */
    public double indicatorBlue = 0.611;
    /** No tag in view. */
    public double indicatorRed = 0.280;
    /** The light off: before START, and the dark half of a blink. */
    public double indicatorOff = 0;
    /** With one tag in view the light blinks: this long on, this long off, in milliseconds. */
    public double indicatorBlinkMs = 300;

    // ---- LED strip: goBILDA Prism RGB LED driver, over I2C (docs/decisions/0017) ---------

    /** The Prism driver: type "goBILDA Prism LED Driver" on an I2C port the Pinpoint is not on. */
    public String cfgStrip = "strip";
    /** How many LEDs are on the strip. One 200 mm Prism strip has 12. Chained strips add up. */
    public int stripLedCount = 12;
    /** Strip brightness, 0 to 100. Start low; 100 is very bright and warm. */
    public int stripBrightness = 50;

    // ---- Safe Mode: LOCKED (docs/decisions/0004 and 0007) --------------------------------
    // Safe Mode protects the people standing next to the robot, so these three are not
    // per-OpMode settings. "static final" means: one value, shared by every OpMode, that
    // can't be changed while the program runs. Writing
    // "properties.safeModeMaxPower = 1;" in an OpMode will not compile.
    //
    // The key is checked once at INIT. "Remove Before Flight": key in = Safe Mode, key
    // removed = full speed.
    // NOW (bench testing): the touch sensor "liftHomeSwitch". Pressed = pretend the key is
    //   removed (full speed). Not pressed = key in (Safe Mode).
    // LATER (the real key): REV Magnetic Limit Switch. Change the name below to the magnetic
    //   switch's name, and change keyPressedMeansKeyIn to true.

    /**
     * TEMPORARY, 2026-10-17: Safe Mode is switched OFF while Pedro's Foresight numbers are
     * tested at the speed they were measured at. The robot runs at full speed, key or no
     * key, and the Driver Station says so in capitals. Set this back to true when the
     * autos are proven (docs/decisions/0014).
     */
    public boolean safeModeEnabled = false;
    /** Fastest any drive motor may go in Safe Mode. 0.25 is one quarter of full power. */
    public static final double safeModeMaxPower = 0.25;
    /** The key sensor. Configure it as a "REV Touch Sensor" (the magnetic switch too). */
    public static final String cfgSafeModeKey = "liftHomeSwitch";
    /**
     * true: the sensor reads "pressed" when the key is IN (magnetic switch, magnet near).
     * false: "pressed" means the key is REMOVED (touch sensor stand-in for testing).
     */
    public static final boolean keyPressedMeansKeyIn = false;
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Auto - Shoot and Park, Alpha 1
 * Shoots our preloaded POLLEN at the hive, then drives into our LOADING ZONE (rungs A-3 + A-1 in
 * TeamCode/docs/biobuzz-auto-strategy.md). It is AutoLeaveAndPark plus a firing sequence.
 *
 *   1. FIND_HIVE         : the turret turns until the camera is locked on the hive, and the
 *                          flywheel starts spinning at PRE_REV_FLYWHEEL_RPM so it is already
 *                          warm. As soon as the camera is locked, the shot RPM comes from the
 *                          shot table (from ty) and is kept.
 *   2. WAIT_FOR_SPEED    : the FLYWHEEL gets to the target RPM and stays there, and the turret is
 *                          on target.
 *   3. FEED_BALLS        : the gate goes CLOSED -> OPENED and the INTAKE feeds balls up the chute,
 *                          but only while the flywheel is at speed. Each ball slows the flywheel
 *                          for a moment, so the intake pauses until the flywheel recovers. Balls
 *                          go one at a time. This lasts FEED_SECONDS.
 *   4. LIFT_LAST_BALL    : the gate goes to LIFTED (it pushes the last ball in) with the intake
 *                          still running, for GATE_LIFT_SECONDS. Then the gate goes back to CLOSED,
 *                          and the intake and shooter stop.
 *   5. DRIVE_TO_WAYPOINT : slide to the right of the hive (so we do not clip it).
 *   6. DRIVE_TO_PARK     : drive down the field into the LOADING ZONE.
 *   7. DONE              : the robot holds its spot.
 *
 * IF THE SHOT CANNOT HAPPEN, SKIP IT AND PARK (we never give up the park points):
 *   - the camera is not locked on the hive after LOCK_TIMEOUT_SECONDS, or
 *   - the flywheel is not at speed after SPIN_UP_TIMEOUT_SECONDS.
 *   The screen says which one happened ("Shot skipped").
 *
 * WHY "AT SPEED" CHECKS ONLY THE FLYWHEEL (a change from Alpha 1's first test)
 *   ShooterSubsystem.isAtSpeed() needs BOTH wheels within 25 RPM, and in the first robot test it
 *   only came true after about 4 seconds. The hood wheel has far fewer encoder counts per turn
 *   (28, against the flywheel's 103.8) and runs about twice as fast, so its speed reading is a
 *   lot noisier. Here we check ONLY the flywheel (the heavy wheel, so a steady reading) and
 *   assume the hood wheel, which is set by the same ratio, is close enough. The hood wheel's
 *   error is still shown on the screen so we can check that. ShooterSubsystem and the TeleOp are
 *   NOT changed: they still use both wheels.
 *
 * THE INTAKE FEEDS THE BALLS
 *   One motor runs the intake AND the transfer chute (the TeleOp's Right Bumper "pulls balls in
 *   and carries them up the chute"). In the first robot test the gate moved but nothing pushed the
 *   balls, because the intake was off. Now it runs during the shot.
 *
 * WHAT THIS OPMODE DOES NOT DO
 *   - It does not move while shooting. The robot shoots from its start spot.
 *   - It cannot count balls (no sensor). FEED_SECONDS is a time. The screen shows how many times
 *     the flywheel dipped during the feed, which is about one dip per ball shot.
 *
 * Pedro's Follower owns the drive motors and the Pinpoint, so this OpMode does NOT use
 * DriveSubsystem or PinpointSubsystem. The shooter, gate, intake, turret, camera and shot table
 * are the same classes the TeleOp uses.
 *
 * WHERE THE ROBOT STARTS (BLUE, the default)   (all the numbers are in pedro/FieldConstants.java)
 *   Tile D6, in the middle of the tile, with the robot's BACK against the TOP wall, facing DOWN
 *   the field toward the hive (the intake and the turret point at the hive). Measure from the
 *   LEFT wall to the middle of the robot: about 82.5 inches.
 *   The robot must not touch a flower, no part of it may be in the LOADING ZONE, and it must hold
 *   exactly 4 POLLEN.
 *   RED (press B in INIT) is the same plan turned 180 degrees: tile C1, back against the BOTTOM
 *   wall, facing UP. Red has not been driven yet, so check it slowly the first time.
 *
 * BEFORE YOU RUN IT (once per robot)
 *   - The turret must have been pointing STRAIGHT AHEAD when the robot was powered on (the turret
 *     home switch is off in code, so 0 degrees is where it pointed at power-up), and it should be
 *     pointing roughly straight ahead now.
 *   - The Limelight needs about 30 seconds after power-up before it sees anything.
 *   - The driver station config must have the "limelight" Ethernet device (like the TeleOp).
 *   - To drive, Pedro must be tuned (TeamCode/docs/pedro-setup-and-auto-guide.md) and
 *     FORESIGHT_TUNED set to true. If it is NOT tuned, this OpMode still does the SHOT but does
 *     not drive, so you can test the firing sequence on its own. (Until Pedro is tuned, use
 *     AutoEmergencyLeave to get the LEAVE points.)
 *   - Test the shot with the robot on its start spot before you trust the park: check that the
 *     camera sees the hive from there (see the INIT screen), and that the ty on the screen is
 *     near a row of the shot table.
 *
 * HOW TO RUN IT
 *   1. Put the robot on its start spot, with 4 balls loaded, and leave it STILL (INIT
 *      recalibrates the Pinpoint's gyro).
 *   2. Select "Auto - Shoot and Park - Alpha 1" and press INIT. Wait for "Pinpoint ready".
 *   3. Check the INIT screen: ALLIANCE (gamepad1 X = BLUE, B = RED), camera connected, "Hive
 *      visible: YES" and the table RPM for the start spot.
 *      TIP: type that "Table RPM from here" number into PRE_REV_FLYWHEEL_RPM, so the flywheel
 *      never has to change its target when the camera locks.
 *   4. Press START. (The 30 second timer should be ON, like in a match.)
 *
 * WHAT TO LOOK AT AFTER A RUN (the "Run log" lines stay on the screen)
 *   When the camera locked, when the flywheel was ready, how far each wheel was from its target at
 *   that moment, the ty and the table RPM, and how many flywheel dips happened during the feed.
 *   Write them in TeamCode/docs/biobuzz-test-and-match-log.md.
 *
 * WHAT TO TUNE (all at the top of this file, change one at a time)
 *   FEED_SECONDS and GATE_LIFT_SECONDS: practice until the balls leave the way you want, then
 *   leave them alone for matches. INTAKE_WAITS_FOR_SPEED (false = the intake runs the whole feed,
 *   useful to check that the balls move at all). PRE_REV_FLYWHEEL_RPM, LOCK_TIMEOUT_SECONDS,
 *   SPIN_UP_TIMEOUT_SECONDS, FLYWHEEL_AT_SPEED_TOLERANCE_RPM.
 *
 * Run it 5 times in a row before you trust it, and write each run in
 * TeamCode/docs/biobuzz-test-and-match-log.md. Once it is solid, change group = "Testing" to
 * "Competition" below.
 */
@Autonomous(name = "Auto - Shoot and Park - Alpha 1", group = "Testing",
        preselectTeleOp = "TeleOp - Alpha 14")
public class ShootAndParkAuto extends OpMode {

    // ------------------------------------------------------------------
    // Settings (change these to change the Auto)
    // ------------------------------------------------------------------
    // How long the gate stays OPENED while the intake feeds balls, in seconds. The intake only
    // runs while the flywheel is at speed, so this is the time the feed is ALLOWED to run, not the
    // time the intake is on. Four balls with a recovery after each one take a few seconds.
    // Practice with this until all 4 balls go, then keep it the same for matches.
    private static final double FEED_SECONDS = 3.0;

    // How long the gate stays LIFTED (with the intake still running) to push the last ball in.
    private static final double GATE_LIFT_SECONDS = 0.5;

    // true  = the intake runs only while the flywheel is at speed (balls go one at a time).
    // false = the intake runs the whole feed no matter what (to check the balls move at all).
    private static final boolean INTAKE_WAITS_FOR_SPEED = true;

    // The flywheel speed to warm up at while the camera is still finding the hive. It is the same
    // number the TeleOp's speed dial starts at. The shot table's RPM takes over as soon as the
    // camera is locked. Best: set it to the "Table RPM from here" number the INIT screen shows at
    // the start spot, so the target does not change. It must be 1458 or lower (ShooterSubsystem
    // cuts it there).
    private static final double PRE_REV_FLYWHEEL_RPM = 1300.0;

    // How long to wait for the camera to lock onto the hive before skipping the shot.
    private static final double LOCK_TIMEOUT_SECONDS = 3.0;

    // How long to wait for the flywheel to reach speed (after the camera locks) before skipping.
    // The first robot test needed about 4 seconds, so this is 6 to leave room.
    private static final double SPIN_UP_TIMEOUT_SECONDS = 6.0;

    // The flywheel counts as "at speed" when it is within this many RPM of its target, and stays
    // there for FLYWHEEL_AT_SPEED_HOLD_SECONDS. (The same two numbers ShooterSubsystem uses for
    // BOTH wheels, but here only the flywheel is checked.)
    private static final double FLYWHEEL_AT_SPEED_TOLERANCE_RPM = 25.0;
    private static final double FLYWHEEL_AT_SPEED_HOLD_SECONDS = 0.15;

    // The most time one drive step is allowed to take. (Same idea as in AutoLeaveAndPark.)
    private static final double LEG_TIMEOUT_SECONDS = 6.0;

    // Same numbers as BiobuzzTeleOp's auto-align. We only pick a new turret target when the camera
    // says we are more than this far off, so the turret can settle (degrees).
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;

    // The camera counts as LOCKED when the hive's tx is this small (degrees) and the turret says
    // it is on target.
    private static final double LOCKED_TOLERANCE_DEGREES = 2.0;

    // ------------------------------------------------------------------
    // The steps of this Auto. The robot is always in exactly one of these.
    // ------------------------------------------------------------------
    private enum AutoState {
        FIND_HIVE,
        WAIT_FOR_SPEED,
        FEED_BALLS,
        LIFT_LAST_BALL,
        DRIVE_TO_WAYPOINT,
        DRIVE_TO_PARK,
        DONE
    }

    // ------------------------------------------------------------------
    // Things this OpMode uses
    // ------------------------------------------------------------------
    private Follower follower;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShotTable shotTable;

    private VisionSubsystem.Alliance alliance = VisionSubsystem.Alliance.BLUE;

    private Pose startPose;
    private Pose waypointPose;
    private Pose parkPose;

    // The two paths are built when the alliance is chosen (INIT), never inside loop().
    private Path waypointPath;
    private Path parkPath;

    private AutoState autoState = AutoState.DONE;

    // Times the current step (waiting for the camera, the flywheel, the feed, or the lift).
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Times one drive step, so a step that never settles cannot use up the whole Auto.
    private final ElapsedTime legTimer = new ElapsedTime();

    // The flywheel RPM the shot table gave us when the camera locked. It is saved so the target
    // does not wobble with the camera's ty while the flywheel is getting to speed.
    private double shotFlywheelRpm = 0.0;

    // Why the shot was skipped, or "" if it was not. Shown on the screen.
    private String skipReason = "";

    // The flywheel-only "at speed" check. flywheelReady is worked out once per loop, after the
    // shooter has read its encoders. The two helpers below are only for working it out.
    private boolean flywheelReady = false;
    private boolean flywheelWasInTolerance = false;
    private final ElapsedTime flywheelHoldTimer = new ElapsedTime();

    // The "Run log": what happened, so one robot test tells us where the time went. -1 means
    // "has not happened (yet)".
    private final ElapsedTime runTimer = new ElapsedTime();
    private double lockedAtSeconds = -1.0;
    private double lockedTyDegrees = 0.0;
    private double readyAtSeconds = -1.0;
    private double flywheelErrorAtReadyRpm = 0.0;   // actual minus target, so negative = too slow
    private double hoodWheelErrorAtReadyRpm = 0.0;
    private int flywheelDipCount = 0;               // about one per ball shot
    private boolean feedWasReady = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Create the Follower. This recalibrates the Pinpoint, so keep the robot still.
        follower = Constants.create(hardwareMap);

        // Each subsystem gets its own hardware from the hardware map.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate moves to CLOSED here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        // The intake starts stopped.
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        // The turret does not move at INIT. It only starts reading its encoder.
        turretSubsystem = new TurretSubsystem(hardwareMap);
        // The Limelight starts polling here. It needs about 30 seconds to boot after power-up.
        visionSubsystem = new VisionSubsystem(hardwareMap);
        shotTable = new ShotTable();

        // BLUE is the default. This builds the poses and paths and tells Pedro where we start.
        applyAlliance(VisionSubsystem.Alliance.BLUE);

        telemetry.addLine("Pinpoint ready. Keep the robot still, then press START.");
        addStatusTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The Follower is idle here, so this only reads the Pinpoint and keeps the wheels stopped.
        follower.update();

        // Read the camera now, so we can check before START that it sees the hive.
        visionSubsystem.update();

        // gamepad1 X picks BLUE, gamepad1 B picks RED. Nothing happens if it is already chosen.
        if (gamepad1.x && alliance != VisionSubsystem.Alliance.BLUE) {
            applyAlliance(VisionSubsystem.Alliance.BLUE);
        } else if (gamepad1.b && alliance != VisionSubsystem.Alliance.RED) {
            applyAlliance(VisionSubsystem.Alliance.RED);
        }

        addStatusTelemetry();
        addPoseTelemetry();
        addInitCameraTelemetry();

        // The turret is NOT driven during INIT (the rules say nothing may move). We only read it.
        turretSubsystem.update();
        turretSubsystem.addTelemetry(telemetry);
        shooterServoSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // The rules say the robot has not moved since INIT, so it is on its start spot. Tell
        // Pedro that again, in case the robot was nudged while the drive team was setting up.
        follower.setPose(startPose);
        follower.update();

        // Start the run log over.
        runTimer.reset();
        lockedAtSeconds = -1.0;
        readyAtSeconds = -1.0;
        flywheelDipCount = 0;
        flywheelReady = false;
        flywheelWasInTolerance = false;

        // Start looking for the hive and warming up the flywheel right away.
        skipReason = "";
        autoState = AutoState.FIND_HIVE;
        stateTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. Pedro reads the Pinpoint and sets the wheel powers. It must run
        //    every loop, or the robot does not move (or stops steering).
        follower.update();
        visionSubsystem.update();

        // 2. Decide what to do next, and tell the subsystems. (flywheelReady is from the end of
        //    the last loop, which is only 20 milliseconds old.)
        switch (autoState) {
            case FIND_HIVE:
                // Warm the flywheel up while we look. The turret turns until the camera is centered
                // on the hive.
                shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);
                aimTurretAtHive();

                if (isHiveLocked()) {
                    // Locked. Ask the shot table how fast to shoot from here, and keep that number.
                    lockedAtSeconds = runTimer.seconds();
                    lockedTyDegrees = visionSubsystem.getTyDegrees();
                    shotFlywheelRpm = shotTable.getFlywheelRpm(lockedTyDegrees);
                    shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                    stateTimer.reset();
                    autoState = AutoState.WAIT_FOR_SPEED;
                } else if (stateTimer.seconds() >= LOCK_TIMEOUT_SECONDS) {
                    skipShotAndPark("the camera never locked onto the hive");
                }
                break;

            case WAIT_FOR_SPEED:
                // Same number every loop is safe: ShooterSubsystem ignores a repeat request.
                shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                aimTurretAtHive();

                // The firing gate: the flywheel at speed (and held there) AND the turret on target.
                if (flywheelReady && turretSubsystem.isOnTarget()) {
                    // Write down how the wheels looked at this moment, for the run log.
                    readyAtSeconds = runTimer.seconds();
                    flywheelErrorAtReadyRpm =
                            shooterSubsystem.getFlywheelRpm() - shooterSubsystem.getTargetFlywheelRpm();
                    hoodWheelErrorAtReadyRpm =
                            shooterSubsystem.getHoodWheelRpm() - shooterSubsystem.getTargetHoodWheelRpm();

                    // Open the gate and start feeding.
                    shooterServoSubsystem.goToOpened();
                    feedWasReady = true;
                    stateTimer.reset();
                    autoState = AutoState.FEED_BALLS;
                } else if (stateTimer.seconds() >= SPIN_UP_TIMEOUT_SECONDS) {
                    skipShotAndPark("the flywheel never reached speed");
                }
                break;

            case FEED_BALLS:
                // The gate is OPENED. Feed one ball at a time: the intake runs only while the
                // flywheel is at speed. (This is the same rule ShotTableAlpha uses.)
                if (flywheelReady || !INTAKE_WAITS_FOR_SPEED) {
                    intakeSubsystem.intake();
                } else {
                    intakeSubsystem.stop();
                }

                // Every time the flywheel drops out of "at speed", a ball has just gone through.
                if (feedWasReady && !flywheelReady) {
                    flywheelDipCount = flywheelDipCount + 1;
                }
                feedWasReady = flywheelReady;

                if (stateTimer.seconds() >= FEED_SECONDS) {
                    shooterServoSubsystem.goToLifted();
                    stateTimer.reset();
                    autoState = AutoState.LIFT_LAST_BALL;
                }
                break;

            case LIFT_LAST_BALL:
                // The gate is LIFTED to push the last ball in. The intake keeps running, whether
                // or not the flywheel has recovered, so the last ball is not left behind.
                intakeSubsystem.intake();

                if (stateTimer.seconds() >= GATE_LIFT_SECONDS) {
                    shooterServoSubsystem.goToClosed();
                    intakeSubsystem.stop();
                    shooterSubsystem.stop();
                    startDriving();
                }
                break;

            case DRIVE_TO_WAYPOINT:
                // isBusy() stays true until the robot has arrived and settled.
                if (!follower.isBusy() || legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
                    legTimer.reset();
                    follower.follow(parkPath);
                    autoState = AutoState.DRIVE_TO_PARK;
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy() || legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                // Nothing left to do. The Follower keeps holding the last pose.
                break;
        }

        // The subsystems need update() once per loop, AFTER they get their commands. Then work out
        // the flywheel-only "at speed" answer from the fresh reading.
        shooterSubsystem.update();
        turretSubsystem.update();
        intakeSubsystem.update();
        updateFlywheelReady();

        // 3. Telemetry
        addStatusTelemetry();
        telemetry.addData("State", autoState);
        if (!skipReason.isEmpty()) {
            telemetry.addData("Shot skipped because", skipReason);
        }
        addShotTelemetry();
        addRunLogTelemetry();
        addPoseTelemetry();
        telemetry.addData("Distance to end of path (in)", "%.2f", follower.distanceToEndpoint());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Put the Follower in IDLE and tell the drivetrain to stop right now. (follower.stop()
        // only changes the mode. The wheels are zeroed by the next update(), and there is none.)
        follower.stop();
        follower.drivetrain.stop();

        shooterSubsystem.stop();
        intakeSubsystem.stop();
        // The turret floats. The hub keeps its encoder count, so the TeleOp knows its angle.
        turretSubsystem.stop();
        visionSubsystem.stop();
    }

    // ------------------------------------------------------------------
    // The steps
    // ------------------------------------------------------------------
    /*
     * Turns the turret toward the hive. Call it every loop while we are looking and waiting.
     * It is the TeleOp's auto-align without the "follow the robot's spin" part, because the robot
     * does not move while it shoots.
     */
    private void aimTurretAtHive() {
        if (!visionSubsystem.hasTarget()) {
            // The camera does not see the hive. Leave the turret where it is.
            return;
        }

        double txDegrees = visionSubsystem.getTxDegrees();

        // Pick a new target if the turret is not in POSITION mode yet (the align just started),
        // or if the camera says we are far enough off. Picking a new target every loop would keep
        // the turret from ever settling.
        boolean notAimingYet = turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION;
        boolean farOff = Math.abs(txDegrees) > ALIGN_REAIM_THRESHOLD_DEGREES;
        if (notAimingYet || farOff) {
            // tx is positive to the RIGHT, but turret angles are positive to the LEFT, so subtract.
            double newTargetAngle = turretSubsystem.getAngleDegrees() - txDegrees;
            turretSubsystem.setTargetAngle(newTargetAngle);
        }
    }

    /*
     * True when the camera sees the hive, the hive is nearly centered in the view, and the turret
     * says it is on target.
     */
    private boolean isHiveLocked() {
        return visionSubsystem.hasTarget()
                && Math.abs(visionSubsystem.getTxDegrees()) <= LOCKED_TOLERANCE_DEGREES
                && turretSubsystem.isOnTarget();
    }

    /*
     * Works out flywheelReady: true when the FLYWHEEL (only) has been within
     * FLYWHEEL_AT_SPEED_TOLERANCE_RPM of its target for FLYWHEEL_AT_SPEED_HOLD_SECONDS. Call it
     * once per loop, right after shooterSubsystem.update(). One reading outside the band drops it
     * at once, so after a ball slows the flywheel the intake pauses until it recovers.
     */
    private void updateFlywheelReady() {
        double targetRpm = shooterSubsystem.getTargetFlywheelRpm();
        double errorRpm = Math.abs(shooterSubsystem.getFlywheelRpm() - targetRpm);

        // A target of 0 means the shooter is stopped, so it can never be "at speed".
        boolean inTolerance = targetRpm > 0.0 && errorRpm <= FLYWHEEL_AT_SPEED_TOLERANCE_RPM;

        if (!inTolerance) {
            flywheelWasInTolerance = false;
            flywheelReady = false;
            return;
        }

        // Inside the band. Start the hold timer the first time, then wait out the hold time.
        if (!flywheelWasInTolerance) {
            flywheelHoldTimer.reset();
            flywheelWasInTolerance = true;
        }
        flywheelReady = flywheelHoldTimer.seconds() >= FLYWHEEL_AT_SPEED_HOLD_SECONDS;
    }

    /*
     * Give up on the shot (and say why), stop the shooter and intake, and go park.
     */
    private void skipShotAndPark(String reason) {
        skipReason = reason;
        shooterSubsystem.stop();
        intakeSubsystem.stop();
        shooterServoSubsystem.goToClosed();
        startDriving();
    }

    /*
     * The shooting part is over (or skipped). Start the first drive step.
     */
    private void startDriving() {
        // Hold the turret where it is while we drive, so it does not swing around.
        turretSubsystem.holdCurrentAngle();

        if (!Constants.FORESIGHT_TUNED) {
            // Do not drive on another robot's braking numbers. The shot (if any) is already done.
            autoState = AutoState.DONE;
            return;
        }

        legTimer.reset();
        follower.follow(waypointPath);
        autoState = AutoState.DRIVE_TO_WAYPOINT;
    }

    // ------------------------------------------------------------------
    // Alliance
    // ------------------------------------------------------------------
    /*
     * Picks the alliance: tells the camera which hive is ours, builds the three poses and the two
     * paths, and tells Pedro where the robot is standing. Only call it in INIT, never in loop().
     */
    private void applyAlliance(VisionSubsystem.Alliance newAlliance) {
        alliance = newAlliance;
        visionSubsystem.setAlliance(alliance);

        startPose = FieldConstants.startPose(alliance);
        waypointPose = FieldConstants.waypointPose(alliance);
        parkPose = FieldConstants.parkPose(alliance);

        // A path is a line between poses. ".constant(startPose)" means: keep facing the heading
        // of startPose the whole way, so the robot never turns. (It slides sideways instead.)
        waypointPath = Paths.line(startPose, waypointPose).constant(startPose);
        parkPath = Paths.line(waypointPose, parkPose).constant(startPose);

        // Tell Pedro where the robot is standing right now.
        follower.setPose(startPose);
    }

    // ------------------------------------------------------------------
    // Telemetry helpers
    // ------------------------------------------------------------------
    private void addStatusTelemetry() {
        telemetry.addData("ALLIANCE (gamepad1 X = BLUE, B = RED)", alliance);
        if (alliance == VisionSubsystem.Alliance.BLUE) {
            telemetry.addLine("Start: tile D6, back to the TOP wall, facing DOWN");
        } else {
            telemetry.addLine("Start: tile C1, back to the BOTTOM wall, facing UP");
        }

        if (Constants.FORESIGHT_TUNED) {
            telemetry.addData("Foresight", "tuned, OK to drive");
        } else {
            telemetry.addLine("!!! NOT TUNED: this OpMode will shoot but NOT drive. !!!");
            telemetry.addLine("Tune Foresight (see the Pedro guide), paste the numbers into");
            telemetry.addLine("pedro/Constants.java, then set FORESIGHT_TUNED = true.");
        }
    }

    // Checks to read BEFORE START: does the camera see the hive from the start spot, and what
    // would the shot table say?
    private void addInitCameraTelemetry() {
        telemetry.addData("Camera connected", visionSubsystem.isCameraConnected());
        if (visionSubsystem.hasTarget()) {
            double tyDegrees = visionSubsystem.getTyDegrees();
            telemetry.addData("Hive visible", "YES (ty %.1f)", tyDegrees);
            telemetry.addData("Table RPM from here", "%.0f  (PRE_REV_FLYWHEEL_RPM is %.0f)",
                    shotTable.getFlywheelRpm(tyDegrees), PRE_REV_FLYWHEEL_RPM);
            if (!shotTable.isInRange(tyDegrees)) {
                telemetry.addLine("Warning: ty is OUTSIDE the shot table. It uses the nearest row.");
            }
        } else {
            telemetry.addData("Hive visible", "NO (the Limelight needs ~30 s after power-up)");
        }
    }

    // What the shot is doing right now (while the Auto runs).
    private void addShotTelemetry() {
        telemetry.addData("Hive visible", visionSubsystem.hasTarget());
        telemetry.addData("tx (deg)", "%.1f", visionSubsystem.getTxDegrees());
        telemetry.addData("Turret on target", turretSubsystem.isOnTarget());
        telemetry.addData("Flywheel RPM (target / actual)", "%.0f / %.0f",
                shooterSubsystem.getTargetFlywheelRpm(), shooterSubsystem.getFlywheelRpm());
        telemetry.addData("Hood wheel RPM (target / actual)", "%.0f / %.0f",
                shooterSubsystem.getTargetHoodWheelRpm(), shooterSubsystem.getHoodWheelRpm());
        telemetry.addData("FLYWHEEL at speed (this Auto's check)", flywheelReady);
        telemetry.addData("Both wheels at speed (ShooterSubsystem, info only)",
                shooterSubsystem.isAtSpeed());
        telemetry.addData("Gate", shooterServoSubsystem.getState());
        telemetry.addData("Intake", intakeSubsystem.getState());
    }

    // The run log. It fills in as the Auto goes and stays on the screen after it ends.
    private void addRunLogTelemetry() {
        telemetry.addLine("--- RUN LOG ---");
        telemetry.addData("Run time (s)", "%.1f", runTimer.seconds());
        if (lockedAtSeconds < 0.0) {
            telemetry.addData("Camera locked at", "not yet");
        } else {
            telemetry.addData("Camera locked at (s)", "%.2f  (ty %.1f, table RPM %.0f)",
                    lockedAtSeconds, lockedTyDegrees, shotFlywheelRpm);
        }
        if (readyAtSeconds < 0.0) {
            telemetry.addData("Flywheel ready at", "not yet");
        } else {
            telemetry.addData("Flywheel ready at (s)", "%.2f", readyAtSeconds);
            telemetry.addData("Error at that moment (RPM)", "flywheel %.0f, hood wheel %.0f",
                    flywheelErrorAtReadyRpm, hoodWheelErrorAtReadyRpm);
            telemetry.addData("Flywheel dips during the feed (about 1 per ball)", flywheelDipCount);
        }
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose X (in)", "%.2f", pose.x());
        telemetry.addData("Pose Y (in)", "%.2f", pose.y());
        telemetry.addData("Pose heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }
}

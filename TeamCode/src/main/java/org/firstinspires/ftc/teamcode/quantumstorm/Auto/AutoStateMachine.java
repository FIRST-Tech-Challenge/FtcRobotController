package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.Navigator;
import org.firstinspires.ftc.teamcode.quantumstorm.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.quantumstorm.Indexer.SoftwareIndexer;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;
import org.firstinspires.ftc.teamcode.quantumstorm.Vision.LimelightCamera;

/**
 * Shared BioBuzz autonomous routine (see "BioBuzz Autonomous Design").
 * Not an OpMode itself: each alliance/position OpMode (RedTeamOneAutonomous, ...)
 * creates one with its AutoStartPosition.
 *
 * States (design-doc name in brackets):
 *   INIT     [Init]          self-check; ballCount = preload; dest = Hive 1 -> MOVING
 *   MOVING   [Moving]        drive to dest; on arrival enter the state stored with dest
 *                            (Hive -> SHOOTING, pickup zone -> INTAKE)
 *   SHOOTING [ReadyToShoot]  not enough balls -> go pick up. Else camera up, reposition if
 *                            the Hive flipped, align to AprilTag, shoot until empty -> go pick up
 *   INTAKE   [ReadyToPickup] Intake at Garden, Flower, or ground for a 30-second
 *                            color-pipeline scan; detections do not control the robot
 *   ERROR    [Error]         recoverable and retries left -> resume dest; else PARK
 *   PARK     [Parked]        drive to park (touching the perimeter) and stay
 * Global guard every loop: isAutoTimeUp() -> PARK from any state.
 *
 * Every step is non-blocking: each loop checks conditions once and returns.
 *
 * Usage inside an OpMode's runOpMode():
 *     AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.RED_1);
 *     waitForStart();
 *     auto.run();
 */
public class AutoStateMachine {

    public enum State {
        INIT,
        MOVING,
        SHOOTING,
        INTAKE,
        ERROR,
        PARK,
    }

    // Steps inside SHOOTING
    private enum ShootStep { CHECK_BALLS, AIM, FIRE }

    // Where the next pickup happens
    private enum PickupSource { GARDEN, FLOWER, GROUND }

    private final LinearOpMode opMode;
    private final AutoStartPosition position;

    private final PinpointOdometry odometry;
    private final Navigator navigator;
    private final Shooter shooter;
    private final Intake intake;
    private final Indexer indexer;
    private final LimelightCamera camera;

    private State state = State.INIT;
    private final ElapsedTime stateTimer = new ElapsedTime();   // time in the current state/step
    private final ElapsedTime matchTimer = new ElapsedTime();   // starts when leaving INIT
    private boolean matchTimerStarted = false;

    // MOVING: destination and the state to enter on arrival (design doc: dest)
    private Pose2D target;
    private String targetName = "";
    private State stateAfterMove;

    // SHOOTING
    private ShootStep shootStep = ShootStep.CHECK_BALLS;
    private boolean hiveFlipped = false;

    // INTAKE
    private PickupSource pickupSource = PickupSource.GARDEN;
    private boolean gardenPickedUp = false;
    private boolean flowerPickedUp = false;

    // ERROR / PARK
    private String errorReason = "";
    private boolean errorRecoverable = false;
    private int retryCount = 0;
    private boolean odometryFailed = false;
    private boolean parked = false;

    /** Call during INIT. Robot must be placed at position.start and be still. */
    public AutoStateMachine(LinearOpMode opMode, AutoStartPosition position) {
        this.opMode = opMode;
        this.position = position;

        MecanumDrive drive = new MecanumDrive(opMode.hardwareMap);
        odometry = new PinpointOdometry(opMode.hardwareMap, position.start);
        navigator = new Navigator(drive, odometry);
        shooter = new Shooter(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);
        indexer = new SoftwareIndexer();   // TODO: sensor-based Indexer
        camera = new LimelightCamera(opMode.hardwareMap);

        opMode.telemetry.addData("Autonomous", position.label);
        opMode.telemetry.addData("Camera", camera.getConnectionStatus());
        opMode.telemetry.addLine("Place robot at the " + position.label + " start, then press PLAY");
        opMode.telemetry.update();
    }

    /** Runs the routine until the OpMode ends. Call after waitForStart(). */
    public void run() {
        while (opMode.opModeIsActive()) {

            navigator.update();
            camera.updateRobotOrientation(odometry.getHeading());
            checkForFaults();

            switch (state) {
                case INIT:     runInit();     break;
                case MOVING:   runMoving();   break;
                case SHOOTING: runShooting(); break;
                case INTAKE:   collectBalls(); break;
                case ERROR:    runError();    break;
                case PARK:     parkRobot();   break;
            }

            // Global guard, checked last so it wins over whatever the states decided
            if (isAutoTimeUp() && state != State.PARK) {
                setState(State.PARK);
            }

            camera.updateColorPipelineScan();
            addTelemetry();
        }

        stopEverything();
        camera.stop();
    }

    // =========================================================
    // STATES
    // =========================================================

    private void runInit() {
        matchTimer.reset();
        matchTimerStarted = true;

        // Self-check: a failed check goes straight to ERROR
        if (!camera.isConnected()) {
            fail(camera.getConnectionStatus(), false);
            return;
        }

        indexer.setBallCount(Constants.AUTO_PRELOAD_BALLS);
        moveThen(position.hive1, "Hive 1", State.SHOOTING);
    }

    private void runMoving() {
        if (navigator.moveTo(target)) {
            setState(stateAfterMove);
        } else if (navigator.isTimedOut()) {
            fail("Could not reach " + targetName, true);
        }
    }

    private void runShooting() {
        switch (shootStep) {
            case CHECK_BALLS:
                if (indexer.getBallCount() < Constants.AUTO_BALLS_TO_SHOOT) {
                    goToPickup();
                    return;
                }
                shooter.spinUp();          // spins up while aligning
                setShootStep(ShootStep.AIM);
                break;

            case AIM:
                // Hive tipped: its active Cell is now on the far face, shoot from there
                if (!hiveFlipped && camera.isHiveFlipped()) {
                    hiveFlipped = true;
                    moveThen(shootingPose(), "Hive 1 far face", State.SHOOTING);
                    return;
                }
                if (isReadyToShoot()) {
                    navigator.stop();
                    shooter.feed();
                    setShootStep(ShootStep.FIRE);
                } else if (stateTimer.seconds() > Constants.AUTO_APRILTAG_TIMEOUT_SEC) {
                    fail(camera.isAprilTagVisible() ? "Could not align to AprilTag" : "AprilTag not found", true);
                } else if (camera.isAprilTagVisible()) {
                    double angle = camera.getAprilTagAngle();
                    if (Math.abs(angle) >= Constants.AUTO_AIM_TOLERANCE_DEG) {
                        double turn = Math.copySign(
                                Range.clip(Math.abs(angle * Constants.AUTO_AIM_TURN_KP),
                                        Constants.AUTO_AIM_MIN_TURN_POWER, Constants.MOVE_MAX_POWER),
                                angle);
                        navigator.driveRobotRelative(0, 0, turn);
                    } else {
                        // Stay aligned while waiting for the flywheel to spin up.
                        navigator.stop();
                    }
                } else {
                    navigator.stop();
                }
                break;

            case FIRE:
                if (indexer.getBallCount() > 0 && stateTimer.seconds() >= Constants.AUTO_FEED_TIME_SEC) {
                    // Fed long enough: should be empty. A sensor-based indexer will disagree if jammed.
                    indexer.setBallCount(0);
                    if (indexer.getBallCount() > 0) {
                        fail("Balls stuck in indexer", true);
                        return;
                    }
                }
                if (indexer.getBallCount() == 0) {
                    shooter.stop();
                    goToPickup();
                }
                break;
        }
    }

    /** INTAKE state (design doc: CollectBalls). */
    private void collectBalls() {
        switch (pickupSource) {
            case GARDEN:
            case FLOWER:
                // Color detections are telemetry-only. Stop intake when full or timed out,
                // but stay here until all three pipelines have had their 10-second scan.
                if (isFull() || stateTimer.seconds() > Constants.AUTO_PICKUP_TIMEOUT_SEC) {
                    intake.stop();
                }
                if (stateTimer.seconds() >= Constants.LIMELIGHT_COLOR_SCAN_CYCLE_SEC) {
                    if (pickupSource == PickupSource.GARDEN) {
                        gardenPickedUp = true;
                    } else {
                        flowerPickedUp = true;
                    }
                    finishPickup();
                }
                break;

            case GROUND:
                if (isFull() || stateTimer.seconds() > Constants.AUTO_GROUND_SEARCH_TIMEOUT_SEC) {
                    intake.stop();
                }
                navigator.stop();
                if (stateTimer.seconds() >= Constants.LIMELIGHT_COLOR_SCAN_CYCLE_SEC) {
                    finishPickup();
                }
                break;
        }
    }

    private void runError() {
        stopEverything();
        // TODO: re-localize odometry from the Limelight AprilTag bot pose

        if (errorRecoverable && retryCount < Constants.AUTO_MAX_RETRIES) {
            retryCount++;
            setState(State.MOVING);   // resume the destination we were heading to
        } else {
            setState(State.PARK);
        }
    }

    /** PARK state (design doc: ParkRobot). */
    private void parkRobot() {
        if (parked) {
            return;
        }
        if (odometryFailed) {
            // Can't navigate without odometry: stop where we are
            navigator.stop();
            parked = true;
        } else if (navigator.moveTo(position.park) || navigator.isTimedOut()) {
            // Arrived, or pushed against the perimeter/something: stop and stay
            navigator.stop();
            parked = true;
        }
    }

    // =========================================================
    // GUARDS
    // =========================================================

    /** Aligned to the Hive AprilTag and the flywheel has had time to spin up. */
    private boolean isReadyToShoot() {
        return camera.isAprilTagVisible()
                && Math.abs(camera.getAprilTagAngle()) < Constants.AUTO_AIM_TOLERANCE_DEG
                && stateTimer.seconds() >= Constants.AUTO_SHOOTER_SPINUP_SEC;
    }

    private boolean isAutoTimeUp() {
        return matchTimerStarted && matchTimer.seconds() >= Constants.AUTO_PARK_TIME_SEC;
    }

    private boolean isFull() {
        return indexer.getBallCount() >= Constants.AUTO_BALLS_TO_SHOOT;
    }

    // =========================================================
    // TRANSITION HELPERS
    // =========================================================

    /** Set the MOVING destination and the state to enter when it is reached. */
    private void moveThen(Pose2D newTarget, String name, State nextState) {
        target = newTarget;
        targetName = name;
        stateAfterMove = nextState;
        setState(State.MOVING);
    }

    /** Pick the next source (Garden, then Flower, then ground) and go there. */
    private void goToPickup() {
        if (!gardenPickedUp) {
            pickupSource = PickupSource.GARDEN;
            moveThen(position.garden, "Garden", State.INTAKE);
        } else if (!flowerPickedUp) {
            pickupSource = PickupSource.FLOWER;
            moveThen(position.flower, "Flower", State.INTAKE);
        } else {
            pickupSource = PickupSource.GROUND;
            setState(State.INTAKE);   // search from where we are
        }
    }

    private void finishPickup() {
        intake.stop();
        // Without ball sensors, assume the pickup filled the indexer
        indexer.setBallCount(Constants.AUTO_BALLS_TO_SHOOT);
        moveThen(shootingPose(), "Hive 1", State.SHOOTING);
    }

    private Pose2D shootingPose() {
        return hiveFlipped ? position.hive1Flipped : position.hive1;
    }

    /** Go to ERROR. Recoverable errors retry (resume dest); others end in PARK. */
    private void fail(String reason, boolean recoverable) {
        errorReason = reason;
        errorRecoverable = recoverable;
        setState(State.ERROR);
    }

    private void checkForFaults() {
        if (odometryFailed || state == State.PARK) {
            return;
        }
        if (odometry.getStatus().name().startsWith("FAULT")) {
            odometryFailed = true;
            fail("Pinpoint " + odometry.getStatus(), false);
        }
    }

    private void setState(State newState) {
        // Leaving a state: make sure nothing keeps running from it
        navigator.stop();
        if (state == State.SHOOTING) {
            shooter.stop();
        }
        if (state == State.INTAKE) {
            intake.stop();
        }

        state = newState;
        stateTimer.reset();
        if (newState == State.INTAKE) {
            camera.setTiltAngleFromNeutralDegrees(Constants.LIMELIGHT_SERVO_MAX_OFFSET_DEGREES);
            camera.startColorPipelineScan();
        } else {
            camera.setTiltAngleFromNeutralDegrees(newState == State.SHOOTING
                    ? -Constants.LIMELIGHT_SERVO_MAX_OFFSET_DEGREES : 0.0);
            camera.selectAprilTagPipeline();
        }

        // Entering a state
        if (newState == State.SHOOTING) {
            shootStep = ShootStep.CHECK_BALLS;
        } else if (newState == State.INTAKE) {
            intake.start();
        } else if (newState == State.PARK) {
            stopEverything();
            parked = false;
        }
    }

    private void setShootStep(ShootStep step) {
        shootStep = step;
        stateTimer.reset();
    }

    private void stopEverything() {
        navigator.stop();
        shooter.stop();
        intake.stop();
    }

    private void addTelemetry() {
        opMode.telemetry.addData("Autonomous", position.label);
        opMode.telemetry.addData("Time", "%.1f s", matchTimerStarted ? matchTimer.seconds() : 0.0);
        opMode.telemetry.addData("State", state);
        if (state == State.MOVING) {
            opMode.telemetry.addData("Destination", targetName);
        } else if (state == State.SHOOTING) {
            opMode.telemetry.addData("Shoot step", shootStep);
        } else if (state == State.INTAKE) {
            opMode.telemetry.addData("Pickup", pickupSource);
        }
        opMode.telemetry.addData("Balls", indexer.getBallCount());
        opMode.telemetry.addData("Garden / Flower done", "%b / %b", gardenPickedUp, flowerPickedUp);
        if (!errorReason.isEmpty()) {
            opMode.telemetry.addData("Last error", "%s (retries %d/%d)",
                    errorReason, retryCount, Constants.AUTO_MAX_RETRIES);
        }
        navigator.addTelemetry(opMode.telemetry);
        camera.addTelemetry(opMode.telemetry);
        opMode.telemetry.update();
    }
}

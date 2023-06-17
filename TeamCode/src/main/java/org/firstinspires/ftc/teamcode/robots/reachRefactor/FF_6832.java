package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Crane;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.IntSupplier;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

/** Controls
 * --Pregame--
 * left bumper - decrement state
 * right bumper - increment state
 * start - toggle active
 * x - set starting position to blue upper
 * b - set starting position to red lower
 * a - set starting position to blue lower
 * y - set starting position to red upper
 * dpad up - initialize / shutdown vision provider
 * dpad left - increment vision provider index
 * dpad down - toggle debug telemetry
 * dpad right - save vision output to filesystem
 * left trigger - toggle numerical dashboard
 * right trigger - toggle anti tipping
 * right stick button - toggle smoothing
 * left stick button - "test" init, points the shoulder straight up to make sure servo positions are good
 *
 * --Tele-Op--
 * gamepad 1: x - set gripper for intake
 * gamepad 1: b - lift gripper
 * gamepad 1: y - transfer
 * gamepad 1: a - toggle duck spinner
 * gamepad 1: tank drive
 *
 * gamepad 2: x - articulate crane to home
 * gamepad 2: b - dump crane bucket
 * gamepad 2: a - toggle duck spinner
 * gamepad 2: y - transfer
 * gamepad 2: dpad right - rotate turret 90 degrees right
 * gamepad 2: dpad up - articulate crane to high tier
 * gamepad 2: dpad left - articulate crane to middle tier
 * gamepad 2: dpad down - articulate crane to lowest tier
 * gamepad 2: left bumper - decrement chassis length stage
 * gamepad 2: right bumper - increment chassis length stage
 * gamepad 2: arcade drive
 *
 * --Manual Diagnostic--
 * gamepad 1: left bumper - decrement diagnostic index
 * gamepad 1: right bumper - increment diagnostic index
 * gamepad 1: right stick y - fine adjustment
 * gamepad 1: left stick y - coarse adjustment
 */
@Config
@TeleOp(name = "AAAA FF_6832")
@Disabled
public class FF_6832 extends OpMode {
    // constants
    public static double TANK_DRIVE_JOYSTICK_DIFF_DEADZONE = 0.2;
    public static double AVERAGE_LOOP_TIME_SMOOTHING_FACTOR = 0.1;
    public static boolean DEFAULT_DEBUG_TELEMETRY_ENABLED = false;
    public static double FORWARD_SCALING_FACTOR = 24; // scales the target linear robot velocity from tele-op controls
    public static double ROTATE_SCALING_FACTOR = 1; // scales the target angular robot velocity from tele-op controls
    public static double[] CHASSIS_LENGTH_LEVELS = new double[] {
            MIN_CHASSIS_LENGTH,
            MIN_CHASSIS_LENGTH + (MAX_CHASSIS_LENGTH - MIN_CHASSIS_LENGTH) / 3,
            MIN_CHASSIS_LENGTH + 2 * (MAX_CHASSIS_LENGTH - MIN_CHASSIS_LENGTH) / 3,
            MAX_CHASSIS_LENGTH
    };
    public static double DIAGNOSTIC_SERVO_STEP_MULTIPLIER_SLOW = 1;
    public static double DIAGNOSTIC_SERVO_STEP_MULTIPLIER_FAST = 5;
    public static double DRIVE_VELOCITY_EXPONENT = 1;
    public static double FORWARD_SMOOTHING_FACTOR = 0.3;
    public static double ROTATE_SMOOTHING_FACTOR = 0.25;
    public static double CHASSIS_LENGTH_SCALING_FACTOR = 1;
    public static double MAINTAIN_HEADING_DELAY = 1;
    public static double RUMBLE_DURATION = 0.5;
    public static double MAX_DX = 12;
    public static double MAX_DY = 12;

    private Robot robot;
    private Autonomous auto;
    private FtcDashboard dashboard;
    private ExponentialSmoother forwardSmoother, rotateSmoother;

    // global state
    private boolean active, initializing, debugTelemetryEnabled, numericalDashboardEnabled, smoothingEnabled, antiTippingEnabled;
    private Alliance alliance;
    private Position startingPosition;
    private GameState gameState;
    private int gameStateIndex;
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private long startTime;

    // vision state
    private int visionProviderIndex;
    private boolean visionProviderFinalized;

    // tele-op state
    private boolean dynamicChassisLengthEnabled, endGameHandled;
    private double forward1, rotate1, forward2, rotate2;
    private double velocityBoost;
    private boolean maintainHeadingTriggered;
    private long turningStopTime;
    public static double MAX_VELOCITY_BOOST = 1;
    private boolean deeznuts;

    // diagnostic state
    private DiagnosticStep diagnosticStep;
    private int diagnosticIndex;

    // max angular tuner state
    private double maxAngularVelocity, lastAngularVelocity, maxAngularAcceleration;
    private long lastAngularTunerUpdateTime;
    public static double ANGULAR_TUNER_RUNTIME = 4.0;

    // max linear tuner state
    private double maxVelocity, lastVelocity, maxAcceleration;
    private long lastLinearTunerUpdateTime;
    public static double LINEAR_TUNER_RUNTIME = 4.0;

    // track width tuner state
    public static double TRACK_WIDTH_TUNER_ANGLE = 180; // deg
    public static int TRACK_WIDTH_TUNER_NUM_TRIALS = 5;
    public static float TRACK_WIDTH_TUNER_DELAY = 0.5f; // s
    double headingAccumulator, lastHeading;
    int trackWidthTrial;
    MovingStatistics trackWidthStats = new MovingStatistics(TRACK_WIDTH_TUNER_NUM_TRIALS);
    StateMachine trackWidthTurn;
    Stage trackWidthTurnStage = new Stage();

    // timing
    private long lastLoopClockTime, loopTime;
    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother, averageUpdateTimeSmoother;

    public enum GameState {
        AUTONOMOUS("Autonomous", true),
        LINEAR_AUTONOMOUS("Linear Autonomous", true),
        NO_RR("no RR", true),
        JANK_AUTO("Jank Auto", true),

        TELE_OP("Tele-Op"),
        MANUAL_DIAGNOSTIC("Manual Diagnostic"),
        MAX_ANGULAR_TUNER("Max Angular Tuner"),
        MAX_LINEAR_TUNER("Max Linear Tuner"),
        TRACK_WIDTH_TUNER("Track Width Tuner"),
        CRANE_DEBUG("Crane Debug"),

        BACK_AND_FORTH("Back And Forth"),
        SQUARE("Square"),
        SQUARENORR("Square No RR"),
        TURN("Turn"),
        LENGTH_TEST("Length Test"),
        DIAGONAL_TEST("Diagonal Test");

        private final String name;
        private final boolean autonomous;

        GameState(String name, boolean autonomous) {
            this.name = name;
            this.autonomous = autonomous;
        }

        GameState(String name) {
            this(name, false);
        }

        public String getName() { return name; }

        public boolean isAutonomous() { return autonomous; }

        public static GameState getGameState(int index) {
            return GameState.values()[index];
        }

        public static int getNumGameStates() {
            return GameState.values().length;
        }

        public static int indexOf(GameState gameState) {
            return Arrays.asList(GameState.values()).indexOf(gameState);
        }
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // global state
        active = true;
        initializing = true;
        debugTelemetryEnabled = DEFAULT_DEBUG_TELEMETRY_ENABLED;
        gameState = GameState.TELE_OP;

        // timing
        lastLoopClockTime = System.nanoTime();
        loopTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);
        averageUpdateTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);

        // gamepads
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        robot = new Robot(hardwareMap, false);
        alliance = Alliance.BLUE;
        startingPosition = Position.START_BLUE_UP;
        robot.driveTrain.setPoseEstimate(startingPosition.getPose());
        auto = new Autonomous(robot);

        forwardSmoother = new ExponentialSmoother(FORWARD_SMOOTHING_FACTOR);
        rotateSmoother = new ExponentialSmoother(ROTATE_SMOOTHING_FACTOR);

        auto.createVisionProvider(VisionProviders.DEFAULT_PROVIDER_INDEX);
        auto.visionProvider.initializeVision(hardwareMap);
        visionProviderFinalized = true;

        auto.build(startingPosition);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry.setMsTransmissionInterval(25);
        //start the calibration motion
        robot.crane.calibrateShoulder();
        robot.articulate(Robot.Articulation.INIT);
    }

    private void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
                gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
                gameStateIndex += 1;

            if(gameStateIndex < 0)
                gameStateIndex = GameState.getNumGameStates() - 1;
            gameStateIndex %= GameState.getNumGameStates();
            gameState = GameState.getGameState(gameStateIndex);
        }

        if (stickyGamepad1.back || stickyGamepad2.back)
            active = !active;
    }

    private void handleVisionProviderSwitch() {
        if(!active) {
            if(!visionProviderFinalized) {
                if (stickyGamepad1.dpad_left || stickyGamepad2.dpad_left) {
                    visionProviderIndex = (visionProviderIndex + 1) % VisionProviders.VISION_PROVIDERS.length; // switch vision provider
                    auto.createVisionProvider(visionProviderIndex);
                }
                if (stickyGamepad1.dpad_up || stickyGamepad2.dpad_up) {
                    auto.visionProvider.initializeVision(hardwareMap); // this is blocking
                    visionProviderFinalized = true;
                }
            } else if (stickyGamepad1.dpad_up || stickyGamepad2.dpad_up) {
                auto.visionProvider.shutdownVision(); // also blocking, but should be very quick
                visionProviderFinalized = false;
            }
        }
        else if((stickyGamepad1.dpad_right || stickyGamepad2.dpad_right) && visionProviderFinalized)
        {
            auto.visionProvider.saveDashboardImage();
        }
        if(visionProviderFinalized)
            auto.visionProvider.update();
    }

    private void handlePregameControls() {
        Position previousStartingPosition = startingPosition;
        if(stickyGamepad1.x || stickyGamepad2.x) {
            alliance = Alliance.BLUE;
            startingPosition = Position.START_BLUE_UP;
        }
        if(stickyGamepad1.a || stickyGamepad2.a) {
            alliance = Alliance.BLUE;
            startingPosition = Position.START_BLUE_DOWN;
        }
        if(stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Alliance.RED;
            startingPosition = Position.START_RED_DOWN;
        }
        if(stickyGamepad1.y || stickyGamepad2.y) {
            alliance = Alliance.RED;
            startingPosition = Position.START_RED_UP;
        }
        if(previousStartingPosition != startingPosition) {
            robot.driveTrain.setPoseEstimate(startingPosition.getPose());
            auto.build(startingPosition);
        }

        if(stickyGamepad1.dpad_up || stickyGamepad2.dpad_up)
            debugTelemetryEnabled = !debugTelemetryEnabled;
        if(stickyGamepad1.dpad_down || stickyGamepad2.dpad_down)
            if (robot.crane.shoulderInitialized)
                robot.articulate(Robot.Articulation.START_DOWN); //stow crane to the starting position
            else
                robot.crane.configureShoulder(); //setup the shoulder - do this only when the
        if(stickyGamepad1.left_trigger || stickyGamepad2.left_trigger)
            numericalDashboardEnabled = !numericalDashboardEnabled;
        if(stickyGamepad1.right_trigger || stickyGamepad2.right_trigger)
            antiTippingEnabled = !antiTippingEnabled;
        if(stickyGamepad1.right_stick_button || stickyGamepad2.right_stick_button)
            smoothingEnabled = !smoothingEnabled;
        if(stickyGamepad1.left_stick_button || stickyGamepad2.left_stick_button)
            robot.crane.articulate(Crane.Articulation.TEST_INIT);
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        handleStateSwitch();
        handleVisionProviderSwitch();
        handlePregameControls();

        update();
    }

    private void rumble() {
        gamepad1.rumble((int) (RUMBLE_DURATION * 1000));
        gamepad2.rumble((int) (RUMBLE_DURATION * 1000));
    }

    //Code that runs ONCE after the driver hits PLAY
    @Override
    public void start() {
        initializing = false;

        trackWidthTurn = Utils.getStateMachine(trackWidthTurnStage)
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(
                        robot.driveTrain.trajectorySequenceBuilder(
                                robot.driveTrain.getPoseEstimate()
                        )
                                .turn(TRACK_WIDTH_TUNER_ANGLE)
                                .build()
                ))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .addTimedState(() -> TRACK_WIDTH_TUNER_DELAY, () -> {}, () -> {})
                .build();
        auto.visionProvider.shutdownVision();

        robot.articulate(Robot.Articulation.START);
        if(!gameState.equals(GameState.MANUAL_DIAGNOSTIC)) {
            robot.driveTrain.setMaintainChassisLengthEnabled(true);
            robot.driveTrain.setChassisLength(CHASSIS_LENGTH_LEVELS[0]);
            robot.driveTrain.setMaintainHeading(robot.driveTrain.getPoseEstimate().getHeading());
        }
        lastLoopClockTime = System.nanoTime();
        startTime = System.currentTimeMillis();

        rumble();
    }

    private void handleArcadeDrive(Gamepad gamepad) {
        forward1 = Math.pow(-gamepad.left_stick_y, DRIVE_VELOCITY_EXPONENT) * FORWARD_SCALING_FACTOR;
        rotate1 = Math.pow(-gamepad.right_stick_x, DRIVE_VELOCITY_EXPONENT) * ROTATE_SCALING_FACTOR;
    }

    private void handleArcadeDriveReversed(Gamepad gamepad) {
        forward2 = Math.pow(gamepad.left_stick_y, DRIVE_VELOCITY_EXPONENT) * FORWARD_SCALING_FACTOR;
        rotate2 = Math.pow(-gamepad.right_stick_x, DRIVE_VELOCITY_EXPONENT) * ROTATE_SCALING_FACTOR;
    }

    private void sendDriveCommands() {
        double forward, rotate;
        if(smoothingEnabled) {
            forward = forwardSmoother.update((forward2 + forward1) * velocityBoost);
            rotate = rotateSmoother.update((rotate2 + rotate1) * velocityBoost);
        } else {
            forward = (forward1 + forward2) * velocityBoost;
            rotate = (rotate1 + rotate2) * velocityBoost;
        }

        if(!approxEquals(rotate, 0)) {
            robot.driveTrain.setMaintainHeadingEnabled(false);
            turningStopTime = System.nanoTime();
            maintainHeadingTriggered = false;
        } else if((System.nanoTime() - turningStopTime) * 1e-9 > MAINTAIN_HEADING_DELAY && !maintainHeadingTriggered) {
            robot.driveTrain.setMaintainHeadingEnabled(true);
            robot.driveTrain.setMaintainHeading(robot.driveTrain.getPoseEstimate().getHeading());
            maintainHeadingTriggered = true;
        }

        if(antiTippingEnabled)
            robot.driveTrain.setDrivePowerSafe(new Pose2d(forward, 0, rotate));
        else
            robot.driveTrain.setDriveVelocity(new Pose2d(forward, 0, rotate));
    }

    private boolean duckSpinnerPowerResetHandled = false;
    private void handleTeleOp() { // apple
        // gamepad 1
        if (stickyGamepad1.x) {
            if (robot.isDoubleDuckEnabled()){
                if (robot.gripper.getPitchTargetPos() == Gripper.PITCH_DOWN)
                    robot.gripper.liftDuck();
                else
                    robot.gripper.setDuck();
            }
            else {
                robot.gripper.set();
                robot.driveTrain.setChassisLength(MIN_CHASSIS_LENGTH);
            }
        }
        if(stickyGamepad1.b)
            robot.articulate(robot.isDoubleDuckEnabled() ? Robot.Articulation.DOUBLE_DUCK_DUMP_AND_SET_CRANE_FOR_TRANSFER : Robot.Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER);

        // gamepad 2
        if(stickyGamepad2.left_bumper) //go home - it's the safest place to retract if the bucket is about to colide with something
            robot.crane.articulate(Crane.Articulation.HOME);
        if(stickyGamepad2.b)  //dump bucket - might be able to combine this with Cycle Complete
            robot.articulate(robot.isDoubleDuckEnabled() ? Robot.Articulation.DOUBLE_DUCK_DUMP_AND_SET_CRANE_FOR_TRANSFER : Robot.Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER);
        if(stickyGamepad2.x) {
            if(robot.isDoubleDuckEnabled())
                robot.gripper.setDuck();
            else
                robot.gripper.set();
            robot.driveTrain.setChassisLength(MIN_CHASSIS_LENGTH);
        }

        // joint gamepad controls
        if((stickyGamepad1.dpad_right || stickyGamepad2.dpad_right) && robot.crane.getArticulation() == Crane.Articulation.MANUAL)
            robot.crane.articulate(Crane.Articulation.HIGH_TIER_RIGHT);
//        if((stickyGamepad1.dpad_down || stickyGamepad2.dpad_down) && robot.crane.getArticulation() == Crane.Articulation.MANUAL)
//            robot.crane.articulate(Crane.Articulation.SHARED_SHIPPING_HUB);
        if((stickyGamepad1.dpad_left || stickyGamepad2.dpad_left) && robot.crane.getArticulation() == Crane.Articulation.MANUAL)
            robot.crane.articulate(Crane.Articulation.HIGH_TIER_LEFT);
        if((stickyGamepad1.dpad_up || stickyGamepad2.dpad_up) && robot.crane.getArticulation() == Crane.Articulation.MANUAL)
            robot.crane.articulate(Crane.Articulation.HOME);
        if((stickyGamepad1.y || stickyGamepad2.y) && robot.crane.getArticulation() == Crane.Articulation.MANUAL) //todo - this should trigger a Swerve_Cycle_Complete articulation in Pose
            robot.articulate(Robot.Articulation.TRANSFER);

        if(stickyGamepad1.right_bumper || stickyGamepad2.right_bumper) {
            dynamicChassisLengthEnabled = false;
            robot.driveTrain.setChassisLength(robot.driveTrain.getTargetChassisLength() == CHASSIS_LENGTH_LEVELS[0] ? CHASSIS_LENGTH_LEVELS[CHASSIS_LENGTH_LEVELS.length - 1] : CHASSIS_LENGTH_LEVELS[0]);
        }
//        if(stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
//            dynamicChassisLengthEnabled = true;
//        if(dynamicChassisLengthEnabled) {
//            double chassisLength = Range.clip(robot.driveTrain.getTargetChassisLength() + CHASSIS_LENGTH_SCALING_FACTOR * loopTime * 1e-9 * ((forward1 - forward2) / 2), MIN_CHASSIS_LENGTH, MAX_CHASSIS_LENGTH);
//            robot.driveTrain.setChassisLength(chassisLength);
//        }
        if(notTriggerDeadZone(gamepad1.left_trigger))
            velocityBoost = (1 + gamepad1.left_trigger * MAX_VELOCITY_BOOST);
        if(notTriggerDeadZone((gamepad2.left_trigger)))
            velocityBoost = (1 + gamepad2.left_trigger * MAX_VELOCITY_BOOST);
        if(!notTriggerDeadZone(gamepad1.left_trigger) && !notTriggerDeadZone(gamepad2.left_trigger))
            velocityBoost = 1;

//        if(stickyGamepad1.right_trigger || stickyGamepad2.right_trigger)
//            robot.articulate(alliance == Alliance.RED ? Robot.Articulation.AUTO_HIGH_TIER_RED : Robot.Articulation.AUTO_HIGH_TIER_BLUE);

        if(gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3)
            robot.driveTrain.setDuckSpinnerPower(0.5);
        else if(gamepad1.a || gamepad2.a)
            robot.driveTrain.setDuckSpinnerPower(DriveTrain.DUCK_SPINNER_POWER * alliance.getMod());
        else
            robot.driveTrain.setDuckSpinnerPower(0);

//            robot.setAutoDumpEnabled(!robot.isAutoDumpEnabled());
//            robot.setDoubleDuckEnabled(!robot.isDoubleDuckEnabled());

        if(gamepad1.touchpad_finger_1) {
            Pose2d pose = robot.driveTrain.getPoseEstimate();
            Vector2d turretPose = pose.vec().minus(
                    new Vector2d(
                            robot.driveTrain.getChassisLength(),
                            0
                    ).rotated(pose.getHeading())
            );

            Vector2d finalPose = turretPose.minus(
                    new Vector2d(
                            map(gamepad1.touchpad_finger_1_x, -1, 1, -MAX_DX, MAX_DX),
                            map(gamepad1.touchpad_finger_1_y, -1, 1, 0, MAX_DY)
                    ).rotated(pose.getHeading() - Math.toRadians(90))
            );

            robot.handleAutoCrane(new Pose2d(finalPose.getX(), finalPose.getY()), HIGH_TIER_SHIPPING_HUB_HEIGHT);
        }

        handleArcadeDrive(gamepad1);
        handleArcadeDriveReversed(gamepad2);

        sendDriveCommands();
    }

    private enum DiagnosticStep {
        DRIVETRAIN_LEFT_MOTOR, DRIVETRAIN_RIGHT_MOTOR, DRIVETRAIN_MIDDLE_MOTOR, DRIVETRAIN_MIDDLE_SWIVEL_MOTOR,
        CRANE_SHOULDER_SERVO, CRANE_ELBOW_SERVO, CRANE_WRIST_SERVO, TURRET_MOTOR,
        GRIPPER_SERVO, GRIPPER_PITCH_SERVO,
        DUCK_SPINNER;

        public static DiagnosticStep getDiagnosticStep(int index) {
            return DiagnosticStep.values()[index];
        }

        public static int getNumDiagnosticSteps() {
            return DiagnosticStep.values().length;
        }
    }

    private void handleDiagnosticMotorControls(DoubleConsumer setTargetVelocity) {
        setTargetVelocity.accept(-gamepad1.right_stick_y);
    }

    private void handleDiagnosticServoControls(IntSupplier getTargetPos, IntConsumer setTargetPos) {
        if(notJoystickDeadZone(gamepad1.right_stick_y))
            setTargetPos.accept(servoClip((int) (getTargetPos.getAsInt() - gamepad1.right_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLIER_SLOW)));
        else if(notJoystickDeadZone(gamepad1.left_stick_y))
            setTargetPos.accept(servoClip((int) (getTargetPos.getAsInt() - gamepad1.left_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLIER_FAST)));
    }

    private void handleDiagnosticServoAngleControls(DoubleSupplier getTargetPosAngle, DoubleConsumer setTargetAngle, double minAngle, double maxAngle) {
        if(notJoystickDeadZone(gamepad1.right_stick_y))
            setTargetAngle.accept(Range.clip(getTargetPosAngle.getAsDouble() - gamepad1.right_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLIER_SLOW, minAngle, maxAngle));
        else if(notJoystickDeadZone(gamepad1.left_stick_y))
            setTargetAngle.accept(Range.clip(getTargetPosAngle.getAsDouble() - gamepad1.left_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLIER_FAST, minAngle, maxAngle));
    }

    private void handleManualDiagnostic() {
        if(stickyGamepad1.right_bumper)
            diagnosticIndex++;
        else if(stickyGamepad1.left_bumper)
            diagnosticIndex--;
        if(diagnosticIndex < 0)
            diagnosticIndex = DiagnosticStep.getNumDiagnosticSteps() - 1;
        diagnosticIndex %= DiagnosticStep.getNumDiagnosticSteps();
        diagnosticStep = DiagnosticStep.getDiagnosticStep(diagnosticIndex);

        switch(diagnosticStep) {
            case DRIVETRAIN_LEFT_MOTOR:
                handleDiagnosticMotorControls(robot.driveTrain::setLeftVelocity);
                break;
            case DRIVETRAIN_RIGHT_MOTOR:
                handleDiagnosticMotorControls(robot.driveTrain::setRightVelocity);
                break;
            case DRIVETRAIN_MIDDLE_MOTOR:
                handleDiagnosticMotorControls(robot.driveTrain::setSwerveVelocity);
                break;
            case DRIVETRAIN_MIDDLE_SWIVEL_MOTOR:
                robot.driveTrain.setSwivelAngle(-gamepad1.right_stick_y);
                break;
            case CRANE_SHOULDER_SERVO:
                handleDiagnosticServoAngleControls(robot.crane::getShoulderTargetAngle, robot.crane::setShoulderTargetAngle, Crane.SHOULDER_DEG_MIN, Crane.SHOULDER_DEG_MAX);
                break;
            case CRANE_ELBOW_SERVO:
                handleDiagnosticServoAngleControls(robot.crane::getElbowTargetAngle, robot.crane::setElbowTargetAngle, Crane.ELBOW_DEG_MIN, Crane.ELBOW_DEG_MAX);
                break;
            case CRANE_WRIST_SERVO:
                handleDiagnosticServoAngleControls(robot.crane::getWristTargetAngle, robot.crane::setWristTargetAngle, Crane.WRIST_DEG_MIN, Crane.WRIST_DEG_MAX);
                break;
            case TURRET_MOTOR:
                robot.crane.turret.setTargetHeading(robot.crane.turret.getTargetHeading() - gamepad1.right_stick_y);
                break;
            case GRIPPER_SERVO:
                handleDiagnosticServoControls(robot.gripper::getTargetPos, robot.gripper::setTargetPosDiag);
                break;
            case GRIPPER_PITCH_SERVO:
                handleDiagnosticServoControls(robot.gripper::getPitchTargetPos, robot.gripper::setPitchTargetPosDiag);
                break;
            case DUCK_SPINNER:
                robot.driveTrain.setDuckSpinnerPower(-gamepad1.right_stick_y);
                break;
        }
    }

    private void handleCraneDebug() {
        if(stickyGamepad1.dpad_up || stickyGamepad2.dpad_up)
            robot.crane.articulate(Crane.Articulation.HIGH_TIER);
        if(stickyGamepad1.dpad_right || stickyGamepad2.dpad_right)
            robot.crane.articulate(Crane.Articulation.MIDDLE_TIER);
        if(stickyGamepad1.dpad_down || stickyGamepad2.dpad_down)
            robot.crane.articulate(Crane.Articulation.LOWEST_TIER);
        if(stickyGamepad1.dpad_left || stickyGamepad2.dpad_left)
            robot.crane.articulate(Crane.Articulation.HOME);
        if(stickyGamepad1.y || stickyGamepad2.y) //todo - this should trigger a Swerve_Cycle_Complete articulation in Pose
            robot.crane.articulate(Crane.Articulation.TRANSFER);
        if(stickyGamepad1.b || stickyGamepad2.b) //todo - this should trigger a Swerve_Cycle_Complete articulation in Pose
            robot.articulate(Robot.Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER);
        if(stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
            robot.crane.articulate(Crane.Articulation.TEST_1);
        if(stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
            robot.crane.articulate(Crane.Articulation.TEST_1);
        if(stickyGamepad1.right_stick_button || stickyGamepad2.right_stick_button)
            robot.crane.articulate(Crane.Articulation.TEST_1);
    }

    private void changeGameState(GameState state) {
        active = false;
        gameState = state;
        gameStateIndex = GameState.indexOf(state);
        startTime = System.currentTimeMillis();
    }

    //Main loop that repeats after hitting PLAY
    @Override
    public void loop() {
        handleStateSwitch();

        if (active) {
            long currentTime = System.currentTimeMillis();
            if (!endGameHandled && gameState == GameState.TELE_OP && (currentTime - startTime) * 1e-3 >= 80) {
//                robot.articulate(Robot.Articulation.START_END_GAME);
                endGameHandled = true;
                rumble();
            }
            switch(gameState) {
                case TELE_OP:
                    handleTeleOp();
                    break;
                case MANUAL_DIAGNOSTIC:
                    handleManualDiagnostic();
                    break;
                case MAX_ANGULAR_TUNER:
                    robot.driveTrain.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    robot.driveTrain.setDrivePower(new Pose2d(0, 0, 1));

                    if((currentTime - startTime) * 1e-3 < ANGULAR_TUNER_RUNTIME) {
                        Pose2d poseVelocity = robot.driveTrain.getPoseVelocity();
                        double angularVelocity = poseVelocity.getHeading();
                        maxAngularVelocity = Math.max(angularVelocity, maxAngularVelocity);

                        long currentUpdateTime = System.nanoTime();
                        if(lastAngularTunerUpdateTime == 0)
                            lastAngularTunerUpdateTime = currentUpdateTime;

                        double angularAcceleration = (lastAngularVelocity - angularVelocity) / ((currentUpdateTime - lastAngularTunerUpdateTime) * 1e-9);
                        lastAngularTunerUpdateTime = currentUpdateTime;
                        maxAngularAcceleration = Math.max(angularAcceleration, maxAngularAcceleration);
                        lastAngularVelocity = angularVelocity;
                    }
                    break;
                case MAX_LINEAR_TUNER:
                    robot.driveTrain.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    robot.driveTrain.setDrivePower(new Pose2d(1, 0, 0));

                    if((currentTime - startTime) * 1e-3 < LINEAR_TUNER_RUNTIME) {
                        Pose2d poseVelocity = robot.driveTrain.getPoseVelocity();
                        double velocity = poseVelocity.vec().norm();
                        maxVelocity = Math.max(velocity, maxAngularVelocity);

                        long currentUpdateTime = System.nanoTime();
                        if(lastLinearTunerUpdateTime == 0)
                            lastLinearTunerUpdateTime = currentUpdateTime;

                        double acceleration = (lastVelocity - velocity) / ((currentUpdateTime - lastLinearTunerUpdateTime) * 1e-9);
                        lastLinearTunerUpdateTime = currentUpdateTime;
                        maxAcceleration = Math.max(acceleration, maxAcceleration);
                        lastVelocity = velocity;
                    }
                    break;
                case TRACK_WIDTH_TUNER:
                    if(trackWidthTurn.execute()) {
                        double trackWidth = TRACK_WIDTH * Math.toRadians(TRACK_WIDTH_TUNER_ANGLE) / headingAccumulator;
                        trackWidthStats.add(trackWidth);
                        trackWidthTurnStage.resetStage();
                        if(trackWidthTrial < TRACK_WIDTH_TUNER_NUM_TRIALS) {
                            trackWidthTrial++;
                            trackWidthTurn.execute();
                        }
                    } else {
                        double heading = robot.driveTrain.getPoseEstimate().getHeading();
                        headingAccumulator += wrapAngleRad(heading - lastHeading);
                    }
                    break;
                case CRANE_DEBUG:
                    handleCraneDebug();
                    break;
                case AUTONOMOUS:
                    if(auto.getStateMachine(startingPosition, Autonomous.Mode.SPLINE).execute())
                        changeGameState(GameState.TELE_OP);
                    break;
                case LINEAR_AUTONOMOUS:
                    if(auto.getStateMachine(startingPosition, Autonomous.Mode.LINEAR).execute())
                        changeGameState(GameState.TELE_OP);
                    break;
                case JANK_AUTO:
                    if(auto.getStateMachine(startingPosition, Autonomous.Mode.SIMPLE).execute())
                        changeGameState(GameState.TELE_OP);
                    break;
                case NO_RR:
                    if(auto.getStateMachine(startingPosition, Autonomous.Mode.NO_RR).execute())
                        changeGameState(GameState.TELE_OP);
                    break;
                case BACK_AND_FORTH:
                    auto.backAndForth.execute();
                    break;
                case SQUARE:
                    auto.square.execute();
                    break;
                case SQUARENORR:
                    auto.squareNoRR.execute();
                    break;
                case TURN:
                    auto.turn.execute();
                    break;
                case LENGTH_TEST:
                    if(auto.lengthTest.execute())
                        changeGameState(GameState.TELE_OP);
                    break;
                case DIAGONAL_TEST:
                    if(auto.diagonalTest.execute())
                        changeGameState(GameState.TELE_OP);
                    break;
            }
        } else {
            handlePregameControls();
        }

        update();
    }

    private void updateTiming() {
        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopClockTime;
        averageLoopTime = loopTimeSmoother.update(loopTime);
        lastLoopClockTime = loopClockTime;
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet) {
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);

        if(robot.driveTrain.getVoltage() <= LOW_BATTERY_VOLTAGE) {
            telemetryMap = new LinkedHashMap<>();
            for(int i = 0; i < 20; i++) {
                telemetryMap.put(i +
                        (System.currentTimeMillis() / 500 % 2 == 0 ? "**BATTERY VOLTAGE LOW**" : "  BATTERY VOLTAGE LOW  "),
                        (System.currentTimeMillis() / 500 % 2 == 0 ? "**CHANGE BATTERY ASAP!!**" : "  CHANGE BATTERY ASAP!!  "));
            }
        }
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            if(numericalDashboardEnabled)
                packet.put(entry.getKey(), entry.getValue());
            else
                packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }

    private void update() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        // handling dashboard changes
        forwardSmoother.setSmoothingFactor(FORWARD_SMOOTHING_FACTOR);
        rotateSmoother.setSmoothingFactor(ROTATE_SMOOTHING_FACTOR);

        TelemetryPacket packet = new TelemetryPacket();

        long updateStartTime = System.nanoTime();
        robot.update(packet.fieldOverlay());
        long updateTime = (System.nanoTime() - updateStartTime);
        double averageUpdateTime = averageUpdateTimeSmoother.update(updateTime);

        Map<String, Object> opModeTelemetryMap = new LinkedHashMap<>();

        // handling op mode telemetry
        opModeTelemetryMap.put("Active", active);
        if(initializing) {
            opModeTelemetryMap.put("Starting Position", startingPosition);
            opModeTelemetryMap.put("Anti-Tipping Enabled", antiTippingEnabled);
            opModeTelemetryMap.put("Smoothing Enabled", smoothingEnabled);
        }
        opModeTelemetryMap.put("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Last Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));
        if(debugTelemetryEnabled) {
            opModeTelemetryMap.put("Average Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageUpdateTime * 1e-6), (int) (1 / (averageUpdateTime * 1e-9))));
            opModeTelemetryMap.put("Last Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (updateTime * 1e-6), (int) (1 / (updateTime * 1e-9))));
        }

        switch(gameState) {
            case TELE_OP:
                opModeTelemetryMap.put("Double Duck", robot.isDoubleDuckEnabled());
                break;
            case MANUAL_DIAGNOSTIC:
                opModeTelemetryMap.put("Diagnostic Step", diagnosticStep);
                break;
            case TRACK_WIDTH_TUNER:
                opModeTelemetryMap.put("Effective track width", Misc.formatInvariant("%.2f (SE = %.3f)", trackWidthStats.getMean(), trackWidthStats.getStandardDeviation() / Math.sqrt(TRACK_WIDTH_TUNER_NUM_TRIALS)));
                break;
            case MAX_ANGULAR_TUNER:
                opModeTelemetryMap.put("Max Angular Velocity", Math.toDegrees(maxAngularVelocity));
                opModeTelemetryMap.put("Max Angular Acceleration", Math.toDegrees(maxAngularAcceleration));
                break;
            case MAX_LINEAR_TUNER:
                opModeTelemetryMap.put("Max Velocity", maxVelocity);
                opModeTelemetryMap.put("Max Acceleration", maxAcceleration);
                break;
        }
        handleTelemetry(opModeTelemetryMap,  Misc.formatInvariant("(%d): %s", gameStateIndex, gameState.getName()), packet);

        // handling subsystem telemetry
        for(TelemetryProvider telemetryProvider: robot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);
        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

        // handling vision telemetry
        Map<String, Object> visionTelemetryMap = auto.visionProvider.getTelemetry(debugTelemetryEnabled);
        visionTelemetryMap.put("Backend",
                Misc.formatInvariant("%s (%s)",
                        VisionProviders.VISION_PROVIDERS[visionProviderIndex].getSimpleName(),
                        visionProviderFinalized ?
                                "finalized" :
                                System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  "
                )
        );
        handleTelemetry(visionTelemetryMap, auto.visionProvider.getTelemetryName(), packet);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        if(!initializing)
            dashboard.sendImage(robot.getBitmap());

        updateTiming();
    }
}

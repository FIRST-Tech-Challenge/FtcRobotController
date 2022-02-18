package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Crane;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
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
 * gamepad 1: dpad up - enable duck game
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
@TeleOp(name = "AAAArefactored FF_6832")
public class FF_6832 extends OpMode {
    // constants
    public static double TANK_DRIVE_JOYSTICK_DIFF_DEADZONE = 0.2;
    public static double AVERAGE_LOOP_TIME_SMOOTHING_FACTOR = 0.1;
    public static boolean DEFAULT_DEBUG_TELEMETRY_ENABLED = false;
    public static double FORWARD_SCALING_FACTOR = 48; // scales the target linear robot velocity from tele-op controls
    public static double ROTATE_SCALING_FACTOR = 4; // scales the target angular robot velocity from tele-op controls
    public static double[] CHASSIS_LENGTH_LEVELS = new double[] {
            MIN_CHASSIS_LENGTH,
            MIN_CHASSIS_LENGTH + (MAX_CHASSIS_LENGTH - MIN_CHASSIS_LENGTH) / 3,
            MIN_CHASSIS_LENGTH + 2 * (MAX_CHASSIS_LENGTH - MIN_CHASSIS_LENGTH) / 3,
            MAX_CHASSIS_LENGTH
    };
    public static int DIAGNOSTIC_SERVO_STEP_MULTIPLER_SLOW = 5;
    public static int DIAGNOSTIC_SERVO_STEP_MULTIPLER_FAST = 15;
    public static double DRIVE_VELOCITY_EXPONENT = 1;
    public static double FORWARD_SMOOTHING_FACTOR = 0.3;
    public static double ROTATE_SMOOTHING_FACTOR = 0.3;
    public static double CHASSIS_LENGTH_SCALING_FACTOR = 0.05;

    private Robot robot;
    private Autonomous auto;
    private FtcDashboard dashboard;
    private ExponentialSmoother forwardSmoother, rotateSmoother;

    // global state
    private boolean active, initializing, debugTelemetryEnabled, numericalDashboardEnabled, smoothingEnabled, antiTippingEnabled;
    private Alliance ALLIANCE;
    private Position startingPosition;
    private GameState gameState;
    private int gameStateIndex;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    // vision state
    private int visionProviderIndex;
    private boolean visionProviderFinalized;

    // tele-op state
    private int chassisDistanceLevelIndex;
    private double forward1, rotate1, forward2, rotate2, forward, rotate;

    // diagnostic state
    private DiagnosticStep diagnosticStep;
    private int diagnosticIndex;
    private int servoTargetPos;

    // timing
    private long lastLoopClockTime, loopTime;
    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother;

    public enum GameState {
        TELE_OP("Tele-Op"),
        AUTONOMOUS("Autonomous"),
        LINEAR_AUTONOMOUS("Linear Autonomous"),
        MANUAL_DIAGNOSTIC("Manual Diagnostic"),
        BACK_AND_FORTH("Back And Forth"),
        SQUARE("Square"),
        TURN("Turn"),
        LENGTH_TEST("Length Test");

        private final String name;

        GameState(String name) {
            this.name = name;
        }

        public String getName() { return name; }

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

        // gamepads
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        robot = new Robot(hardwareMap, false);
        ALLIANCE = Alliance.BLUE;
        startingPosition = Position.START_BLUE_UP;
        robot.driveTrain.setPoseEstimate(startingPosition.getPose());
        auto = new Autonomous(robot);

        forwardSmoother = new ExponentialSmoother(FORWARD_SMOOTHING_FACTOR);
        rotateSmoother = new ExponentialSmoother(ROTATE_SMOOTHING_FACTOR);

        // vision
        auto.createVisionProvider(VisionProviders.DEFAULT_PROVIDER_INDEX);

        auto.visionProvider.initializeVision(hardwareMap);
        visionProviderFinalized = true;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry.setMsTransmissionInterval(25);

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

        if (stickyGamepad1.start || stickyGamepad2.start)
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
    }

    private void handlePregameControls() {
        if(stickyGamepad1.x || stickyGamepad2.x) {
            ALLIANCE = Alliance.BLUE;
            startingPosition = Position.START_BLUE_UP;
        }
        if(stickyGamepad1.a || stickyGamepad2.a) {
            ALLIANCE = Alliance.BLUE;
            startingPosition = Position.START_BLUE_DOWN;
        }
        if(stickyGamepad1.b || stickyGamepad2.b) {
            ALLIANCE = Alliance.RED;
            startingPosition = Position.START_RED_DOWN;
        }
        if(stickyGamepad1.y || stickyGamepad2.y) {
            ALLIANCE = Alliance.RED;
            startingPosition = Position.START_RED_UP;
        }
        if(stickyGamepad1.dpad_up || stickyGamepad2.dpad_up)
            debugTelemetryEnabled = !debugTelemetryEnabled;
        if(stickyGamepad1.dpad_down || stickyGamepad2.dpad_down)
            robot.articulate(Robot.Articulation.START); //stow crane to the starting position
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

    //Code that runs ONCE after the driver hits PLAY
    @Override
    public void start() {
        initializing = false;

        auto.build();
        auto.visionProvider.shutdownVision();

        robot.articulate(Robot.Articulation.START);
        if(!gameState.equals(GameState.MANUAL_DIAGNOSTIC)) {
            robot.driveTrain.setMaintainChassisLengthEnabled(true);
            robot.driveTrain.setChassisLength(CHASSIS_LENGTH_LEVELS[0]);
        }
        lastLoopClockTime = System.nanoTime();
    }


    private void handleArcadeDrive(Gamepad gamepad) {
        forward1 = Math.pow(gamepad.left_stick_y, DRIVE_VELOCITY_EXPONENT) * FORWARD_SCALING_FACTOR;
        rotate1 = Math.pow(-gamepad.right_stick_x, DRIVE_VELOCITY_EXPONENT) * ROTATE_SCALING_FACTOR;
    }

    private void handleArcadeDriveFunkyTest(Gamepad gamepad) {
        forward2 = Math.pow(-gamepad.left_stick_y, DRIVE_VELOCITY_EXPONENT) * FORWARD_SCALING_FACTOR;
        rotate2 = Math.pow(-gamepad.right_stick_x, DRIVE_VELOCITY_EXPONENT) * ROTATE_SCALING_FACTOR;
    }

    private void handleTankDrive(Gamepad gamepad) {
        double left = Math.pow(-gamepad.left_stick_y, DRIVE_VELOCITY_EXPONENT);
        double right = Math.pow(-gamepad.right_stick_y, DRIVE_VELOCITY_EXPONENT);

        forward2 = (right + left) / 2.0 * FORWARD_SCALING_FACTOR;
        rotate2 = (right - left) / 2.0 * ROTATE_SCALING_FACTOR * .4;

        if(Math.abs(right - left) < TANK_DRIVE_JOYSTICK_DIFF_DEADZONE)
            rotate2 = 0;
    }

    private void sendDriveCommands() {
        if(smoothingEnabled) {
            forward = forwardSmoother.update(forward2 + forward1);
            rotate = rotateSmoother.update(rotate2 + rotate1);
        } else {
            forward = forward1 + forward2;
            rotate = rotate1 + rotate2;
        }
        if(antiTippingEnabled)
            robot.driveTrain.setDrivePowerSafe(new Pose2d(forward, 0, rotate));
        else
            robot.driveTrain.setDrivePower(new Pose2d(forward, 0, rotate));
    }

    private void handleTeleOp() { // apple
        // gamepad 1
        if (stickyGamepad1.x)
            robot.gripper.set();
        if(stickyGamepad1.b)
            robot.gripper.lift();
        if(stickyGamepad1.a)
            robot.driveTrain.toggleDuckSpinner(ALLIANCE.getMod());

        // gamepad 2
        if(stickyGamepad2.x) //go home - it's the safest place to retract if the bucket is about to colide with something
            robot.crane.articulate(Crane.Articulation.HOME);
        if(stickyGamepad2.b)  //dump bucket - might be able to combine this with Cycle Complete
            robot.articulate(Robot.Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER);
        if(stickyGamepad2.a) //spin carousel
            robot.driveTrain.toggleDuckSpinner(ALLIANCE.getMod());
        if(stickyGamepad2.right_trigger)
            robot.turret.incrementOffset(1);
        if(stickyGamepad2.left_trigger)
            robot.turret.incrementOffset(-1);

        // joint gamepad controls
        if(stickyGamepad2.dpad_right)
            robot.crane.articulate(Crane.Articulation.HIGH_TIER_RIGHT);
        if(stickyGamepad2.dpad_down)
            //robot.crane.articulate(Crane.Articulation.LOWEST_TIER);
            robot.crane.articulate(Crane.Articulation.LOWEST_TIER);
        if(stickyGamepad2.dpad_left)
            //robot.crane.articulate(Crane.Articulation.MIDDLE_TIER);
            robot.crane.articulate(Crane.Articulation.HIGH_TIER_LEFT);
        if(stickyGamepad2.dpad_up)
            robot.crane.articulate(Crane.Articulation.HOME);
        if(stickyGamepad2.y) //todo - this should trigger a Swerve_Cycle_Complete articulation in Pose
            robot.articulate(Robot.Articulation.TRANSFER);

        if(stickyGamepad1.dpad_down)
            robot.driveTrain.setDuckGameEnabled(!robot.driveTrain.isDuckGameEnabled());

        if(stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
            robot.driveTrain.setMaintainChassisLengthEnabled(!robot.driveTrain.isMaintainChassisLengthEnabled());
        if(stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
            robot.articulate(Robot.Articulation.AUTO_HIGH_TIER);

//        double chassisLength = Range.clip(robot.driveTrain.getTargetChassisLength() + CHASSIS_LENGTH_SCALING_FACTOR * loopTime * 1e-9 * ((forward2 - forward1) / 2), MIN_CHASSIS_LENGTH, MAX_CHASSIS_LENGTH);
        robot.driveTrain.setChassisLength(CHASSIS_LENGTH_LEVELS[0]);

        handleArcadeDriveFunkyTest(gamepad1);
        handleArcadeDrive(gamepad2);

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
        servoTargetPos = getTargetPos.getAsInt();
        if(notJoystickDeadZone(gamepad1.right_stick_y))
            setTargetPos.accept(servoClip((int) (getTargetPos.getAsInt() - gamepad1.right_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLER_SLOW)));
        else if(notJoystickDeadZone(gamepad1.left_stick_y))
            setTargetPos.accept(servoClip((int) (getTargetPos.getAsInt() - gamepad1.left_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLER_FAST)));
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
                handleDiagnosticServoControls(robot.crane::getShoulderTargetPos, robot.crane::setShoulderTargetPosRaw);
                break;
            case CRANE_ELBOW_SERVO:
                handleDiagnosticServoControls(robot.crane::getElbowTargetPos, robot.crane::setElbowTargetPosRaw);
                break;
            case CRANE_WRIST_SERVO:
                handleDiagnosticServoControls(robot.crane::getWristTargetPos, robot.crane::setWristTargetPosRaw);
                break;
            case TURRET_MOTOR:
                robot.crane.turret.setTargetAngle(robot.crane.turret.getTargetAngle() - gamepad1.right_stick_y);
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

    //Main loop that repeats after hitting PLAY
    @Override
    public void loop() {
        handleStateSwitch();

        if (active) {
            switch(gameState) {
                case TELE_OP:
                    handleTeleOp();
                    break;
                case AUTONOMOUS:
                    StateMachine autoStateMachine = auto.getStateMachine(startingPosition, true);
                    if(autoStateMachine.execute()) {
                        active = false;
                        gameState = GameState.TELE_OP;
                        gameStateIndex = GameState.indexOf(GameState.TELE_OP);
                    }
                    break;
                case LINEAR_AUTONOMOUS:
                    StateMachine linearAutoStateMachine = auto.getStateMachine(startingPosition, false);
                    if(linearAutoStateMachine.execute()) {
                        active = false;
                        gameState = GameState.TELE_OP;
                        gameStateIndex = GameState.indexOf(GameState.TELE_OP);
                    }
                    break;
                case MANUAL_DIAGNOSTIC:
                    handleManualDiagnostic();
                    break;
                case BACK_AND_FORTH:
                    auto.backAndForth.execute();
                    break;
                case SQUARE:
                    auto.square.execute();
                    break;
                case TURN:
                    auto.turn.execute();
                    break;
                case LENGTH_TEST:
                    if(auto.lengthTest.execute()) {
                        active = false;
                        gameState = GameState.TELE_OP;
                        gameStateIndex = GameState.indexOf(GameState.TELE_OP);
                    }
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

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = String.format("%s: %s", entry.getKey(), entry.getValue());
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
        if(initializing) {
            auto.visionProvider.update();
            robot.driveTrain.setPoseEstimate(startingPosition.getPose());
        }

        stickyGamepad1.update();
        stickyGamepad2.update();

        TelemetryPacket packet = new TelemetryPacket();
        Map<String, Object> opModeTelemetryMap = new LinkedHashMap<>();

        // handling op mode telemetry
        opModeTelemetryMap.put("Active", active);
        if(initializing) {
            opModeTelemetryMap.put("Starting Position", startingPosition);
            opModeTelemetryMap.put("Anti-Tipping Enabled", antiTippingEnabled);
            opModeTelemetryMap.put("Smoothing Enabled", smoothingEnabled);
        }
        opModeTelemetryMap.put("Chassis Level Index", String.format("%d / %d", chassisDistanceLevelIndex, CHASSIS_LENGTH_LEVELS.length));
        opModeTelemetryMap.put("Average Loop Time", String.format("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Last Loop Time", String.format("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));

        switch(gameState) {
            case MANUAL_DIAGNOSTIC:
                opModeTelemetryMap.put("Diagnostic Step", diagnosticStep);
                opModeTelemetryMap.put("Servo Target Pos", servoTargetPos);
                break;
        }
        handleTelemetry(opModeTelemetryMap,  String.format("(%d): %s", gameStateIndex, gameState.getName()), packet);

        robot.update(packet.fieldOverlay());

        // handling subsystem telemetry
        for(TelemetryProvider telemetryProvider: robot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);
        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

        // handling vision telemetry
        Map<String, Object> visionTelemetryMap = auto.visionProvider.getTelemetry(debugTelemetryEnabled);
        visionTelemetryMap.put("Backend",
                String.format("%s (%s)",
                        VisionProviders.VISION_PROVIDERS[visionProviderIndex].getSimpleName(),
                        visionProviderFinalized ?
                                "finalized" :
                                System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  "
                )
        );
        handleTelemetry(visionTelemetryMap, auto.visionProvider.getTelemetryName(), packet);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        // handling dashboard changes
        forwardSmoother.setSmoothingFactor(FORWARD_SMOOTHING_FACTOR);
        rotateSmoother.setSmoothingFactor(ROTATE_SMOOTHING_FACTOR);

        updateTiming();
    }
}

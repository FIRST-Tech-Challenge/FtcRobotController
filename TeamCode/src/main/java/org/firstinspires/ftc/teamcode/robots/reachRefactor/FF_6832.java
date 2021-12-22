package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Crane;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.IntConsumer;
import java.util.function.IntSupplier;

/** Controls
 * Pregame
 * x - set alliance to blue
 * b - set alliance to red
 * a - toggle desmos drive
 * y - toggle drivetrain smoothing
 *
 * dpad up - initialize / shutdown vision provider
 * dpad left - increment vision provider index
 * dpad down - toggle debug telemetry
 * dpad right - toggle arcade drive for teleop
 * left bumper - decrement state
 * right bumper - increment state
 *
 * Tele-Op
 * gamepad 1: left bumper - raise gripper
 * gamepad 1: right bumper - lower gripper
 * gamepad 1: left trigger - open gripper
 * gamepad 1: right trigger - close gripper
 * gamepad 1: tank drive
 *
 * gamepad 2: b - toggle gripper
 * gamepad 2: a - toggle duck spinner
 * gamepad 2: dpad left - articulate crane to home
 * gamepad 2: dpad up - articulate crane to high teir
 * gamepad 2: dpad right - articulate crane to transfer
 * gamepad 2: right bumper - increment chassis length stage
 * gamepad 2: left bumper - decrement chassis length stage
 * gamepad 2: arcade drive
 *
 * left stick y - forward
 * right stick x - rotate
 * guide - emergency stop
 */
@TeleOp(name = "refactored FF_6832")
public class FF_6832 extends OpMode {
    private Robot robot;
    private Autonomous auto;
    private FtcDashboard dashboard;

    // global state
    private boolean active, initializing, debugTelemetryEnabled;
    private Constants.Alliance alliance;
    private GameState gameState;
    private int gameStateIndex;
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private boolean gamepad1JoysticksActive, gamepad2JoysticksActive;

    // vision state
    private int visionProviderIndex;
    private boolean visionProviderFinalized;

    // tele-op state
    private boolean usingDesmosDrive, smoothingEnabled;
    private int chassisDistanceLevelIndex;

    // diagnostic state
    private DiagnosticStep diagnosticStep;
    private int diagnosticIndex;
    private boolean diagnosticFinished;

    // timing
    private long lastLoopClockTime, loopTime;
    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother;

    // constants
    public static double TRIGGER_DEADZONE_THRESHOLD = 0.4;
    public static double JOYSTICK_DEADZONE_THRESHOLD = 0.05;
    public static double AVERAGE_LOOP_TIME_SMOOTHING_FACTOR = 0.1;
    public static boolean DEFAULT_DEBUG_TELEMETRY_ENABLED = true;
    public static double FORWARD_SCALING_FACTOR = 3; // scales the target linear robot velocity from tele-op controls
    public static double ROTATE_SCALING_FACTOR = 5; // scales the target angular robot velocity from tele-op controls
    public static double[] CHASSIS_DISTANCE_LEVELS = new double[] {
            Constants.MIN_CHASSIS_LENGTH,
            Constants.MIN_CHASSIS_LENGTH + (Constants.MAX_CHASSIS_LENGTH - Constants.MIN_CHASSIS_LENGTH) / 3,
            Constants.MIN_CHASSIS_LENGTH + 2 * (Constants.MAX_CHASSIS_LENGTH - Constants.MIN_CHASSIS_LENGTH) / 3,
            Constants.MAX_CHASSIS_LENGTH
    };
    public static int DIAGNOSTIC_SERVO_STEP_MULTIPLER_SLOW = 10;
    public static int DIAGNOSTIC_SERVO_STEP_MULTIPLER_FAST = 30;

    public enum GameState {
        TELE_OP("Tele-Op"),
        AUTONOMOUS("Autonomous"),
        AUTONOMOUS_DIAGNOSTIC("Autonomous Diagnostic"),
        MANUAL_DIAGNOSTIC("Manual Diagnostic");

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

        robot = new Robot(hardwareMap);
        alliance = Constants.Alliance.BLUE;
        auto = new Autonomous(robot);

        // vision
        auto.createVisionProvider(VisionProviders.DEFAULT_PROVIDER_INDEX);
        auto.visionProvider.initializeVision(hardwareMap);
        visionProviderFinalized = true;

        dashboard = FtcDashboard.getInstance();

        robot.articulate(Robot.Articulation.INIT);
    }

    private void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper)
                gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper)
                gameStateIndex += 1;

            gameStateIndex %= GameState.getNumGameStates();
            gameStateIndex = Math.abs(gameStateIndex);
            gameState = GameState.getGameState(gameStateIndex);
        }

        if (stickyGamepad1.start)
            active = !active;
    }

    private void handleVisionProviderSwitch() {
        if(!active) {
            if(!visionProviderFinalized) {
                if (stickyGamepad1.dpad_left) {
                    visionProviderIndex = (visionProviderIndex + 1) % VisionProviders.VISION_PROVIDERS.length; // switch vision provider
                    auto.createVisionProvider(visionProviderIndex);
                }
                if (stickyGamepad1.dpad_up) {
                    auto.visionProvider.initializeVision(hardwareMap); // this is blocking
                    visionProviderFinalized = true;
                }
            } else if (stickyGamepad1.dpad_up) {
                auto.visionProvider.shutdownVision(); // also blocking, but should be very quick
                visionProviderFinalized = false;
            }
        }
    }

    private void handlePregameControls() {
        if(stickyGamepad1.x)
            alliance = Constants.Alliance.BLUE;
        if(stickyGamepad1.b)
            alliance = Constants.Alliance.RED;

        if(stickyGamepad1.y)
            smoothingEnabled = !smoothingEnabled;
        if(stickyGamepad1.dpad_down)
            debugTelemetryEnabled = !debugTelemetryEnabled;
        if(stickyGamepad1.a)
            usingDesmosDrive = !usingDesmosDrive;
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        handleStateSwitch();
        handleVisionProviderSwitch();
        handlePregameControls();

        update();
    }

    @Override
    public void start() {
        lastLoopClockTime = System.nanoTime();
        initializing = false;
        if(gameState.equals(GameState.AUTONOMOUS) || gameState.equals(GameState.TELE_OP))
            robot.articulate(Robot.Articulation.START);
        auto.visionProvider.shutdownVision();
    }

    private void handleTeleOpDriveArcade() {
        double forward = Math.pow(-gamepad2.left_stick_y, 3) * FORWARD_SCALING_FACTOR;
        double rotate = Math.pow(gamepad2.right_stick_x, 3) * ROTATE_SCALING_FACTOR;

        if(usingDesmosDrive)
            robot.driveTrain.driveDesmos(forward, rotate, loopTime / 1e9, smoothingEnabled);
        else
            robot.driveTrain.drive(forward, rotate, smoothingEnabled);
    }

    private void handleTeleOpDriveTank() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        double forward = (left + right) / 2 * FORWARD_SCALING_FACTOR;
        double rotate = (right - left) / 2 * ROTATE_SCALING_FACTOR;


        if(usingDesmosDrive)
            robot.driveTrain.driveDesmos(forward, rotate, loopTime / 1e9, smoothingEnabled);
        else
            robot.driveTrain.drive(forward, rotate, smoothingEnabled);
    }

    private void handleTeleOp() {
        // gamepad 1
        if(stickyGamepad1.left_bumper)
            robot.gripper.pitchGripper(true);
        if(stickyGamepad1.right_bumper)
            robot.gripper.pitchGripper(false);

        if(UtilMethods.notDeadZone(gamepad1.right_trigger, TRIGGER_DEADZONE_THRESHOLD))
            robot.gripper.actuateGripper(false);
        if(UtilMethods.notDeadZone(gamepad1.left_trigger, TRIGGER_DEADZONE_THRESHOLD))
            robot.gripper.actuateGripper(true);

        if(stickyGamepad1.b)
            robot.gripper.toggleGripper();

        if(stickyGamepad1.a)
            robot.driveTrain.handleDuckSpinnerToggle(alliance.getMod());

        if(gamepad1JoysticksActive && !gamepad2JoysticksActive)
            handleTeleOpDriveTank();

        // gamepad 2
        if(stickyGamepad2.b)
            robot.gripper.toggleGripper();

        if(stickyGamepad2.a)
            robot.driveTrain.handleDuckSpinnerToggle(alliance.getMod());

        if(stickyGamepad2.dpad_left)
            robot.crane.articulate(Crane.Articulation.HOME);
        if(stickyGamepad2.dpad_up)
            robot.crane.articulate(Crane.Articulation.HIGH_TEIR);
        if(stickyGamepad2.dpad_right)
            robot.articulate(Robot.Articulation.TRANSFER);

        if(stickyGamepad2.right_bumper)
            chassisDistanceLevelIndex++;
        else if(stickyGamepad2.left_bumper)
            chassisDistanceLevelIndex--;

        chassisDistanceLevelIndex = Math.abs(chassisDistanceLevelIndex % CHASSIS_DISTANCE_LEVELS.length);
        robot.driveTrain.setTargetChassisDistance(CHASSIS_DISTANCE_LEVELS[chassisDistanceLevelIndex]);

        if(gamepad2JoysticksActive && !gamepad1JoysticksActive)
            handleTeleOpDriveArcade();

        if(stickyGamepad1.guide || stickyGamepad2.guide)
            robot.stop();
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
        setTargetPos.accept(UtilMethods.servoClip((int) (getTargetPos.getAsInt() - gamepad1.right_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLER_SLOW)));
        setTargetPos.accept(UtilMethods.servoClip((int) (getTargetPos.getAsInt() - gamepad1.left_stick_y * DIAGNOSTIC_SERVO_STEP_MULTIPLER_FAST)));
    }

    private void handleManualDiagnostic() {
        if(stickyGamepad1.right_bumper)
            diagnosticIndex++;
        else if(stickyGamepad1.left_bumper)
            diagnosticIndex--;
        diagnosticIndex %= DiagnosticStep.getNumDiagnosticSteps();
        diagnosticIndex = Math.abs(diagnosticIndex);
        diagnosticStep = DiagnosticStep.getDiagnosticStep(diagnosticIndex);

        switch(diagnosticStep) {
            case DRIVETRAIN_LEFT_MOTOR:
                handleDiagnosticMotorControls(robot.driveTrain::setFrontLeftTargetVelocity);
                break;
            case DRIVETRAIN_RIGHT_MOTOR:
                handleDiagnosticMotorControls(robot.driveTrain::setFrontRightTargetVelocity);
                break;
            case DRIVETRAIN_MIDDLE_MOTOR:
                handleDiagnosticMotorControls(robot.driveTrain::setMiddleTargetVelocity);
                break;
            case DRIVETRAIN_MIDDLE_SWIVEL_MOTOR:
                robot.driveTrain.setMaintainSwivelAngleEnabled(false);
                robot.driveTrain.setMaintainSwivelAngleCorrection(-gamepad1.right_stick_y);
                break;
            case CRANE_SHOULDER_SERVO:
                handleDiagnosticServoControls(robot.crane::getShoulderTargetPos, robot.crane::setShoulderTargetPos);
                break;
            case CRANE_ELBOW_SERVO:
                handleDiagnosticServoControls(robot.crane::getElbowTargetPos, robot.crane::setElbowTargetPos);
                break;
            case CRANE_WRIST_SERVO:
                handleDiagnosticServoControls(robot.crane::getWristTargetPos, robot.crane::setWristTargetPos);
                break;
            case TURRET_MOTOR:
                robot.crane.turret.setTargetAngle(robot.crane.turret.getTargetAngle() - gamepad1.right_stick_y);
                break;
            case GRIPPER_SERVO:
                handleDiagnosticServoControls(robot.gripper::getTargetPos, robot.gripper::setTargetPos);
                break;
            case GRIPPER_PITCH_SERVO:
                handleDiagnosticServoControls(robot.gripper::getPitchTargetPos, robot.gripper::setPitchTargetPos);
                break;
            case DUCK_SPINNER:
                robot.driveTrain.duckSpinner.setPower(-gamepad1.right_stick_y);
                break;
        }
    }

    @Override
    public void loop() {
        handleStateSwitch();

        if (active) {
            switch(gameState) {
                case TELE_OP:
                    handleTeleOp();
                    break;
                case AUTONOMOUS:
                    if (alliance == Constants.Alliance.RED && auto.autonomousRed.execute()) {
                        active = false;
                        gameState = GameState.TELE_OP;
                        gameStateIndex = GameState.indexOf(GameState.TELE_OP);
                    } else if (alliance == Constants.Alliance.BLUE && auto.autonomousBlue.execute()) {
                        active = false;
                        gameState = GameState.TELE_OP;
                        gameStateIndex = GameState.indexOf(GameState.TELE_OP);
                    }
                    break;
                case AUTONOMOUS_DIAGNOSTIC:
                    if(!diagnosticFinished && robot.articulate(Robot.Articulation.DIAGNOSTIC))
                            diagnosticFinished = true;
                    break;
                case MANUAL_DIAGNOSTIC:
                    handleManualDiagnostic();
                    break;
            }
        } else {
            handleStateSwitch();
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

        for(Map.Entry<String, Object> entry: telemetryMap.entrySet()) {
            String line = String.format("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
            packet.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }

    private void updateTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        Map<String, Object> opModeTelemetryMap = new HashMap<>();

        // handling op mode telemetry
        if(debugTelemetryEnabled) {
            opModeTelemetryMap.put("Average Loop Time", String.format("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
            opModeTelemetryMap.put("Last Loop Time", String.format("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));
            opModeTelemetryMap.put("Using Desmos Drive", usingDesmosDrive);
        }
        opModeTelemetryMap.put("Active", active);
        opModeTelemetryMap.put("State", String.format("(%d): %s", gameStateIndex, gameState));
        opModeTelemetryMap.put("Debug Telemetry Enabled", debugTelemetryEnabled);
        opModeTelemetryMap.put("Using Desmos Drive", usingDesmosDrive);
        opModeTelemetryMap.put("Chassis Level Index", String.format("%d / %d", chassisDistanceLevelIndex, CHASSIS_DISTANCE_LEVELS.length));

        switch(gameState) {
            case TELE_OP:
                opModeTelemetryMap.put("Smoothing Enabled", smoothingEnabled);
                break;
            case AUTONOMOUS:
                break;
            case AUTONOMOUS_DIAGNOSTIC:
                break;
            case MANUAL_DIAGNOSTIC:
                opModeTelemetryMap.put("Diagnostic Step", diagnosticStep);
                break;
        }
        handleTelemetry(opModeTelemetryMap, gameState.getName(), packet);

        // handling subsystem telemetry
        for(TelemetryProvider telemetryProvider: robot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);
        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

        // handling vision telemetry
        handleTelemetry(auto.visionProvider.getTelemetry(debugTelemetryEnabled), auto.visionProvider.getTelemetryName(), packet);

        robot.drawFieldOverlay(packet);
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }

    private boolean joysticksActive(Gamepad gamepad) {
        return  UtilMethods.notDeadZone(gamepad.left_stick_x, JOYSTICK_DEADZONE_THRESHOLD) ||
                UtilMethods.notDeadZone(gamepad.left_stick_y, JOYSTICK_DEADZONE_THRESHOLD) ||
                UtilMethods.notDeadZone(gamepad.right_stick_x, JOYSTICK_DEADZONE_THRESHOLD) ||
                UtilMethods.notDeadZone(gamepad.right_stick_y, JOYSTICK_DEADZONE_THRESHOLD);
    }

    private void update() {
        if(initializing)
            auto.visionProvider.update();

        gamepad1JoysticksActive = joysticksActive(gamepad1);
        gamepad2JoysticksActive = joysticksActive(gamepad2);

        updateTelemetry();

        updateTiming();

        stickyGamepad1.update();
        stickyGamepad2.update();

        robot.update();
    }
}

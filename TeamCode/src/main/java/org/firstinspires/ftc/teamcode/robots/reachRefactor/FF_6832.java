package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import android.graphics.Bitmap;

import org.ejml.simple.SimpleMatrix;
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
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

/** Controls
 * Pregame
 * x - set alliance to blue
 * b - set alliance to red
 * a - toggle FTC dashboard
 * y - toggle drivetrain smoothing
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

    // global state
    private boolean active, initializing;
    private Constants.GameState gameState;
    private int gameStateIndex, visionProviderIndex;
    private boolean visionProviderFinalized;
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private Map<org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position, Integer> positionFrequencies;
    private org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position mostFrequentPosition;
    private boolean usingDesmosDrive;
    private boolean dashboardEnabled, debugTelemetryEnabled, smoothingEnabled;
    private boolean gamepad1JoysticksActive, gamepad2JoysticksActive;
    private int chassisDistanceLevelIndex;

    // Telemetry
    private FtcDashboard dashboard;

    // TPM Calibration state
    private boolean TPMCalibrationInitialized;
    private SimpleMatrix TPMCalibrationStartingTicks;
    private double averageTPMCalibrationTicksTraveled;

    // timing
    private long lastLoopClockTime, loopTime;
    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother;
//    private long[] times;

    // Constants
    public static double TRIGGER_DEADZONE_THRESHOLD = 0.4;
    public static double JOYSTICK_DEADZONE_THRESHOLD = 0.4;
    public static double AVERAGE_LOOP_TIME_SMOOTHING_FACTOR = 0.1;
    public static boolean DEFAULT_DASHBOARD_ENABLED = true;
    public static double FORWARD_SCALING_FACTOR = 3; // scales the target linear robot velocity from tele-op controls
    public static double ROTATE_SCALING_FACTOR = 5; // scales the target angular robot velocity from tele-op controls
    public static double[] CHASSIS_DISTANCE_LEVELS = new double[] {
            Constants.MIN_CHASSIS_LENGTH,
            Constants.MIN_CHASSIS_LENGTH + (Constants.MAX_CHASSIS_LENGTH - Constants.MIN_CHASSIS_LENGTH) / 3,
            Constants.MIN_CHASSIS_LENGTH + 2 * (Constants.MAX_CHASSIS_LENGTH - Constants.MIN_CHASSIS_LENGTH) / 3,
            Constants.MAX_CHASSIS_LENGTH
    };

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // global state
        active = true;
        initializing = true;
        dashboardEnabled = DEFAULT_DASHBOARD_ENABLED;
        gameState = Constants.GameState.TELE_OP;

        // TPM calibration state
        TPMCalibrationInitialized = false;
        TPMCalibrationStartingTicks = new SimpleMatrix(1, 3);

        // timing
        lastLoopClockTime = System.nanoTime();
        loopTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);

        // gamepads
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        robot = new Robot(hardwareMap);
        robot.setAlliance(Constants.Alliance.BLUE);
        auto = new Autonomous(robot);

        // vision
//        robot.createVisionProvider(VisionProviders.DEFAULT_PROVIDER_INDEX);
//        auto.visionProvider.initializeVision(hardwareMap);
//        visionProviderFinalized = true;

        if(dashboardEnabled)
            dashboard = FtcDashboard.getInstance();

        positionFrequencies = new HashMap<org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position, Integer>() {{
            put(org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position.LEFT, 0);
            put(org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position.MIDDLE, 0);
            put(org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position.RIGHT, 0);
        }};
    }

    private void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper)
                gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper)
                gameStateIndex += 1;

            gameStateIndex %= Constants.GameState.getNumGameStates();
            gameState = Constants.GameState.getGameState(gameStateIndex);
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

    public void toggleIsDashboardEnabled() {
        dashboardEnabled = !dashboardEnabled;
        if(dashboard == null)
            dashboard = FtcDashboard.getInstance();
    }

    private void handlePregameControls() {
        if(stickyGamepad1.x)
            robot.setAlliance(Constants.Alliance.BLUE);
        if(stickyGamepad1.b)
            robot.setAlliance(Constants.Alliance.RED);

        if(stickyGamepad1.a)
            toggleIsDashboardEnabled();
        if(stickyGamepad1.y)
            smoothingEnabled = !smoothingEnabled;
        if(stickyGamepad1.dpad_down) {
            debugTelemetryEnabled = !debugTelemetryEnabled;
        }
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        handleStateSwitch();
//        handleVisionProviderSwitch();
        handlePregameControls();

        update();
    }

    @Override
    public void start() {
//        updateMostFrequentPosition();
//        robot.setMostFrequentPosition(mostFrequentPosition);
        lastLoopClockTime = System.nanoTime();
        initializing = false;
        robot.articulate(Robot.Articulation.START);
//        auto.visionProvider.shutdownVision();
    }

    private void handleTeleOpDriveArcade(Gamepad gamepad) {
        double forward = Math.pow(-gamepad.left_stick_y, 3) * FORWARD_SCALING_FACTOR;
        double rotate = Math.pow(gamepad.right_stick_x, 3) * ROTATE_SCALING_FACTOR;

        if(usingDesmosDrive)
            robot.driveTrain.driveDesmos(forward, rotate, loopTime / 1e9);
        else
            robot.driveTrain.drive(forward, rotate);
    }

    private void handleTeleOpDriveTank(Gamepad gamepad) {
        double left = -gamepad.left_stick_y;
        double right = -gamepad.right_stick_y;

        double forward = (left + right) / 2 * FORWARD_SCALING_FACTOR;
        double rotate = (right - left) / 2 * ROTATE_SCALING_FACTOR;


        if(usingDesmosDrive)
            robot.driveTrain.driveDesmos(forward, rotate, loopTime / 1e9);
        else
            robot.driveTrain.drive(forward, rotate);
    }

    private void handleEmergencyStop() {
        if(stickyGamepad1.guide || stickyGamepad2.guide)
            robot.stop();
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
            robot.driveTrain.handleDuckSpinnerToggle(robot.getAlliance().getMod());

        if(gamepad1JoysticksActive && !gamepad2JoysticksActive)
            handleTeleOpDriveTank(gamepad1);

        // gamepad 2
        if(stickyGamepad2.b)
            robot.gripper.toggleGripper();

        if(stickyGamepad2.a)
            robot.driveTrain.handleDuckSpinnerToggle(robot.getAlliance().getMod());

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

        if(gamepad2JoysticksActive && !gamepad1JoysticksActive)
            handleTeleOpDriveArcade(gamepad2);

        chassisDistanceLevelIndex = Math.abs(chassisDistanceLevelIndex % CHASSIS_DISTANCE_LEVELS.length);
        robot.driveTrain.setTargetChassisDistance(CHASSIS_DISTANCE_LEVELS[chassisDistanceLevelIndex]);
    }

    private Stage diagnosticStage = new Stage();
    private StateMachine diagnostic = UtilMethods.getStateMachine(diagnosticStage)
            // testing drivetrain
            .addTimedState(3f, () -> {
                robot.driveTrain.drive(0.25, 0);
            }, () -> {})
            .addTimedState(3f, () -> {
                robot.driveTrain.drive(0, Math.PI / 2);
            }, () -> {})

            // testing crane
            .addState(() -> robot.crane.articulate(Crane.Articulation.HOME))
            .addState(() -> robot.crane.articulate(Crane.Articulation.LOWEST_TEIR))
            .addState(() -> robot.crane.articulate(Crane.Articulation.HIGH_TEIR))
            .addState(() -> robot.crane.articulate(Crane.Articulation.MIDDLE_TEIR))
            .addState(() -> robot.crane.articulate(Crane.Articulation.STARTING))
            .addState(() -> robot.crane.articulate(Crane.Articulation.CAP))
            .addState(() -> robot.articulate(Robot.Articulation.TRANSFER))

            // testing gripper
            .addState(() -> robot.crane.articulate(Crane.Articulation.HOME))
            .addTimedState(3f, () -> {
                robot.gripper.togglePitch();
            }, () -> {})
            .addTimedState(3f, () -> {
                robot.gripper.toggleGripper();
            }, () -> {})

            // testing turret
            .addTimedState(3f,
                    () -> robot.crane.turret.setTargetAngle(Math.PI / 2),
                    () -> {})
            .addTimedState(3f,
                    () -> robot.crane.turret.setTargetAngle(Math.PI),
                    () -> {})
            .addTimedState(3f,
                    () -> robot.crane.turret.setTargetAngle(3 * Math.PI / 2),
                    () -> {})
            .addTimedState(3f,
                    () -> robot.crane.turret.setTargetAngle(2 * Math.PI),
                    () -> {})
            .build();
    private void handleDiagnostic() {
        diagnostic.execute();
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
                    if (robot.getAlliance().equals(Constants.Alliance.RED)
                            && auto.autonomousRed.execute()) {
                        active = false;
                        gameState = Constants.GameState.TELE_OP;
                        gameStateIndex = Constants.GameState.indexOf(Constants.GameState.TELE_OP);
                    } else if (robot.getAlliance().equals(Constants.Alliance.BLUE)
                            && auto.autonomousBlue.execute()) {
                        active = false;
                        gameState = Constants.GameState.TELE_OP;
                        gameStateIndex = Constants.GameState.indexOf(Constants.GameState.TELE_OP);
                    }
                    break;
//                case DIAGNOSTIC:
//                    handleDiagnostic();
//                    break;
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

    private void updateMostFrequentPosition() {
        int mostFrequentPositionCount = -1;
        for(Map.Entry<org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position, Integer> entry: positionFrequencies.entrySet()) {
            int positionFrequency = entry.getValue();
            if(positionFrequency > mostFrequentPositionCount) {
                mostFrequentPositionCount = positionFrequency;
                mostFrequentPosition = entry.getKey();
            }
        }
    }

    private void sendVisionImage() {
        Mat mat = auto.visionProvider.getDashboardImage();
        if(mat != null && mat.width() > 0 && mat.height() > 0) {
            Bitmap bm = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(mat, bm);
            dashboard.sendImage(bm);
        }
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
            if(initializing)
                opModeTelemetryMap.put("Most Frequent Detected Position", mostFrequentPosition);
            opModeTelemetryMap.put("Average Loop Time", String.format("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
            opModeTelemetryMap.put("Last Loop Time", String.format("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));
        }
        opModeTelemetryMap.put("Active", active);
        opModeTelemetryMap.put("State", String.format("(%d): %s", gameStateIndex, gameState));
        opModeTelemetryMap.put("Dashboard Enabled", dashboardEnabled);
        opModeTelemetryMap.put("Debug Telemetry Enabled", debugTelemetryEnabled);
        opModeTelemetryMap.put("Using Desmos Drive", usingDesmosDrive);
        opModeTelemetryMap.put("Chassis Level Index", String.format("%d / %d", chassisDistanceLevelIndex, CHASSIS_DISTANCE_LEVELS.length));

        switch(gameState) {
            case TELE_OP:
                opModeTelemetryMap.put("Smoothing Enabled", smoothingEnabled);
                break;
            case AUTONOMOUS:
                break;
        }
        handleTelemetry(opModeTelemetryMap, gameState.getName(), packet);

        // handling subsystem telemetry
        for(TelemetryProvider telemetryProvider: robot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);
        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

        // handling vision telemetry
        Map<String, Object> visionTelemetryMap = new HashMap<>();
        if(initializing) {
//            visionTelemetryMap.put("Detected Position", auto.visionProvider.getPosition());
        }
//        handleTelemetry(visionTelemetryMap, auto.visionProvider.getTelemetryName(), packet);

        if(dashboardEnabled) {
//            if(auto.visionProvider.canSendDashboardImage())
//                sendVisionImage();
//            robot.drawFieldOverlay(packet);
            dashboard.sendTelemetryPacket(packet);
        }


        telemetry.update();
    }

    private boolean joysticksActive(Gamepad gamepad) {
        return  UtilMethods.notDeadZone(gamepad.left_stick_x, JOYSTICK_DEADZONE_THRESHOLD) &&
                UtilMethods.notDeadZone(gamepad.left_stick_y, JOYSTICK_DEADZONE_THRESHOLD) &&
                UtilMethods.notDeadZone(gamepad.right_stick_x, JOYSTICK_DEADZONE_THRESHOLD) &&
                UtilMethods.notDeadZone(gamepad.right_stick_y, JOYSTICK_DEADZONE_THRESHOLD);
    }

    private void update() {
        if(initializing) {
//            if(robot.isDebugTelemetryEnabled())
//                updateMostFrequentPosition();
//            auto.visionProvider.update();
//            Position position = auto.visionProvider.getPosition();
//            if(position != null)
//                positionFrequencies.put(position, positionFrequencies.get(position) + 1);
        }

        gamepad1JoysticksActive = joysticksActive(gamepad1);
        gamepad2JoysticksActive = joysticksActive(gamepad2);

        updateTelemetry();

        updateTiming();

        stickyGamepad1.update();
        stickyGamepad2.update();

        robot.update();
    }
}

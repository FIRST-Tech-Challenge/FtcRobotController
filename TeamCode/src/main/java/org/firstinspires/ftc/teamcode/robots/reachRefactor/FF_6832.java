package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import android.graphics.Bitmap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
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
 * left bumper - decrement state
 * right bumper - increment state
 *
 * Tele-Op
 * dpad up - toggle desmos drive
 * dpad down - toggle debug telemetry
 * right stick y - forward
 * left stick x - rotate
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
    private Map<Position, Integer> positionFrequencies;
    private Position mostFrequentPosition;
    private boolean usingDesmosDrive;
    private boolean dashboardEnabled, debugTelemetryEnabled, smoothingEnabled;

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

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // global state
        active = true;
        initializing = true;
        dashboardEnabled = Constants.DEFAULT_DASHBOARD_ENABLED;
        gameState = Constants.GameState.TELE_OP;

        // TPM calibration state
        TPMCalibrationInitialized = false;
        TPMCalibrationStartingTicks = new SimpleMatrix(1, 3);

        // timing
        lastLoopClockTime = System.nanoTime();
        loopTimeSmoother = new ExponentialSmoother(Constants.AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);

        // gamepads
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        robot = new Robot(hardwareMap);
        auto = new Autonomous(robot);

        // vision
//        robot.createVisionProvider(VisionProviders.DEFAULT_PROVIDER_INDEX);
//        auto.visionProvider.initializeVision(hardwareMap);
//        visionProviderFinalized = true;

        if(dashboardEnabled)
            dashboard = FtcDashboard.getInstance();

        positionFrequencies = new HashMap<Position, Integer>() {{
            put(Position.LEFT, 0);
            put(Position.MIDDLE, 0);
            put(Position.RIGHT, 0);
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
//        auto.visionProvider.shutdownVision();
    }

    private void handleTeleOpDrive() {
        double forward = Math.pow(-gamepad1.right_stick_y, 3) * Constants.FORWARD_SCALING_FACTOR;
        double rotate = Math.pow(gamepad1.left_stick_x, 3) * Constants.ROTATE_SCALING_FACTOR;


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
        handleTeleOpDrive();
        handleEmergencyStop();

        if(stickyGamepad1.dpad_up) {
            usingDesmosDrive = !usingDesmosDrive;
        }
        if(stickyGamepad1.dpad_down) {
            debugTelemetryEnabled = !debugTelemetryEnabled;
        }
    }

    private void handleTPMCalibration() {
        handleTeleOpDrive();
        handleEmergencyStop();

        // initializing TPM calibration state
        if(!TPMCalibrationInitialized) {
            TPMCalibrationStartingTicks = robot.driveTrain.getWheelTicks().cols(0, 1);
            TPMCalibrationInitialized = true;
        }

        // calculating average wheel ticks traveled
        SimpleMatrix ticks = robot.driveTrain.getWheelTicks().cols(0, 1);
        SimpleMatrix ones = new SimpleMatrix(new double[][] {{1, 1, 1}});
        averageTPMCalibrationTicksTraveled = ones.mult(ticks.minus(TPMCalibrationStartingTicks)).divide(3).get(0);
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
                case TPM_CALIBRATION:
                    handleTPMCalibration();
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

    private void updateMostFrequentPosition() {
        int mostFrequentPositionCount = -1;
        for(Map.Entry<Position, Integer> entry: positionFrequencies.entrySet()) {
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
        packet.addLine(telemetryName);
        packet.putAll(telemetryMap);
        packet.addLine("");

        telemetry.addLine(telemetryName);
        for(Map.Entry<String, Object> entry: telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
        telemetry.addLine();
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

        switch(gameState) {
            case TELE_OP:
                opModeTelemetryMap.put("Smoothing Enabled", smoothingEnabled);
                break;
            case AUTONOMOUS:
                break;
            case TPM_CALIBRATION:
                opModeTelemetryMap.put("Average Ticks Traveled", averageTPMCalibrationTicksTraveled);
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

    private void update() {
        if(initializing) {
//            if(robot.isDebugTelemetryEnabled())
//                updateMostFrequentPosition();
//            auto.visionProvider.update();
//            Position position = auto.visionProvider.getPosition();
//            if(position != null)
//                positionFrequencies.put(position, positionFrequencies.get(position) + 1);
        }

        updateTelemetry();

        updateTiming();

        stickyGamepad1.update();
        stickyGamepad2.update();

        robot.update();
    }
}

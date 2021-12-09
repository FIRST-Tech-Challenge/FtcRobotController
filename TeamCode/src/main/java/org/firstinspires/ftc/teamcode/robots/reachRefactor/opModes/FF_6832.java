package org.firstinspires.ftc.teamcode.robots.reachRefactor.opModes;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;

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
 * right stick y - forward
 * left stick x - rotate
 * guide - emergency stop
 */
@TeleOp(name = "refactored FF_6832")
public class FF_6832 extends OpMode {
    private Robot robot;

    // global state
    private boolean active, initializing;
    private Constants.GameState gameState;
    private int gameStateIndex, visionProviderIndex;
    private boolean visionProviderFinalized;
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private Map<Position, Integer> positionFrequencies;
    private Position mostFrequentPosition;

    // TPM Calibration state
    private boolean TPMCalibrationInitialized;
    private SimpleMatrix TPMCalibrationStartingTicks;
    private double averageTPMCalibrationTicksTraveled;

    // timing
    private long lastLoopClockTime, loopTime;
    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // global state
        active = true;
        initializing = true;
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

        robot = new Robot(hardwareMap, telemetry, Constants.DEFAULT_DASHBOARD_ENABLED);

        // vision
        robot.createVisionProvider(VisionProviders.DEFAULT_PROVIDER_INDEX);
        robot.visionProvider.initializeVision(hardwareMap);
        visionProviderFinalized = true;

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
                    robot.createVisionProvider(visionProviderIndex);
                }
                if (stickyGamepad1.dpad_up) {
                    robot.visionProvider.initializeVision(hardwareMap); // this is blocking
                    visionProviderFinalized = true;
                }
            } else if (stickyGamepad1.dpad_up) {
                robot.visionProvider.shutdownVision(); // also blocking, but should be very quick
                visionProviderFinalized = false;
            }
        }
    }

    private void handlePregameControls() {
        if(stickyGamepad1.x)
            robot.setAlliance(Constants.Alliance.BLUE);
        if(stickyGamepad1.b)
            robot.setAlliance(Constants.Alliance.RED);

        if(stickyGamepad1.a)
            robot.toggleIsDashboardEnabled();
        if(stickyGamepad1.y)
            robot.driveTrain.toggleSmoothingEnabled();
        if(stickyGamepad1.dpad_down) {
            robot.toggleIsDebugTelemetryEnabled();
        }
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
        robot.setMostFrequentPosition(mostFrequentPosition);
        lastLoopClockTime = System.nanoTime();
        initializing = false;
    }

    private void handleTeleOpDrive() {
        double forward = Math.pow(-gamepad1.right_stick_y, 3) * Constants.FORWARD_SCALING_FACTOR;
        double rotate = Math.pow(gamepad1.left_stick_x, 3) * Constants.ROTATE_SCALING_FACTOR;

        robot.driveTrain.drive(forward, rotate);
    }

    private void handleEmergencyStop() {
        if(stickyGamepad1.guide || stickyGamepad2.guide)
            robot.stop();
    }

    private void handleTeleOp() {
        handleTeleOpDrive();
        handleEmergencyStop();
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
                            && robot.articulate(Robot.Articulation.AUTONOMOUS_RED)) {
                        active = false;
                        gameState = Constants.GameState.TELE_OP;
                        gameStateIndex = Constants.GameState.indexOf(Constants.GameState.TELE_OP);
                    } else if (robot.getAlliance().equals(Constants.Alliance.BLUE)
                            && robot.articulate(Robot.Articulation.AUTONOMOUS_BLUE)) {
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

    private void handleTelemetry() {
        if(initializing) {
            robot.addTelemetryData("Detected Position", robot.visionProvider.getPosition());
        }

        if(robot.isDebugTelemetryEnabled()) {
            if(initializing) {
                // finding the most frequently detected position
                int mostFrequentPositionCount = -1;
                for(Map.Entry<Position, Integer> entry: positionFrequencies.entrySet()) {
                    int positionFrequency = entry.getValue();
                    if(positionFrequency > mostFrequentPositionCount) {
                        mostFrequentPositionCount = positionFrequency;
                        mostFrequentPosition = entry.getKey();
                    }
                }

                robot.addTelemetryData("Most Frequent Detected Position", mostFrequentPosition);
            }
            robot.addTelemetryData("Average Loop Time", String.format("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
            robot.addTelemetryData("Last Loop Time", String.format("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));
        }
        robot.addTelemetryData("Active", active);
        robot.addTelemetryData("State", String.format("(%d): %s", gameStateIndex, gameState));
        robot.addTelemetryData("Smoothing Enabled", robot.driveTrain.isSmoothingEnabled());
        robot.addTelemetryData("Dashboard Enabled", robot.isDashboardEnabled());
        robot.addTelemetryData("Debug Telemetry Enabled", robot.isDebugTelemetryEnabled());

        switch(gameState) {
            case TELE_OP:
                break;
            case AUTONOMOUS:
                break;
            case TPM_CALIBRATION:
                robot.addTelemetryData("Average Ticks Traveled", averageTPMCalibrationTicksTraveled);
                break;
        }
    }

    private void update() {
        if(initializing) {
            robot.visionProvider.update();
            Position position = robot.visionProvider.getPosition();
            if(position != null)
                // updating frequency of position detections
                positionFrequencies.put(position, positionFrequencies.get(position) + 1);
        }

        handleTelemetry();

        updateTiming();

        stickyGamepad1.update();
        stickyGamepad2.update();

        robot.update();
    }
}

/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.util.jackiebot;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.drive.mecanum.DriveConstants;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.drive.mecanum.TrajectoryFollowerState;
import com.hfrobots.tnt.corelib.drive.mecanum.TurnState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StateMachine;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.hfrobots.tnt.season1819.TntPose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Autonomous(name="Jackie Auto Test", group="Utilities")
@Disabled
@SuppressWarnings("unused")
public class JackiebotAuto extends OpMode {
    /**
     * Describes the tuned characteristics of the drivebase for Jackiebot
     */
    static class JackieDriveConstants extends DriveConstants {
        private static DriveConstraints DRIVE_CONSTRAINTS = new DriveConstraints(40, 20.0, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0);
        private static PIDCoefficients TRANSLATIONAL_PID_COEFFICIENTS = new PIDCoefficients(4.2D, 0, 0);
        private static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0.295D, 0, 0);

        @Override
        public DriveConstraints getBaseConstraints() {
            return DRIVE_CONSTRAINTS;
        }

        @Override
        public PIDCoefficients getTranslationalPID() {
            return TRANSLATIONAL_PID_COEFFICIENTS;
        }

        @Override
        public PIDCoefficients getHeadingPid() {
            return HEADING_PID_COEFFICIENTS;
        }
    }

    private Ticker ticker;

    private RoadRunnerMecanumDriveREV driveBase;

    private StateMachine stateMachine;

    // The routes our robot knows how to do
    private enum Routes {
        MOVE_FOUNDATION("Move Foundation"),
        SCAN_SKYSTONES("Scan Skystones"),
        PARK_LEFT_NEAR_POS("Park from left to near"),
        PARK_RIGHT_NEAR_POS("Park from right to near"),
        PARK_LEFT_FAR_POS("Park from left to far"),
        PARK_RIGHT_FAR_POS("Park from right to far");

        final String description;

        Routes(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    private int selectedRoutesIndex = 0;

    private Routes[] possibleRoutes = Routes.values();

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Constants.Alliance currentAlliance = Constants.Alliance.RED;

    private int initialDelaySeconds = 0;

    @Override
    public void init() {
        ticker = createAndroidTicker();

        setupDriverControls();

        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);
        driveBase = new RoadRunnerMecanumDriveREV(new JackieDriveConstants(), simplerHardwareMap, true);

        stateMachine = new StateMachine(telemetry);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    private boolean configLocked = false;

    @Override
    public void init_loop() {
        if (driversGamepad == null) { // safety, need to double check whether we actually need this
            // not ready yet init() hasn't been called
            return;
        }

        if (!configLocked) {
            doAutoConfig();

            if (lockButton.getRise()) {
                configLocked = true;
            }
        } else {
            if (unlockButton.getRise()) {
                configLocked = false;
            }
        }

        if (configLocked) {
            telemetry.addData("00", "LOCKED: Press Rt stick unlock");
        } else {
            telemetry.addData("00", "UNLOCKED: Press Lt stick lock");
        }

        telemetry.addData("01", "Alliance: %s", currentAlliance);
        telemetry.addData("02", "Task: %s", possibleRoutes[selectedRoutesIndex].getDescription());
        telemetry.addData("03", "Delay %d sec", initialDelaySeconds);

        updateTelemetry(telemetry);
    }

    private Ticker createAndroidTicker() {
        return new Ticker() {
            public long read() {
                return android.os.SystemClock.elapsedRealtimeNanos();
            }
        };
    }

    private void doAutoConfig() {
        // Use driver dpad up/down to select which route to run
        if (driverDpadDown.getRise()) {
            selectedRoutesIndex--;

            if (selectedRoutesIndex < 0) {
                selectedRoutesIndex = possibleRoutes.length - 1; // why?
            }
        } else if (driverDpadUp.getRise()) {
            selectedRoutesIndex++;

            if (selectedRoutesIndex > possibleRoutes.length - 1) { // why -1?
                selectedRoutesIndex = 0;
            }
        }

        // use left/right bumper to decrease/increase delay

        if (driverLeftBumper.getRise()) {
            initialDelaySeconds -= 1;

            if (initialDelaySeconds < 0) {
                initialDelaySeconds = 0;
            }
        } else if (driverRightBumper.getRise()) {
            initialDelaySeconds += 1;

            if (initialDelaySeconds > 10) {
                initialDelaySeconds = 10;
            }
        }

        // Alliance selection
        if (driverBRedButton.getRise()) {
            currentAlliance = Constants.Alliance.RED;
        } else if (driverXBlueButton.getRise()) {
            currentAlliance = Constants.Alliance.BLUE;
        }
    }

    private boolean stateMachineSetup = false;

    @Override
    public void loop() {
        try {
            if (!stateMachineSetup) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case PARK_LEFT_NEAR_POS:
                        setupParkFromLeftNear();

                        break;
                    case PARK_RIGHT_NEAR_POS:
                        setupParkFromRightNear();
                        break;
                    case PARK_LEFT_FAR_POS:
                        setupParkFromLeftFar();
                        break;
                    case PARK_RIGHT_FAR_POS:
                        setupParkFromRightFar();
                        break;
                    case SCAN_SKYSTONES:
                        setupScanSkytones();
                        break;
                    case MOVE_FOUNDATION:
                        setupMoveFoundation();
                        break;
                    default:
                        stateMachine.addSequential(newDoneState("Default done"));
                        break;
                }

                if (initialDelaySeconds != 0) {
                    stateMachine.addStartDelay(initialDelaySeconds);
                }

                stateMachineSetup = true;
            }

            stateMachine.doOneStateLoop();

            telemetry.update(); // send all telemetry to the drivers' station
        } catch (Throwable t) {
            // Better logging than the FTC SDK provides :(
            Log.e(LOG_TAG, "Exception during state machine", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException)t;
            }

            RuntimeException rte = new RuntimeException();
            rte.initCause(t);

            throw rte;
        }
    }

    protected void setupParkFromLeftNear() {
        setupParkCommon("Park from the left near", 20, 2);
    }

    protected void setupParkFromLeftFar() {
        setupParkCommon("Park from the left far", 20, 16);
    }

    protected void setupParkFromRightNear() {
        // Robot starts against wall, to right of tape line
        setupParkCommon("Park from the right near", -20, 2);
    }

    protected void setupParkFromRightFar() {
        setupParkCommon("Park from the right far", -20, 16);
    }

    protected void setupParkCommon(String stateName, final double strafeDistance, final double forwardDistance) {
        // Robot starts against wall, to left or right of tape line

        State parkTrajectoryState = new TrajectoryFollowerState(stateName,
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
                @Override
                protected Trajectory createTrajectory() {
                    TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                    trajectoryBuilder.forward(forwardDistance);

                    if (strafeDistance < 0) {
                        trajectoryBuilder.strafeLeft(Math.abs(strafeDistance));
                    } else {
                        trajectoryBuilder.strafeRight(Math.abs(strafeDistance));
                    }

                    return trajectoryBuilder.build();
                }
        };

        stateMachine.addSequential(parkTrajectoryState);
        stateMachine.addSequential(newDoneState("Done!"));
    }

    protected void setupScanSkytones() {
        // Robot starts against wall, depot side of tape line

        State straightTrajectoryState = new TrajectoryFollowerState("Straight",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                return driveBase.trajectoryBuilder().lineTo(TntPose2d.toVector2d(0, 28), new ConstantInterpolator(0))
                        .lineTo(TntPose2d.toVector2d(-20, 28), new ConstantInterpolator(0))
                        .build();
            }
        };

        stateMachine.addSequential(straightTrajectoryState);
        stateMachine.addSequential(newDoneState("Done!"));
    }

    protected void setupMoveFoundation() {
        // Robot starts against wall, left side on tile seam nearest to tape

        State splineTrajectoryState = new TrajectoryFollowerState("Spline",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                return driveBase.trajectoryBuilder().splineTo(TntPose2d.toPose2d(9, 28))
                        .build();
            }
        };

        // GRIP!

        // Turn 90 degrees clockwise
        State turnWithBase = new TurnState("Turn with base",
                telemetry, new Turn(Rotation.CW, 90), driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000));

        // Forward 5-6"

        State forwardTrajectoryState = new TrajectoryFollowerState("Spline",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                return driveBase.trajectoryBuilder().forward(6)
                        .build();
            }
        };

        // UNGRIP

        // Win!

        stateMachine.addSequential(splineTrajectoryState);

        // REMOVE AFTER TESTING
        {
            stateMachine.addSequential(newDelayState("wait to turn", 2));
            stateMachine.addSequential(turnWithBase);
            stateMachine.addSequential(forwardTrajectoryState);
        }

        stateMachine.addSequential(newDoneState("Done!"));
    }

    /**
     * Creates an instance of the "delay" state which waits the number of seconds before
     * advancing to the next state
     */
    protected State newDelayState() {
        return newDelayState("start delay", initialDelaySeconds);
    }

    // TODO: Move to new model of controllers after first league meet
    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected RangeInput driverRightStickX;

    protected RangeInput driverRightStickY;

    protected RangeInput driveForwardReverse;

    protected RangeInput driveStrafe;

    protected RangeInput driveRotate;

    protected DebouncedButton driverDpadUp;

    protected DebouncedButton driverDpadDown;

    protected DebouncedButton driverDpadLeft;

    protected DebouncedButton driverDpadRight;

    protected DebouncedButton driverXBlueButton;

    protected DebouncedButton driverBRedButton;

    protected DebouncedButton driverYYellowButton;

    protected DebouncedButton driverAGreenButton;

    protected DebouncedButton driverRightBumper;

    protected DebouncedButton driverLeftBumper;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected NinjaGamePad driversGamepad;

    private void setupDriverControls() {
        driversGamepad = new NinjaGamePad(gamepad1);
        driverLeftStickX = driversGamepad.getLeftStickX();
        driverLeftStickY = driversGamepad.getLeftStickY();
        driverRightStickX = driversGamepad.getRightStickX();
        driverRightStickY = driversGamepad.getRightStickY();

        driverDpadDown = new DebouncedButton(driversGamepad.getDpadDown());
        driverDpadUp = new DebouncedButton(driversGamepad.getDpadUp());
        driverDpadLeft = new DebouncedButton(driversGamepad.getDpadLeft());
        driverDpadRight = new DebouncedButton(driversGamepad.getDpadRight());
        driverAGreenButton = new DebouncedButton(driversGamepad.getAButton());
        driverBRedButton = new DebouncedButton(driversGamepad.getBButton());
        driverXBlueButton = new DebouncedButton(driversGamepad.getXButton());
        driverYYellowButton = new DebouncedButton(driversGamepad.getYButton());
        driverLeftBumper = new DebouncedButton(driversGamepad.getLeftBumper());
        driverRightBumper = new DebouncedButton(driversGamepad.getRightBumper());
        lockButton = new DebouncedButton(driversGamepad.getLeftStickButton());
        unlockButton = new DebouncedButton(driversGamepad.getRightStickButton());
    }

    protected State newDelayState(String name, final int numberOfSeconds) {
        return new State(name, telemetry) {

            private long startTime = 0;
            private long thresholdTimeMs = TimeUnit.SECONDS.toMillis(numberOfSeconds);

            @Override
            public void resetToStart() {
                startTime = 0;
            }

            @Override
            public void liveConfigure(DebouncedGamepadButtons buttons) {

            }

            @Override
            public State doStuffAndGetNextState() {
                if (startTime == 0) {
                    startTime = System.currentTimeMillis();
                    return this;
                }

                long now = System.currentTimeMillis();
                long elapsedMs = now - startTime;

                if (elapsedMs > thresholdTimeMs) {
                    return nextState;
                }

                telemetry.addData("04", "Delay: %d of %d ms", elapsedMs, thresholdTimeMs);
                return this;
            }
        };
    }

    /**
     * Creates an instance of the "done" state which stops the robot and should be the
     * "end" state of all of our robot's state machines
     */
    protected State newDoneState(String name) {
        return new State(name, telemetry) {
            private boolean issuedStop = false;

            @Override
            public State doStuffAndGetNextState() {
                if (!issuedStop) {
                    driveBase.setMotorPowers(0, 0, 0, 0);

                    issuedStop = true;
                }

                return this;
            }

            @Override
            public void resetToStart() {
                issuedStop = false;
            }

            @Override
            public void liveConfigure(DebouncedGamepadButtons buttons) {

            }
        };
    }
}

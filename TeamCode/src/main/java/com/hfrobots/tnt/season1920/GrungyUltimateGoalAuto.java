/*
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
 */

package com.hfrobots.tnt.season1920;

import android.util.Log;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.drive.mecanum.TrajectoryFollowerState;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.hfrobots.tnt.season2021.StarterStackDetectorPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Autonomous(name="00 Grungy UG Auto")
@SuppressWarnings("unused")
public class GrungyUltimateGoalAuto extends OpMode {
    private Ticker ticker;

    private OnOffButton unsafe = new OnOffButton() {
        @Override
        public boolean isPressed() {
            return true;
            //Danger Will Robinson
        }

        @Override
        public DebouncedButton debounced() {
            return null;
        }
    };

    private RoadRunnerMecanumDriveREV driveBase;

    private StateMachine stateMachine;

    private enum ParkOrLane {
        WALL,
        NEUTRAL_SKYBRIDGE
    }

    // The routes our robot knows how to do
    private enum Routes {
        DEPOSIT_WOBBLE_GOAL("Deposit Wobble Goal");

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

    private FoundationGripMechanism foundationGripper;

    private DeliveryMechanism deliveryMechanism;

    // Detector object
    //private TntSkystoneDetector detector;

    //private EasyOpenCvPipelineAndCamera pipelineAndCamera;

    private SkystoneGrabber skystoneGrabber;

    //private ParkingSticks parkingSticks;

    private CapstoneMechanism capstoneMechanism;

    @Override
    public void init() {
        ticker = createAndroidTicker();

        setupDriverControls();

        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);

        driveBase = new RoadRunnerMecanumDriveREV(new SkystoneDriveConstants(),
                simplerHardwareMap, true);

        foundationGripper = new FoundationGripMechanism(simplerHardwareMap);

        stateMachine = new StateMachine(telemetry);

        deliveryMechanism = new DeliveryMechanism(simplerHardwareMap, telemetry, ticker);

        deliveryMechanism.setUnsafe(unsafe);

        skystoneGrabber = new SkystoneGrabber(simplerHardwareMap);

        capstoneMechanism = new CapstoneMechanism(simplerHardwareMap, telemetry, ticker);
    }

    private com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera pipelineAndCamera;

    private void setupOpenCvCameraAndPipeline() {
        OpenCvPipeline detector = StarterStackDetectorPipeline.builder().telemetry(telemetry).build();

        com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.EasyOpenCvPipelineAndCameraBuilder pipelineBuilder =
                com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.builder();

        pipelineBuilder.hardwareMap(hardwareMap).telemetry(telemetry).openCvPipeline(detector);

        pipelineAndCamera = pipelineBuilder.build();

        pipelineAndCamera.createAndRunPipeline();
    }

    @Override
    public void start() {
        super.start();
        foundationGripper.initPos();
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

            if (initialDelaySeconds > 25) {
                initialDelaySeconds = 25;
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
            capstoneMechanism.periodicTask(); // just to maintain holding
        } catch (Throwable t) {
            Log.e(LOG_TAG, "Capstone error", t);
        }

        try {
            if (!stateMachineSetup) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case DEPOSIT_WOBBLE_GOAL:
                        setupDeliverWobbleGoal();
                        break;
                    default:
                        stateMachine.addSequential(newDoneState("Default done"));
                        break;
                }

                if (initialDelaySeconds != 0) {
                    stateMachine.addStartDelay(initialDelaySeconds, Ticker.systemTicker());
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

    enum Target {
        A, B, C;
    }

    protected void setupDeliverWobbleGoal() {
        // Robot starts against wall, depot side of tape line

        //1. Find the skystone


        //State detectionState = new SkystoneDetectionState(pipelineAndCamera,
        //        detector, telemetry, ticker);

        //2. Drive straight to the quarry (-3 inches)
        //
        // Targets, A, B, C


        //3. strafe to the skystone (centered)
        final Target target = Target.A;

        State toTargetZone = new TrajectoryFollowerState("To skystone",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();


                switch (target) {
                    case A:
                            trajectoryBuilder.forward(58);
                        break;
                    case B:
                        trajectoryBuilder.forward(82);

                        if(currentAlliance == Constants.Alliance.RED) {
                            trajectoryBuilder.strafeLeft(24);
                        } else {
                            trajectoryBuilder.strafeRight(24);
                        }
                        break;
                    case C:
                        trajectoryBuilder.forward(106);
                        break;
                }

                return trajectoryBuilder.build();
            }
        };

        State wobbleGoalWaitState = newDelayState("Deposit Wobble Goal", 4);

        State toParkedPosition = new TrajectoryFollowerState("Parking",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                switch (target) {
                    case A:
                        trajectoryBuilder.strafeLeft(22).forward(12);
                        break;
                    case B:
                        trajectoryBuilder.back(14);
                        break;
                    case C:
                        trajectoryBuilder.back(38);
                        break;
                }

                return trajectoryBuilder.build();
            }
        };

        // toParkedPosition
        // ejectWobbleGoalState
        // wobbleGoalWaitState
        // wobbleGoalCoolDownState

        // toTargetZone

        State ejectWobbleGoalState = new State("Eject wobble goal", telemetry) {

            @Override
            public State doStuffAndGetNextState() {
                deliveryMechanism.setIntakeVelocity(-.2);
                return nextState;
            }

            @Override
            public void resetToStart() {

            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };

        State wobbleGoalCoolDownState = new State("Stop the delivey Mech",telemetry){
            @Override
            public State doStuffAndGetNextState() {
                deliveryMechanism.setIntakeVelocity(0);
                return nextState;
            }

            @Override
            public void resetToStart() {

            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };

        stateMachine.addSequential(toTargetZone);
        stateMachine.addSequential(ejectWobbleGoalState);
        stateMachine.addSequential(wobbleGoalWaitState);
        stateMachine.addSequential(wobbleGoalCoolDownState);
        stateMachine.addSequential(toParkedPosition);
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
            public void liveConfigure(NinjaGamePad gamePad) {

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
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };
    }
}

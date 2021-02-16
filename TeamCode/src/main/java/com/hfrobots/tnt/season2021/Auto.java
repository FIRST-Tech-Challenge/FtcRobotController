/*
 Copyright (c) 2021 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season2021;

import android.util.Log;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.drive.mecanum.TrajectoryFollowerState;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.hfrobots.tnt.season1920.CapstoneMechanism;
import com.hfrobots.tnt.season1920.DeliveryMechanism;
import com.hfrobots.tnt.season1920.FoundationGripMechanism;
import com.hfrobots.tnt.season1920.GrungyUltimateGoalAuto;
import com.hfrobots.tnt.season1920.SkystoneDriveConstants;
import com.hfrobots.tnt.season1920.SkystoneGrabber;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Autonomous(name="00 UltGoal Auto")
public class Auto extends OpMode {
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
    private StarterStackDetectorPipeline starterStackDetectorPipeline;
    private WobbleGoal wobbleGoal;

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

    private RevBlinkinLedDriver blinkinLed;

    private OperatorControls operatorControls;

    @Override
    public void init() {
        ticker = createAndroidTicker();

        setupDriverControls();

        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);

        driveBase = new RoadRunnerMecanumDriveREV(new SkystoneDriveConstants(),
                simplerHardwareMap, true);

        stateMachine = new StateMachine(telemetry);

        setupOpenCvCameraAndPipeline();

        ScoringMechanism scoringMechanism = ScoringMechanism.builder()
                .hardwareMap(hardwareMap)
                .telemetry(telemetry)
                .ticker(Ticker.systemTicker()).build();

        wobbleGoal = WobbleGoal.builder().hardwareMap(hardwareMap)
                .telemetry(telemetry).ticker(Ticker.systemTicker()).build();

        operatorControls = OperatorControls.builder().operatorGamepad(new NinjaGamePad(gamepad2))
                .scoringMechanism(scoringMechanism)
                .wobbleGoal(wobbleGoal).build();

        blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
    }

    private com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera pipelineAndCamera;

    private void setupOpenCvCameraAndPipeline() {
        starterStackDetectorPipeline = StarterStackDetectorPipeline.builder().telemetry(telemetry).build();

        com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.EasyOpenCvPipelineAndCameraBuilder pipelineBuilder =
                com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.builder();

        pipelineBuilder.hardwareMap(hardwareMap).telemetry(telemetry).openCvPipeline(starterStackDetectorPipeline);

        pipelineAndCamera = pipelineBuilder.build();

        pipelineAndCamera.createAndRunPipeline();
    }

    @Override
    public void start() {
        super.start();
        starterStackDetectorPipeline.setStartLookingForRings(true);
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

        if (operatorControls != null) {
            operatorControls.periodicTask();
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

            wobbleGoal.periodicTask();

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

    private Target deliverToTarget = Target.A; // To start with

    protected void setupDeliverWobbleGoal() {
        State detectorState = new StopwatchTimeoutSafetyState(
                "Detecting starter stack",
                telemetry,
                Ticker.systemTicker(),
                TimeUnit.SECONDS.toMillis(10)) {
            @Override
            public State doStuffAndGetNextState() {
                StarterStackDetectorPipeline.RingsDetected ringsDetected = starterStackDetectorPipeline.getRingsDetected();

                switch (ringsDetected) {

                    case UNKNOWN:
                        // Keep looking if not timed out
                        if (!isTimedOut()){
                            return this;
                        }

                        deliverToTarget = Target.A;
                        Log.d(LOG_TAG, "Unknown rings detected, Zone A selected" );

                        break;
                    case ZERO:
                        deliverToTarget = Target.A;
                        Log.d(LOG_TAG, "Zero rings detected, Zone A selected" );

                        break;
                    case ONE:
                        deliverToTarget = Target.B;
                        Log.d(LOG_TAG, "One rings detected, Zone B selected" );

                        break;
                    case FOUR:
                        deliverToTarget = Target.C;
                        Log.d(LOG_TAG, "Four rings detected, Zone C selected" );

                        break;
                    case SOME:
                        //if we see orange it is more likely we see between 1 and 4 and should lean towards 4
                        deliverToTarget = Target.C;
                        Log.d(LOG_TAG, "Some rings detected, Zone C selected" );

                        break;
                }

                return nextState;
            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };

        State toTargetZone = new TrajectoryFollowerState("To skystone",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();


                switch (deliverToTarget) {
                    case A:
                        trajectoryBuilder.forward(58);
                        break;
                    case B:
                        trajectoryBuilder.forward(82);

                        if(currentAlliance == Constants.Alliance.RED) {
                            trajectoryBuilder.strafeLeft(28);
                        } else {
                            trajectoryBuilder.strafeRight(24); //blue Needs testing
                        }
                        break;
                    case C:
                        trajectoryBuilder.forward(106);
                        break;
                }

                return trajectoryBuilder.build();
            }
        };

        State wobbleGoalToPlaceState = new StopwatchTimeoutSafetyState("wobbleMovingToPlace",
                telemetry, ticker, TimeUnit.SECONDS.toMillis(10)) {
            @Override
            public State doStuffAndGetNextState() {
                Class<? extends State> wobbleStateClass = wobbleGoal.getCurrentState().getClass();

                if (!wobbleStateClass.equals(WobbleGoal.AutoPlaceState.class)) {
                    wobbleGoal.gotoPlaceState();

                    return this;
                }

                if (isTimedOut()) {
                    resetTimer();

                    return nextState;
                }

                if (!wobbleStateClass.equals(WobbleGoal.PlaceState.class)) {
                    return nextState;
                }

                return this;
            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };

        State toParkedPosition = new TrajectoryFollowerState("Parking",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                switch (deliverToTarget) {
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

        State dropWobbleGoalState = new State("Drop wobble goal", telemetry) {

            @Override
            public State doStuffAndGetNextState() {
                return nextState;
            }

            @Override
            public void resetToStart() {

            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };

        State wobbleGoalCoolDownState = new State("Close servo, back to stow", telemetry){
            @Override
            public State doStuffAndGetNextState() {
                Class<? extends State> wobbleStateClass = wobbleGoal.getCurrentState().getClass();

                if (!wobbleStateClass.equals(WobbleGoal.AutoStowState.class)) {
                    wobbleGoal.gotoStowState();
                }

                return nextState;
            }

            @Override
            public void resetToStart() {

            }

            @Override
            public void liveConfigure(NinjaGamePad gamePad) {

            }
        };

        stateMachine.addSequential(detectorState);
        stateMachine.addSequential(toTargetZone);
        stateMachine.addSequential(wobbleGoalToPlaceState);
        stateMachine.addSequential(dropWobbleGoalState);
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

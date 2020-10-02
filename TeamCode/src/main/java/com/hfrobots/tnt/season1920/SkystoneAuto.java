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

import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.drive.mecanum.TrajectoryFollowerState;
import com.hfrobots.tnt.corelib.drive.mecanum.TurnState;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.hfrobots.tnt.season1920.opencv.DetectionZone;
import com.hfrobots.tnt.season1920.opencv.EasyOpenCvPipelineAndCamera;
import com.hfrobots.tnt.season1920.opencv.TntSkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.jetbrains.annotations.NotNull;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Autonomous(name="00 Skystone Auto")
@SuppressWarnings("unused")
public class SkystoneAuto extends OpMode {
    private Ticker ticker;

    private RoadRunnerMecanumDriveREV driveBase;

    private StateMachine stateMachine;

    private enum ParkOrLane {
        WALL,
        NEUTRAL_SKYBRIDGE
    }

    // The routes our robot knows how to do
    private enum Routes {
        MOVE_FOUNDATION_WALL("Move Foundation - Wall"),
        MOVE_FOUNDATION_BRIDGE("Move Foundation - Bridge"),
        DELIVER_SKYSTONE("Deliver Skystone - Bridge"),
        DELIVER_SKYSTONE_WALL_PATH("Deliver Skystone - Wall"),
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

    private FoundationGripMechanism foundationGripper;

    private DeliveryMechanism deliveryMechanism;

    // Detector object
    private TntSkystoneDetector detector;

    private EasyOpenCvPipelineAndCamera pipelineAndCamera;

    private SkystoneGrabber skystoneGrabber;

    private ParkingSticks parkingSticks;

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

        skystoneGrabber = new SkystoneGrabber(simplerHardwareMap);

        parkingSticks = new ParkingSticks(simplerHardwareMap);

        capstoneMechanism = new CapstoneMechanism(simplerHardwareMap, telemetry, ticker);

        setupOpenCvCameraAndPipeline();


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
                    case DELIVER_SKYSTONE:
                        setupBetterDeliverSkystoneNotDepot();
                        //setupDeliverSkytone();
                        break;
                    case DELIVER_SKYSTONE_WALL_PATH:
                        setupBetterDeliverSkystoneWallPath();
                        break;
                    case MOVE_FOUNDATION_WALL:
                        setupMoveFoundation(ParkOrLane.WALL);
                        break;
                    case MOVE_FOUNDATION_BRIDGE:
                        setupMoveFoundation(ParkOrLane.NEUTRAL_SKYBRIDGE);
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

    protected void setupParkFromLeftNear() {
        setupParkCommon("Park from the left near", 22, 0);
    }

    protected void setupParkFromLeftFar() {
        setupParkCommon("Park from the left far", 22, 18);
    }

    protected void setupParkFromRightNear() {
        // Robot starts against wall, to right of tape line
        setupParkCommon("Park from the right near", -22, 0);
    }

    protected void setupParkFromRightFar() {
        setupParkCommon("Park from the right far", -22, 18);
    }

    protected void setupParkCommon(String stateName, final double strafeDistance, final double forwardDistance) {
        // Robot starts against wall, to left or right of tape line

        State parkTrajectoryState = new TrajectoryFollowerState(stateName,
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (forwardDistance > 0){
                    trajectoryBuilder.forward(forwardDistance);
                }

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

    protected void setupDeliverSkytone() {
        // Robot starts against wall, depot side of tape line

        //1. Find the skystone


        State detectionState = new SkystoneDetectionState(pipelineAndCamera,
                detector, telemetry, ticker);

        //2. Drive straight to the quarry (-3 inches)

        State toQuarryState = createForwardTrajectory("To quarry", 27);

        //3. strafe to the skystone (centered)

        State toSkystoneState = new TrajectoryFollowerState("To skystone",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();
                // [ inner ][ middle ][ outer ]
                // fixMe left vs. right is alliance specific, below is red alliance, blue is flip-flopped

                String whichSkystone = detector.getBestScoringZone().getZoneName();

                switch (whichSkystone) {
                    case TntSkystoneDetector.INNER_ZONE_NAME:
                        if(currentAlliance == Constants.Alliance.RED){
                            trajectoryBuilder.strafeLeft(20);
                        } else {
                            trajectoryBuilder.strafeLeft(22);
                        }
                        break;
                    case TntSkystoneDetector.MIDDLE_ZONE_NAME:
                        if(currentAlliance == Constants.Alliance.RED){
                            trajectoryBuilder.strafeLeft(10);
                        } else {
                            trajectoryBuilder.strafeLeft(12);
                        }
                        break;
                    case TntSkystoneDetector.OUTER_ZONE_NAME:
                        if(currentAlliance == Constants.Alliance.RED){
                            trajectoryBuilder.strafeLeft(1);
                        } else {
                            trajectoryBuilder.strafeLeft(2);
                        }
                        break;
                }

                return trajectoryBuilder.build();
            }
        };

        State servoToGrabState = new RunnableState("grab the skystone", telemetry,
                new Runnable() {
                    @Override
                    public void run() {
                        skystoneGrabber.grab();
                    }
                });

        // For red alliance
        Turn turn = new Turn(Rotation.CW, 75);

        if (currentAlliance == Constants.Alliance.BLUE) {
            turn = turn.invert();
        }

        // Turn 90 degrees clockwise
        State turnToStone = new TurnState("turnToStone",
                telemetry, turn, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000));


        //6. move forward (how far/)
        State gobbleGobble = createForwardTrajectory("Yum", 4 + 1);

        //7. count for score (entire robot must cross line)

        // 7a. Go backwards 7-10

        State byeBye = createBackwardsTrajectory("bye bye", 10 + 1);


        // 7b turn 90 degrees, clockwise for red, ccw for blue
        // 7c. Go straight ????

        State toDeliver = new TrajectoryFollowerState("To Deliver ",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();
                // [ 1 ][ 2 ][ 3 ]

                // fixMe distance need to be shorter for blue alliance
                // fixMe left vs. right is allince specific, below is red alliance, blue is flip-flopped
                String whichSkystone = detector.getBestScoringZone().getZoneName();

                switch (whichSkystone) {
                    case TntSkystoneDetector.INNER_ZONE_NAME:
                        if(currentAlliance == Constants.Alliance.RED){
                            trajectoryBuilder.forward(72 - 5);
                        } else {
                            trajectoryBuilder.forward(56 - 17);
                        }
                        break;

                    case TntSkystoneDetector.MIDDLE_ZONE_NAME:
                        if(currentAlliance == Constants.Alliance.RED) {
                            trajectoryBuilder.forward(64 - 5);
                        } else {
                            trajectoryBuilder.forward(64 - 17);
                        }

                        break;

                    case TntSkystoneDetector.OUTER_ZONE_NAME:
                        if(currentAlliance == Constants.Alliance.RED){
                            trajectoryBuilder.forward(56 - 5);
                        } else {
                            trajectoryBuilder.forward(72 - 17);
                        }
                        break;
                }

                return trajectoryBuilder.build();
            }
        };


        State servoToNotGrabState =new RunnableState("un-grab the skystone", telemetry,
                new Runnable() {
                    @Override
                    public void run() {
                        skystoneGrabber.stow();
                    }
                });


        State getCentered = new TrajectoryFollowerState("get centered to park ",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if(currentAlliance == Constants.Alliance.RED){
                    trajectoryBuilder.strafeLeft(3);
                } else {
                    trajectoryBuilder.strafeRight(3);
                }

                return trajectoryBuilder.build();
            }
        };

        State backupToParkState = createBackwardsTrajectory("parking", 20);

        State strafeToPark = new TrajectoryFollowerState("strafe parking ",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if(currentAlliance == Constants.Alliance.RED){
                    trajectoryBuilder.strafeLeft(6);
                } else {
                    trajectoryBuilder.strafeRight(6);
                }

                return trajectoryBuilder.build();
            }
        };

        stateMachine.addSequential(detectionState);
        stateMachine.addSequential(toQuarryState);
        stateMachine.addSequential(toSkystoneState);
        stateMachine.addSequential(gobbleGobble);
        stateMachine.addSequential(servoToGrabState);
        stateMachine.addSequential(newDelayState("waiting for servo", 1));

        //stateMachine.addSequential(startIntakeState);

        stateMachine.addSequential(byeBye);
        stateMachine.addSequential(turnToStone);
        stateMachine.addSequential(toDeliver);
        stateMachine.addSequential(servoToNotGrabState);

        stateMachine.addSequential(newDelayState("waiting for servo again", 1));

        if(currentAlliance == Constants.Alliance.BLUE) {
            stateMachine.addSequential(getCentered);
        }

        stateMachine.addSequential(backupToParkState);
        stateMachine.addSequential(strafeToPark);

        stateMachine.addSequential(newDoneState("Done!"));
    }

    protected void setupBetterDeliverSkystoneWallPath() {
        setupBeterDeliverSkystone(0,16);
    }

    protected void setupBetterDeliverSkystoneNotDepot() {
        setupBeterDeliverSkystone(0,0);
    }

    protected void setupBeterDeliverSkystone(double deltaStart, double deltaRowBack){

        final double distanceDifferenceForward;


        final double distanceDeltaStart = deltaStart;
        final double distanceDeltaRowBack = deltaRowBack;

        //1. Find the skystone
        State detectionState = new SkystoneDetectionState(pipelineAndCamera,
                detector, telemetry, ticker);

        //2. Drive straight to the quarry (-3 inches)
        State toQuarryState = createForwardTrajectory("To quarry", 27);

        //3. Calculate difference from median distances
        String whichSkystone = detector.getBestScoringZone().getZoneName();

        //4. Strafe to the skystone
        State toSkystoneState = new TrajectoryFollowerState("To skystone",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                final double distanceDifferenceStrafe;

                //3. Calculate difference from median distances
                String whichSkystone = getSkystoneZone();

                switch (whichSkystone) {
                    case TntSkystoneDetector.INNER_ZONE_NAME:
                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceStrafe = 19;
                        } else {
                            distanceDifferenceStrafe = 20;
                        }
                        break;

                    case TntSkystoneDetector.MIDDLE_ZONE_NAME:
                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceStrafe = 9;
                        } else {
                            distanceDifferenceStrafe = 10;
                        }

                        break;

                    case TntSkystoneDetector.OUTER_ZONE_NAME:
                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceStrafe = 0;
                        } else {
                            distanceDifferenceStrafe = 0;
                        }
                        break;

                    default:

                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceStrafe = 9;
                        } else {
                            distanceDifferenceStrafe = 10;
                        }
                }

                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if(currentAlliance == Constants.Alliance.RED){
                    trajectoryBuilder.strafeLeft(1 + distanceDifferenceStrafe);
                } else {
                    trajectoryBuilder.strafeLeft(2 + distanceDifferenceStrafe);
                }
                return trajectoryBuilder.build();
            }

        };

        State servoToGrabState = new RunnableState("grab the skystone", telemetry,
                new Runnable() {
                    @Override
                    public void run() {
                        skystoneGrabber.grab();
                    }
                });

        // For red alliance
        Turn turn = new Turn(Rotation.CW, 75);

        if (currentAlliance == Constants.Alliance.BLUE) {
            turn = turn.invert();
        }

        // Turn 90 degrees clockwise
        State turnToStone = new TurnState("turnToStone",
                telemetry, turn, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000));


        //6. move forward (how far/)
        State gobbleGobble = createForwardTrajectory("Yum", 4 + 1);

        //7. count for score (entire robot must cross line)

        // 7a. Go backwards 7-10

        State byeBye = createBackwardsTrajectory("bye bye", 10 + 1 +
                distanceDeltaRowBack);


        // 7b turn 90 degrees, clockwise for red, ccw for blue
        // 7c. Go straight ????

        State toDeliver = new TrajectoryFollowerState("To Deliver ",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                //3. Calculate difference from median distances
                String whichSkystone = getSkystoneZone();

                final double distanceDifferenceForward;

                switch (whichSkystone) {
                    case TntSkystoneDetector.INNER_ZONE_NAME:
                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceForward = 8;
                        } else {
                            distanceDifferenceForward = -8;
                        }
                        break;

                    case TntSkystoneDetector.MIDDLE_ZONE_NAME:
                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceForward = 0;
                        } else {
                            distanceDifferenceForward = 0;
                        }

                        break;

                    case TntSkystoneDetector.OUTER_ZONE_NAME:
                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceForward = -8;
                        } else {
                            distanceDifferenceForward = 8;
                        }
                        break;

                    default:

                        if (currentAlliance == Constants.Alliance.RED) {
                            distanceDifferenceForward = 0;
                        } else {
                            distanceDifferenceForward = 0;
                        }
                }

                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();
                    if(currentAlliance == Constants.Alliance.RED) {
                        trajectoryBuilder.forward(64 - 5 + distanceDifferenceForward +
                                distanceDeltaStart);
                    } else {
                        trajectoryBuilder.forward(64 - 17 + distanceDifferenceForward +
                                distanceDeltaStart);
                    }

                return trajectoryBuilder.build();
            }
        };


        State servoToNotGrabState =new RunnableState("un-grab the skystone", telemetry,
                new Runnable() {
                    @Override
                    public void run() {
                        skystoneGrabber.stow();
                    }
                });


        final boolean onWall = deltaRowBack > 0;

        State getCentered = new TrajectoryFollowerState("get centered to park ",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if(currentAlliance == Constants.Alliance.RED){
                    if (onWall) {
                        trajectoryBuilder.strafeRight(3);
                    }else{
                        trajectoryBuilder.strafeLeft(3);
                    }
                } else {
                    if (onWall) {
                        trajectoryBuilder.strafeLeft(3);
                    }else{
                        trajectoryBuilder.strafeRight(3);
                    }

                }

                return trajectoryBuilder.build();
            }
        };

        State backupToParkState = createBackwardsTrajectory("parking", 20);

        State strafeToPark = new TrajectoryFollowerState("strafe parking ",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if(currentAlliance == Constants.Alliance.RED){
                    if (onWall) {
                        trajectoryBuilder.strafeRight(6);
                    }else{
                        trajectoryBuilder.strafeLeft(6);
                    }
                } else {
                    if (onWall) {
                        trajectoryBuilder.strafeLeft(6);
                    }else{
                        trajectoryBuilder.strafeRight(6);
                    }
                }

                return trajectoryBuilder.build();
            }
        };

        stateMachine.addSequential(detectionState);
        stateMachine.addSequential(toQuarryState);
        stateMachine.addSequential(toSkystoneState);
        stateMachine.addSequential(gobbleGobble);
        stateMachine.addSequential(servoToGrabState);

        /*public StopwatchDelayState(@NonNull String name,
                               @NonNull Telemetry telemetry,
                               @NonNull Ticker ticker,
                               long val,
                               @NonNull TimeUnit unit)*/


        stateMachine.addSequential(new StopwatchDelayState("waiting for servo", telemetry,
                ticker, 1500, TimeUnit.MILLISECONDS));

        stateMachine.addSequential(byeBye);
        stateMachine.addSequential(turnToStone);
        stateMachine.addSequential(toDeliver);
        stateMachine.addSequential(servoToNotGrabState);

        stateMachine.addSequential(newDelayState("waiting for servo again", 1));

        if(currentAlliance == Constants.Alliance.BLUE) {
            stateMachine.addSequential(getCentered);
        }

        stateMachine.addSequential(backupToParkState);
        stateMachine.addSequential(strafeToPark);

        stateMachine.addSequential(newDoneState("Done!"));

    }

    @NotNull
    private TrajectoryFollowerState createForwardTrajectory(final String name,
                                                            final double inchesForward) {
        return new TrajectoryFollowerState(name,
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.forward(inchesForward);

                return trajectoryBuilder.build();
            }
        };
    }

    private TrajectoryFollowerState createBackwardsTrajectory(final String name,
                                                            final double inchesBackwards) {
        return new TrajectoryFollowerState(name,
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.back(inchesBackwards);

                return trajectoryBuilder.build();
            }
        };
    }

    protected void setupMoveFoundation(@NonNull ParkOrLane whereToPark) {
        // Robot starts against wall, left side on tile seam nearest to tape
        // Gripper hooks are forward, which means robot is facing backwards (!)

        // Does alliance color change which way we turn in any of these steps?
        // YASSSS!

        State splineTrajectoryState = new TrajectoryFollowerState("Spline",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                BaseTrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder().back(4);

                //if (currentAlliance == Constants.Alliance.RED) {
                //    trajectoryBuilder.strafeLeft(11.5);
                //} else {
                //    trajectoryBuilder.strafeRight(11.5);
                //}

                trajectoryBuilder.back(27);

                return trajectoryBuilder.build();
            }
        };

        // GRIP!
        State gripState = new RunnableState("grip", telemetry, new Runnable() {
            @Override
            public void run() {
                foundationGripper.down();
            }
        });

        // Pull back a bit
        State pullBackState = new TrajectoryFollowerState("Pull",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                BaseTrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder().forward(12);
                if(currentAlliance == Constants.Alliance.BLUE)
                {
                    trajectoryBuilder.strafeLeft(20);
                }else{
                    trajectoryBuilder.strafeRight(20);
                }

                return trajectoryBuilder.build();
            }
        };

        // For red alliance
        Turn turn = new Turn(Rotation.CW, 90);

        if (currentAlliance == Constants.Alliance.BLUE) {
            turn = turn.invert();
        }

        // Turn 90 degrees clockwise
        State turnWithBase = new TurnState("Turn with base",
                telemetry, turn, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000));

        // Forward 5-6"

        State pushToWallTrajectoryState = new TrajectoryFollowerState("Push to wall",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(5 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                if (currentAlliance == Constants.Alliance.RED) {
                    return driveBase.trajectoryBuilder().back(18)
                            .build();
                } else {
                    return driveBase.trajectoryBuilder().back(24)
                            .build();
                }
            }
        };

        // UNGRIP

        State ungripState = new RunnableState("ungrip", telemetry, new Runnable() {
            @Override
            public void run() {
                foundationGripper.up();
            }
        });

        State getClearState = new TrajectoryFollowerState("get clear",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(5 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                return driveBase.trajectoryBuilder().forward(10)
                        .build();
            }
        };

        final State strafeToParkState;

        if (whereToPark == ParkOrLane.NEUTRAL_SKYBRIDGE) {
            // strafe  3-4
            strafeToParkState = new TrajectoryFollowerState("StrafeToPark",
                    telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
                @Override
                protected Trajectory createTrajectory() {
                    BaseTrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                    if (currentAlliance == Constants.Alliance.RED) {
                        trajectoryBuilder.strafeRight(6);
                    } else {
                        trajectoryBuilder.strafeLeft(6);
                    }

                    return trajectoryBuilder.build();
                }
            };
        } else {
            strafeToParkState = new TrajectoryFollowerState("StrafeToPark",
                    telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
                @Override
                protected Trajectory createTrajectory() {
                    BaseTrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                    if (currentAlliance == Constants.Alliance.RED) {
                        trajectoryBuilder.strafeLeft(18);
                    } else {
                        trajectoryBuilder.strafeRight(21);
                    }

                    return trajectoryBuilder.build();
                }
            };

        }

        State backToParkState = new TrajectoryFollowerState("back to park",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                BaseTrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder().forward(33);

                return trajectoryBuilder.build();
            }
        };

        final State snugUpState;

        if (whereToPark == ParkOrLane.NEUTRAL_SKYBRIDGE) {
            snugUpState = new TrajectoryFollowerState("snug up bridge",
                    telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
                @Override
                protected Trajectory createTrajectory() {
                    BaseTrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                    if (currentAlliance == Constants.Alliance.RED) {
                        trajectoryBuilder.strafeRight(6);
                    } else {
                        trajectoryBuilder.strafeLeft(6);
                    }

                    return trajectoryBuilder.build();
                }
            };
        } else {
            snugUpState = new TrajectoryFollowerState("snug up wall",
                    telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
                @Override
                protected Trajectory createTrajectory() {
                    BaseTrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                    if (currentAlliance == Constants.Alliance.RED) {
                        trajectoryBuilder.strafeLeft(6);
                    } else {
                        trajectoryBuilder.strafeRight(6);
                    }

                    return trajectoryBuilder.build();
                }
            };

        }

        // Win!

        stateMachine.addSequential(splineTrajectoryState);
        stateMachine.addSequential(gripState);
        stateMachine.addSequential(newDelayState("wait for grip", 1));
        stateMachine.addSequential(pullBackState);
        stateMachine.addSequential(turnWithBase);
        stateMachine.addSequential(pushToWallTrajectoryState);
        stateMachine.addSequential(ungripState);
        stateMachine.addSequential(newDelayState("wait for un-grip", 1));
        stateMachine.addSequential(getClearState);
        stateMachine.addSequential(strafeToParkState);
        stateMachine.addSequential(backToParkState);
        stateMachine.addSequential(snugUpState);

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

    private void setupOpenCvCameraAndPipeline() {
        detector = new TntSkystoneDetector(); // Create detector
        detector.setTelemetry(telemetry);

        EasyOpenCvPipelineAndCamera.EasyOpenCvPipelineAndCameraBuilder pipelineBuilder = EasyOpenCvPipelineAndCamera.builder();

        pipelineBuilder.hardwareMap(hardwareMap).telemetry(telemetry).detector(detector);

        pipelineAndCamera = pipelineBuilder.build();

        pipelineAndCamera.createAndRunPipeline();
    }

    private String skystoneZone = null;

    private String getSkystoneZone() {
        if (skystoneZone == null) {
            return TntSkystoneDetector.MIDDLE_ZONE_NAME;
        }

        return skystoneZone;
    }

    class SkystoneDetectionState extends StopwatchTimeoutSafetyState {

        private final TntSkystoneDetector detector;

        private final EasyOpenCvPipelineAndCamera pipelineAndCamera;

        protected SkystoneDetectionState(@NonNull EasyOpenCvPipelineAndCamera pipelineAndCamera,
                                         @NonNull TntSkystoneDetector detector,
                                         @NonNull Telemetry telemetry,
                                         @NonNull Ticker ticker) {
            // FIXME: How much time, really?
            super("Skystone detector", telemetry, ticker, TimeUnit.SECONDS.toMillis(8));
            this.detector = detector;
            this.pipelineAndCamera = pipelineAndCamera;
        }

        @Override
        public State doStuffAndGetNextState() {
            detector.startSearching();

            if (isTimedOut()) {
                skystoneZone = detector.getBestScoringZone().getZoneName();

                cleanupPipelineAndStuff();

                return nextState;
            }

            // FIXME: Can we early exit, if so, what are the parameters
            // We know that it's related to best detection zone, so I'll give you *that*
            // You need to decide "what else"

            DetectionZone stoneZone = detector.getBestScoringZone();
            DetectionZone secondBestZone = detector.getSecondBestScoringZone(); // MIGHT BE NULL

            Log.d(LOG_TAG, "amount of black:" + stoneZone.getTotalBlackArea());

            double bestBlackArea = stoneZone.getTotalBlackArea();

            if ((secondBestZone != null && bestBlackArea - secondBestZone.getTotalBlackArea() > 500)
                    || bestBlackArea > 850) {

                skystoneZone = detector.getBestScoringZone().getZoneName();

                cleanupPipelineAndStuff();

                return nextState;
            }

            return this;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }

        private void cleanupPipelineAndStuff() {
            // What kind of things do we want to do here, perhaps to conserve CPU/Battery?
            detector.stopPipeline();

        }
    }
}

/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1819;

import lombok.NonNull;
import android.util.Log;

import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.ServoPositionState;
import com.ftc9929.corelib.state.State;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Autonomous(name="Old RoverRuckus Auto")
@Disabled
@SuppressWarnings("unused")
public class RoverRuckusAutonomous extends RoverRuckusHardware {

    private State currentState = null;

    // The routes our robot knows how to do
    private enum Routes {
        DESCEND_ONLY("Descend Only"),
        FACING_DEPOT("Facing Depot (Gold)"),
        FACING_CRATER("Facing Crater (Silver)"),
        SAMPLE_FACING_DEPOT("Sample Facing Depot (Gold)"),
        SAMPLE_FACING_CRATER("Sample Facing Crater (Silver)");

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

    // change these constraints to something reasonable for your drive
    DriveConstraints baseConstraints = null; //new DriveConstraints(25.0,
            //40.0,
            //Math.PI / 2,
            //Math.PI / 2);

    MecanumConstraints mecanumConstraints = mecanumConstraints  = new MecanumConstraints(
            baseConstraints, RoadrunnerMecanumDriveAdapter.TRACK_WIDTH
            , RoadrunnerMecanumDriveAdapter.WHEEL_BASE);

    public RoverRuckusAutonomous() {
        imuNeeded = false; // for now...
    }

    @Override
    public void init() {
        super.init();
        setDefaults();

        for (DcMotor motor : mecanumDrive.motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        try {
            tensorFlowThread = new TensorflowThread(tfResultsMailbox, hardwareMap, TimeUnit.SECONDS.toMillis(30));
            tensorFlowThread.initDetection();
        } catch (Throwable t) {
            Log.e(LOG_TAG, "Tensor flow thread could not be initialized", t);
        }
    }

    @Override
    public void start() {
        super.start();
        logBatteryState("Auto.start()");
    }

    @Override
    public void stop() {
        super.stop();

        if (tensorFlowThread != null) {
            tensorFlowThread.shutdownTensorFlow();
        }

        logBatteryState("Auto.stop()");
    }

    private void setDefaults() {
        currentAlliance = Constants.Alliance.RED;
        selectedRoutesIndex = 0;
        initialDelaySeconds = 0;
    }

    private boolean configLocked = false;

    // Called repeatedly after init button has been pressed and init() has completed (we think)
    @Override
    public void init_loop() {
        if (driversGamepad == null) { // safety, need to double check whether we actually need this
            // not ready yet init() hasn't been called
            return;
        }

        handleAscender();

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

    @Override
    public void loop() {
        long cycleStartTimeMs = System.currentTimeMillis();

        try {
            if (currentState == null) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                final State selectedState;
                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case DESCEND_ONLY:
                        selectedState = descendOnly();

                        if (tensorFlowThread != null) {
                            tensorFlowThread.shutdownTensorFlow();
                        }

                        break;
                    case FACING_DEPOT:
                        selectedState = facingDepot();

                        if (tensorFlowThread != null) {
                            tensorFlowThread.shutdownTensorFlow();
                        }

                        break;
                    case FACING_CRATER:
                        selectedState = facingCrater();

                        if (tensorFlowThread != null) {
                            tensorFlowThread.shutdownTensorFlow();
                        }

                        break;
                    case SAMPLE_FACING_DEPOT:
                        selectedState = facingDepotWithSampling();
                        break;
                    case SAMPLE_FACING_CRATER:
                        selectedState = facingCraterWithSampling();
                        break;
                    default:
                        selectedState = newDoneState("Default done");
                }

                if (initialDelaySeconds != 0) {
                    currentState = newDelayState();
                    currentState.setNextState(selectedState);
                } else {
                    currentState = selectedState;
                }
            }

            State nextState = currentState.doStuffAndGetNextState();

            if (nextState != currentState) {
                // We've changed states alert the driving team, log for post-match analysis
                telemetry.addData("00-State", "From %s to %s", currentState, nextState);
                Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                        + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
            }

            currentState = nextState;
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

        long cycleStopTimeMs = System.currentTimeMillis();
        Log.d(Constants.LOG_TAG, "Cycle time (ms): " + (cycleStopTimeMs - cycleStartTimeMs));
    }

    /**
     * Creates an instance of the "delay" state which waits the number of seconds before
     * advancing to the next state
     */
    protected State newDelayState() {
        return newDelayState("start delay", initialDelaySeconds);
    }

    protected State descendOnly() {
        State initialState = new DescenderState(telemetry);

        MecanumStrafeDistanceState awayFromLanderOne = new MecanumStrafeDistanceState(
                "away from lander one", telemetry, mecanumDrive, 1.0,
                TimeUnit.SECONDS.toMillis(5));
        initialState.setNextState(awayFromLanderOne);

        MecanumDriveDistanceState offTheHook = new MecanumDriveDistanceState("off the hook",
                telemetry, mecanumDrive, 1.5, TimeUnit.SECONDS.toMillis(5));
        awayFromLanderOne.setNextState(offTheHook);

        MecanumStrafeDistanceState awayFromLanderFinal = new MecanumStrafeDistanceState(
                "away from lander Final", telemetry, mecanumDrive, 1.5,
                TimeUnit.SECONDS.toMillis(5));
        offTheHook.setNextState(awayFromLanderFinal);
        awayFromLanderFinal.setNextState(newDoneState("done"));

        return initialState;
    }

    protected State facingDepot() {
        // FIXME: After league meet, we should not copy-and-paste the descent,
        //        it should come from a shared method

        State initialState = new DescenderState(telemetry);

        MecanumStrafeDistanceState awayFromLanderOne = new MecanumStrafeDistanceState(
                "away from lander one", telemetry, mecanumDrive, 1.0,
                TimeUnit.SECONDS.toMillis(5));
        initialState.setNextState(awayFromLanderOne);

        MecanumDriveDistanceState offTheHook = new MecanumDriveDistanceState("off the hook",
                telemetry, mecanumDrive, 1.5, TimeUnit.SECONDS.toMillis(5));
        awayFromLanderOne.setNextState(offTheHook);

        MecanumStrafeDistanceState awayFromLander = new MecanumStrafeDistanceState(
                "away from lander Final", telemetry, mecanumDrive, 1.5 + 14,
                TimeUnit.SECONDS.toMillis(5));
        offTheHook.setNextState(awayFromLander);

        // WARNING: From this point, once we start using RoadRunner trajectories, we cannot
        // go back to our old style drive/strafe distance states! (slightly different motor
        // setup required, done in the state itself)

        // --------------------------------------------------------------
        // move forward 34.5 inches (subtract 4" from this for "from landing")
        // turn (counter - mm) clockwise 45 degrees
        // --------------------------------------------------------------

        // Remember, right hand rule, turns go CCW for + angles
        double turnInDegrees = 45; // note - not alliance specific!

        Trajectory toAlignWithWallTrajectory = new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(0, 34.5 - 4), new ConstantInterpolator(0))
                //.turnTo(Math.toRadians(turnInDegrees))
                .build();

        TrajectoryFollowerState toAlignWithWallState = new TrajectoryFollowerState(
                "To align with wall",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toAlignWithWallTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        awayFromLander.setNextState(toAlignWithWallState);

        // --------------------------------------------------------------
        // strafe right 9.7 inches (I'd round to 10 or so...MM)
        // move backward 45.4 inches
        // --------------------------------------------------------------

        Trajectory toWallThenDepotTrajectory = new TrajectoryBuilder(TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(10, 0), new ConstantInterpolator(0)) // strafe
                .lineTo(TntPose2d.toVector2d(10, -49.4), new ConstantInterpolator(0)).build(); // to crater

        TrajectoryFollowerState toWallThenDepotState = new TrajectoryFollowerState(
                "To wall, then depot",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toWallThenDepotTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        toAlignWithWallState.setNextState(toWallThenDepotState);

        // --------------------------------------------------------------
        // (drop off team marker)
        // --------------------------------------------------------------

        State dropTeamMarker = new ServoPositionState("drop tm", telemetry, teamMarkerServo, TEAM_MARKER_DUMP_POS);
        toWallThenDepotState.setNextState(dropTeamMarker);

        State waitForDrop = newDelayState("wait-for-drop", 2);
        dropTeamMarker.setNextState(waitForDrop);

        State storeTeamMarkerMech = new ServoPositionState("store-tm", telemetry, teamMarkerServo, /* MM TEAM_MARKER_STOWED_STATE */ TEAM_MARKER_DUMP_POS);
        waitForDrop.setNextState(storeTeamMarkerMech);

        // --------------------------------------------------------------
        // move forward 68.25 inches
        // (parked at crater)
        // --------------------------------------------------------------

        Trajectory toCraterTrajectory = new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(10, 62.3),
                        new ConstantInterpolator(0)).build(); // Always a constant interpolator to hold heading

        TrajectoryFollowerState toCraterState = new TrajectoryFollowerState(
                "To crater",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toCraterTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        storeTeamMarkerMech.setNextState(toCraterState);

        // FIXME: Probably need a bit more power to end up breaking crater plane

        toCraterState.setNextState(newDoneState("Done!"));

        return initialState;
    }

    protected State facingCrater() {
        // FIXME: After league meet, we should not copy-and-paste the descent,
        //        it should come from a shared method

        State initialState = new DescenderState(telemetry);

        MecanumStrafeDistanceState awayFromLanderOne = new MecanumStrafeDistanceState(
                "away from lander one", telemetry, mecanumDrive, 1.0,
                TimeUnit.SECONDS.toMillis(5));
        initialState.setNextState(awayFromLanderOne);

        MecanumDriveDistanceState offTheHook = new MecanumDriveDistanceState("off the hook",
                telemetry, mecanumDrive, 1.5, TimeUnit.SECONDS.toMillis(5));
        awayFromLanderOne.setNextState(offTheHook);

        MecanumStrafeDistanceState awayFromLander = new MecanumStrafeDistanceState(
                "away from lander Final", telemetry, mecanumDrive, 1.5 + 14,
                TimeUnit.SECONDS.toMillis(5));
        offTheHook.setNextState(awayFromLander);

        // WARNING: From this point, once we start using RoadRunner trajectories, we cannot
        // go back to our old style drive/strafe distance states! (slightly different motor
        // setup required, done in the state itself)

        // --------------------------------------------------------------
        // move forward 34.5 inches (subtract 4" from this for "from landing")
        // turn (counter - mm) clockwise 135 degrees
        // --------------------------------------------------------------

        // Remember, right hand rule, turns go CCW for + angles
        double turnInDegrees = 135 + 90; // note - not alliance specific!

        Trajectory toAlignWithWallTrajectory = new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(0, 34.5 - 4), new ConstantInterpolator(0))
                //.turnTo(Math.toRadians(turnInDegrees))
                .build();

        TrajectoryFollowerState toAlignWithWallState = new TrajectoryFollowerState(
                "To align with wall",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toAlignWithWallTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        awayFromLander.setNextState(toAlignWithWallState);

        // --------------------------------------------------------------
        // strafe LEFT 9.7 inches (I'd round to 10 or so...MM)
        // move backward 45.4 inches
        // --------------------------------------------------------------

        Trajectory toWallThenDepotTrajectory = new TrajectoryBuilder(TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(-10, 0), new ConstantInterpolator(0)) // strafe
                .lineTo(TntPose2d.toVector2d(-10, -49.4), new ConstantInterpolator(0)).build(); // to crater

        TrajectoryFollowerState toWallThenDepotState = new TrajectoryFollowerState(
                "To wall, then depot",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toWallThenDepotTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        toAlignWithWallState.setNextState(toWallThenDepotState);

        // --------------------------------------------------------------
        // (drop off team marker)
        // --------------------------------------------------------------

        State dropTeamMarker = new ServoPositionState("drop tm", telemetry, teamMarkerServo, TEAM_MARKER_DUMP_POS);
        toWallThenDepotState.setNextState(dropTeamMarker);

        State waitForDrop = newDelayState("wait-for-drop", 2);
        dropTeamMarker.setNextState(waitForDrop);

        State storeTeamMarkerMech = new ServoPositionState("store-tm", telemetry, teamMarkerServo,TEAM_MARKER_STOWED_STATE);
        waitForDrop.setNextState(storeTeamMarkerMech);

        // --------------------------------------------------------------
        // move forward 68.25 inches
        // (parked at crater)
        // --------------------------------------------------------------

        Trajectory toCraterTrajectory = new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(-10, 62.3),
                        new ConstantInterpolator(0)).build(); // Always a constant interpolator to hold heading

        TrajectoryFollowerState toCraterState = new TrajectoryFollowerState(
                "To crater",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toCraterTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        waitForDrop.setNextState(toCraterState);

        // FIXME: Probably need a bit more power to end up breaking crater plane

        toCraterState.setNextState(newDoneState("Done!"));

        return initialState;
    }

    protected State facingCraterWithSampling() {
        // FIXME: After league meet, we should not copy-and-paste the descent,
        //        it should come from a shared method

        State initialState = new StartTensorFlowDetectionState(telemetry);

        State descendState = new DescenderState(telemetry);
        initialState.setNextState(descendState);

        MecanumStrafeDistanceState awayFromLanderOne = new MecanumStrafeDistanceState(
                "away from lander one", telemetry, mecanumDrive, 1.0,
                TimeUnit.SECONDS.toMillis(5));
        descendState.setNextState(awayFromLanderOne);

        MecanumDriveDistanceState offTheHook = new MecanumDriveDistanceState("off the hook",
                telemetry, mecanumDrive, 1.5, TimeUnit.SECONDS.toMillis(5));
        awayFromLanderOne.setNextState(offTheHook);

        MecanumStrafeDistanceState awayFromLander = new MecanumStrafeDistanceState(
                "away from lander Final", telemetry, mecanumDrive, 1.5 + 14 + 1.0,
                TimeUnit.SECONDS.toMillis(5));
        offTheHook.setNextState(awayFromLander);

        // WARNING: From this point, once we start using RoadRunner trajectories, we cannot
        // go back to our old style drive/strafe distance states! (slightly different motor
        // setup required, done in the state itself)

        // --------------------------------------------------------------
        // move forward 34.5 inches (subtract 4" from this for "from landing")
        // turn (counter - mm) clockwise 135 degrees
        // --------------------------------------------------------------

        double turnInDegrees = 135 + 90; // note - not alliance specific!

        Trajectory sampleLeftThenAlignWithWallTrajectory =
                createSampleLeftThenAlignWithWallTrajectory(turnInDegrees);

        Trajectory sampleCenterThenAlignWithWallTrajectory =
                createSampleCenterThenAlignWithWallTrajectory(turnInDegrees);

        Trajectory sampleRightThenAlignWithWallTrajectory =
                createSampleRightThenAlignWithWallTrajectory(turnInDegrees);

        TrajectoryFollowerState sampleThenAlignWithWallState = new MineralTrajectoryState(
                "Sample, then align with wall",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                sampleLeftThenAlignWithWallTrajectory,
                sampleCenterThenAlignWithWallTrajectory,
                sampleRightThenAlignWithWallTrajectory,
                tfResultsMailbox,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);

        awayFromLander.setNextState(sampleThenAlignWithWallState);


        // --------------------------------------------------------------
        // strafe LEFT 9.7 inches (I'd round to 10 or so...MM)
        // move backward 45.4 inches
        // --------------------------------------------------------------

        Trajectory toWallThenDepotTrajectory = new TrajectoryBuilder(TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(-10, 0), new ConstantInterpolator(0)) // strafe
                .lineTo(TntPose2d.toVector2d(-10, -49.4), new ConstantInterpolator(0)).build(); // to crater

        TrajectoryFollowerState toWallThenDepotState = new TrajectoryFollowerState(
                "To wall, then depot",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toWallThenDepotTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        sampleThenAlignWithWallState.setNextState(toWallThenDepotState);

        // --------------------------------------------------------------
        // (drop off team marker)
        // --------------------------------------------------------------

        State dropTeamMarker = new ServoPositionState("drop tm", telemetry, teamMarkerServo, TEAM_MARKER_DUMP_POS);
        toWallThenDepotState.setNextState(dropTeamMarker);

        State waitForDrop = newDelayState("wait-for-drop", 2);
        dropTeamMarker.setNextState(waitForDrop);

        State storeTeamMarkerMech = new ServoPositionState("store-tm", telemetry, teamMarkerServo,TEAM_MARKER_STOWED_STATE);
        waitForDrop.setNextState(storeTeamMarkerMech);

        // --------------------------------------------------------------
        // move forward 68.25 inches
        // (parked at crater)
        // --------------------------------------------------------------

        Trajectory toCraterTrajectory = new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(-10, 62.3),
                        new ConstantInterpolator(0)).build(); // Always a constant interpolator to hold heading

        TrajectoryFollowerState toCraterState = new TrajectoryFollowerState(
                "To crater",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toCraterTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        waitForDrop.setNextState(toCraterState);

        // FIXME: Probably need a bit more power to end up breaking crater plane

        toCraterState.setNextState(newDoneState("Done!"));

        return initialState;
    }

    protected Queue<TensorflowThread.GOLD_MINERAL_POSITION> tfResultsMailbox = new LinkedBlockingQueue<>();

    protected State facingDepotWithSampling() {
        // FIXME: After league meet, we should not copy-and-paste the descent,
        //        it should come from a shared method

        State initialState = new StartTensorFlowDetectionState(telemetry);

        State descendState = new DescenderState(telemetry);
        initialState.setNextState(descendState);

        MecanumStrafeDistanceState awayFromLanderOne = new MecanumStrafeDistanceState(
                "away from lander one", telemetry, mecanumDrive, 1.0,
                TimeUnit.SECONDS.toMillis(5));
        descendState.setNextState(awayFromLanderOne);

        MecanumDriveDistanceState offTheHook = new MecanumDriveDistanceState("off the hook",
                telemetry, mecanumDrive, 1.5, TimeUnit.SECONDS.toMillis(5));
        awayFromLanderOne.setNextState(offTheHook);

        MecanumStrafeDistanceState awayFromLander = new MecanumStrafeDistanceState(
                "away from lander Final", telemetry, mecanumDrive, 1.5 + 14 + 1.0,
                TimeUnit.SECONDS.toMillis(5));
        offTheHook.setNextState(awayFromLander);

        // WARNING: From this point, once we start using RoadRunner trajectories, we cannot
        // go back to our old style drive/strafe distance states! (slightly different motor
        // setup required, done in the state itself)

        // --------------------------------------------------------------
        // move forward 34.5 inches (subtract 4" from this for "from landing")
        // turn (counter - mm) clockwise 45 degrees
        // --------------------------------------------------------------

        double turnInDegrees = 45; // note - not alliance specific!

        Trajectory sampleLeftThenAlignWithWallTrajectory =
                createSampleLeftThenAlignWithWallTrajectory(turnInDegrees);

        Trajectory sampleCenterThenAlignWithWallTrajectory =
                createSampleCenterThenAlignWithWallTrajectory(turnInDegrees);

        Trajectory sampleRightThenAlignWithWallTrajectory =
                createSampleRightThenAlignWithWallTrajectory(turnInDegrees);

        TrajectoryFollowerState sampleThenAlignWithWallState = new MineralTrajectoryState(
                "Sample then align with wall",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                sampleLeftThenAlignWithWallTrajectory,
                sampleCenterThenAlignWithWallTrajectory,
                sampleRightThenAlignWithWallTrajectory,
                tfResultsMailbox,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);

        awayFromLander.setNextState(sampleThenAlignWithWallState);

        // --------------------------------------------------------------
        // strafe right 9.7 inches (I'd round to 10 or so...MM)
        // move backward 45.4 inches
        // --------------------------------------------------------------

        Trajectory toWallThenDepotTrajectory = new TrajectoryBuilder(TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(10, 0), new ConstantInterpolator(0)) // strafe
                .lineTo(TntPose2d.toVector2d(10, -49.4), new ConstantInterpolator(0)).build(); // to crater

        TrajectoryFollowerState toWallThenDepotState = new TrajectoryFollowerState(
                "To wall, then depot",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toWallThenDepotTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        sampleThenAlignWithWallState.setNextState(toWallThenDepotState);

        // --------------------------------------------------------------
        // (drop off team marker)
        // --------------------------------------------------------------

        State dropTeamMarker = new ServoPositionState("drop tm", telemetry, teamMarkerServo, TEAM_MARKER_DUMP_POS);
        toWallThenDepotState.setNextState(dropTeamMarker);

        State waitForDrop = newDelayState("wait-for-drop", 2);
        dropTeamMarker.setNextState(waitForDrop);

        State storeTeamMarkerMech = new ServoPositionState("store-tm", telemetry, teamMarkerServo, /* MM TEAM_MARKER_STOWED_STATE */ TEAM_MARKER_DUMP_POS);
        waitForDrop.setNextState(storeTeamMarkerMech);

        // --------------------------------------------------------------
        // move forward 68.25 inches
        // (parked at crater)
        // --------------------------------------------------------------

        Trajectory toCraterTrajectory = new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(10, 62.3),
                        new ConstantInterpolator(0)).build(); // Always a constant interpolator to hold heading

        TrajectoryFollowerState toCraterState = new TrajectoryFollowerState(
                "To crater",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                toCraterTrajectory,
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        storeTeamMarkerMech.setNextState(toCraterState);

        // FIXME: Probably need a bit more power to end up breaking crater plane

        toCraterState.setNextState(newDoneState("Done!"));

        return initialState;
    }


    @NonNull
    private Trajectory createSampleLeftThenAlignWithWallTrajectory(double turnToStrafeInDegrees) {
        // forward 4.5, turn 21 degrees CW
        return new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(0, 8.0), new ConstantInterpolator(0)) // get to mineral
                //.turnTo(Math.toRadians(-85)) // turn towards mineral
                //.turnTo(Math.toRadians(0)) // turn back
                .lineTo(TntPose2d.toVector2d(0, 34.5 - 6.0 /* distance traveled to mineral */), new ConstantInterpolator(0))
                //.turnTo(Math.toRadians(turnToStrafeInDegrees))
                .build();
    }

    @NonNull
    private Trajectory createSampleCenterThenAlignWithWallTrajectory(double turnToStrafeInDegrees) {
        // forward 1.75, turn 21 degrees CCW
        return new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(0, -3 /* FIXME: how far and in what direction do we drive? */), new ConstantInterpolator(0)) // get to mineral
                //.turnTo(Math.toRadians(85)) // turn towards mineral
                //.turnTo(Math.toRadians(0)) // turn back
                .lineTo(TntPose2d.toVector2d(0, 34.5 - 4 + 3 /* distance traveled to mineral */), new ConstantInterpolator(0))
                //.turnTo(Math.toRadians(turnToStrafeInDegrees))
                .build();
    }

    @NonNull
    private Trajectory createSampleRightThenAlignWithWallTrajectory(double turnToStrafeInDegrees) {
        // backwards 12.5, turn 21 degrees CCW
        return new TrajectoryBuilder(
                TntPose2d.toPose2d(0, 0, 0), mecanumConstraints) // Always starting from 0, 0, 0
                .lineTo(TntPose2d.toVector2d(0, -19), new ConstantInterpolator(0)) // get to mineral
                //.turnTo(Math.toRadians(75)) // turn towards mineral
                //.turnTo(Math.toRadians(0)) // turn back
                .lineTo(TntPose2d.toVector2d(0, 34.5 - 4 + 5 /* distance traveled to mineral */), new ConstantInterpolator(0))
                //.turnTo(Math.toRadians(turnToStrafeInDegrees))
                .build();
    }

    class DescenderState extends TimeoutSafetyState {
        public DescenderState(final Telemetry telemetry) {
            super("descending", telemetry, TimeUnit.SECONDS.toMillis(15));
            //FixMe how long do we do this?
            // Not 27 didn't allow for run of the off the hook step. I was watching the robot and
            // it keep descending during auto and didn't leave enough time to descend. I think it
            // would be ok to go into drive state if we reach time ou bc if we have not reach the
            // ground then we won't go anywhere, if we have then we will still get full auto.
                // I didn't see the wheels move during off auto's. check logs.
                // - Lauren

            //tried 15 may still not be right - lauren 11/18
                //we removed tape on acDc mech and it worked w/ this code - lauren l8r 11/18
        }

        private boolean stateStarted = false;


        @Override
        public State doStuffAndGetNextState() {
            if (!stateStarted) {
                ascenderDescender.extendToMax();

                stateStarted = true;

                return this;
            } else {
                if (isTimedOut()) {
                    Log.d(Constants.LOG_TAG, "Descender timed out after " + safetyTimeoutMillis + " ms");

                    ascenderDescender.stopMoving();

                    return newDoneState("Descender timed out");

                } else if (ascenderDescender.hasExtended()) {
                    ascenderDescender.stopMoving();

                    return nextState;
                } else {
                    return this;
                }
            }
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {
            // do nothing
        }
    }


    private TensorflowThread tensorFlowThread;

    class StartTensorFlowDetectionState extends TimeoutSafetyState {

        protected StartTensorFlowDetectionState(Telemetry telemetry) {
            super("Start TensorFlow", telemetry, TimeUnit.SECONDS.toMillis(7));
        }

        @Override
        public State doStuffAndGetNextState() {
            try {
                if (tensorFlowThread != null) {
                    tensorFlowThread.start();

                    Log.d(LOG_TAG, "Started TensorFlow detection thread");
                }
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Could not start TensorFlow thread", t);
            }

            return nextState;
        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }
}
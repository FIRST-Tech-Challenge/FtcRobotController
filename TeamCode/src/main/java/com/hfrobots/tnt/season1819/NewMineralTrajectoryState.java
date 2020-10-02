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

package com.hfrobots.tnt.season1819;

import android.util.Log;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.ServoPositionState;
import com.ftc9929.corelib.state.State;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Queue;
import java.util.concurrent.TimeUnit;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class NewMineralTrajectoryState extends TimeoutSafetyState {
    // You need 3 Trajectory segment sets - one for each mineral
    private final MineralTrajectorySegments rightTrajectorySegments;

    private final MineralTrajectorySegments centerTrajectorySegments;

    private final MineralTrajectorySegments leftTrajectorySegments;

    // You need 1 "mailbox", which is of type Queue<GOLD_MINERAL_POSITION>
    private final Queue<TensorflowThread.GOLD_MINERAL_POSITION> mailbox;

    private boolean mineralInitialized = false;

    private final Servo mineralMoverArmServo;

    private final Servo mineralMoverFlagServo;

    private final DriveConstraints baseConstraints;

    private final MecanumConstraints mecanumConstraints;

    private final HardwareMap hardwareMap;

    private State currentState;

    /**
     * A simple structure to hold the 3 segments of the full trajectory
     * that moves us to, through, and beyond the gold mineral
     */
    public static class MineralTrajectorySegments {
        private final Trajectory drivePastTrajectory;

        private final Trajectory driveThroughTrajectory;

        private final Trajectory toTurnTrajectory;

        public MineralTrajectorySegments(final Trajectory drivePastTrajectory,
                                         final Trajectory driveThroughTrajectory,
                                         final Trajectory toTurnTrajectory) {
            this.drivePastTrajectory = drivePastTrajectory; // FIXME M3 Changes
            this.driveThroughTrajectory = driveThroughTrajectory; // FIXME M3 Changes
            this.toTurnTrajectory = toTurnTrajectory; // FIXME - M3 Changes

        }
    }

    public NewMineralTrajectoryState(final String name,
                                     final Telemetry telemetry,
                                     final long safetyTimeoutMillis,
                                     Servo mineralMoverArmServo,
                                     Servo mineralMoverFlagServo,
                                     final MineralTrajectorySegments leftTrajectorySegments,
                                     final MineralTrajectorySegments centerTrajectorySegments,
                                     final MineralTrajectorySegments rightTrajectorySegments,
                                     final Queue<TensorflowThread.GOLD_MINERAL_POSITION> mailbox,
                                     final DriveConstraints baseConstraints,
                                     final MecanumConstraints mecanumConstraints,
                                     final HardwareMap hardwareMap) {
        super(name, telemetry, safetyTimeoutMillis);

        this.mineralMoverArmServo = mineralMoverArmServo;
        this.mineralMoverFlagServo = mineralMoverFlagServo;

        this.baseConstraints = baseConstraints;
        this.mecanumConstraints = mecanumConstraints;
        this.hardwareMap = hardwareMap;

        this.leftTrajectorySegments = leftTrajectorySegments;
        this.centerTrajectorySegments = centerTrajectorySegments;
        this.rightTrajectorySegments = rightTrajectorySegments;
        this.mailbox = mailbox;
    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {
        // not really implemented
    }

    private void initialize() {
        MineralTrajectorySegments mineralTrajectorySegments = null;

        if (!mailbox.isEmpty()) {
            TensorflowThread.GOLD_MINERAL_POSITION mineralPosition = mailbox.poll();

            if (mineralPosition != null) {
                if (mineralPosition.equals(TensorflowThread.GOLD_MINERAL_POSITION.LEFT)) {
                    Log.d(LOG_TAG, "Using LEFT trajectory segments");

                    mineralTrajectorySegments = leftTrajectorySegments;
                } else if (mineralPosition.equals(TensorflowThread.GOLD_MINERAL_POSITION.CENTER)) {
                    Log.d(LOG_TAG, "Using CENTER trajectory segments");

                    mineralTrajectorySegments = centerTrajectorySegments;
                } else if (mineralPosition.equals(TensorflowThread.GOLD_MINERAL_POSITION.RIGHT)) {
                    Log.d(LOG_TAG, "Using RIGHT trajectory segments");

                    mineralTrajectorySegments = rightTrajectorySegments;
                }
            }
        }

        if (mineralTrajectorySegments == null /* didn't find it */) {
                // Correct 33% of the time! Bonus, shortest travel, less error-prone
            Log.d(LOG_TAG, "Using DEFAULT (CENTER) trajectory segments");

            mineralTrajectorySegments = centerTrajectorySegments;
        }

        // (2) Set the m3 flag to the out position
        ServoPositionState mCubedServoFlagOutState = new ServoPositionState("m3 flag out",
                telemetry,
                mineralMoverFlagServo,
                RoverRuckusHardware.M3_FLAG_DEPLOYED_POSITION); // FIXME M3 Changes


        // (1) Drive past gold mineral

        TrajectoryFollowerState toPastMineralState  = new TrajectoryFollowerState(
                "Drive past gold mineral",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                mineralTrajectorySegments.drivePastTrajectory, /* FIXME M3 changes */ // MM Put back to broken
                baseConstraints,
                mecanumConstraints,
                hardwareMap);

        mCubedServoFlagOutState.setNextState(toPastMineralState);

        // (4) Move the m3 arm down
        ServoPositionState mCubedServoDownState = new ServoPositionState("m3 arm down",
                telemetry,
                mineralMoverArmServo,
                RoverRuckusHardware.M3_ARM_OUT_POSITION); // FIXME M3 Changes

        toPastMineralState.setNextState(mCubedServoDownState);

        // (5) Wait for the plate to move down // maybe this one
        DelayState waitForMineralMoverDownState = new DelayState(
                "Wait for M3 down",
                telemetry, 500,
                TimeUnit.MILLISECONDS);
        mCubedServoDownState.setNextState(waitForMineralMoverDownState);

        // (6) Drive through the gold mineral
        TrajectoryFollowerState driveTroughGoldMineralState = new TrajectoryFollowerState(
                "Drive through gold mineral",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                mineralTrajectorySegments.driveThroughTrajectory, /* FIXME M3 changes */ // MM put back to broken
                baseConstraints,
                mecanumConstraints,
                hardwareMap);

        waitForMineralMoverDownState.setNextState(driveTroughGoldMineralState);

        // (7) Raise the m3 arm
        ServoPositionState mCubedServoUpState = new ServoPositionState("raise m3 arm",
                telemetry,
                mineralMoverArmServo,
                RoverRuckusHardware.M3_ARM_UP_POSITION); // FIXME M3 Changes
        driveTroughGoldMineralState.setNextState(mCubedServoUpState);

        // (8) Wait for the m3 arm to raise
        DelayState waitForServoRaisedState = new DelayState("wait for M3 up",
                telemetry, 500,
                TimeUnit.MILLISECONDS);
        mCubedServoUpState.setNextState(waitForServoRaisedState);

        // (9) retract the m3 flag
        ServoPositionState mCubedServoPlateInState = new ServoPositionState("raise m3 plate",
                telemetry,
                mineralMoverFlagServo,
                RoverRuckusHardware.M3_FLAG_STOWED_POSITION); // FIXME M3 Changes
        waitForServoRaisedState.setNextState(mCubedServoPlateInState);

        // (10) wait for the m3 flag to retract
        DelayState waitForFlagInState = new DelayState("wait for M3 flag in",
                telemetry, 500,
                TimeUnit.MILLISECONDS);
        mCubedServoPlateInState.setNextState(waitForFlagInState);

        // (11) drive to the turn, and complete it
        TrajectoryFollowerState driveToTurnPointState = new TrajectoryFollowerState(
                "Drive to turn point",
                telemetry,
                TimeUnit.SECONDS.toMillis(30 /* FIXME */),
                mineralTrajectorySegments.toTurnTrajectory, /* FIXME M3 changes */ // MM Put back broken
                baseConstraints,
                mecanumConstraints,
                hardwareMap);
        waitForFlagInState.setNextState(driveToTurnPointState);

        // (12) Continue on with the rest of the trajectories and states
        driveToTurnPointState.setNextState(nextState); // this is the next state of the "outer" state machine

        currentState = mCubedServoFlagOutState;
    }


    @Override
    public State doStuffAndGetNextState() {
        if (!mineralInitialized) {

            initialize();

            mineralInitialized = true;
        }

        State nextState = currentState.doStuffAndGetNextState();

        if (nextState != currentState) {
            // We've changed states alert the driving team, log for post-match analysis
            telemetry.addData("00-Sub-State", "From %s to %s", currentState, nextState);
            Log.d(LOG_TAG, String.format("Sub-State transition from %s to %s", currentState.getClass()
                    + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
        }

        currentState = nextState;

        return currentState;
    }
}

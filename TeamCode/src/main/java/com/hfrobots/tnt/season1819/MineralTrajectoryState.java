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

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.hfrobots.tnt.corelib.state.State;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Queue;

public class MineralTrajectoryState extends TrajectoryFollowerState {
    // You need 3 Trajectories - one for each mineral
    private final Trajectory rightTrajectory;

    private final Trajectory centerTrajectory;

    private final Trajectory leftTrajectory;

    // You need 1 "mailbox", which is of type Queue<GOLD_MINERAL_POSITION>
    private final Queue<TensorflowThread.GOLD_MINERAL_POSITION> mailbox;

    private boolean mineralInitialized = false;

    // You then need to add them as arguments to the following constructor - I suggest
    // adding them as the existing trajectory

    public MineralTrajectoryState(String name, Telemetry telemetry, long safetyTimeoutMillis,
                                  Trajectory leftTrajectory,
                                  Trajectory centerTrajectory,
                                  Trajectory rightTrajectory,
                                  Queue<TensorflowThread.GOLD_MINERAL_POSITION> mailbox,
                                  DriveConstraints baseConstraints,
                                  MecanumConstraints constraints,
                                  HardwareMap hardwareMap) {
        super(name, telemetry, safetyTimeoutMillis, centerTrajectory, baseConstraints, constraints, hardwareMap);
        this.leftTrajectory = leftTrajectory;
        this.centerTrajectory = centerTrajectory;
        this.rightTrajectory = rightTrajectory;
        this. mailbox = mailbox;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!mineralInitialized) {
            if (!mailbox.isEmpty()) {
                TensorflowThread.GOLD_MINERAL_POSITION mineralPosition = mailbox.poll();

                if (mineralPosition != null) {
                    if (mineralPosition.equals(TensorflowThread.GOLD_MINERAL_POSITION.LEFT)) {
                        this.trajectory = leftTrajectory;
                    } else if (mineralPosition.equals(TensorflowThread.GOLD_MINERAL_POSITION.CENTER)) {
                        this.trajectory = centerTrajectory;
                    } else if (mineralPosition.equals(TensorflowThread.GOLD_MINERAL_POSITION.RIGHT)) {
                        this.trajectory = rightTrajectory;
                    }
                } else {
                    // Correct 33% of the time! Bonus, shortest travel, less error-prone
                    this.trajectory = centerTrajectory;
                }
            }

            mineralInitialized = true;
        }

        return super.doStuffAndGetNextState();
    }
}

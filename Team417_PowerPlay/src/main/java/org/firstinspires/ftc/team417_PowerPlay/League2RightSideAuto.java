/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf -> pdf that includes all the id's for april tags

package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous (name="RIGHT SIDE")
public class League2RightSideAuto extends BaseAutonomous {

    @Override
    public void runOpMode() {

        initializeAuto();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory traject2 = drive.trajectoryBuilder(startPose, false)
                .forward(48)
                .build();
        Trajectory traject3 = drive.trajectoryBuilder(traject2.end(), false)
                .back(19.5)
                .build();
        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .strafeRight(15)
                .build();
        Trajectory traject5 = drive.trajectoryBuilder(traject4.end(), false)
                .back(3)
                .build();
        Trajectory right = drive.trajectoryBuilder(traject5.end(), false)
                .strafeRight(12)
                .build();
        Trajectory correctRight = drive.trajectoryBuilder(right.end(), false)
                .forward(5)
                .build();
        Trajectory middle = drive.trajectoryBuilder(traject5.end(), false)
                .strafeLeft(21)
                .build();
        Trajectory left = drive.trajectoryBuilder(traject5.end(), false)
                .strafeLeft(40)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            detectAprilTag();
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        updateTelemetryAfterStart();

        grabberServo.setPosition(GRABBER_CLOSED);
        raiseAndHoldArmGroundJunctionPosition();

        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);

        raiseAndHoldArmLowJunctionPosition();
        drive.followTrajectory(traject4);
        motorArm.setPower(0);
        sleep(800);
        // open servo
        grabberServo.setPosition(GRABBER_OPEN);
        drive.followTrajectory(traject5);


        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(left);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(middle);
        } else {
            drive.followTrajectory(right);
            drive.followTrajectory(correctRight);
        }

    }
}
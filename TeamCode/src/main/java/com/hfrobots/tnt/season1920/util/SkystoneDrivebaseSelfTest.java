/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1920.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.hfrobots.tnt.season1920.SkystoneDriveConstants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/*
 * An op-mode that runs the drivebase motors a given distance and checks that all
 * encoders are reporting the correct direction, and that the delta difference is
 * within tolerances.
 */
@Config
@TeleOp(name="Drive Selftest", group = "util")
@Disabled
public class SkystoneDrivebaseSelfTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);
        RoadRunnerMecanumDriveREV drive = new RoadRunnerMecanumDriveREV(
                new SkystoneDriveConstants(), simplerHardwareMap, true);


        telemetry.log().add("Press play to begin the drive base self test");
        telemetry.log().add("Make sure the robot drive wheels can move freely");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        List<Double> startWheelPositions = drive.getWheelPositions();

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(24)
                .build();


        drive.followTrajectorySync(trajectory);

        List<Double> endWheelPositions = drive.getWheelPositions();

        // frontLeft, rearLeft, rearRight, frontRight
        double frontLeftDelta = endWheelPositions.get(0) - startWheelPositions.get(0);
        double rearLeftDelta = endWheelPositions.get(1) - startWheelPositions.get(1);
        double rearRightDelta = endWheelPositions.get(2) - startWheelPositions.get(2);
        double frontRightDelta = endWheelPositions.get(3) - startWheelPositions.get(3);

        boolean encoderDirectionGood = frontLeftDelta > 0 && rearLeftDelta > 0 &&
                rearRightDelta > 0 && frontRightDelta > 0;

        telemetry.log().clear();
        telemetry.log().add(String.format("delta (fl, rl, rr, fr): %f4.2, %f4.2, %f4.2, %f4.2",
                frontLeftDelta, rearLeftDelta, rearRightDelta, frontRightDelta));

        telemetry.log().add("dir: " + (encoderDirectionGood ? "pass" : "fail"));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}

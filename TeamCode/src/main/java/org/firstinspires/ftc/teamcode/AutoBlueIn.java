/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name="AutoBlueIn", group="Autonomous LinearOpMode")
//@Disabled
public class AutoBlueIn extends LinearOpMode {
    Robot2024 robot;
    private DistanceSensor sensorRange;
    private double distance;
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @SuppressLint("DefaultLocale")
    @Override
    //public void runOpMode() throws InterruptedException {
    public void runOpMode() throws InterruptedException {
        robot = new Robot2024((this));
        robot.initializeRobot();
        telemetry.addData("Status", "Initialized");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range2");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
         while (opModeIsActive()) {
             double NumberOfSamples=0;
//             NumberOfSamples = 0;/
             double Sum=0;
//             Sum = 0;
             double Average =0;
             while (NumberOfSamples<100) {
                 distance = sensorRange.getDistance(DistanceUnit.INCH);
                 Sum = Sum + distance;
                 NumberOfSamples = NumberOfSamples + 1;
                 Average = Sum / NumberOfSamples;
                 telemetry.addData("AverageDistance:", Average);
                 telemetry.addData("distance: ", distance);
                 telemetry.update();
             }
                 Average = Sum / NumberOfSamples;
                 /// if distance < 30 = drive foward x drop pixel, else, move next location, and repeat. Else, go to third location, repeat.
                 distance = sensorRange.getDistance(DistanceUnit.INCH);
                 if (Average<24) {
                     robot.moveRobotAuto(robot.RIGHT, 0.5, 5);
                     robot.moveRobotAuto(robot.REVERSE, 0.5, 14);
//                      robot.dump_bucket();
                    robot.moveRobotAuto(robot.FORWARD, 0.5, 3);
//                     robot.raise_bucket();
                     robot.moveRobotAuto(robot.RIGHT, 0.5, 15);
                     robot.moveRobotAuto(robot.REVERSE, 0.5, 12);
                     robot.rotateRobotAuto2(robot.TURN_LEFT, 90, 0.5);
                     robot.moveRobotAuto(robot.REVERSE, 0.5, 12);
//                    robot.rollSweeperOut(0.5);
//                    robot.raiseElevatorToPosition(1, 1000);
//                    robot.dump_bucket();
//                    sleep(1000);
//                    robot.raise_bucket();
//                     robot.raiseElevatorToPosition(1, 0);
                     robot.moveRobotAuto(robot.RIGHT, 0.5, 22);
                 }

             robot.moveRobotAuto(robot.LEFT, 0.3, 9);
             sleep(1000);
             NumberOfSamples = 0;
             Sum = 0;
             Average = 0;

             while (NumberOfSamples<100) {
                 distance = sensorRange.getDistance(DistanceUnit.INCH);
                 Sum = Sum + distance;
                 NumberOfSamples = NumberOfSamples + 1;
                 Average = Sum/NumberOfSamples;
                 telemetry.addData("AverageDistance:", Average);
                 telemetry.addData("distance: ", distance);
                 telemetry.update();
             }
             Average = Sum / NumberOfSamples;
             if (Average<27) {
                 robot.moveRobotAuto(robot.RIGHT, 0.5, 9);
                 robot.moveRobotAuto(robot.REVERSE, 0.3, 29);
//                robot.dump_bucket();
                 robot.moveRobotAuto(robot.FORWARD, 0.5, 3);
//                 robot.raise_bucket();
//                 robot.rollSweeperOut(0.5);
                 robot.rotateRobotAuto2(robot.TURN_LEFT, 90, 0.5);
                 robot.moveRobotAuto(robot.REVERSE, 0.3, 32);
//                 robot.rollSweeperOut(0.5);
//                 robot.raiseElevatorToPosition(1, 1000);
//                 robot.dump_bucket();
//                 sleep(1000);
//                 robot.raise_bucket();
//                 robot.raiseElevatorToPosition(1, 0);
                 robot.moveRobotAuto(robot.RIGHT, 0.3, 23);
             }
             robot.moveRobotAuto(robot.RIGHT, 0.3, 5);
             robot.moveRobotAuto(robot.REVERSE, 0.3, 17);
             robot.rotateRobotAuto2(robot.TURN_RIGHT, 90, 0.5);
             robot.moveRobotAuto(robot.REVERSE, 0.3, 5);
//            robot.dump_bucket();
             robot.moveRobotAuto(robot.FORWARD, 0.3, 12);
//             robot.raise_bucket();
//             robot.rollSweeperOut(0.5);
             robot.moveRobotAuto(robot.REVERSE, 0.3, 17);
             robot.rotateRobotAuto2(robot.TURN_RIGHT, 180, 0.5);
             robot.moveRobotAuto(robot.REVERSE, 0.3, 30);
//             robot.raiseElevatorToPosition(1, 1000);
//             robot.dump_bucket();
//             sleep(1000);
//             robot.raise_bucket();
//             robot.raiseElevatorToPosition(1, 0);
             robot.moveRobotAuto(robot.RIGHT, 0.3, 24);

             telemetry.addData("RUNNING ", robot.getTicks());
           telemetry.update();
         }

   }
}
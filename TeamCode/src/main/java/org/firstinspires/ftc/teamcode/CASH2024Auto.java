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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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

@Autonomous(name="DONTUSETHISRED", group="Autonomous LinearOpMode")
//@Disabled
public class CASH2024Auto extends LinearOpMode {
    Robot2024 robot;
    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    private double distance;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @SuppressLint("DefaultLocale")
    @Override
    //public void runOpMode() throws InterruptedException {
    public void runOpMode() throws InterruptedException {
        robot = new Robot2024((this));
        robot.initializeRobot();
   //     robot.initializeImplements();
        telemetry.addData("Status", "Initialized");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
         while (opModeIsActive()) {
             sleep(1000);
             robot.moveRobotAuto(robot.RIGHT, 0.3, 12);

             double NumberOfSamples=0;
//             NumberOfSamples = 0;/
             double Sum=0;
//             Sum = 0;
             double Average =0;
             while (NumberOfSamples<50) {
                 distance = sensorRange.getDistance(DistanceUnit.INCH);
                 Sum = Sum + distance;
                 NumberOfSamples = NumberOfSamples + 1;
                 Average = Sum/NumberOfSamples;
                 telemetry.addData("AverageDistance:", Average);
                 telemetry.addData("distance: ", distance);
                 telemetry.update();
             }
             Average = Sum / NumberOfSamples;
             /// if distance < 30 = drive foward x drop pixel, else, move next location, and repeat. Else, go to third location, repeat.
             distance = sensorRange.getDistance(DistanceUnit.INCH);
             if (Average<40) {
               //robot.moveRobotAuto(robot.FORWARD, 0.3, 26);
                 /// drop pixel (WIP)
                 robot.moveRobotAuto(robot.LEFT, 0.5, 5);
                robot.moveRobotAuto(robot.REVERSE, 0.5, 15);
           //    robot.dump_bucket();
               sleep(1000);
                 robot.moveRobotAuto(robot.FORWARD, 0.5, 12);
           //     robot.raise_bucket();
          //      robot.rollSweeperOut(1);
                 robot.rotateRobotAuto2(robot.TURN_RIGHT, 90, 0.5);
                 robot.moveRobotAuto(robot.REVERSE, 0.5, 80);
                 robot.moveRobotAuto(robot.RIGHT, 0.5, 29);
//             robot.moveRobotAuto(robot.REVERSE, 0.5, 7);
           //      robot.raiseElevatorToPosition(1, 2500);
         //       sleep(3000);
          //       robot.dump_bucket();
         //        sleep(1000);
         //        robot.raise_bucket();
                 //     robot.raiseElevatorToPosition(1, 0);
                 //    robot.moveRobotAuto(robot.REVERSE, 0.5, 6);
                 sleep(30000);
             }

             robot.moveRobotAuto(robot.LEFT, 0.3, 11);
             sleep(1000);
            NumberOfSamples = 0;
            Sum = 0;
            Average = 0;
                         while (NumberOfSamples<50) {
                 distance = sensorRange.getDistance(DistanceUnit.INCH);
                 Sum = Sum + distance;
                 NumberOfSamples = NumberOfSamples + 1;
                 Average = Sum/NumberOfSamples;
                 telemetry.addData("AverageDistance:", Average);
                 telemetry.addData("distance: ", distance);
                 telemetry.update();
             }
             Average = Sum / NumberOfSamples;
             if (Average<50) {
                 robot.moveRobotAuto(robot.REVERSE, 0.3, 31);
            //     robot.dump_bucket();
                 sleep(1000);
                 robot.moveRobotAuto(robot.FORWARD, 0.5, 26);
           //      robot.raise_bucket();
           //      robot.rollSweeperOut(0.5);
                 robot.rotateRobotAuto2(robot.TURN_RIGHT, 90, 0.5);
                 robot.moveRobotAuto(robot.REVERSE, 0.5, 70);
           //      robot.rollSweeperOut(0);
                 robot.moveRobotAuto(robot.RIGHT, 0.5, 29);
//                 robot.raiseElevatorToPosition(1, 2500);
//                 sleep(3000);
//                 robot.dump_bucket();
//                 sleep(1000);
//                 robot.raise_bucket();
                 sleep(30000);
             }
             robot.moveRobotAuto(robot.REVERSE, 0.5, 24);
             robot.rotateRobotAuto2(robot.TURN_RIGHT, 90, 0.5);
             robot.moveRobotAuto(robot.REVERSE, 0.5, 3);
       //      robot.dump_bucket();
             sleep(1000);
             robot.moveRobotAuto(robot.FORWARD, 0.5, 5);
         //    robot.raise_bucket();
       //      robot.rollSweeperOut(1);
             robot.moveRobotAuto(robot.LEFT, 0.5, 26);
             robot.moveRobotAuto(robot.REVERSE, 0.5, 70);
         //    robot.rollSweeperOut(0);
             robot.moveRobotAuto(robot.RIGHT, 0.5, 29);
//             robot.raiseElevatorToPosition(1, 4000);
//             sleep(3000);
//             robot.dump_bucket();
//             sleep(1000);
//             robot.raise_bucket();
             sleep(30000);


                  telemetry.addData("RUNNING ", robot.getTicks());
           telemetry.update();
         }

   }
}
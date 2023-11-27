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

@Autonomous(name="AutoRedIn_Schultz", group="Autonomous LinearOpMode")
//@Disabled
public class AutoRedIn_Schultz extends LinearOpMode {
    Robot2024 robot;
    //This sensor is used to detect the team prop.  There are two of them, one on left and one on
    //right.  The each sensor is used for a different start location of the robot depending on
    //color and alliance.
    private DistanceSensor sensorRange;
    //Variable to hold the distance value measured from the Distance sensor
    private double distance;

    //Variable that holds the runtime of the operation
    private ElapsedTime runtime = new ElapsedTime();

    //Toggle between running with just the drivng of the robot if we are using second test bed to practice.
    //Set this to true to run with full robot
    //Set this to false if you want to run just the driving portion of the bot.
    private boolean initimpliments = true;

     /*
     * Code to run ONCE when the driver hits INIT
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        //create the robot object that basically activates everything in Robot2024.java file.
        robot = new Robot2024((this));
        //initializes the Driving portion of the robot
        robot.initializeRobot();
        if ( initimpliments == true ) {
            robot.initializeImplements();  //NOTE:  We reset everything because this is beginning of match
        }
        telemetry.addData("Status", "Initialized");

        //Setup the Distance Sensor for sensing the team prop.
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            //First step is to reset the bucket so that it is held into position.
            robot.reset_pixle_bucket();

            //Step 1:  Setup robot to scan the first position for the team prop
            robot.moveRobotAuto(robot.LEFT, 0.5, 4);
            robot.moveRobotAuto(robot.REVERSE, 0.3, 6);

            //Get Ave Distance from distance sensor
            double Average = getAverageDistanceFromSensor(sensorRange);

            //If the Average is less than the below value we have determined that the robot found the
            // team prop
            double MIN_DISTANCE_TO_PROP = 24;

            //If it is at location 1
            if (Average<MIN_DISTANCE_TO_PROP) {
                telemetry.addLine("Found Team Prop at Location:  #1");
                telemetry.update();

                robot.moveRobotAuto(robot.LEFT, 0.5, 2);
                robot.moveRobotAuto(robot.REVERSE, 0.5, 5);
                if ( initimpliments == true ) {
                    robot.raiseElevatorToPosition_Autonomous(.5,robot.DELIVER_PIXLE_POSITION);
                    robot.sweeperCommand(1.0);
                    sleep(1000);
                    robot.sweeperCommand(0.0);
                }
                robot.moveRobotAuto(robot.FORWARD, 0.5, 3);
                robot.moveRobotAuto(robot.LEFT, 0.5, 18);
                robot.moveRobotAuto(robot.REVERSE, 0.5, 11);
                robot.rotateRobotAuto2(robot.TURN_RIGHT, 90, 0.5);
                if ( initimpliments == true ) {
                    robot.raiseElevatorToPosition_Autonomous(1, robot.ELEVATOR_MID_POSITION);
                    robot.dump_pixle();
                    sleep(1000);
                    robot.reset_pixle_bucket();
                    robot.raiseElevatorToPosition_Autonomous(.5, 0);
                }
                robot.moveRobotAuto(robot.LEFT, 0.5, 15);
                robot.moveRobotAuto(robot.REVERSE, 0.5, 5);

                sleep(30000);
            }//end of poistion 1 work

            telemetry.addLine("Didn't find team prop at location 1. Moving to chech number 2");
            telemetry.update();

            robot.moveRobotAuto(robot.RIGHT, 0.3, 10);
            sleep(1000);
            Average = getAverageDistanceFromSensor(sensorRange);

            if (Average<27) {
                telemetry.addLine("Found Team Prop at Location:  #2");
                telemetry.update();
                robot.moveRobotAuto(robot.REVERSE, 0.3, 19);
                if ( initimpliments == true ) {
                    robot.raiseElevatorToPosition_Autonomous(.5,robot.DELIVER_PIXLE_POSITION);
                    robot.sweeperCommand(1.0);
                    sleep(1000);
                    robot.sweeperCommand(0.0);
                }

                robot.moveRobotAuto(robot.FORWARD, 0.5, 3);
                robot.rotateRobotAuto2(robot.TURN_RIGHT, 90, 0.5);
                robot.moveRobotAuto(robot.REVERSE, 0.3, 28);
                if ( initimpliments == true ) {
                    robot.raiseElevatorToPosition_Autonomous(1, robot.ELEVATOR_MID_POSITION);
                    robot.dump_pixle();
                    sleep(1000);
                    robot.reset_pixle_bucket();
                    robot.raiseElevatorToPosition_Autonomous(.5, 0);
                }

                robot.moveRobotAuto(robot.LEFT, 0.3, 15);
                robot.moveRobotAuto(robot.REVERSE, 0.5, 6);
                sleep(30000);
            }//end of second position

            telemetry.addLine("Didn't find 1 or 2 so assume #3");
            telemetry.update();
            //Now we know that the pixel is at the last location so just go there and drop pixle
            robot.moveRobotAuto(robot.REVERSE, 0.3, 19);
            robot.rotateRobotAuto2(robot.TURN_LEFT, 90, 0.5);
            if ( initimpliments == true ) {
                robot.raiseElevatorToPosition_Autonomous(.5,robot.DELIVER_PIXLE_POSITION);
                robot.sweeperCommand(1.0);
                sleep(1000);
                robot.sweeperCommand(0.0);
            }

            robot.moveRobotAuto(robot.FORWARD, 0.3, 29);
            robot.rotateRobotAuto2(robot.TURN_LEFT, 180, 0.5);
            if ( initimpliments == true ) {
                robot.raiseElevatorToPosition_Autonomous(1, robot.ELEVATOR_MID_POSITION);
                robot.dump_pixle();
                sleep(1000);
                robot.reset_pixle_bucket();
                robot.raiseElevatorToPosition_Autonomous(.5, 0);
            }

            robot.moveRobotAuto(robot.LEFT, 0.3, 15);
            robot.moveRobotAuto(robot.REVERSE, 0.5, 6);

            telemetry.addData("Done ", robot.getTicks());
            telemetry.update();
            sleep(30000);
         }

   }

   //This method is what is used to get the average from the desired sensor.
   double getAverageDistanceFromSensor(DistanceSensor dist_sensor){
       double NumberOfSamples=0;
       double Sum=0;
       double Average;
       double dist;
       while ( NumberOfSamples <= 100  && opModeIsActive() ) {
           dist = dist_sensor.getDistance(DistanceUnit.INCH);
           Sum = Sum + dist;
           NumberOfSamples = NumberOfSamples + 1;
           telemetry.addData("distance: ", dist);
           telemetry.update();
       }
       Average = Sum / NumberOfSamples;
       telemetry.addData("Average: ", Average);
       telemetry.update();
       return Average;
   }
}
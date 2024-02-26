/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy RobotMovement.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear OpMode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

@TeleOp(name="TeleOp - RobotMovement", group="Robot")
public class RobotMovement extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    org.firstinspires.ftc.teamcode.RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double left = 0;
        double right = 0;
        double a1 = 0;
        double a2 = 0;
        double wristOffset = 0;
        boolean clawOpen = false;
        double a1ArmSpeed = 0.2;
        double a2ArmSpeed = 0.3;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            left = gamepad1.left_stick_y;
            right  =  -gamepad1.right_stick_y;

            robot.driveRobot(left, right);

            if (gamepad1.dpad_down)
                wristOffset += robot.WRIST_SERVO_SPEED;
            else if (gamepad1.dpad_up)
                wristOffset -= robot.WRIST_SERVO_SPEED;
            wristOffset = Range.clip(wristOffset, -0.5, 0.5);

            //A1 CONTROL
            if (gamepad2.left_stick_y != 0)
            {
                a1 = -gamepad2.left_stick_y * a1ArmSpeed;
            }
            else{
                a1 = 0;
            }


            //A2 CONTROL
            if (gamepad2.right_stick_y != 0)
            {
                a2 = -gamepad2.right_stick_y * a2ArmSpeed;
            }
            else{
                a2 = 0;
            }


            //CLAW CONTROL
            if(gamepad2.y)
            {
                clawOpen = !clawOpen;
            }

            //PLANE LAUNCHER
            if(gamepad1.ps && gamepad2.ps){
                robot.firePlaneLauncher(0.3);
            }
            else{
                robot.firePlaneLauncher(0);
            }

            //SPEED CONTROLLER
            if(gamepad2.left_bumper){
                a1ArmSpeed = 0.05;
            }
            else{
                a1ArmSpeed = 0.2;
            }
            if(gamepad2.right_bumper){
                a2ArmSpeed = 0.15;
            }
            else{
                a2ArmSpeed = 0.3;
            }

            robot.setA1Power(a1);
            robot.setA2Power(a2);
            robot.setClaw(clawOpen);

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("-", "-------");

            telemetry.addData("Left", "%.2f", left);
            telemetry.addData("Right",  "%.2f", right);
            telemetry.addData("A1",  a1);
            telemetry.addData("A2",  a2);
            telemetry.addData("Wrist Position",  "Offset = %.2f", wristOffset);
            telemetry.addData("Claw State",  "State = " + clawOpen);
            telemetry.addData("A1 Arm Speed", a1ArmSpeed);
            telemetry.addData("A2 Arm Speed", a2ArmSpeed);
            telemetry.update();
        }
    }
}
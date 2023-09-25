/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: Omni Linear - Autonomous", group="Linear Opmode")
public class autonomous extends LinearOpMode {
    public int direction = 1;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //@Override
    public void stopM() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void driveForward(double power, int miliseconds) {
        direction = 1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        sleep(miliseconds);
        stopM();
    }


    public void driveBackward(double power, int miliseconds) {
        direction = -1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        sleep(miliseconds);
        stopM();
    }

    public void rotateRight(double power, int miliseconds) {
        direction = 1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction * -1);
        sleep(miliseconds);
        stopM();
    }

    public void rotateLeft(double power, int miliseconds) {
        direction = -1;
        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction * -1);
        sleep(miliseconds);
        stopM();
    }

    public void straifRight(double power, int miliseconds) {
        direction = 1;
        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction);
        sleep(miliseconds);
        stopM();
    }

    public void straifLeft(double power, int miliseconds) {
        direction = -1;
        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction);
        sleep(miliseconds);
        stopM();
    }

    public void returnToLocaiton(double power, int miliseconds, int direction) {

        leftFrontDrive.setPower(power * direction);
        rightFrontDrive.setPower(power * direction);
        leftBackDrive.setPower(power * direction);
        rightBackDrive.setPower(power * direction);
        sleep(miliseconds);
        stopM();

        leftFrontDrive.setPower(power * direction * -1);
        rightFrontDrive.setPower(power * direction * -1);
        leftBackDrive.setPower(power * direction * -1);
        rightBackDrive.setPower(power * direction * -1);
        sleep(miliseconds);
        stopM();
    }

    public void runOpMode() {
        // Initialize your hardware here (motor hardware mappings)

        waitForStart(); // Wait for the start button to be pressed
            driveForward(1,1000);
            rotateLeft(1, 500);
        // Your autonomous code here:
        // Example: driveForward(0.5, 1000); // Drive forward at half power for 1 second

        // Stop all motors when autonomous is done
        stopM();
    }


}




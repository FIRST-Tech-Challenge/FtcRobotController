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

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Finalizable;

/*
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
@Disabled
public class MainMovement extends LinearOpMode {
    //move forward: all = cc
    //move backwards all = c
    //strafe left = BL, FR = cc, BR, FL = c
    private DcMotor BLDrive; //Initializes Back-Left direct current motor for the driving function of our robot, gary.
    private DcMotor BRDrive;
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    final float joystickDeadzone = 0.1f;

    boolean usingLStick;
    public float LjoystickX = gamepad1.left_stick_x;
    public float LjoystickY = gamepad1.left_stick_y;
    public float RjoystickX = gamepad1.right_stick_x;
    public float RjoystickY = gamepad1.right_stick_y;
    @Override
    public void runOpMode() {

        //psuedocode (:skull:, :fire:, :splash:, :articulated-lorry:, :flushed:, :weary:, :sob:);
        BLDrive  = hardwareMap.get(DcMotor.class, "bl_drive");
        BRDrive  = hardwareMap.get(DcMotor.class, "br_drive");
        FLDrive  = hardwareMap.get(DcMotor.class, "fl_drive");
        FRDrive  = hardwareMap.get(DcMotor.class, "fr_drive");

        epicRotationMovement();
        legendaryStrafeMovement();

    }

    private void setMotorPowers(float BL, float BR, float FL, float FR, float speed){
        //set all the motor powers to the floats defined
        BLDrive.setPower(BL*speed);
        BRDrive.setPower(BR*speed);
        FLDrive.setPower(FL*speed);
        FRDrive.setPower(FR*speed);
    }



    private void epicRotationMovement() {
        // rotates the robot if left stick is not being used (movement takes priorities)
        if (!(Math.abs(RjoystickX) <= joystickDeadzone) && !(Math.abs(RjoystickY) <= joystickDeadzone) && !usingLStick) {
            if(gamepad1.right_stick_x > 0) {
                setMotorPowers(1, -1,1,-1,gamepad1.right_stick_x); // moves wheels at speed how far x is from origin
            } else if (gamepad1.right_stick_x < 0) {
                setMotorPowers(-1, 1,-1,1,gamepad1.right_stick_x); // moves wheels at speed how far x is from origin
            }
        }
    }

    //    _                     _          _   _                 _            _                                      __   _
    //   (_)  __ _   __   ___  | |__      (_) (_)  __ _   __ _  | |  _  _    | |__   __ _   _ _    __   _ _   ___   / _| | |_
    //   | | / _` | / _| / _ \ | '_ \     | | | | / _` | / _` | | | | || |   | '_ \ / _` | | ' \  / _| | '_| / _ \ |  _| |  _|
    //  _/ | \__,_| \__| \___/ |_.__/    _/ | |_| \__, | \__, | |_|  \_, |   |_.__/ \__,_| |_||_| \__| |_|   \___/ |_|    \__|
    // |__/                             |__/      |___/  |___/       |__/

    private void legendaryStrafeMovement() {
        float minSpeed = 0.15f;
        double addSpeed = Math.sqrt(LjoystickX*LjoystickX + LjoystickY*LjoystickY);
        float netS = minSpeed + (float)addSpeed; //net speed

        // calculates the angle of the joystick in radians -> degrees
        double LangleInRadians = Math.atan2(LjoystickY, LjoystickX);
        double LangleInDegrees = LangleInRadians * (180 / Math.PI);

        // find quadrant of joystick then convert to strafing movement
        if (!(Math.abs(LjoystickX) <= joystickDeadzone) && !(Math.abs(LjoystickY) <= joystickDeadzone)) {
            usingLStick = true;
            //if not in dead zone
            if (LangleInDegrees >= -22.5 && LangleInDegrees <= 22.5) {
                // right quadrant, move right
                setMotorPowers(-1,1,1,-1,netS);
                System.out.println("Left Stick in RIGHT quadrant");

            } else if (LangleInDegrees > 22.5 && LangleInDegrees < 67.5) {
                // top-right quadrant
                setMotorPowers(0,1,1,0,netS);
                System.out.println("LeftStick in TOP-RIGHT quadrant");

            } else if (LangleInDegrees > -67.5 && LangleInDegrees < -22.5) {
                // bottom-right quadrant
                setMotorPowers(-1,0,0,-1,netS);
                System.out.println("Left Stick in BOTTOM-RIGHT quadrant");

            } else if (LangleInDegrees >= 67.5 && LangleInDegrees <= 112.5) {
                // top quadrant
                setMotorPowers(1,1,1,1,netS);
                System.out.println("Left Stick in TOP quadrant");

            } else if (LangleInDegrees > -112.5 && LangleInDegrees < -67.5) {
                // bottom quadrant
                setMotorPowers(-1,-1,-1,-1,netS);
                System.out.println("Left Stick in BOTTOM quadrant");

            } else if (LangleInDegrees > 112.5 && LangleInDegrees < 157.5) {
                // top-left quadrant
                setMotorPowers(1,0,0,1,netS);
                System.out.println("Left Stick in TOP-LEFT quadrant");

            } else if (LangleInDegrees > -157.5 && LangleInDegrees < -112.5) {
                // bottom-left quadrant
                setMotorPowers(0,-1,-1,0,netS);
                System.out.println("Left Stick in BOTTOM-LEFT quadrant");

            } else if (LangleInDegrees >= 157.5 || LangleInDegrees <= -157.5) {
                // left quadrant
                setMotorPowers(1,-1,-1,1,netS);
                System.out.println("Left Stick in LEFT quadrant");

            }

        } else {
            usingLStick = false;
        }

    }

}

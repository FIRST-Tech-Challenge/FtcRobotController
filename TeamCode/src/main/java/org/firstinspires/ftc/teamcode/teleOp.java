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

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="teleOp", group="Iterative Opmode")
//@Disabled
public class teleOp extends OpMode
{
    //Declare runtime variable
    private ElapsedTime runtime = new ElapsedTime();

    //Set Motor objects
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    Motor shooter;
    Motor backIntake;

    //Set Servo objects
    //CRServo leftLift;
    //CRServo rightLift;

    //Initialize
    @Override
    public void init() {
        //Define motors/servos hardware maps
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        shooter = new Motor(hardwareMap, "shooter", 28, 6000);
        backIntake = new Motor(hardwareMap, "intake", 5, 6);

        //leftLift = new CRServo(hardwareMap, "leftLift");
        //rightLift = new CRServo(hardwareMap, "rightLift");

        //Set Run modes
        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        shooter.setRunMode(Motor.RunMode.RawPower);
        backIntake.setRunMode(Motor.RunMode.RawPower);

        //Set Directions
        leftFront.setInverted(true);
        rightFront.setInverted(false);
        leftBack.setInverted(true);
        rightBack.setInverted(false);

        shooter.setInverted(false);
        backIntake.setInverted(false);

        //Initialized
        telemetry.addData("Status", "Initialized");
    }

    //Repeat loop prior to hitting play
    @Override
    public void init_loop() {
    }

    //Play once
    @Override
    public void start() {
        runtime.reset();
    }

    //Play loop
    @Override
    public void loop() {
        //Set variables for motor powers
        //Wheels
        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;

        //Used for Calculations:
        double forward = 0;
        double side = 0;
        double turn = 0;

        /////////////
        //GAMEPAD 1//
        /////////////

        //DRIVE
        forward = -gamepad1.left_stick_y;
        side = gamepad1.left_stick_x; //Positive means right
        turn = gamepad1.right_stick_x; //Positive means turn right

        leftFrontPower = (forward + side + turn) / 2;
        leftBackPower = (forward - side + turn) / 2;
        rightFrontPower = (forward - side - turn) / 2;
        rightBackPower = (forward + side - turn) / 2;

        //Boost; Slow; Normal
        if (gamepad1.x) {
            //BOOST

            leftFrontPower = leftFrontPower * 2;
            leftBackPower = leftBackPower * 2;
            rightFrontPower = rightFrontPower * 2;
            rightBackPower = rightBackPower * 2;

        } else if (gamepad1.a) {
            //SLOW

            leftFrontPower = leftFrontPower * 0.6;
            leftBackPower = leftBackPower * 0.6;
            rightFrontPower = rightFrontPower * 0.6;
            rightBackPower = rightBackPower * 0.6;

        } else {
            //This is normal.  Don't put anything here.
        }

        //Intake

        //Back
        if (gamepad1.left_trigger > 0.1) {
            //Outake takes priority
            backIntake.set(gamepad1.left_trigger);
        } else {
            //Intake
            backIntake.set(gamepad1.right_trigger);
        }

        /////////////
        //GAMEPAD 2//
        /////////////

        //Shooter
        shooter.set(gamepad2.left_stick_y);

        // Send power to wheel motors
        leftFront.set(leftFrontPower);
        rightFront.set(rightFrontPower);
        leftBack.set(leftBackPower);
        rightBack.set(rightBackPower);

    }

    //Stop code
    @Override
    public void stop() {
    }

}

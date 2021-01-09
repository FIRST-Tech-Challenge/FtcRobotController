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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name="arcadeDrive", group="Iterative Opmode")

public class arcadeDrive extends OpMode {
    // Declare OpMode members.

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFoward = null;
    public DcMotor rightReverse = null;
    public DcMotor leftReverse = null;
    public DcMotor rightFoward = null;
    public DcMotor intake = null;

    //Constants
    final double COUNTS_PER_MOTOR_REV = 1440; //Counts to rotations, testing later
    final double DRIVE_GEAR_REDUCTION = 1.0; //If gears are added
    final double WHEEL_DIAMETER_INCHES = 4.0; //Wheel size
    final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES; //Circumference of wheel


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //Name motors

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFoward = hardwareMap.get(DcMotor.class, "left_foward_drive");
        rightReverse = hardwareMap.get(DcMotor.class, "right_reverse_drive");
        leftReverse = hardwareMap.get(DcMotor.class, "left_reverse_drive");
        rightFoward = hardwareMap.get(DcMotor.class, "right_foward_drive");
        intake = hardwareMap.get(DcMotor.class, "intake_intial");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFoward.setDirection(DcMotor.Direction.FORWARD);
        rightReverse.setDirection(DcMotor.Direction.REVERSE);
        leftReverse.setDirection(DcMotor.Direction.FORWARD);
        rightFoward.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        //Reset encoders
        leftFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset power
        leftFoward.setPower(0);
        rightReverse.setPower(0);
        leftReverse.setPower(0);
        rightFoward.setPower(0);
        intake.setPower(0);

        //use encoders
        leftFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    //This function converts the points from the joysticks to degrees
    public double driveAngle(double x, double y) {
        double degree = (((Math.atan2(y, x)) * 180 / Math.PI)) + 90;
        if (degree > 180) {
            degree -= 360;
        }
        return degree;
    }
    //This function converts the degrees into the power needed for the motors
    public void motorPower(double angle, double forwardPower, double sidePower, double turnPower)
    {
        if(Math.abs(forwardPower) > 0.05 && Math.abs(turnPower) > 0.05 && Math.abs(sidePower) < 0.05)
        {
            if(turnPower > 0)
            {
                leftReverse.setPower(forwardPower);
                leftFoward.setPower(forwardPower);
                rightReverse.setPower(0);
                rightFoward.setPower(0);
            }
            else if(turnPower < 0)
            {
                leftReverse.setPower(0);
                leftFoward.setPower(0);
                rightReverse.setPower(forwardPower);
                rightFoward.setPower(forwardPower);
            }

        }
        else if (Math.abs(turnPower) > 0.05 && Math.abs(forwardPower) < 0.05 && Math.abs(sidePower) < 0.05)
        {
            leftFoward.setPower(-turnPower);
            leftReverse.setPower(-turnPower);
            rightFoward.setPower(turnPower);
            rightReverse.setPower(turnPower);
        }
        else
        {
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;
            double power;

            double radians = (angle * (Math.PI / 180)); //This is to the radians out of the degrees to use the equation

            if (Math.abs(forwardPower) >= Math.abs(sidePower)) {
                power = (Math.abs(forwardPower)); // To determine the power it should use
            } else if (Math.abs(forwardPower) <= Math.abs(sidePower)) {
                power = (Math.abs(sidePower)); // To determine the power it should use
            } else {
                power = 0;
            }

            leftFrontPower = (power * Math.sin(radians + (Math.PI / 4)));
            rightFrontPower = (power * Math.cos(radians + (Math.PI / 4)));
            leftBackPower = (power * Math.cos(radians + (Math.PI / 4)));
            rightBackPower = (power * Math.sin(radians + (Math.PI / 4)));


            leftFoward.setPower(-leftFrontPower);
            rightFoward.setPower(-rightFrontPower);
            leftReverse.setPower(-leftBackPower);
            rightReverse.setPower(-rightBackPower);
        }

    }


    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double forwardPower;
        double sidePower;
        double turnPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double forwardBackward = gamepad1.left_stick_y;
        double sideWays = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;



        forwardPower = Range.clip(forwardBackward, -1.0, 1.0);
        sidePower = Range.clip(sideWays, -1.0, 1.0);
        turnPower = Range.clip(turn, -1.0, 1.0);


        double degrees = driveAngle(sideWays, forwardBackward);

        // calls upon the function  motorpower to set the powers of the motor based on the input of your joysticks on your controller 1
        motorPower(degrees, forwardPower, sidePower, turnPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "degrees (%.2f)", degrees);
        telemetry.addData("Path2",  "Running at %7d :%7d",
                leftFoward.getCurrentPosition(),
                rightFoward.getCurrentPosition());
        telemetry.update();
    }

}

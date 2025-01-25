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

package org.firstinspires.ftc.McQueen;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: MecaMeca", group="Robot")

public class MecaMecaConfigFile extends OpMode {

    /* Declare OpMode members. */
    public DcMotor  frontLeft  = null;
    public DcMotor  frontRight   = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor armLift = null;
    public DcMotor armExtend = null;
    public Servo claw = null;
    public Servo rotate = null;
    public Servo wrist = null;
    public Servo elbow = null;
    public Servo upperWrist = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // left_wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // right_wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        /* REMINDER
        This is bot-centric meca drive!!
        Thank you Game Manual 0-chan
        https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
         */

        //double left, leftResult;
        //double right, rightResult;
            double horizontal = -gamepad1.left_stick_y; // L-stick up/down - Remember, Y stick value is reversed
            double vertical = gamepad1.left_stick_x * 1.1; // L-stick left/right - Counteract imperfect strafing (by multiplying by 1.1)
            double turn = gamepad1.right_stick_x; // R-stick left/right

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(turn), 1);
             // Denominator
            double frontLeftPower = (horizontal + vertical + turn) / denominator;
            double backLeftPower = (horizontal - vertical + turn) / denominator;
            double frontRightPower = (horizontal - vertical - turn) / denominator;
            double backRightPower = (horizontal + vertical - turn) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        //left = -1 * speedMult * gamepad1.left_stick_y;
        //right = -1 * speedMult * gamepad1.right_stick_y;

        //leftResult = left * left * Math.signum(left);
        //rightResult = right * right * Math.signum(right);
        //leftResult = left;
        //rightResult = right;

        //left_wheels.setPower(leftResult);
        //right_wheels.setPower(rightResult);

        // Send telemetry message to signify robot running (and show the variable values);
        //telemetry.addData("left",  "%.2f", left);
        //telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

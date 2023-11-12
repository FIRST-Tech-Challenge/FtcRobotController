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
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drive", group="Iterative Opmode")
public class DriveTrain extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private DcMotor intakeMotor = null;
    private Servo clawServo = null;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    private double Kg = 0.07;
    //private PIDController pid = new PIDController(0.06, 0, 0);

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        clawServo = hardwareMap.servo.get("claw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;

        // mecanum drive
        double y = gamepad1.left_stick_y * -1;
        double x = gamepad1.left_stick_x * 1.5;
        double pivot = gamepad1.right_stick_x;

        frontLeftPower = (pivot+y+x);
        rearLeftPower = (pivot+y-x);
        frontRightPower = (-pivot+y-x);
        rearRightPower = (-pivot+y+x);


        // gamepad 1 controls
        if(gamepad1.left_bumper) {
            frontLeft.setPower(frontLeftPower * 0.35);
            frontRight.setPower(frontRightPower * 0.35);
            rearLeft.setPower(rearLeftPower * 0.35);
            rearRight.setPower(rearRightPower * 0.35);
        }
        else {
            frontLeft.setPower(frontLeftPower * .85);
            frontRight.setPower(frontRightPower * .85);
            rearLeft.setPower(rearLeftPower * .85);
            rearRight.setPower(rearRightPower * .85);
        }
        if(gamepad1.dpad_left) {
            frontLeft.setPower(-0.5);
            rearRight.setPower(-0.5);
            frontRight.setPower(0.5);
            rearLeft.setPower(0.5);
        }
        if(gamepad1.dpad_right) {
            frontLeft.setPower(0.5);
            rearRight.setPower(0.5);
            frontRight.setPower(-0.5);
            rearLeft.setPower(-0.5);
        }

        if(gamepad1.right_bumper) {
            intakeMotor.setPower(1);
        }
        else {
            intakeMotor.setPower(0);
        }

        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
        // gamepad 1 controls
        double power = 0;
//        if(gamepad1.a) {
//            //power = pid.PIDControl(1000, liftMotor.getCurrentPosition());
//            liftMotor1.setPower(power);
//            liftMotor2.setPower(power);
//        }
//        if(gamepad1.x) {
//            //power = pid.PIDControl(2000, liftMotor.getCurrentPosition());
//            liftMotor1.setPower(power);
//            liftMotor2.setPower(power);
//        }
//        if(gamepad1.y) {
//            //power = pid.PIDControl(3000, liftMotor.getCurrentPosition());
//            liftMotor1.setPower(power);
//            liftMotor2.setPower(power);
//        }

        // gamepad 2 controls
        if(gamepad2.y) {
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        } else if (gamepad2.a) {
            liftMotor1.setPower(-.8);
            liftMotor2.setPower(-.8);
        } else if (gamepad2.right_stick_y > 0.8) {
            liftMotor1.setPower(-0.8);
            liftMotor2.setPower(-0.8);
        } else if (gamepad2.right_stick_y < -0.8) {
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        } else if (gamepad2.left_bumper && gamepad2.a) {
            liftMotor1.setPower(-.4);
            liftMotor2.setPower(-.4);
        }
//        else {
//            liftMotor1.setPower(Kg);
//            liftMotor2.setPower(Kg);
//        }


//        if(gamepad2.left_bumper) {
//            clawServo.setPosition(0.26);
//        } else if (gamepad2.right_bumper) {
//            clawServo.setPosition(0.5);
//        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), rearLeft (%.2f), frontRight (%.2f), rearRight (%.2f)", frontLeftPower, rearLeftPower, frontRightPower, rearRightPower);
        telemetry.addData("Lift Position", liftMotor1.getCurrentPosition());
        telemetry.update();
    }

//    public void liftToPosition(int target) {
//        int currentPosition1 = liftMotor1.getCurrentPosition();
//        int currentPosition2 = liftMotor2.getCurrentPosition();
//        while(Math.abs(currentPosition1 - target) > 6 && Math.abs(currentPosition2 - target) > 6) {
//            currentPosition1 = liftMotor1.getCurrentPosition();
//            currentPosition2 = liftMotor2.getCurrentPosition();
//            int targetPosition = target;
//            double power1 = returnPower(targetPosition, liftMotor1.getCurrentPosition());
//            double power2 = returnPower(targetPosition, liftMotor2.getCurrentPosition());
//            liftMotor1.setPower(power1);
//            liftMotor2.setPower(power2);
//            telemetry.addData("current position", currentPosition1);
//            telemetry.addData("targetPosition", targetPosition);
//            telemetry.update();
//        }
//    }

    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


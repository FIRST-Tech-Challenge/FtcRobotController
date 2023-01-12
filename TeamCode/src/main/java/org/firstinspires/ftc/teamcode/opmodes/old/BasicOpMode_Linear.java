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

package org.firstinspires.ftc.teamcode.opmodes.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//CAUTION: DEPRECATED SINCE 2023/01/11. MODIFY Autonomous_root.java INSTEAD.//
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//@TeleOp(name="Mecanum Drivetrain TeleOp :)", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private static final ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private Servo gripper = null;

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 2-0.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

        //bring arm back down to home position before resetting encoders
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        //resetting encoders at home level
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting the motors into the necessary mode for using the encoders
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timeElapsed = new ElapsedTime();

        int lowJunction = -420;
        int middleJunction = -630;
        int highJunction = -910;
        int armTarget = 0;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            double left_y = gamepad1.left_stick_y;
            double left_x = gamepad1.left_stick_x;
            double strafe_side = gamepad1.right_stick_x;

            if (gamepad2.x) armTarget = lowJunction;
            if (gamepad2.b) armTarget = middleJunction;
            if (gamepad2.y) armTarget = highJunction;
            if (gamepad2.a) armTarget = 0;

            if (gamepad2.left_bumper) {
                gripper.setPosition(0.75);
                armTarget = 0;
            }
            if (gamepad2.right_bumper) gripper.setPosition(0.47);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;

            if (Math.abs(left_y) < 0.2) {
                leftFrontPower = -left_x * 0.8 - strafe_side * 0.6;
                rightFrontPower = left_x * 0.8 + strafe_side * 0.6;
                leftBackPower = left_x * 0.8 - strafe_side * 0.6;
                rightBackPower = -left_x * 0.8 + strafe_side * 0.6;
            } else {
                leftFrontPower = (left_y - left_x) * 0.8 - strafe_side * 0.6;
                rightFrontPower = (left_y + left_x) * 0.8 + strafe_side * 0.6;
                leftBackPower = (left_y + left_x) * 0.8 - strafe_side * 0.6;
                rightBackPower = (left_y - left_x) * 0.8 + strafe_side * 0.6;
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.right_bumper) {
                leftFrontPower *= 0.5;
                rightBackPower *= 0.5;
                rightFrontPower *= 0.5;
                leftBackPower *= 0.5;
            }
            else {
                leftFrontPower *= 1;
                rightBackPower *= 1;
                rightFrontPower *= 1;
                leftBackPower *= 1;
            }

            // Send calculated power to wheels
            motorFL.setPower(leftFrontPower);
            motorFR.setPower(rightFrontPower);
            motorBL.setPower(leftBackPower);
            motorBR.setPower(rightBackPower);

            boolean targetChanged = false;

            if (gamepad2.left_trigger > 0) {
                SetArmPower(0);
                targetChanged = true;
            }
            if (gamepad2.right_trigger > 0) {
                SetArmPower(-gamepad2.right_trigger);
                targetChanged = true;
            }

            if(targetChanged) armTarget = (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;

            if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            leftLift.setTargetPosition(armTarget);
            rightLift.setTargetPosition(armTarget);
            if (rightLift.isBusy() && leftLift.isBusy()) {
                if (leftLift.getCurrentPosition() < leftLift.getTargetPosition() && rightLift.getCurrentPosition() < rightLift.getTargetPosition()) {
                    if ((leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2 < -700 && armTarget != middleJunction) {
                        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        SetArmPower(0.0);
                    }
                    else {
                        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        SetArmPower(0.0);
                    }
                } else {
                    leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    SetArmPower(1.0);
                }
                telemetry.addData("Arm Position", (leftLift.getCurrentPosition() + rightLift.getCurrentPosition()) / 2.0);
                telemetry.addData("Target Position", (leftLift.getTargetPosition() + rightLift.getTargetPosition()) / 2.0);
                telemetry.addData("Arm Power", (leftLift.getPower() + rightLift.getPower()) / 2.0);
                telemetry.update();
            }
        }
    }

    void SetArmPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }
}
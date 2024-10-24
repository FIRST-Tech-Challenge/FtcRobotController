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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Mecanum Driver", group="TeleOp")
public class MecanumDriver extends OpMode {
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();

    private CRServo servo;
    private final static double TURN_POWER = 2.0;
    private final static double FORWARD_POWER = 1.0;
    private final static double VIPER_POWER = 0.7;
    private final static double PIVOT_POWER = 0.7;
    private final static double STRAFE_POWER = FORWARD_POWER * 1.192;
    private final static double SPEED_MULTIPLIER = 2.3;
    public final boolean isFieldCentric = true;
//    private int slidePosition = 0;
//    private int encoderMax = 750;
//    private int encoderMin = 250;

    private Arm arm;

    @Override
    public void init() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BACKLEFT");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        arm = new Arm(hardwareMap);

        telemetry.addData("Status", "Initialized");

        IMU gyro = hardwareMap.get(IMU.class, "imu2");
//        servo = hardwareMap.get(CRServo.class, "intake");
        gyro.resetYaw();

     //   robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
//        robotController.continuousDrive(gamepad1.left_stick_y * SPEED_MULTIPLIER * FORWARD_POWER,
//                gamepad1.left_stick_x * SPEED_MULTIPLIER * STRAFE_POWER,
//                gamepad1.right_stick_x * TURN_POWER, isFieldCentric);
        //arm.setPivotTargetPosition(0, PIVOT_POWER);
//        arm.setViperTargetPosition(1000, VIPER_POWER);
        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            arm.setViperToContinuousMode();
            arm.moveViperSlides((gamepad1.left_trigger - gamepad1.right_trigger) * VIPER_POWER);
        } else if (arm.getViperMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            arm.stopVipers();
        }

        if (gamepad1.dpad_up) {
            arm.setPivotToContinuousMode();
            arm.movePivot(PIVOT_POWER);
        } else if (gamepad1.dpad_down) {
            arm.setPivotToContinuousMode();
            arm.movePivot(-PIVOT_POWER);
        } else if (arm.getPivotMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            arm.stopPivot();
        }

        if (gamepad1.left_bumper) {
            arm.setViperTargetPosition(1000, VIPER_POWER);
        } else if (gamepad1.right_bumper) {
            arm.setViperTargetPosition(30, VIPER_POWER);
        }

//        servo.setPower(1);
//        double servoPosition = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, 0.0, 1.0);


        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Viper Slide Current Position", arm.getViperCurrentPosition());
        telemetry.addData("Viper Slide Target Position", arm.getViperTargetPosition());
        telemetry.addData("Left Pivot Current Position", arm.getLeftPivotCurrentPosition());
        telemetry.addData("Right Pivot Current Position", arm.getRightPivotCurrentPosition());
        telemetry.addData("Left Pivot Target Position", arm.getLeftPivotTargetPosition());
        telemetry.addData("Right Pivot Target Position", arm.getRightPivotTargetPosition());
//        telemetry.addData("servoPosition", servoPosition);
//        robotController.sendTelemetry(telemetry);

//        if (gamepad1.left_bumper) {
//            slidePosition = encoderMax;
//            moveViperSlides();
//
//        } else if (gamepad1.right_bumper) {
//            slidePosition = encoderMin;
//            moveViperSlides();
//        } else {
//            viperSlide1.setPower(0);
//            viperSlide2.setPower(0);
//        }



    }
//
//    public void moveViperSlides() {
//        moveSlide1();
//        moveSlide2();
//    }
//
//    public void moveSlide1() {
//        viperSlide1.setTargetPosition(slidePosition);
//    }
//
//    public void moveSlide2() {
//        viperSlide2.setTargetPosition(slidePosition);
//    }


}

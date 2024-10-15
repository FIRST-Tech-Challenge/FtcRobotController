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
    private final static double STRAFE_POWER = FORWARD_POWER * 1.192;
    private final static double SPEED_MULTIPLIER = 2.3;
    public final boolean isFieldCentric = true;
    private int slidePosition = 0;

    private DcMotor viperSlide1;
    private DcMotor viperSlide2;

    @Override
    public void init() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BACKLEFT");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");

        viperSlide1 = hardwareMap.get(DcMotor.class, "VIPERRIGHT");
        viperSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide2 = hardwareMap.get(DcMotor.class, "VIPERLEFT");
        viperSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");

        frontRight.setDirection(DcMotor.Direction.REVERSE);

        IMU gyro = hardwareMap.get(IMU.class, "imu2");
        servo = hardwareMap.get(CRServo.class, "intake");




        gyro.resetYaw();

        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro);
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
        robotController.continuousDrive(gamepad1.left_stick_y * SPEED_MULTIPLIER * FORWARD_POWER,
                gamepad1.left_stick_x * SPEED_MULTIPLIER * STRAFE_POWER,
                gamepad1.right_stick_x * TURN_POWER, isFieldCentric);
        servo.setPower(1);
        double servoPosition = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, 0.0, 1.0);


        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("servoPosition", servoPosition);
        robotController.sendTelemetry(telemetry);

        if (gamepad1.left_bumper) {
            slidePosition = 1;
            moveViperSlides();

        } else if (gamepad1.right_bumper) {
            slidePosition = -1;
            moveViperSlides();
        } else {
            slidePosition = 0;
            moveViperSlides();
        }


    }

    public void moveViperSlides() {
        moveSlide1();
        moveSlide2();
    }

    public void moveSlide1() {
        viperSlide1.setTargetPosition(slidePosition);
    }

    public void moveSlide2() {
        viperSlide2.setTargetPosition(slidePosition);
    }


}

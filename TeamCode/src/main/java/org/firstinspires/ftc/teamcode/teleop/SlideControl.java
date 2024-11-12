package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MotorPair;

@TeleOp(name = "SlideControl")
public class SlideControl extends OpMode {
    MotorPair slideRotate, slideLift;

    @Override
    public void init() {
        this.slideRotate = new MotorPair(
                1000,
                0.5,
                hardwareMap.get(DcMotor.class, "leftRotation"),
                hardwareMap.get(DcMotor.class, "rightRotation"),
                DcMotor.Direction.REVERSE);
        this.slideLift = new MotorPair(
                2500,
                1.0,
                hardwareMap.get(DcMotor.class, "leftSlide"),
                hardwareMap.get(DcMotor.class, "rightSlide"));
        this.slideRotate.resetPosition();
        this.slideLift.resetPosition();
    }

    @Override
    public void loop() {
        if (this.gamepad2.guide) {
            if (this.gamepad2.dpad_right) {
                this.slideRotate.resetPosition();
            }
            if (this.gamepad2.b) {
                this.slideLift.resetPosition();
            }
            if (this.gamepad2.dpad_up) {
                this.slideRotate.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.slideRotate.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.slideRotate.left.setPower(-0.6);
                this.slideRotate.right.setPower(-0.6);
            } else if (this.gamepad2.dpad_down) {
                this.slideRotate.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.slideRotate.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.slideRotate.left.setPower(0.4);
                this.slideRotate.right.setPower(0.4);
            } else {
                final double power = this.gamepad2.left_stick_x * -0.8;
                this.slideRotate.left.setPower(power);
                this.slideRotate.right.setPower(power);
            }
            if (this.gamepad2.left_stick_y != 0) {
                double power = -this.gamepad2.left_stick_y * 0.2;
                this.slideLift.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.slideLift.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.slideLift.left.setPower(power);
                this.slideLift.right.setPower(power);
            }
        } else {
            if (this.gamepad2.dpad_up) {
                this.slideRotate.move(-15);
            } else if (this.gamepad2.dpad_down) {
                this.slideRotate.move(15);
            } else if (this.gamepad2.dpad_left) {
                this.slideRotate.setPosition(0);
            } else {
                this.slideRotate.move((int) (this.gamepad2.left_stick_x * 20));
            }
            this.slideLift.move((int) (-this.gamepad2.left_stick_y * 50));
            this.slideRotate.update();
            this.slideLift.update();
        }

        this.telemetry.addData("LeftRotation", "%d; %d", this.slideRotate.getLeftPosition(), this.slideRotate.resetting);
        this.telemetry.addData("RightRotation", "%d", this.slideRotate.getRightPosition());
        this.telemetry.addData("LeftPosition", "%d; %d", this.slideLift.getLeftPosition(), this.slideLift.resetting);
        this.telemetry.addData("RightPosition", "%d", this.slideLift.getRightPosition());
        this.telemetry.update();
    }
}

package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2021.ResourceClasses.Button;

public abstract class MasterTeleOp extends MasterOpMode {
    int position = 0;

    public void driveRobot() {
        motorFrontLeft.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
        motorBackLeft.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
        motorFrontRight.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        motorBackRight.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
    }

    public void driveSlow() {
        if (driver1.getLeftTriggerValue() > 0.5) {
            motorFrontLeft.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) / 4);
            motorBackLeft.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) / 4);
            motorFrontRight.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) / 4);
            motorBackRight.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) / 4);
        }
    }

    public void driveLeftCarousel() {
        if (driver1.isButtonPressed(Button.LEFT_BUMPER)) {
            motorLeftDuck.setPower(-0.75);
            motorRightDuck.setPower(-0.75);
        }
    }

    public void driveRightCarousel() {
        if (driver1.isButtonPressed(Button.RIGHT_BUMPER)) {
            motorLeftDuck.setPower(0.75);
            motorRightDuck.setPower(0.75);
        }
    }

    public void driveGrabber() {
        if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() < 0.15) {
            servoGrabber.setPosition(0.45);
        } else if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() > 0.15) {
            servoGrabber.setPosition(0.0);
        }
    }

    public void driveArm() {
        int x;

        if (driver2.isButtonJustPressed(Button.DPAD_UP) && position == 0) {
            // todo - motor at level 1 alliance hub
            motorArm.setTargetPosition(300);

            while (motorArm.getCurrentPosition() < 300) {
                x = 300 - motorArm.getCurrentPosition();
                motorArm.setPower(-0.00004 * Math.pow((x - 150), 2) + 0.9);
            }

            //todo - servo parallel to ground
            servoArm.setPosition(0.4);

            position++;
        }

        if (driver2.isButtonJustPressed(Button.DPAD_UP) && position == 1) {
            // todo - motor at level 2 alliance hub
            motorArm.setTargetPosition(600);

            while (motorArm.getCurrentPosition() < 600) {
                x = 600 - motorArm.getCurrentPosition();
                motorArm.setPower(-0.00004 * Math.pow((x - 150), 2) + 0.9);
            }

            //todo - servo parallel to ground
            servoArm.setPosition(0.3);

            position++;
        }
    }

    public void driveArmManual() {
        if (Math.abs(driver2.getRightStickY()) > 0.1) {
            motorArm.setPower(driver2.getRightStickY() / 5);
            // todo - servo parallel to ground
        }
    }

    public void driveBelt() {
        if (Math.abs(driver2.getLeftStickY()) > 0.1) {
            motorBelt.setPower(driver2.getLeftStickY());
        }
    }

    // todo - make sure this resets properly
    public void resetArmAndServo() {
        if (driver2.getLeftTriggerValue() > 0.5 && driver2.getRightTriggerValue() > 0.5) {
            motorArm.setPower(0.75);
            servoArm.setPosition(0.0);
            servoGrabber.setPosition(0.81);
            motorArm.setTargetPosition(-10);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
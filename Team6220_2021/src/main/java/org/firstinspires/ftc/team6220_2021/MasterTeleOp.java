package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2021.ResourceClasses.Button;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

public abstract class MasterTeleOp extends MasterOpMode {
    public void driveRobot() {
        if (driver1.getLeftTriggerValue() > 0.25) {
            motorFL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * 0.25);
            motorBL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * 0.25);
            motorFR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * 0.25);
            motorBR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * 0.25);
        } else {
            motorFL.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            motorBL.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            motorFR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            motorBR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        }
    }

    public void driveLeftCarousel() {
        if (driver2.isButtonJustPressed(Button.LEFT_BUMPER)) {
            stopMotors();
            motorLeftDuck.setPower(-0.75);
            motorRightDuck.setPower(-0.75);
            pauseMillis(1500);
        } else {
            motorLeftDuck.setPower(0.0);
            motorRightDuck.setPower(0.0);
        }
    }

    public void driveRightCarousel() {
        if (driver2.isButtonJustPressed(Button.RIGHT_BUMPER)) {
            stopMotors();
            motorLeftDuck.setPower(0.75);
            motorRightDuck.setPower(0.75);
            pauseMillis(1500);
        } else {
            motorLeftDuck.setPower(0.0);
            motorRightDuck.setPower(0.0);
        }
    }

    public void driveGrabber() {
        if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() < 0.4) {
            servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);
        } else if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() > 0.4) {
            servoGrabber.setPosition(Constants.CLOSED_GRABBER_POSITION);
        }
    }

    public void driveArm() {
        if (driver2.isButtonJustPressed(Button.DPAD_LEFT)) {
            stopMotors();
            motorArm.setTargetPosition(Constants.ARM_SHARED_HUB_LEVEL);
            pauseMillis(500);
            servoArm.setPosition(Constants.SERVO_ARM_SHARED_HUB_POSITION);

        } else if (driver2.isButtonJustPressed(Button.DPAD_UP)) {
            stopMotors();
            motorArm.setTargetPosition(Constants.ARM_ALLIANCE_HUB_3RD_LEVEL);
            pauseMillis(500);
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 2400.0);

        } else if (driver2.isButtonJustPressed(Button.DPAD_RIGHT)) {
            stopMotors();
            motorArm.setTargetPosition(400);
            pauseMillis(500);
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - (motorArm.getCurrentPosition() / 2400.0) + 0.3);

        } else if (driver2.isButtonJustPressed(Button.DPAD_DOWN)) {
            stopMotors();
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
            pauseMillis(500);
            motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);

        } else if (driver2.isButtonJustPressed(Button.A)) {
            stopMotors();
            motorArm.setTargetPosition(Constants.ARM_CAPPING_LEVEL);
            pauseMillis(500);
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 2400.0);
        }
    }

    public void driveArmManual() {
        if (Math.abs(driver2.getLeftStickY()) > 0.1) {
            motorArm.setTargetPosition(motorArm.getCurrentPosition() + (int) (driver2.getLeftStickY() * -25));
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 2400.0);
        }

        if (Math.abs(driver2.getRightStickY()) > 0.1) {
            motorBelt.setTargetPosition(motorBelt.getCurrentPosition() + (int) (driver2.getRightStickY() * 50));
        }
    }

    public void reset() {
        servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
        servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);

        motorArm.setPower(0.5);
        motorArm.setTargetPosition(-25);
        pauseMillis(500);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setPower(0.5);
        motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 2400.0);
    }

    public void stopMotors() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
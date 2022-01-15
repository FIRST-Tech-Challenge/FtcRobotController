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

    // todo - fix to be manual
    public void driveLeftCarousel() {
        if (driver2.isButtonJustPressed(Button.LEFT_BUMPER)) {
            motorLeftDuck.setPower(-0.75);
            motorRightDuck.setPower(-0.75);
            pauseMillis(1500);
        } else {
            motorLeftDuck.setPower(0.0);
            motorRightDuck.setPower(0.0);
        }
    }

    // todo - fix to be manual
    public void driveRightCarousel() {
        if (driver2.isButtonJustPressed(Button.RIGHT_BUMPER)) {
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

    // todo - test positions
    public void driveArm() {
        if (driver2.isButtonJustPressed(Button.DPAD_LEFT)) {
            motorArm.setTargetPosition(Constants.ARM_SHARED_HUB_LEVEL);
            motorBelt.setTargetPosition(Constants.BELT_RESET);
            pauseMillis(750);
            servoArm.setPosition(Constants.SERVO_ARM_SHARED_HUB_POSITION);

        } else if (driver2.isButtonJustPressed(Button.DPAD_UP)) {
            motorArm.setTargetPosition(Constants.ARM_ALLIANCE_HUB_3RD_LEVEL);
            motorBelt.setTargetPosition(Constants.BELT_RESET);
            pauseMillis(750);
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 3000.0);

        } else if (driver2.isButtonJustPressed(Button.DPAD_RIGHT)) {
            motorBelt.setTargetPosition(-500);
            motorArm.setTargetPosition(400);
            pauseMillis(750);
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - (motorArm.getCurrentPosition() / 3000.0) + 0.3);

        } else if (driver2.isButtonJustPressed(Button.DPAD_DOWN)) {
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
            motorBelt.setTargetPosition(Constants.BELT_RESET);
            pauseMillis(750);
            motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);

        } else if (driver2.isButtonJustPressed(Button.A)) {
            motorArm.setTargetPosition(Constants.ARM_CAPPING_LEVEL);
            motorBelt.setTargetPosition(Constants.BELT_RESET);
            pauseMillis(750);
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 3000.0);
        }
    }

    // todo - implement vertical and horizontal position using math
    // todo - smooth and/or concurrent and/or small steps
    public void driveArmManual() {
        if (Math.abs(driver2.getLeftStickY()) > 0.1) {
            motorArm.setTargetPosition(motorArm.getCurrentPosition() + (int) (driver2.getLeftStickY() * 25));
            pauseMillis(100);
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 3000.0);
        }

        if (Math.abs(driver2.getRightStickY()) > 0.1) {
            motorBelt.setTargetPosition(motorBelt.getCurrentPosition() + (int) (driver2.getRightStickY() * 50));
        }
    }

    public void reset() {
        servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
        servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);

        motorBelt.setPower(0.5);
        motorBelt.setTargetPosition(Constants.BELT_RESET);

        motorArm.setPower(0.5);
        motorArm.setTargetPosition(-50);
        pauseMillis(500);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setPower(0.5);
        motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 3000.0);
    }
}
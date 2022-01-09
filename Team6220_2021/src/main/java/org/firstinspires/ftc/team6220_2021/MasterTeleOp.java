package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2021.ResourceClasses.Button;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

public abstract class MasterTeleOp extends MasterOpMode {
    int position = 0;

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
        if (driver1.isButtonJustPressed(Button.LEFT_BUMPER)) {
            motorLeftDuck.setPower(-0.75);
            motorRightDuck.setPower(-0.75);
            pauseMillis(1000);
        } else {
            motorLeftDuck.setPower(0.0);
            motorRightDuck.setPower(0.0);
        }
    }

    // todo - fix to be manual
    public void driveRightCarousel() {
        if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER)) {
            motorLeftDuck.setPower(0.75);
            motorRightDuck.setPower(0.75);
            pauseMillis(1000);
        } else {
            motorLeftDuck.setPower(0.0);
            motorRightDuck.setPower(0.0);
        }
    }

    // todo - move arm backwards only when grabber closes
    // todo - and move arm forwards only when grabber opens
    // todo - only if necessary
    public void driveGrabber() {
        if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() < 0.4) {
            servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);
        } else if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() > 0.4) {
            servoGrabber.setPosition(Constants.CLOSED_GRABBER_POSITION);
        }
    }

    public void driveArm() {
        if (driver2.isButtonJustPressed(Button.DPAD_LEFT) && position == 0) {
            motorArm.setTargetPosition(Constants.ARM_SHARED_HUB_LEVEL);
            motorBelt.setTargetPosition(Constants.BELT_RESET);

            pauseMillis(750);

            servoArm.setPosition(Constants.SERVO_ARM_SHARED_HUB_POSITION);
            position = 1;

        } else if (driver2.isButtonJustPressed(Button.DPAD_RIGHT) && position == 0) {
            motorArm.setTargetPosition(Constants.ARM_ALLIANCE_HUB_3RD_LEVEL);
            motorBelt.setTargetPosition(Constants.BELT_RESET);

            pauseMillis(750);

            servoArm.setPosition(0.45 - 3 * motorArm.getCurrentPosition() / 8000.0);
            position = 2;

        } else if (driver2.isButtonJustPressed(Button.A) && position == 0) {
            motorArm.setTargetPosition(Constants.ARM_CAPPING_LEVEL);
            motorBelt.setTargetPosition(Constants.BELT_RESET);

            pauseMillis(750);

            servoArm.setPosition(0.45 - 3 * motorArm.getCurrentPosition() / 8000.0);
            position = 3;

        } else if (driver2.isButtonJustPressed(Button.DPAD_UP) && position == 0) {
            motorArm.setTargetPosition(Constants.ARM_BACKWARDS_ALLIANCE_HUB_3RD_LEVEL);
            motorBelt.setTargetPosition(Constants.BELT_RESET);

            pauseMillis(1500);

            servoArm.setPosition(0.6 - 3 * (motorArm.getCurrentPosition() - 1200) / 8000.0);
            position = 4;

        } else if (driver2.isButtonJustPressed(Button.DPAD_DOWN)) {
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
            motorBelt.setTargetPosition(Constants.BELT_RESET);
            motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);

            position = 0;
        }
    }

    public void driveArmManual() {
        if (Math.abs(driver2.getLeftStickY()) > 0.1) {
            motorArm.setTargetPosition(motorArm.getCurrentPosition() + (int) driver2.getLeftStickY());
            motorBelt.setTargetPosition(motorBelt.getCurrentPosition() - (int) driver2.getLeftStickY() * 4);
        }

        if (Math.abs(driver2.getRightStickY()) > 0.1) {
            motorArm.setTargetPosition(motorArm.getCurrentPosition() + (int) driver2.getLeftStickY());
            motorBelt.setTargetPosition(motorBelt.getCurrentPosition() - (int) driver2.getLeftStickY() * 4);
        }

        // 2k belt 100 arm at 45 deg
        // just belt at 0 deg
    }

    public void reset() {
        if (driver2.getLeftTriggerValue() > 0.5 && driver2.getRightTriggerValue() > 0.5) {
            servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
            servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);
            motorBelt.setTargetPosition(Constants.BELT_RESET);

            motorArm.setTargetPosition(-50);
            pauseMillis(500);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setPower(0.5);
            motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            servoArm.setPosition(0.45 - 3 * motorArm.getCurrentPosition() / 8000.0);
        }
    }
}
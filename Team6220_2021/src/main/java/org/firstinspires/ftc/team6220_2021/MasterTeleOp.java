package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2021.ResourceClasses.Button;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

public abstract class MasterTeleOp extends MasterOpMode {
    int position = 0;
    int x;

    public void driveRobot() {
        motorFL.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
        motorBL.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
        motorFR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        motorBR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
    }

    public void driveSlow() {
        if (driver1.getLeftTriggerValue() > 0.5) {
            motorFL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) / 4);
            motorBL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) / 4);
            motorFR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) / 4);
            motorBR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) / 4);
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

    // todo - make arm move backwards only when grabber closes
    // todo - and forwards when grabber opens
    public void driveGrabber() {
        if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() < 0.15) {
            servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);
        } else if (driver2.isButtonJustPressed(Button.X) && servoGrabber.getPosition() > 0.15) {
            servoGrabber.setPosition(Constants.CLOSED_GRABBER_POSITION);
        }
    }

    // todo - fix servo and arm positions in constants class
    // todo - calculate servo ratio 1/x relationship scientifically
    public void driveArm() {
        if (driver2.isButtonJustPressed(Button.DPAD_LEFT) && position == 0) {
            // todo - motor at shared hub
            motorArm.setTargetPosition(Constants.ARM_SHARED_HUB_LEVEL);

            for (int i = 1; i <= 10; i++) {
                double startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 40) {
                    motorArm.setPower(0.1 * i);
                }

                if (Math.abs(motorArm.getCurrentPosition() - Constants.ARM_SHARED_HUB_LEVEL) < 100) {
                    break;
                }
            }

            //todo - servo shared hub
            servoArm.setPosition(Constants.SERVO_ARM_SHARED_HUB_POSITION);
            position = 1;

        } else if (driver2.isButtonJustPressed(Button.DPAD_RIGHT) && position == 0) {
            // todo - motor at 3rd level
            motorArm.setTargetPosition(Constants.ARM_ALLIANCE_HUB_3RD_LEVEL);

            for (int i = 1; i <= 10; i++) {
                double startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 40) {
                    motorArm.setPower(0.1 * i);
                }

                if (Math.abs(motorArm.getCurrentPosition() - Constants.ARM_ALLIANCE_HUB_3RD_LEVEL) < 100) {
                    break;
                }
            }

            //todo - servo ratio
            servoArm.setPosition(0.0);
            position = 2;

        } else if (driver2.isButtonJustPressed(Button.A) && position == 0) {
            // todo - motor at capping
            motorArm.setTargetPosition(Constants.ARM_CAPPING_LEVEL);

            for (int i = 1; i <= 10; i++) {
                double startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 40) {
                    motorArm.setPower(0.1 * i);
                }

                if (Math.abs(motorArm.getCurrentPosition() - Constants.ARM_CAPPING_LEVEL) < 100) {
                    break;
                }
            }

            //todo - servo ratio
            servoArm.setPosition(0.0);
            position = 3;

        } else if (driver2.isButtonJustPressed(Button.B) && position == 3) {
            // todo - motor at capping backwards
            motorArm.setTargetPosition(Constants.ARM_BACKWARDS_CAPPING_LEVEL);

            for (int i = 1; i <= 5; i++) {
                double startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 40) {
                    motorArm.setPower(0.1 * i);
                }

                if (Math.abs(motorArm.getCurrentPosition() - Constants.ARM_BACKWARDS_CAPPING_LEVEL) < 100) {
                    break;
                }
            }

            //todo - servo ratio backwards
            servoArm.setPosition(0.0);
            position = 4;

        } else if (driver2.isButtonJustPressed(Button.DPAD_UP) && position == 2) {
            // todo - motor at 3rd level backwards
            motorArm.setTargetPosition(Constants.ARM_BACKWARDS_ALLIANCE_HUB_3RD_LEVEL);

            for (int i = 1; i <= 5; i++) {
                double startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 40) {
                    motorArm.setPower(0.1 * i);
                }

                if (Math.abs(motorArm.getCurrentPosition() - Constants.ARM_BACKWARDS_ALLIANCE_HUB_3RD_LEVEL) < 100) {
                    break;
                }
            }

            //todo - servo ratio backwards
            servoArm.setPosition(0.0);
            position = 5;

        } else if (driver2.isButtonJustPressed(Button.DPAD_DOWN)) {
            // todo - motor at collecting
            motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);
            motorArm.setPower(1.0);

            //todo - servo ratio
            servoArm.setPosition(0.0);
            position = 0;
        }
    }

    // todo - change
    public void driveArmManual() {
        if (Math.abs(driver2.getRightStickY()) > 0.1) {
            motorArm.setPower(driver2.getRightStickY() / 5);
            // todo - servo parallel to ground
        } else {
            motorArm.setPower(0.0);
        }
    }

    public void driveBelt() {
        if (Math.abs(driver2.getLeftStickY()) > 0.1) {
            motorBelt.setPower(driver2.getLeftStickY());
        } else {
            motorBelt.setPower(0.0);
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
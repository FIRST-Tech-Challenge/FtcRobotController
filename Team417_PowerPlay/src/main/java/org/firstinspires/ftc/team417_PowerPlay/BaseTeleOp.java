package org.firstinspires.ftc.team417_PowerPlay;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

abstract public class BaseTeleOp extends BaseOpMode {
    boolean grabberClosed = false;
    double armPower = 0.0;
    int armLevel = 0;
    int armEncoderPosition = 0;

    boolean armManual = false;

    ElapsedTime time;

    private static double DRIVING_SPEED = 0.7;

    public void initializeTeleOp() {
        initializeHardware();
        time = new ElapsedTime();
        time.reset();
        grabberServo.setPosition(GRABBER_OPEN);
    }

    public void driveUsingControllers() {
        double x = gamepad1.left_stick_x * DRIVING_SPEED;
        double y = -gamepad1.left_stick_y * DRIVING_SPEED;
        double turning = gamepad1.right_stick_x * DRIVING_SPEED;
        y *= 1 - (0.5 * gamepad1.right_trigger);
        x *= 1 - (0.5 * gamepad1.right_trigger);
        turning *= 1 - (0.5 * gamepad1.right_trigger);
        mecanumDrive(x, y, turning);
    }

    public void driveArm() {
        armPower = (gamepad2.left_stick_y * 0.5);
        if (motorArm.getCurrentPosition() > MIN_ARM_POSITION && armPower > 0) {
            armPower = 0;
        } else if (motorArm.getCurrentPosition() < MAX_ARM_POSITION && armPower < 0) {
            armPower = 0;
        }
        motorArm.setPower(armPower);
    }

    public void thingy() {
        if (gamepad2.dpad_down) {
            armEncoderPosition -=5;
        } else if (gamepad2.dpad_up) {
            armEncoderPosition +=5;
        }
        Range.clip(armEncoderPosition, MIN_ARM_POSITION, MAX_ARM_POSITION);
        motorArm.setTargetPosition(armEncoderPosition);
        motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 100.0);
    }
    public void buttonDriveArm() {
        if (gamepad2.right_trigger != 0) {
            if (gamepad2.dpad_down) {
                armEncoderPosition -=5;
            } else if (gamepad2.dpad_up) {
                armEncoderPosition +=5;
            }
            Range.clip(armEncoderPosition, MIN_ARM_POSITION, MAX_ARM_POSITION);
            motorArm.setTargetPosition(armEncoderPosition);

            if (armEncoderPosition >= motorArm.getCurrentPosition()) {
                motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 100.0);

            } else {
                // if need to lower
                if (motorArm.getCurrentPosition() - armEncoderPosition > 10) {
                    motorArm.setPower(-0.2);

                } else {
                    motorArm.setPower(0);
                }
            }
        } else {
            if (time.time() > 0.3) {
                if (gamepad2.left_bumper) {
                    armLevel--;
                    time.reset();
                } else if (gamepad2.right_bumper) {
                    armLevel++;
                    time.reset();
                }
            }

            if (armLevel > 4) {
                armLevel = 4;
            } else if (armLevel < 0) {
                armLevel = 0;
            }
            if (armLevel == 0) { // ground position
                armEncoderPosition = MIN_ARM_POSITION;

            } else if (armLevel == 1) { // ground junction
                armEncoderPosition = GRD_JUNCT_ARM_POSITION;
            } else if (armLevel == 2) { // low junction
                armEncoderPosition = LOW_JUNCT_ARM_POSITION;
            } else if (armLevel == 3) { // middle junction
                armEncoderPosition = MID_JUNCT_ARM_POSITION;
            } else if (armLevel == 4) {
                armEncoderPosition = HIGH_JUNCT_ARM_POSITION;
            }
            motorArm.setTargetPosition(armEncoderPosition);
            // if need to raise arm
            if (armEncoderPosition >= motorArm.getCurrentPosition()) {
                motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 800.0);

            } else {
                // if need to lower
                if (motorArm.getCurrentPosition() - armEncoderPosition > 10) {
                    motorArm.setPower(-0.4);

                } else {
                    motorArm.setPower(0);
                }
                //motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 1000.0);
            }
        }


    }

    public void newButtonDrive() {
        if (gamepad2.dpad_down) {
            armEncoderPosition -= 5;
            armManual = true;
        } else if (gamepad2.dpad_up) {
            armEncoderPosition += 5;
            armManual = true;
        }
        Range.clip(armEncoderPosition, MIN_ARM_POSITION, MAX_ARM_POSITION);
        motorArm.setTargetPosition(armEncoderPosition);
        if (armManual) {
            if (armEncoderPosition >= motorArm.getCurrentPosition()) {
                motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 100.0);

            } else {
                // if need to lower
                if (motorArm.getCurrentPosition() - armEncoderPosition > 10) {
                    motorArm.setPower(-0.2);

                } else {
                    motorArm.setPower(0);
                }
            }
        }
        boolean leftPress = gamepad2.left_bumper;
        boolean rightPress = gamepad2.right_bumper;
        if (leftPress || rightPress) {
            armManual = false;
        }
        if (time.time() > 0.3) {
            if (leftPress) {
                armLevel--;
                time.reset();
            } else if (rightPress) {
                armLevel++;
                time.reset();
            }
        }

        if (armLevel > 4) {
            armLevel = 4;
        } else if (armLevel < 0) {
            armLevel = 0;
        }
        if (!armManual) {
            if (armLevel == 0) { // ground position
                armEncoderPosition = MIN_ARM_POSITION;

            } else if (armLevel == 1) { // ground junction
                armEncoderPosition = GRD_JUNCT_ARM_POSITION;
            } else if (armLevel == 2) { // low junction
                armEncoderPosition = LOW_JUNCT_ARM_POSITION;
            } else if (armLevel == 3) { // middle junction
                armEncoderPosition = MID_JUNCT_ARM_POSITION;
            } else if (armLevel == 4) {
                armEncoderPosition = HIGH_JUNCT_ARM_POSITION;
            }
            if (armEncoderPosition >= motorArm.getCurrentPosition()) {
                motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 800.0);

            } else {
                // if need to lower
                if (motorArm.getCurrentPosition() - armEncoderPosition > 10) {
                    motorArm.setPower(-0.4);

                } else {
                    motorArm.setPower(0);
                }
                //motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 1000.0);
            }
        }
    }

    public void driveGrabber() {
        grabberClosed = grabberToggle.toggle(gamepad2.a);

        if (grabberClosed) {
            grabberServo.setPosition(GRABBER_CLOSED);
        } else {
            grabberServo.setPosition(GRABBER_OPEN);
        }
    }

    public void doTelemetry() {
        telemetry.addData("Arm power", armPower);
        telemetry.addData("Arm target", armEncoderPosition);
        telemetry.addData("Arm current position", motorArm.getCurrentPosition());
        telemetry.addData("Grabber position", grabberServo.getPosition());
        telemetry.addData("Grabber open", grabberClosed);
        telemetry.addData("Arm level", armLevel);
        telemetry.update();
    }
}

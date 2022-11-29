package org.firstinspires.ftc.team417_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode {
    boolean grabberOpen = false;
    double armPower = 0.0;
    int armLevel = 0;
    int armEncoderPosition = 0;
    Toggler headingToggle = new Toggler();
    boolean maintainingHeading = true;
    double errorHeading;

    private static double DRIVING_SPEED = 0.5;

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
    public void buttonDriveArm() {
        if (gamepad2.right_trigger != 0) {
            driveArm();
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

            if (armLevel > 3) {
                armLevel = 3;
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
            }
            motorArm.setTargetPosition(armEncoderPosition);
            motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) / 1000.0);
        }

    }

    public void driveGrabber() {
        grabberOpen = grabberToggle.toggle(gamepad2.a);

        if (grabberOpen) {
            grabberServo.setPosition(GRABBER_OPEN);
        } else {
            grabberServo.setPosition(GRABBER_CLOSED);
        }
    }

    public void doTelemetry() {
        telemetry.addData("Arm power", armPower);
        telemetry.addData("Arm current position", motorArm.getCurrentPosition());
        telemetry.addData("Grabber position", grabberServo.getPosition());
        telemetry.addData("Grabber open", grabberOpen);
        telemetry.addData("Arm level", armLevel);
        telemetry.update();
    }
}

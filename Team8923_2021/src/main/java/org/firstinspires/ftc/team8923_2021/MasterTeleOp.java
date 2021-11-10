package org.firstinspires.ftc.team8923_2021;

abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;

    private Toggle driveSpeedToggle = new Toggle();

    double driveSpeed = 1.0;

    // one joystick moves forwards and backwards, the other controls left and right.
    public void splitArcadeDrive() {
        double forwardPower = gamepad1.left_stick_x;
        double turningPower = gamepad1.right_stick_y;

        double leftPower = turningPower - forwardPower;
        double rightPower = turningPower + forwardPower;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void runDriveSpeed() {
        isSlowMode = driveSpeedToggle.toggle(gamepad1.left_bumper);
        if (isSlowMode) {
            driveSpeed = 0.25;
        } else {
            driveSpeed = 1.0;
        }
    }

    public void runIntake() {
        if (gamepad2.b) {
            motorIntake.setPower(0.8);
        } else if (gamepad2.x) {
            motorIntake.setPower(-0.8);
        } else {
            motorIntake.setPower(0.0);
        }
    }

    public void runCarousel() {
        if (gamepad2.left_trigger > Constants.MINIMUM_TRIGGER_VALUE) {
            motorCarousel.setPower(-0.8);
        } else {
            motorCarousel.setPower(0.0);
        }
    }
}


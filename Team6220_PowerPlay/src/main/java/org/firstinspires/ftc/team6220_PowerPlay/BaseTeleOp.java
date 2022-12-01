package org.firstinspires.ftc.team6220_PowerPlay;

public abstract class BaseTeleOp extends BaseOpMode {
    double xPower;
    double yPower;
    double tPower;

    int targetPosition;

    public void driveChassisWithController() {
        xPower = gamepad1.left_stick_x * (1 - gamepad1.left_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;
        yPower = gamepad1.left_stick_y * (1 - gamepad1.left_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;
        tPower = gamepad1.right_stick_x * (1 - gamepad1.left_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;

        // atan2 determines angle between the two sticks
        // case for driving the robot left and right
        if (Math.abs(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)) > Constants.DRIVE_DEADZONE_DEGREES) {
            driveWithIMU(xPower, 0.0, tPower);

        // case for driving the robot forwards and backwards
        } else if (Math.abs(Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y)) > Constants.DRIVE_DEADZONE_DEGREES) {
            driveWithIMU(0.0, yPower, tPower);

        // case for if the deadzone limits are passed, the robot drives normally
        } else {
            driveWithIMU(xPower, yPower, tPower);
        }
    }

    public void driveGrabberWithController() {
        if (gamepad2.x) {  // press x to close
            driveGrabber(false);
        } else if (gamepad2.a) {  // press a to open
            driveGrabber(true);
        }
    }

    public void driveSlidesWithController() {
        motorLeftSlides.setPower(-gamepad2.left_stick_y);
        motorRightSlides.setPower(-gamepad2.left_stick_y);

        if (motorLeftSlides.getCurrentPosition() < 0) {
            motorLeftSlides.setPower(0.5);
            motorRightSlides.setPower(0.5);
        } else if (motorLeftSlides.getCurrentPosition() > 9000) {
            motorLeftSlides.setPower(-0.5);
            motorRightSlides.setPower(-0.5);
        }
    }

    public void driveTurntableWithController() {
        driveTurntable(1.0, 0);
    }
}

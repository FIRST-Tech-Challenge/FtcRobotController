package org.firstinspires.ftc.team6220_PowerPlay;

public abstract class BaseTeleOp extends BaseOpMode {
    double xPower;
    double yPower;
    double tPower;

    public void driveChassisWithController() {
        xPower = gamepad1.left_stick_x * (1 - gamepad1.right_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;
        yPower = gamepad1.left_stick_y * (1 - gamepad1.right_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;
        tPower = gamepad1.right_stick_x * (1 - gamepad1.right_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;

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
        if (gamepad2.dpad_up) {
            driveSlides(Constants.VERTICAL_SLIDE_HIGH_JUNCTION);
        } else if (gamepad2.dpad_right) {
            driveSlides(Constants.VERTICAL_SLIDE_MEDIUM_JUNCTION);
        } else if (gamepad2.dpad_left) {
            driveSlides(Constants.VERTICAL_SLIDE_LOW_JUNCTION);
        } else if (gamepad2.dpad_down) {
            driveSlides(Constants.VERTICAL_SLIDE_BOTTOM);
        } else if (Math.abs(gamepad2.left_stick_y) > Constants.VERTICAL_SLIDE_DEADZONE) {
            if (motorLVSlides.getTargetPosition() < Constants.VERTICAL_SLIDE_BOTTOM) {
                driveSlides(Constants.VERTICAL_SLIDE_BOTTOM);
            } else if (motorLVSlides.getTargetPosition() > Constants.VERTICAL_SLIDE_TOP) {
                driveSlides(Constants.VERTICAL_SLIDE_TOP);
            } else {
                driveSlides(motorLVSlides.getTargetPosition() + (int) (-gamepad2.left_stick_y * 240));
            }
        }
    }

    public void driveTurntableWithController() {
        driveTurntable(1.0, 0);
    }
}

package org.firstinspires.ftc.team6220_PowerPlay;

public abstract class BaseTeleOp extends BaseOpMode {
    double xPower;
    double yPower;
    double tPower;

    int yPosition;

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

        // dpad quick turns: rotate to an interval of 90 degrees
        if (gamepad1.dpad_up) {
            IMUOriginalAngle = 0.0;
        } else if (gamepad1.dpad_right) {
            IMUOriginalAngle = -90.0;
        } else if (gamepad1.dpad_down) {
            IMUOriginalAngle = 180.0;
        } else if (gamepad1.dpad_left) {
            IMUOriginalAngle = 90.0;
        }

        // bumper button quick turns: rotate by +/-90 degrees
        if (gamepad1.left_bumper) {
            IMUOriginalAngle = loopAddAngle(IMUOriginalAngle, 90.0);
        } else if (gamepad1.right_bumper) {
            IMUOriginalAngle = loopAddAngle(IMUOriginalAngle, -90.0);
        }
    }

    private static double loopAddAngle(double original, double addition) {
        // add to the original imu angle, but loop around if it goes past +/-180
        double newAngle = original + addition;
        if (newAngle > 180.0) {
            newAngle -= 360.0;
        } else if (newAngle < -180.0) {
            newAngle += 360.0;
        }
        return newAngle;
    }

    public void driveGrabberWithController() {

        if (gamepad2.x) {  // press x to close
            driveGrabber(false);
        } else if (gamepad2.a) {  // press a to open
            driveGrabber(true);
        }
    }

    public void driveSlidesWithController() {
        yPosition = (motorLVSlides.getCurrentPosition() + motorRVSlides.getCurrentPosition()) / 2;

        if (motorLVSlides.getTargetPosition() <= 0 || motorRVSlides.getTargetPosition() <= 0) {
            driveSlides(yPosition, 0.0); 
        } else if (Math.abs(gamepad2.left_stick_y) > Constants.VERTICAL_SLIDE_DEADZONE) {
            driveSlides(yPosition + (int) (10 - gamepad2.right_trigger * 5), 1.0);
        } else {
            driveSlides(motorLVSlides.getCurrentPosition(), 0.0);
        }
    }

    public void driveTurntableWithController() {
//        if (Math.abs(D2rightStickX) > Constants.TURNTABLE_DEADZONE) {
//            driveTurntable(D2rightStickX * D2triggerSpeedMod, 0);
//        } else if (Math.abs(D2rightStickX) < Constants.TURNTABLE_DEADZONE) {
//            driveTurntable(0, motorTurntable.getCurrentPosition());
//        }

        driveTurntable(1, Constants.TURNTABLE_DEFAULT_POSITION);
    }
}

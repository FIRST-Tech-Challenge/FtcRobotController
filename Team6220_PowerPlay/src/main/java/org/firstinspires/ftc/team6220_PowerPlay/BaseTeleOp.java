package org.firstinspires.ftc.team6220_PowerPlay;

public abstract class BaseTeleOp extends BaseOpMode {
    double xPower;
    double yPower;
    double tPower;

    int slideTargetPosition = 0;
    int junction = 0;
    int[] junctions = {0, 0};

    public void driveChassisWithController() {
        xPower = stickCurve(gamepad1.left_stick_x, Constants.DRIVE_MOVE_CURVE_FAC, Constants.DRIVE_STICK_DEADZONE)* (1 - gamepad1.left_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;
        yPower = stickCurve(gamepad1.left_stick_y, Constants.DRIVE_MOVE_CURVE_FAC, Constants.DRIVE_STICK_DEADZONE)* (1 - gamepad1.left_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;
        tPower = stickCurve(gamepad1.right_stick_x, Constants.DRIVE_TURN_CURVE_FAC, Constants.DRIVE_STICK_DEADZONE) * (1 - gamepad1.left_trigger * 0.5) * Constants.DRIVE_SPEED_MULTIPLIER;

        // case for driving the robot left and right
        if (Math.abs(Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x))) < Constants.DRIVE_DEADZONE_DEGREES) {
            driveWithIMU(xPower, 0.0, tPower);

            // case for driving the robot forwards and backwards
        } else if (Math.abs(Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x))) > Constants.DRIVE_DEADZONE_DEGREES) {
            driveWithIMU(0.0, yPower, tPower);

            // case for if the deadzone limits are passed, the robot drives normally
        } else {
            driveWithIMU(xPower, yPower, tPower);
        }
    }

    public void driveGrabberWithController() {
        if (gamepad2.x) {
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
        } else if (gamepad2.a) {
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
        } else if (gamepad2.b) {
            driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        }
    }

    public void driveSlidesWithController() {
        slideTargetPosition += (int) (-gamepad2.left_stick_y * 25);

        if (gamepad2.right_bumper && junctions[0] == junctions[1]) {
            junction++;

            if (junction >= 3) {
                junction = 3;
            }

            switch (junction) {
                case 1:
                    slideTargetPosition = Constants.SLIDE_LOW;
                    break;
                case 2:
                    slideTargetPosition = Constants.SLIDE_MEDIUM;
                    break;
                case 3:
                    slideTargetPosition = Constants.SLIDE_HIGH;
                    break;
            }
        } else if (gamepad2.left_bumper && junctions[0] == junctions[1]) {
            junction--;

            if (junction <= 0) {
                junction = 0;
            }

            switch (junction) {
                case 0:
                    slideTargetPosition = Constants.SLIDE_BOTTOM;
                    break;
                case 1:
                    slideTargetPosition = Constants.SLIDE_LOW;
                    break;
                case 2:
                    slideTargetPosition = Constants.SLIDE_MEDIUM;
                    break;
            }
        }

        // don't let target position go below slide bottom position
        if (slideTargetPosition <= Constants.SLIDE_BOTTOM) {
            slideTargetPosition = Constants.SLIDE_BOTTOM;
            // don't let target position go above slide top position
        } else if (slideTargetPosition >= Constants.SLIDE_TOP) {
            slideTargetPosition = Constants.SLIDE_TOP;
        }

        driveSlides(slideTargetPosition);

        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            junctions[0] = junctions[1];
        }

        junctions[1] = junction;
    }

    public void driveTurntableWithController() {
    }
    
    public double stickCurve(double x, double a, double d) {
        x = Math.signum(x)*Math.max(0, Math.min(1, (Math.abs(x)-d)/(1-d))); // Remap x with deadzone
        return a*x+(1-a)*x*x*x;
    }
}

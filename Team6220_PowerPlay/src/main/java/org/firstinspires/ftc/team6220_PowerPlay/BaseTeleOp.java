package org.firstinspires.ftc.team6220_PowerPlay;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class BaseTeleOp extends BaseOpMode {
    double currentAngle;
    double headingDegrees;
    double negativeHeadingRadians;

    double x;
    double y;
    double t;

    double xRotatedVector;
    double yRotatedVector;
    double ratio;

    double xPower;
    double yPower;
    double tPower;

    int slideTargetPosition = 0;

    // variables to keep track of cone stack height
    int previousConeStack = 0;
    int currentConeStack = 0;

    // variables to keep track of junction height
    int previousJunction = 0;
    int currentJunction = 0;

    /**
     * allows the driver to drive the robot field-centric
     * this is more intuitive for the driver and makes it easier to maneuver around the field
     */
    public void driveFieldCentric() {
        // gets the opposite of the heading of the robot
        // the opposite is needed because the imu returns physics coordinates but the vector rotation uses the opposite coordinates
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        headingDegrees = currentAngle - startAngle;
        negativeHeadingRadians = Math.toRadians(-headingDegrees);

        // changes the reference angle so the robot can "snap" to a grid angle
        if (gamepad1.dpad_up) {
            originalAngle = startAngle;
        } else if (gamepad1.dpad_right) {
            originalAngle = startAngle - 90;
        } else if (gamepad1.dpad_left) {
            originalAngle = startAngle + 90;
        } else if (gamepad1.dpad_down) {
            originalAngle = startAngle + 180;
        }

        // stick curve applied on the joystick input
        x = (Constants.DRIVE_CURVE_FACTOR * gamepad1.left_stick_x + (1 - Constants.DRIVE_CURVE_FACTOR) * Math.pow(gamepad1.left_stick_x, 3)) * Constants.MAXIMUM_DRIVE_POWER_TELEOP;
        y = (Constants.DRIVE_CURVE_FACTOR * -gamepad1.left_stick_y + (1 - Constants.DRIVE_CURVE_FACTOR) * Math.pow(-gamepad1.left_stick_y, 3)) * Constants.MAXIMUM_DRIVE_POWER_TELEOP;
        t = (Constants.DRIVE_CURVE_FACTOR * gamepad1.right_stick_x + (1 - Constants.DRIVE_CURVE_FACTOR) * Math.pow(gamepad1.right_stick_x, 3)) * Constants.MAXIMUM_TURN_POWER_TELEOP;

        // vector rotation - main field centric code
        xRotatedVector = x * Math.cos(negativeHeadingRadians) - y * Math.sin(negativeHeadingRadians);
        yRotatedVector = x * Math.sin(negativeHeadingRadians) + y * Math.cos(negativeHeadingRadians);

        // ratio so that max power to any motor is 1 and the proportion of power to other motors is preserved
        ratio = 1 / Math.max(Math.abs(xRotatedVector) + Math.abs(yRotatedVector) + Math.abs(t), 1);

        // sets motor powers to powers given by vector rotation/ratio
        xPower = xRotatedVector * ratio;
        yPower = yRotatedVector * ratio;
        tPower = t * ratio;

        driveWithIMU(xPower, yPower, tPower);
    }

    /**
     * allows the driver to operate the grabber to 3 positions:
     * x - closes the grabber
     * a - partially opens the grabber
     * b - fully opens the grabber
     */
    public void driveGrabberWithController() {
        if (gamepad2.x) {
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
        } else if (gamepad2.a) {
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
        } else if (gamepad2.b) {
            driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        }
    }

    /**
     * allows the driver to operate the slides using the left joystick, dpad, and bumpers
     * left joystick - fine control
     * dpad - raising and lowering the slides to cone stack heights
     * bumpers - raising and lowering the slides to junction heights
     */
    public void driveSlidesWithController() {
        // joystick control
        slideTargetPosition += (int) (-gamepad2.left_stick_y * Constants.SLIDE_TUNING_MODIFIER);

        // cone stack positions - increase position by one if dpad up is just pressed
        if (gamepad2.dpad_up && previousConeStack == currentConeStack) {
            currentConeStack++;

            // don't let dpad control take slides above highest cone height in cone stack
            if (currentConeStack >= 4) {
                currentConeStack = 4;
            }

            // set slide target position to the chosen cone stack height
            switch (currentConeStack) {
                case 1:
                    slideTargetPosition = Constants.SLIDE_STACK_ONE;
                    break;
                case 2:
                    slideTargetPosition = Constants.SLIDE_STACK_TWO;
                    break;
                case 3:
                    slideTargetPosition = Constants.SLIDE_STACK_THREE;
                    break;
                case 4:
                    slideTargetPosition = Constants.SLIDE_STACK_FOUR;
                    break;
            }

        // cone stack positions - decrease position by one if dpad down is just pressed
        } else if (gamepad2.dpad_down && previousConeStack == currentConeStack) {
            currentConeStack--;

            // don't let dpad control take slides below ground height
            if (currentConeStack <= 0) {
                currentConeStack = 0;
            }

            // set slide target position to the chosen cone stack height
            switch (currentConeStack) {
                case 0:
                    slideTargetPosition = Constants.SLIDE_BOTTOM;
                    break;
                case 1:
                    slideTargetPosition = Constants.SLIDE_STACK_ONE;
                    break;
                case 2:
                    slideTargetPosition = Constants.SLIDE_STACK_TWO;
                    break;
                case 3:
                    slideTargetPosition = Constants.SLIDE_STACK_THREE;
                    break;
            }
        }

        // junction positions - increase position by one if right bumper is just pressed
        if (gamepad2.right_bumper && previousJunction == currentJunction) {
            currentJunction++;

            // don't let bumper control take slides above high junction height
            if (currentJunction >= 3) {
                currentJunction = 3;
            }

            // set slide target position to the chosen junction height
            switch (currentJunction) {
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

        // junction positions - decrease position by one if left bumper is just pressed
        } else if (gamepad2.left_bumper && previousJunction == currentJunction) {
            currentJunction--;

            // don't let bumper control take slides below ground height
            if (currentJunction <= 0) {
                currentJunction = 0;
            }

            // set slide target position to the chosen junction height
            switch (currentJunction) {
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

        // makes sure that the previous cone stack positions are only updated when the dpad is just pressed
        if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
            previousConeStack = currentConeStack;
        }

        // makes sure that the previous junction positions are only updated when the bumpers are just pressed
        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            previousJunction = currentJunction;
        }
    }

    /**
     * allows the driver to align with a wall and reset the absolute and reference angles
     * pressing both bumpers resets the angles - used if IMU drift gets too bad
     */
    public void resetIMU() {
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            originalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
    }

    public void teleOpJunctionCentering() {
        if (gamepad1.back) {
            if (grabberCameraPipeline.detected == true) {
                centerJunctionTop(grabberCameraPipeline);
            }
        }
    }
}

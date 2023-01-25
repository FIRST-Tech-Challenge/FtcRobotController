package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;

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

    int stack = 0;
    int[] stacks = {0, 0};

    int junction = 0;
    int[] junctions = {0, 0};

    public void driveFieldCentric() {
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        headingDegrees = currentAngle - startAngle;
        negativeHeadingRadians = Math.toRadians(-headingDegrees);

        if (gamepad1.dpad_up) {
            originalAngle = startAngle;
        } else if (gamepad1.dpad_right) {
            originalAngle = startAngle - 90;
        } else if (gamepad1.dpad_left) {
            originalAngle = startAngle + 90;
        } else if (gamepad1.dpad_down) {
            originalAngle = startAngle + 180;
        }

        x = (Constants.DRIVE_CURVE_FACTOR * gamepad1.left_stick_x + (1 - Constants.DRIVE_CURVE_FACTOR) * Math.pow(gamepad1.left_stick_x, 3)) * Constants.MAXIMUM_DRIVE_POWER_TELEOP;
        y = (Constants.DRIVE_CURVE_FACTOR * -gamepad1.left_stick_y + (1 - Constants.DRIVE_CURVE_FACTOR) * Math.pow(-gamepad1.left_stick_y, 3)) * Constants.MAXIMUM_DRIVE_POWER_TELEOP;
        t = (Constants.DRIVE_CURVE_FACTOR * gamepad1.right_stick_x + (1 - Constants.DRIVE_CURVE_FACTOR) * Math.pow(gamepad1.right_stick_x, 3)) * Constants.MAXIMUM_TURN_POWER_TELEOP;

        xRotatedVector = x * Math.cos(negativeHeadingRadians) - y * Math.sin(negativeHeadingRadians);
        yRotatedVector = x * Math.sin(negativeHeadingRadians) + y * Math.cos(negativeHeadingRadians);
        ratio = 1 / Math.max(Math.abs(xRotatedVector) + Math.abs(yRotatedVector) + Math.abs(t), 1);

        xPower = xRotatedVector * ratio;
        yPower = yRotatedVector * ratio;
        tPower = t * ratio;

        driveWithIMU(xPower, yPower, tPower);
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

        if (gamepad2.dpad_up && stacks[0] == stacks[1]) {
            stack++;

            if (stack >= 4) {
                stack = 4;
            }

            switch (stack) {
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
        } else if (gamepad2.dpad_down && stacks[0] == stacks[1]) {
            stack--;

            if (stack <= 0) {
                stack = 0;
            }

            switch (stack) {
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

        if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
            stacks[0] = stacks[1];
        }

        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            junctions[0] = junctions[1];
        }

        stacks[1] = stack;
        junctions[1] = junction;
    }

    public void resetIMU() {
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            originalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
    }
}

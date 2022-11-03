package org.firstinspires.ftc.team6220_PowerPlay;

public abstract class BaseTeleOp extends BaseOpMode {

    // stick curve method
    public double stickCurve(double input) {
        return (Math.signum(input) * (Math.pow(input, 2)));
    }

    public void teleOpDrive(float leftXPosition, float leftYPosition, float turnPosition) {

        // booleans for if buttons x or a are pressed (to use grabber)
        boolean D2xIsPressed = gamepad2.x;
        boolean D2aIsPressed = gamepad2.a;

        // double for driver 2 left stick y value
        double D2leftStickY = -gamepad2.left_stick_y;

        // driver 1 right trigger value (used for slow mode)
        double D1rightTrigger = gamepad1.right_trigger;

        // driver 2 right trigger value (used for slow mode)
        double D2rightTrigger = gamepad2.right_trigger;

        // converts 0 to 1 inputs to 1 to 0.5 output modifier for driver 1
        double D1triggerSpeedMod = (D1rightTrigger * -0.5) + 1;

        // converts 0 to 1 inputs to 1 to 0.5 output modifier for driver 2
        double D2triggerSpeedMod = (D2rightTrigger * -0.5) + 1;

        // atan2 determines angle between the two sticks
        if (Math.abs(Math.atan2(leftYPosition, leftXPosition)) > Constants.DEADZONE_ANGLE_DEGREES) {
            // case for driving the robot left and right
            driveWithIMU(leftXPosition * D1triggerSpeedMod, 0, turnPosition * D1triggerSpeedMod);
        } else if (Math.abs(Math.atan2(leftXPosition, leftYPosition)) > Constants.DEADZONE_ANGLE_DEGREES) {
            // case for driving the robot up and down
            driveWithIMU(0, leftYPosition * D1triggerSpeedMod, turnPosition * D1triggerSpeedMod);
        } else {
            // case for if the deadzone limits are passed, the robot drives normally
            driveWithIMU(leftXPosition * D1triggerSpeedMod, leftYPosition * D1triggerSpeedMod, turnPosition * D1triggerSpeedMod);
        }

        // grabber open/close method attached to controller buttons
        if (D2xIsPressed) {  // press x to close
            driveGrabber(false);
        } else if (D2aIsPressed) {  // press a to open
            driveGrabber(true);
        }

        // drive slides
        driveSlides((Math.abs(D2leftStickY) > Constants.VERTICAL_SLIDE_DEADZONE? D2leftStickY * D2triggerSpeedMod : 0 ));
    }
}

package org.firstinspires.ftc.team6220_PowerPlay;

public abstract class BaseTeleOp extends BaseOpMode {

    // stick curve method
    public double stickCurve(double input) {
        return (Math.signum(input) * (Math.pow(input, 2)));
    }

    public void teleOpDrive(float leftXPosition, float leftYPosition, float turnPosition) {

        // saving the gamepad inputs into variables
        // booleans for if the buttons are pressed (to use grabber)
        boolean xIsPressed = gamepad2.x;
        boolean aIsPressed = gamepad2.a;

        // atan2 determines angle between the two sticks
        if (Math.abs(Math.atan2(leftYPosition, leftXPosition)) > Constants.DEADZONE_ANGLE_DEGREES) {
            // case for driving the robot left and right
            driveWithIMU(leftXPosition, 0, turnPosition);
        } else if (Math.abs(Math.atan2(leftXPosition, leftYPosition)) > Constants.DEADZONE_ANGLE_DEGREES) {
            // case for driving the robot up and down
            driveWithIMU(0, leftYPosition, turnPosition);
        } else {
            // case for if the deadzone limits are passed, the robot drives normally
            driveWithIMU(leftXPosition, leftYPosition, turnPosition);
        }

        // grabber open/close method attached to controller buttons
        if (xIsPressed) {  // press x to close
            driveGrabber(false);
        } else if (aIsPressed) {  // press a to open
            driveGrabber(true);
        }
    }
}

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

        // 5 degrees is the deadzone angle
        // atan2 determines angle between the two sticks

        if (Math.abs(Math.atan2(leftYPosition, leftXPosition)) > 10) {
            // case for driving the robot left and right
            driveRobot(leftXPosition, 0, turnPosition);
        } else if (Math.abs(Math.atan2(leftXPosition, leftYPosition)) > 10) {
            // case for driving the robot up and down
            driveRobot(0, leftYPosition, turnPosition);
        } else {
            // case for if the deadzone limits are passed, the robot drives normally
            driveRobot(leftXPosition, leftYPosition, turnPosition);
        }

        // grabber open/close method attached to controller buttons
        if (xIsPressed) {  // press x to close
            openGrabber(false);
        } else if (aIsPressed) {  // press a to open
            openGrabber(true);
        }
    }
}

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

        //double for gamepad 2 left stick y value
        double GP2leftStickY = -gamepad2.left_stick_y; //sus

        // gamepad 1 right trigger value (used for slow mode)
        double GP1rightTrigger = gamepad1.right_trigger;
        //converts 0 to 1 inputs to 1 to 0.5 output modifier
        double GP1TriggerSpeedMod = (GP1rightTrigger * -0.5) + 1;

        //gamepad2 speed modifier right trigger (turntable and such)
        double GP2rightTrigger = gamepad2.right_trigger;
        //converts 0 to 1 inputs to 1 to 0.5 output modifier
        double GP2TriggerSpeedMod = (GP2rightTrigger * -0.5) + 1;

        // atan2 determines angle between the two sticks
        if (Math.abs(Math.atan2(leftYPosition, leftXPosition)) > Constants.DEADZONE_ANGLE_DEGREES) {
            // case for driving the robot left and right
            driveWithIMU(leftXPosition * GP1TriggerSpeedMod, 0, turnPosition * GP1TriggerSpeedMod);
        } else if (Math.abs(Math.atan2(leftXPosition, leftYPosition)) > Constants.DEADZONE_ANGLE_DEGREES) {
            // case for driving the robot up and down
            driveWithIMU(0, leftYPosition * GP1TriggerSpeedMod, turnPosition * GP1TriggerSpeedMod);
        } else {
            // case for if the deadzone limits are passed, the robot drives normally
            driveWithIMU(leftXPosition * GP1TriggerSpeedMod, leftYPosition * GP1TriggerSpeedMod, turnPosition * GP1TriggerSpeedMod);
        }

        // grabber open/close method attached to controller buttons
        if (xIsPressed) {  // press x to close
            driveGrabber(false);
        } else if (aIsPressed) {  // press a to open
            driveGrabber(true);
        }

        //drive slides with deadzone of SLIDE_DEADZONE +/-
        if (Math.abs(GP2leftStickY) > Constants.SLIDE_DEADZONE ) {
            driveSlides(GP2leftStickY * GP2TriggerSpeedMod);
        }
    }
}

package org.firstinspires.ftc.team8923_CENTERSTAGE;


abstract public class BaseTeleOp extends BaseOpMode {
    // add mechanism speeds and drive methods when mechanisms is ready
    double slidesSpeed = 0.6;
    double releaseSpeed = 0.9;

    public void driveMechanism() {
        /* controls:
        hold left bumper: compliant wheels rotate in
        hold right bumper: compliant wheels rotate out
        d-pad up and down: linear slides up and down correspondingly
        A button: gondola flip
        B button: gondola release
        press left joystick down: release drone
         */
        if (gamepad2.left_bumper) {
            motorIntakeWheels.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            motorIntakeWheels.setPower(-1.0);
        }

        if (gamepad2.dpad_up) {
            motorSlideLeft.setPower(0.6);
            motorSlideRight.setPower(0.6);
        } if (gamepad2.dpad_down) {
            motorSlideLeft.setPower(-0.6);
            motorSlideRight.setPower(-0.6);
        }
        // CHANGE POSITION VALUE DURING TESTING
        if (gamepad2.a) {
            servoLeftFlipGondola.setPosition(1);
            servoRightFlipGondola.setPosition(1);
        }
        // THIS IS PROBABLY WRONG
        if (gamepad2.b) {
            servoReleasePixel.setPosition(1);
        } else if (gamepad2.b) {
            servoReleasePixel.setPosition(0);
        }
    }
}

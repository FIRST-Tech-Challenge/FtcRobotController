package org.firstinspires.ftc.team8923_CENTERSTAGE;


abstract public class BaseTeleOp extends BaseOpMode {
    // add mechanism speeds and drive methods when mechanisms is ready
    double slidesSpeed = 0.6;
    double releaseSpeed = 0.9;

    private boolean flipGondola = false;

    private Toggle pressToggle = new Toggle();

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
            servoReleasePixel.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            motorIntakeWheels.setPower(-1.0);
            servoReleasePixel.setPower(-1.0);
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
        flipGondola = pressToggle.toggle(gamepad2.a);
        if (flipGondola) {
            servoLeftFlipGondola.setPosition(1.0);
            servoRightFlipGondola.setPosition(1.0);
        } else if (flipGondola) {
            servoLeftFlipGondola.setPosition(0.0);
            servoRightFlipGondola.setPosition((0.0));
        }

        // PLEASE READ THIS!!!!!!!!!!!!!!!!!! PLEASE!!!!!!!!!!!!!!!!!!!!!!!!
        // TEST THIS WHEN PIXEL MECHANISM IS FUNCTIONAL!!!!!!!!!!!!!!!
        /*pixelReleaseOpen = pressToggle.toggle(gamepad2.b);
        if (pixelReleaseOpen) {
            servoReleasePixel.setPower(0.75);
        } else {
            servoReleasePixel.setPower(0.75);
        }*/

    }
}

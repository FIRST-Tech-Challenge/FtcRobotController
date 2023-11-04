package org.firstinspires.ftc.team8923_CENTERSTAGE;

abstract public class BaseTeleOp extends BaseOpMode {

    private boolean flipGondola = false;

    private Toggle pressToggle = new Toggle();

    public void driveMechanism() {
        /* controls:
        hold left bumper: compliant wheels rotate in, pixel release rotates in
        hold right bumper: compliant wheels rotate out, pixel release rotates out
        d-pad up and down: linear slides up and down correspondingly
        A button: gondola flip
        press left joystick down: release drone
         */

        if (gamepad2.left_bumper) {
            motorIntakeWheels.setPower(1.0);
            servoReleasePixel.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            motorIntakeWheels.setPower(-1.0);
            servoReleasePixel.setPower(-1.0);
        } else {
            motorIntakeWheels.setPower(0.0);
            servoReleasePixel.setPower(0.0);
        }

        if (gamepad2.dpad_up) {
            servoSlideLeft.setPower(1.0);
            servoSlideRight.setPower(1.0);
        } if (gamepad2.dpad_down) {
            servoSlideLeft.setPower(-1.0);
            servoSlideRight.setPower(-1.0);
        } else {
            servoSlideLeft.setPower(0.0);
            servoSlideRight.setPower(0.0);
        }

        // change positions after testing
        flipGondola = pressToggle.toggle(gamepad2.a);
        if (flipGondola) {
            servoLeftFlipGondola.setPosition(1.0);
            servoRightFlipGondola.setPosition(1.0);
        } else {
            servoLeftFlipGondola.setPosition(0.0);
            servoRightFlipGondola.setPosition(0.0);
        }

        // release drone
        if (gamepad2.right_stick_button) {
            servoReleaseDrone.setPosition(0.5);
            servoReleaseDrone.setPosition(0.0);
        }
    }
}

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

        // mUST CHANGE ALL OF THIS PLEASE OHMYGOD

        // intake and servo inside gondola rotate together inside when left bumper, out when right bumper
        if (gamepad2.left_bumper) {
            motorIntakeWheels.setPower(0.5);
            servoReleasePixel.setPower(-1.0);
        } else if (gamepad2.right_bumper) {
            motorIntakeWheels.setPower(-1.0);
            servoReleasePixel.setPower(1.0);
        } else {
            motorIntakeWheels.setPower(0.0);
            servoReleasePixel.setPower(0.0);
        }

        // slides controlled by up and down dpad
        if (gamepad2.dpad_up) {
            motorSlides.setPower(1.0);
        } if (gamepad2.dpad_down) {
            motorSlides.setPower(0.0);
        } else {
            motorSlides.setPower(0.0);
        }

        // this does not work lolz anD IDK WHY AAAAAAAAAAAA
        // servoRotateGondola.setPosition(gamepad2.right_stick_x);

        while (gamepad2.dpad_left) {
            servoRotateGondola.setPosition(servoRotateGondola.getPosition()-0.2);
        }
        while (gamepad2.dpad_right) {
            servoRotateGondola.setPosition(servoRotateGondola.getPosition()+0.2);
        }
        // button A flips arms for pixel mechanism
        if (gamepad2.a) {
            // servoLeftFlipGondola.setPosition(0.96);
            servoFlipGondola.setPosition(0.04);
            // servoRotateGondola.setPosition(0.0);
        // button B flips arms back
        } else if (gamepad2.b) {
            // servoLeftFlipGondola.setPosition(0.04);
            servoFlipGondola.setPosition(0.96);
            // servoFlipGondola.setPosition(1.0);
        }

        // release drone with pushing down right joystick, servo should rotate counter-clockwise
        if (gamepad2.right_stick_button) {
            servoReleaseDrone.setPosition(1.0);
        }
    }
}

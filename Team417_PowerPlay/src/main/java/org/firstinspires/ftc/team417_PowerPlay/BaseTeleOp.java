package org.firstinspires.ftc.team417_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode{
    boolean grabberIsOpen = false;
    double armPower = 0.0;
    public void driveUsingControllers() {
        double x = gamepad1.left_stick_x;
        double y = - gamepad1.left_stick_y;
        double turning = gamepad1.right_stick_x;

        mecanumDrive(x, y, turning);
    }
    public void driveArm() {
        armPower = (gamepad2.left_stick_y * 0.5);
        if (motorArm.getCurrentPosition() > 0 && armPower > 0) {
            armPower = 0;
        } else if (motorArm.getCurrentPosition() < -1000 && armPower < 0) {
            armPower = 0;
        }
        motorArm.setPower(armPower);

    }

    public void driveGrabber() {
        grabberIsOpen = grabberToggle.toggle(gamepad2.a);
        if (grabberIsOpen) {
            grabberServo.setPosition(GRABBER_OPEN);
        } else {
            grabberServo.setPosition(GRABBER_CLOSED);
        }
    }

    public void doTelemetry() {
        telemetry.addData("Arm power", armPower);
        telemetry.addData("Arm current position", motorArm.getCurrentPosition());
        telemetry.addData("Grabber position", grabberServo.getPosition());
        telemetry.addData("Grabber open", grabberIsOpen);
        telemetry.update();
    }
}

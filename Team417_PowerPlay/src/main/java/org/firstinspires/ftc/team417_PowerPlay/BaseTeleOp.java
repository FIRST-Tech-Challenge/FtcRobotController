package org.firstinspires.ftc.team417_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode {
    boolean grabberOpen = false;
    double armPower = 0.0;

    private static double DRIVING_SPEED = 0.5;

    public void driveUsingControllers() {
        double x = gamepad1.left_stick_x * DRIVING_SPEED;
        double y = -gamepad1.left_stick_y * DRIVING_SPEED;
        double turning = gamepad1.right_stick_x * DRIVING_SPEED;

        mecanumDrive(x, y, turning);
    }

    public void driveArm() {
        armPower = (gamepad2.left_stick_y * 0.5);
        if (motorArm.getCurrentPosition() > MIN_ARM_POSITION && armPower > 0) {
            armPower = 0;
        } else if (motorArm.getCurrentPosition() < MAX_ARM_POSITION && armPower < 0) {
            armPower = 0;
        }
        motorArm.setPower(armPower);
    }

    public void driveGrabber() {
        grabberOpen = grabberToggle.toggle(gamepad2.a);

        if (grabberOpen) {
            grabberServo.setPosition(GRABBER_OPEN);
        } else {
            grabberServo.setPosition(GRABBER_CLOSED);
        }
    }

    public void doTelemetry() {
        telemetry.addData("Arm power", armPower);
        telemetry.addData("Arm current position", motorArm.getCurrentPosition());
        telemetry.addData("Grabber position", grabberServo.getPosition());
        telemetry.addData("Grabber open", grabberOpen);
        telemetry.update();
    }
}

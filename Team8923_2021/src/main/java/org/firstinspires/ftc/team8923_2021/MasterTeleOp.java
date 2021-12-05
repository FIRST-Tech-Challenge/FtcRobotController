package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.util.Range;

abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;

    private Toggle driveSpeedToggle = new Toggle();

    double driveSpeed = 1.5;
    boolean startScoring = false;

    // one joystick moves forwards and backwards, the other controls left and right.
    public void splitArcadeDrive() {
        double forwardPower = -gamepad1.left_stick_y;
        double turningPower = gamepad1.right_stick_x;

        double leftPower = forwardPower + turningPower;
        double rightPower = forwardPower - turningPower;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void runDriveSpeed() {
        isSlowMode = driveSpeedToggle.toggle(gamepad1.left_bumper);
        if (isSlowMode) {
            driveSpeed = 0.25;
        } else {
            driveSpeed = 1.5;
        }
    }

    public void runIntake() {
        if (gamepad2.b) {
            motorIntake.setPower(0.6);
        } else if (gamepad2.x) {
            motorIntake.setPower(-0.6);
        } else {
            motorIntake.setPower(0.0);
        }
    }

    public void runCarousel() throws InterruptedException {
        if (gamepad2.left_trigger > Constants.MINIMUM_TRIGGER_VALUE) {
            motorCarousel.setPower(-1.0);
        } else if (gamepad2.right_trigger > Constants.MINIMUM_TRIGGER_VALUE) {
            motorCarousel.setPower(1.0);
        } else {
            motorCarousel.setPower(0.0);
        }
    }

    public void deliver() throws InterruptedException {
        if (gamepad2.left_bumper) {
            motorLift.setPower(0.1);
            motorLift.setTargetPosition(10);
            sleep(2000);
            servoGrabber.setPosition(1.0);
        } else if (gamepad2.right_bumper) {
            servoGrabber.setPosition(-0.8);
            motorLift.setPower(-0.1);
        } else {
            motorLift.setPower(0.0);
        }
        /*if(gamepad2.left_bumper && !startScoring) {
            motorLift.setTargetPosition(1);
            //motorLift.setPower(0.2);
        } else if (gamepad2.right_bumper && !startScoring) {
            motorLift.setTargetPosition(0);
            //motorLift.setPower(-0.2);
        } */
        //motorLift.setPower(Math.max((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 75.0), 1.0));

    }

    public void grab() {
        if (gamepad2.dpad_up) {
            servoGrabber.setPosition(1.0);
            sleep(100);
        } else if (gamepad2.dpad_down) {
            servoGrabber.setPosition(0.0);
        } else {
            servoGrabber.setPosition(0.0);
            sleep(100);
        }
    }
}

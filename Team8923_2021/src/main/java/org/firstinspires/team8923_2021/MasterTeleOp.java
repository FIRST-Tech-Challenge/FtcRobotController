package org.firstinspires.team8923_2021;


abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;

    private Toggle driveSpeedToggle = new Toggle();

    double driveSpeed = 1.0;

    int scoreState = 0;
    int flickStage = 0;

    // one joystick moves forwards and backwards, the other controls left and right.
    public void splitArcadeDrive() {
        double forwardPower = gamepad1.left_stick_y;
        double turningPower = gamepad1.right_stick_x;

        double leftPower = turningPower - forwardPower ;
        double rightPower = turningPower + forwardPower;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void runDriveSpeed() {
        isSlowMode = driveSpeedToggle.toggle(gamepad1.left_bumper);
        if (isSlowMode) driveSpeed = 0.25;
        else driveSpeed = 1.0;
    }
}


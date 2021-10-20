package org.firstinspires.team8923_2021;


abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;

    private Toggle driveSpeedToggle = new Toggle();

    double driveSpeed = 1.0;

    int scoreState = 0;
    int flickStage = 0;

    /*public void driveRobot(){
        double y = -gamepad1.left_stick_y * driveSpeed; // y up is negative
        double x = gamepad1.left_stick_x * driveSpeed;
        double rotationalPower = gamepad1.right_stick_x;


        double power = calculateDistance(x, y);
        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveMecanum(angle, power, rotationalPower);
    }*/

    public void splitArcadeDrive() {
        double forwardPower = gamepad1.left_stick_y;
        double turningPower = Math.pow(Math.abs(gamepad1.right_stick_x), 2) * Math.signum(gamepad1.right_stick_x);

        double leftPower = forwardPower - turningPower;
        double rightPower = forwardPower + turningPower;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void runDriveSpeed() {
        isSlowMode = driveSpeedToggle.toggle(gamepad1.left_bumper);
        if (isSlowMode) driveSpeed = 0.25;
        else driveSpeed = 1.0;
    }


    /*private double map(double value, double minInput, double maxInput, double minMappedOutput, double maxMappedOutput) {
        double valueDifference = maxInput - minInput;
        double percentValueDifference = (value - minInput) / valueDifference;
        double mappedDifference = maxMappedOutput - minMappedOutput;

        return percentValueDifference * mappedDifference + minMappedOutput;*/
    }




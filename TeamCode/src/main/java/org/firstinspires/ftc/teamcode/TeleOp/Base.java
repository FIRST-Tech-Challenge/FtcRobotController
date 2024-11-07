package org.firstinspires.ftc.teamcode.TeleOp;

public class Base {

    public final RobotHardware robot;

    // Constructor that takes RobotHardware as a parameter
    public Base(RobotHardware robot) {
        this.robot = robot;
    }

    // Drive control method
    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate power for each motor
        double frontLeftPower = calculateMotorPower(y, x, rx, 1, 1, denominator);
        double backLeftPower = calculateMotorPower(y, x, rx, -1, 1, denominator);
        double frontRightPower = calculateMotorPower(y, x, rx, -1, -1, denominator);
        double backRightPower = calculateMotorPower(y, x, rx, 1, -1, denominator);

        // Set motor powers
        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backRightMotor.setPower(backRightPower);
    }

    // Helper method for calculating motor power
    private double calculateMotorPower(double y, double x, double rx, double xSign, double rxSign, double denominator) {
        return (y + x * xSign + rx * rxSign) / denominator;
    }
}
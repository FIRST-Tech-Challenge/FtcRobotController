package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometer {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    private double xPosition = 0;
    private double yPosition = 0;
    private double heading = 0;

    // Encoder constants
    private static final double TICKS_PER_REVOLUTION = 1440; // Assuming 1440 ticks per revolution
    private static final double WHEEL_RADIUS = 4.0; // Inches, adjust for your robot's wheels
    private static final double ROBOT_TRACK_WIDTH = 14.5; // Distance between left and right wheels (inches)

    public Odometer(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
    }

    // Update the position based on encoder counts
    public void update() {
        // Get encoder counts from motors
        int leftFrontTicks = frontLeftMotor.getCurrentPosition();
        int rightFrontTicks = frontRightMotor.getCurrentPosition();
        int leftBackTicks = backLeftMotor.getCurrentPosition();
        int rightBackTicks = backRightMotor.getCurrentPosition();

        // Calculate the average distance traveled by left and right wheels
        double leftDistance = leftFrontTicks / TICKS_PER_REVOLUTION * (2 * Math.PI * WHEEL_RADIUS);
        double rightDistance = rightFrontTicks / TICKS_PER_REVOLUTION * (2 * Math.PI * WHEEL_RADIUS);

        // Calculate the robot's displacement (simple differential drive model)
        double deltaDistance = (leftDistance + rightDistance) / 2;
        double deltaHeading = (rightDistance - leftDistance) / ROBOT_TRACK_WIDTH;

        // Update robot's position
        xPosition += deltaDistance * Math.cos(heading);
        yPosition += deltaDistance * Math.sin(heading);
        heading += deltaHeading;

        // Normalize heading to be within 0-360 degrees
        if (heading > 2 * Math.PI) heading -= 2 * Math.PI;
        if (heading < 0) heading += 2 * Math.PI;
    }

    // Return current position
    public double[] getPosition() {
        return new double[] {xPosition, yPosition, heading};
    }
}

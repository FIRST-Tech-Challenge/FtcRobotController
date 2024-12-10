package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commons.RobotHardware;

public class AutonomousRobot extends RobotHardware {

    private final DcMotor[] motors = new DcMotor[4];

    // Odometers (Encoders for localization)
    private final Odometer odometer;

    private final Telemetry telemetry;

    // Constructor
    public AutonomousRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        this.init(hardwareMap);
        this.telemetry = telemetry;

        motors[0] = this.frontLeftMotor;
        motors[1] = this.frontRightMotor;
        motors[2] = this.backLeftMotor;
        motors[3] = this.backRightMotor;

        // Initialize Odometer (for localization)
        odometer = new Odometer(this.frontLeftMotor, this.frontRightMotor, this.backLeftMotor, this.backRightMotor);
    }

    // Method to drive robot using motors
    public void drive(double leftPower, double rightPower) {
        this.frontLeftMotor.setPower(leftPower);
        this.frontRightMotor.setPower(rightPower);
        this.backLeftMotor.setPower(leftPower);
        this.backRightMotor.setPower(rightPower);
    }

    // Method to get robot's current position from odometer
    public double[] getPosition() {
        return odometer.getPosition();
    }

    public void moveToTarget(double targetX, double targetY) {
        // Logic to move to a target position using odometry (you can use PID control here for more precision)
        double[] currentPosition = odometer.getPosition();
        double currentX = currentPosition[0];
        double currentY = currentPosition[1];

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        telemetry.addLine("Before while");
        telemetry.update();
        // Basic control loop (ideal for a simpler task, may want to use PID control for more accuracy)
        while (Math.abs(deltaX) > 0.5 || Math.abs(deltaY) > 0.5) {
            double angleToTarget = Math.atan2(deltaY, deltaX); // Angle to the target
            double distanceToTarget = Math.hypot(deltaX, deltaY); // Distance to target

        }
    }
}
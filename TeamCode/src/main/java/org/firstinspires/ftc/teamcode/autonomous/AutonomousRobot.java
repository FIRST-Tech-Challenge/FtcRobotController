package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commons.RobotHardware;

public class AutonomousRobot extends RobotHardware {

    private final DcMotor[] motors = new DcMotor[4];

    // Odometers (Encoders for localization)
    private Odometer odometer;

    private Telemetry telemetry;

    public void init (HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        telemetry.addLine("preInit ");
        telemetry.update();
        super.init(hardwareMap);
        telemetry.addLine("initializing");
        telemetry.update();
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

        // Basic control loop (ideal for a simpler task, may want to use PID control for more accuracy)
        while (Math.abs(deltaX) > 0.5 || Math.abs(deltaY) > 0.5) {
            double angleToTarget = Math.atan2(deltaY, deltaX); // Angle to the target
            double distanceToTarget = Math.hypot(deltaX, deltaY); // Distance to target

            // Simple drive control: move forward towards the target
            drive(0.5, 0.5);  // Set robot to move forward at half power

            // Update position and recalculate deltas
            odometer.update();
            currentPosition = odometer.getPosition();
            currentX = currentPosition[0];
            currentY = currentPosition[1];

            deltaX = targetX - currentX;
            deltaY = targetY - currentY;
        }

        // Stop the robot once we reach the target
        drive(0, 0);

    }
}
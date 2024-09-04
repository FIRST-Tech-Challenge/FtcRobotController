package org.firstinspires.ftc.teamcode.magic;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot;

public class Odometry {
    // Constants for encoder counts per revolution and wheel diameter
    final double COUNTS_PER_REVOLUTION = 100; // Example value, adjust based on your motor encoders
    final double WHEEL_DIAMETER_INCHES = 3.5; // Example value in inches, adjust based on your robot's wheels
    /**
     * makes default autonomous movement faster or slower on a scale from 0-1.
     * normally 0.25.
     * values above 0.6 will experience unacceptable amounts of encoder inaccuracy.
     */
    final double default_autonomous_speed = 0.38;
    private final BaseRobot baseRobot;
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor rearLeftMotor;
    private final DcMotor rearRightMotor;
    public double xPosition;
    public double yPosition;
    public double heading;

    // Constructor
    public Odometry(BaseRobot baseRobot) {
        this.frontLeftMotor = baseRobot.frontLeftMotor;
        this.frontRightMotor = baseRobot.frontRightMotor;
        this.rearLeftMotor = baseRobot.rearLeftMotor;
        this.rearRightMotor = baseRobot.rearRightMotor;
        this.baseRobot = baseRobot;

        // Initialize other variables as needed
        this.xPosition = 0.0;
        this.yPosition = 0.0;
        this.heading = 0.0;
    }

    // Function to update position and heading based on encoder readings
    public void update() {

        // Calculate distance traveled by each wheel
        double frontLeftDistance = (frontLeftMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION) * (Math.PI * WHEEL_DIAMETER_INCHES);
        double frontRightDistance = (frontRightMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION) * (Math.PI * WHEEL_DIAMETER_INCHES);
        double rearLeftDistance = (rearLeftMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION) * (Math.PI * WHEEL_DIAMETER_INCHES);
        double rearRightDistance = (rearRightMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION) * (Math.PI * WHEEL_DIAMETER_INCHES);

        // Calculate robot's movement and update position
        double distanceTraveled = (frontLeftDistance + frontRightDistance + rearLeftDistance + rearRightDistance) / 4.0;
        double deltaHeading = (frontLeftDistance - frontRightDistance + rearLeftDistance - rearRightDistance) / (4.0 * WHEEL_DIAMETER_INCHES);

        double deltaX = distanceTraveled * Math.cos(heading + (deltaHeading / 2.0));
        double deltaY = distanceTraveled * Math.sin(heading + (deltaHeading / 2.0));

        xPosition += deltaX;
        yPosition += deltaY;
        heading += deltaHeading;

        // Normalize heading angle between 0 and 2*pi
        heading = (2 * Math.PI + heading) % (2 * Math.PI);
    }


    // Function to reset position and heading to zero
    public void reset() {
        xPosition = 0.0;
        yPosition = 0.0;
        heading = 0.0;
        // Reset motor encoders for next update
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor mode back to RUN_USING_ENCODER after reset
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveCounts(String direction, double counts) {
        moveCounts(direction, counts, default_autonomous_speed);
    }

    public void moveCounts(String direction, double counts, double speed) {
        reset();
        // Calculate target counts for each motor based on the given counts
        double targetCounts = counts * COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER_INCHES);

        switch (direction.toLowerCase()) {
            case "forward":
                while (Math.abs(frontLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(frontRightMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearRightMotor.getCurrentPosition()) < targetCounts) {
                    setMotorPower(speed); // Example power, adjust as needed
                }
                break;
            case "backward":
                while (Math.abs(frontLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(frontRightMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearRightMotor.getCurrentPosition()) < targetCounts) {
                    setMotorPower(-speed); // Example power, adjust as needed
                }
                break;
            case "left":
                while (Math.abs(frontLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(frontRightMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearRightMotor.getCurrentPosition()) < targetCounts) {
                    setStrafePowers(-speed); // Example power, adjust as needed for left movement
                }
                break;
            case "right":
                while (Math.abs(frontLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(frontRightMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearRightMotor.getCurrentPosition()) < targetCounts) {
                    setStrafePowers(speed); // Example power, adjust as needed for right movement
                }
                break;
            case "tleft":
                while (Math.abs(frontLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(frontRightMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearRightMotor.getCurrentPosition()) < targetCounts) {
                    setMotorPowers(-speed / 1.75, speed / 1.5); // Example power, adjust as needed for right movement
                }
                break;
            case "tright":
                while (Math.abs(frontLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(frontRightMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearLeftMotor.getCurrentPosition()) < targetCounts
                        && Math.abs(rearRightMotor.getCurrentPosition()) < targetCounts) {
                    setMotorPowers(speed / 1.75, -speed / 1.5); // Example power, adjust as needed for right movement
                }
                break;
            default:
                // Invalid direction provided
                break;
        }

        // Stop the robot after reaching the target counts
        setMotorPower(0);
    }


    // Function to move to a specific position
    public void moveToPosition(double targetX, double targetY) {
        // Calculate distance and angle to the target position
        double deltaX = targetX - xPosition;
        double deltaY = targetY - yPosition;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angleToTarget = Math.atan2(deltaY, deltaX);

        // Adjust the robot's heading to face the target angle
        double angleDifference = angleToTarget - heading;

        // Normalize angle difference to be between -pi and pi
        angleDifference = (angleDifference + Math.PI) % (2 * Math.PI) - Math.PI;

        // Set a threshold for both distance and angle to consider the target reached
        final double DISTANCE_THRESHOLD = 1.0; // Example distance threshold in inches, adjust as needed
        final double ANGLE_THRESHOLD = Math.toRadians(5); // Example angle threshold in radians, adjust as needed

        while (distanceToTarget > DISTANCE_THRESHOLD || Math.abs(angleDifference) > ANGLE_THRESHOLD) {
            // Recalculate the differences in position and angle in case they change during movement
            deltaX = targetX - xPosition;
            deltaY = targetY - yPosition;
            distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            angleToTarget = Math.atan2(deltaY, deltaX);

            angleDifference = angleToTarget - heading;
            angleDifference = (angleDifference + Math.PI) % (2 * Math.PI) - Math.PI;

            // Adjust robot's movement direction based on angle difference
            double turnPower = angleDifference * 0.03; // Proportional control factor, adjust as needed

            // Limit the turn power to prevent excessive rotation
            turnPower = Math.max(-1, Math.min(1, turnPower));

            // Set motor powers for both movement and rotation
            double movementPower = distanceToTarget > DISTANCE_THRESHOLD ? 0.5 : 0.0; // Adjust movement power as needed

            setMotorPowers(movementPower + turnPower, movementPower - turnPower);
            // Assuming differential drive with two motors: left and right
        }

        // Stop the robot after reaching the target
        setMotorPower(0.0);
    }

    private void setStrafePowers(double strafePower) {
        frontLeftMotor.setPower(strafePower);
        rearLeftMotor.setPower(-strafePower);
        frontRightMotor.setPower(-strafePower);
        rearRightMotor.setPower(strafePower);
    }


    private void setMotorPowers(double leftPower, double rightPower) {
        // Apply the calculated powers to the left and right motors
        frontLeftMotor.setPower(leftPower);
        rearLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        rearRightMotor.setPower(rightPower);

    }

    private void setMotorPower(double power) {
        setMotorPowers(power, power);
    }
}

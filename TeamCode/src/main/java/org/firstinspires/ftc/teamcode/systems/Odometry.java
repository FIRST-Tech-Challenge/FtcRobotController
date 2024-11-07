package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/**
 * Handles robot position tracking and autonomous movement using wheel encoders.
 * Implements dead reckoning to maintain position awareness and execute precise
 * movements.
 * 
 * Features:
 * - Real-time position and heading tracking
 * - Encoder-based movement commands
 * - Autonomous navigation to specific coordinates
 * - Support for different movement types (forward, strafe, turn)
 */
public class Odometry {
    final double COUNTS_PER_REVOLUTION = Settings.Hardware.COUNTS_PER_REVOLUTION;
    final double WHEEL_DIAMETER_INCHES = Settings.Hardware.WHEEL_DIAMETER_INCHES;
    final double default_autonomous_speed = Settings.Movement.default_autonomous_speed;
    private final BaseRobot baseRobot;
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor rearLeftMotor;
    private final DcMotor rearRightMotor;

    /**
     * Current X position of the robot on the field (in inches)
     */
    public double xPosition;

    /**
     * Current Y position of the robot on the field (in inches)
     */
    public double yPosition;

    /**
     * Current heading of the robot in radians (0 to 2π)
     * 0/2π = facing positive X axis
     * π/2 = facing positive Y axis
     */
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

    /**
     * Updates the robot's position and heading based on encoder readings.
     * Should be called regularly (preferably in a loop) to maintain accurate
     * positioning.
     * 
     * Calculation process:
     * 1. Reads current encoder values from all wheels
     * 2. Converts encoder counts to distance traveled
     * 3. Uses wheel movements to calculate position and heading changes
     * 4. Updates stored position and heading values
     */
    public void update() {

        // Calculate distance traveled by each wheel
        double frontLeftDistance = (frontLeftMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION)
                * (Math.PI * WHEEL_DIAMETER_INCHES);
        double frontRightDistance = (frontRightMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION)
                * (Math.PI * WHEEL_DIAMETER_INCHES);
        double rearLeftDistance = (rearLeftMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION)
                * (Math.PI * WHEEL_DIAMETER_INCHES);
        double rearRightDistance = (rearRightMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION)
                * (Math.PI * WHEEL_DIAMETER_INCHES);

        // Calculate robot's movement and update position
        double distanceTraveled = (frontLeftDistance + frontRightDistance + rearLeftDistance + rearRightDistance) / 4.0;
        double deltaHeading = (frontLeftDistance - frontRightDistance + rearLeftDistance - rearRightDistance)
                / (4.0 * WHEEL_DIAMETER_INCHES);

        double deltaX = distanceTraveled * Math.cos(heading + (deltaHeading / 2.0));
        double deltaY = distanceTraveled * Math.sin(heading + (deltaHeading / 2.0));

        xPosition += deltaX;
        yPosition += deltaY;
        heading += deltaHeading;

        // Normalize heading angle between 0 and 2*pi
        heading = (2 * Math.PI + heading) % (2 * Math.PI);
    }

    /**
     * Resets all position tracking to zero and resets encoder counts.
     * Should be called at the start of autonomous operations or when
     * establishing a new reference point.
     */
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

    /**
     * Moves the robot a specified distance using encoder counts with default speed
     * 
     * @param direction Movement direction ("forward", "backward", "left", "right",
     *                  "tleft", "tright")
     * @param counts    Distance to move in encoder counts
     */
    public void moveCounts(String direction, double counts) {
        moveCounts(direction, counts, default_autonomous_speed);
    }

    /**
     * Moves the robot a specified distance using encoder counts with custom speed
     * 
     * @param direction Movement direction ("forward", "backward", "left", "right",
     *                  "tleft", "tright")
     * @param counts    Distance to move in encoder counts
     * @param speed     Motor power to use (0.0 to 1.0)
     */
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

    /**
     * Navigates the robot to a specific position on the field using proportional
     * control
     * Automatically handles both rotation and translation to reach the target
     * 
     * @param targetX Target X coordinate in inches
     * @param targetY Target Y coordinate in inches
     * 
     *                Movement process:
     *                1. Calculates distance and angle to target
     *                2. Rotates to face target location
     *                3. Moves toward target while maintaining heading
     *                4. Fine-tunes position when near target
     */
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
            // Recalculate the differences in position and angle in case they change during
            // movement
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

    /**
     * Sets strafe movement power for mecanum drive
     * 
     * @param strafePower Power level for strafing (-1.0 to 1.0)
     */
    private void setStrafePowers(double strafePower) {
        frontLeftMotor.setPower(strafePower);
        rearLeftMotor.setPower(-strafePower);
        frontRightMotor.setPower(-strafePower);
        rearRightMotor.setPower(strafePower);
    }

    /**
     * Sets differential drive powers for tank-style movement
     * 
     * @param leftPower  Power for left side motors (-1.0 to 1.0)
     * @param rightPower Power for right side motors (-1.0 to 1.0)
     */
    private void setMotorPowers(double leftPower, double rightPower) {
        // Apply the calculated powers to the left and right motors
        frontLeftMotor.setPower(leftPower);
        rearLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        rearRightMotor.setPower(rightPower);

    }

    /**
     * Sets the same power level to all drive motors
     * 
     * @param power Power level for all motors (-1.0 to 1.0)
     */
    private void setMotorPower(double power) {
        setMotorPowers(power, power);
    }
}

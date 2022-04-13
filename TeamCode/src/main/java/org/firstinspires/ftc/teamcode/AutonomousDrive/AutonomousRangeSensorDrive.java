package org.firstinspires.ftc.teamcode.AutonomousDrive;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousRangeSensorDrive extends AutonomousBasicDrive {
    public AutonomousRangeSensorDrive(HardwareRobot robot) {
        super(robot);
    }

    /**
     *
     * @param maxSpeed - The speed to use when going large distances
     * @param maxRotation - The speed to rotate when making heading corrections
     * @param distanceFromWall - The distance to make the robot parallel to the wall in cm
     * @param timeout - The maximum amount of time to wait until giving up
     * @return true if reached distance, false if timeout occurred first
     */
    public boolean driveToLeftWall(double maxSpeed, double maxRotation, int distanceFromWall, int timeout) {
        boolean reachedDestination = false;
        int allowedError = 2;
        ElapsedTime timer = new ElapsedTime();
        int sensorDistance = robot.readLeftRangeSensor();
        int distanceError = sensorDistance - distanceFromWall;
        timer.reset();
        while(opModeIsActive() && (Math.abs(distanceError) > allowedError) && (timer.milliseconds() < timeout)) {
            robot.readBulkData();

            performRobotActions();
        }

        return reachedDestination;
    }
}

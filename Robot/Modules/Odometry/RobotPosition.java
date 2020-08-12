package org.firstinspires.ftc.teamcode.rework.Robot.Modules.Odometry;

import org.firstinspires.ftc.teamcode.rework.Robot.Auto.PathPlanning.Point;

public class RobotPosition {
    /**
     * Location of the robot.
     */
    private Point location;

    /**
     * Angle the robot is facing, starting with 0 radians straight forward and increasing clockwise.
     */
    private double heading;

    public RobotPosition(Point location, double heading) {
        this.location = location;
        this.heading = heading;
    }

    public void updatePosition(Point robotLocation, double robotHeading) {
        this.location = robotLocation;
        this.heading = robotHeading;
    }

    public Point getLocation() {
        return location;
    }

    public double getHeading() {
        return heading;
    }
}

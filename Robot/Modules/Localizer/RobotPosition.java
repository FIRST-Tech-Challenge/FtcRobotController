package org.firstinspires.ftc.teamcode.rework.Robot.Modules.Localizer;

import org.firstinspires.ftc.teamcode.rework.Robot.Auto.PathPlanning.Point;

public class RobotPosition {
    /**
     * Location of the robot.
     */
    private Point robotLocation;

    /**
     * Angle the robot is facing, starting with 0 radians straight forward and increasing clockwise.
     */
    private double robotHeading;

    public RobotPosition(Point robotLocation, double robotHeading) {
        this.robotLocation = robotLocation;
        this.robotHeading = robotHeading;
    }

    public void updatePosition(Point robotLocation, double robotHeading) {
        this.robotLocation = robotLocation;
        this.robotHeading = robotHeading;
    }

    public Point getRobotLocation() {
        return robotLocation;
    }

    public double getRobotHeading() {
        return robotHeading;
    }
}

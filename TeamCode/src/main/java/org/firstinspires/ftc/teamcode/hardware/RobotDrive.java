package org.firstinspires.ftc.teamcode.hardware;

public interface RobotDrive {
    void setDistanceUnits(DistanceUnit unit);
    DistanceUnit getDistanceUnit();

    void moveRobot(double forward, double rotate);


    void moveForward(double distance);
    /*
    ** The rotate functions rotate the robot around the z axis. Positive is counterclockwise, negative is clockwise.
     */
    void rotateDegrees(double degrees);
    void rotateRadians(double radians);
}

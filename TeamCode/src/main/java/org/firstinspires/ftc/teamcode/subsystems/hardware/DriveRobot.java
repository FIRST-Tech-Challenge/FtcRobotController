package org.firstinspires.ftc.teamcode.subsystems.hardware;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public interface DriveRobot extends SubSystem {
    void turn(double degrees);
    void turnTo(double degrees);
    void driveForward(double distance);
    void driveLeft(double distance);
    void driveReverse(double distance);
    void driveRight(double distance);
}

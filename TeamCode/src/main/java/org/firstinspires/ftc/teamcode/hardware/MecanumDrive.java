package org.firstinspires.ftc.teamcode.hardware;

public interface MecanumDrive extends RobotDrive {

    void moveRect(double forward, double lateral, double rotate);
    void movePolar(double power, double angle, double rotate);
    /*
    ** This moves the robot to the side, positive being to the right and negative being left.
     */
    void moveLaterally(double distance);
}

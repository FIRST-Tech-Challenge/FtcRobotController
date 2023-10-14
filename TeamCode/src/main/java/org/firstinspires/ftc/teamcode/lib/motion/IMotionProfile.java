package org.firstinspires.ftc.teamcode.lib.motion;

public interface IMotionProfile {
    void generateProfile();
    double getDuration();
    double getPosition();
    double getVelocity();
    double getAcceleration();
    double getJerk();
    double getPosition(double timeStamp);
    double getVelocity(double timeStamp);
    double getAcceleration(double timeStamp);
    double getJerk(double timeStamp);
    double getRuntime();
    boolean isDone(double timeStamp);
    boolean isDone();
    void start();
    void setNegative(boolean negative);
    boolean isNegative();
}

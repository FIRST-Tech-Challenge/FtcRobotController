package org.firstinspires.ftc.teamcode.odometry;

public interface IBaseOdometry extends Runnable {

    void setInitPosition(int startXInches, int startYInches, int startHeadingDegrees) throws Exception;
    void stop();

    int getCurrentX();
    int getCurrentY();
    int getCurrentHeading();
}

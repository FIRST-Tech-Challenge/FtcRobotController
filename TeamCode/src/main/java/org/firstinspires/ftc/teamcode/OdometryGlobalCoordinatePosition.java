package org.firstinspires.ftc.teamcode;

import android.net.MacAddress;

public class OdometryGlobalCoordinatePosition implements Runnable {
    HardwareMapV2 robot;

    double globalPositionX, globalPositionY, robotOrientationDegrees, previousOrientationDegrees, previousleftEncoder, previousrightEncoder, previousHorizontalEncoder, changeinrobotorientationDegrees, changeinX, changeinY;
    double ENCODERS_PER_DEGREE; //Get during calibration

    public void updatePositions(double leftEncoder, double rightEncoder, double horizontal){
        previousOrientationDegrees = robotOrientationDegrees;
        changeinrobotorientationDegrees = (leftEncoder - rightEncoder)/ENCODERS_PER_DEGREE;
        robotOrientationDegrees += changeinrobotorientationDegrees;

        double distance = (leftEncoder-previousleftEncoder)+(rightEncoder-previousrightEncoder)/2;
        double Hdistance = horizontal-previousHorizontalEncoder;

        changeinX = (Math.cos(Math.toRadians(robotOrientationDegrees))*distance) + Math.cos(Hdistance);
        changeinY = (Math.sin(Math.toRadians(robotOrientationDegrees))*distance) + Math.sin(Hdistance);

        globalPositionX+=changeinX;
        globalPositionY+=changeinY;

        previousleftEncoder = leftEncoder;
        previousrightEncoder = rightEncoder;
        previousHorizontalEncoder = horizontal;

    }

    public double Xpos(){
        return globalPositionX;
    }

    public double Ypos(){
        return globalPositionY;
    }

    @Override
    public void run() {
        updatePositions(robot.leftVertical.getCurrentPosition(), robot.rightVertical.getCurrentPosition(), robot.horizontal.getCurrentPosition());
    }
}

package org.firstinspires.ftc.teamcode;

import android.net.MacAddress;

public class OdometryGlobalCoordinatePosition implements Runnable {
    HardwareMapV2 robot;
    boolean isRunning = true;

    double globalPositionX, globalPositionY, robotOrientationDegrees, previousOrientationDegrees, previousleftEncoder, previousrightEncoder, previousHorizontalEncoder, changeinrobotorientationDegrees, changeinX, changeinY;
    double ENCODERS_PER_DEGREE; //Get during calibration

    public OdometryGlobalCoordinatePosition(double x, double y, double rotationDeg){
        globalPositionX = x;
        globalPositionY = y;
        robotOrientationDegrees = rotationDeg;
    }

    public void updatePositions(double leftEncoder, double rightEncoder, double horizontal){
        previousOrientationDegrees = robotOrientationDegrees;
        changeinrobotorientationDegrees = (leftEncoder - rightEncoder)/ENCODERS_PER_DEGREE;
        robotOrientationDegrees += changeinrobotorientationDegrees;

        double distance = (leftEncoder-previousleftEncoder)+(rightEncoder-previousrightEncoder)/2;
        double Hdistance = horizontal-previousHorizontalEncoder;

        changeinX = (Math.cos(Math.toRadians(robotOrientationDegrees))*distance) + Math.cos((Math.toRadians(robotOrientationDegrees-90))*Hdistance);
        changeinY = (Math.sin(Math.toRadians(robotOrientationDegrees))*distance) + Math.sin((Math.toRadians(robotOrientationDegrees-90))*Hdistance);

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

    public double theta() { return robotOrientationDegrees; }

    public void stop() {isRunning = false;}

    @Override
    public void run() {
        while (isRunning) {
            updatePositions(robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.intake.getCurrentPosition());
            try {
                Thread.sleep(75);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

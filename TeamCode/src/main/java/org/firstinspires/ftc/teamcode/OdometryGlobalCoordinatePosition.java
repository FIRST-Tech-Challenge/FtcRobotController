package org.firstinspires.ftc.teamcode;

public class OdometryGlobalCoordinatePosition implements Runnable {
    HardwareMapV2 robot;

    double globalPositionX, globalPositionY, robotOrientationDegrees, previousOrientationDegrees, previousleftEncoder, previousrightEncoder, changeinrobotorientationDegrees, changeinX, changeinY;
    double ENCODERS_PER_DEGREE; //Get during calibration

    public void updatePositions(double leftEncoder, double rightEncoder){
        previousOrientationDegrees = robotOrientationDegrees;
        changeinrobotorientationDegrees = (leftEncoder - rightEncoder)/ENCODERS_PER_DEGREE;
        robotOrientationDegrees += changeinrobotorientationDegrees;

        double distance = (leftEncoder-previousleftEncoder)+(rightEncoder-previousrightEncoder)/2;

        changeinX = Math.cos(Math.toRadians(robotOrientationDegrees))*distance;
        changeinY = Math.sin(Math.toRadians(robotOrientationDegrees))*distance;

        globalPositionX+=changeinX;
        globalPositionY+=changeinY;

        previousleftEncoder = leftEncoder;
        previousrightEncoder = rightEncoder;

    }

    public double Xpos(){
        return globalPositionX;
    }

    public double Ypos(){
        return globalPositionY;
    }

    @Override
    public void run() {
        updatePositions(robot.leftVertical.getCurrentPosition(), robot.rightVertical.getCurrentPosition());
    }
}

package org.firstinspires.ftc.teamcode.resourses;

public class Odometry {
  
  public double initX, initY;
  public double dForward, dSideways;
  public double correctionFactor = 1;
  public double ticksPerRevolution = 2000;
  public double mmPerRevolution = 48 * Math.PI;
  public double mmPerInch = 25.4;
  public double odometryPodOldX, odometryPodOldY;
  public double odometryPodX, odometryPodY;
  public static double radiusX = 6.08;
  public static double radiusY = 7.73886;
  
  private double robotX, robotY;
  private double robotAngle;
  private double oldAngle;
  
  double wrap(double angle) {
    while (angle > Math.PI) {
      angle -= 2 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2 * Math.PI;
    }
    return angle;
  }
  
  public void UpdateOdometry() {
    
    double deltaRotation = robotAngle - oldAngle;
    deltaRotation = wrap(deltaRotation);
    dForward = odometryPodY - radiusY * deltaRotation;
    dSideways = odometryPodX - radiusX * deltaRotation;
    robotX = (initX - dForward * Math.sin(robotAngle) + dSideways * Math.cos(robotAngle));
    robotY = (initY + dForward * Math.cos(robotAngle) + dSideways * Math.sin(robotAngle));
    initX = robotX;
    initY = robotY;
    oldAngle = robotAngle;
  }
  
  
  public void UpdateEncoders() {
    odometryPodX = 0;
    odometryPodY = 0;
    //the Y Odometry pod is plugged into port 2 which is the same port as the intake and its the same thing for the transfer
    double tempX = odometryPodX - odometryPodOldX;
    double tempY = odometryPodY - odometryPodOldY;
    
    odometryPodOldX = odometryPodX;
    odometryPodOldY = odometryPodY;
    odometryPodX = tempX;
    odometryPodY = tempY;
    
    odometryPodX = odometryPodX / ticksPerRevolution;
    odometryPodX = mmPerRevolution * odometryPodX;
    odometryPodX = odometryPodX / mmPerInch;
    odometryPodX *= correctionFactor;
    
    odometryPodY = odometryPodY / ticksPerRevolution;
    odometryPodY = mmPerRevolution * odometryPodY;
    odometryPodY = odometryPodY / mmPerInch;
    odometryPodY *= correctionFactor;
  }
  
}
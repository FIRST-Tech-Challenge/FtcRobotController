package org.firstinspires.ftc.teamcode.resourses;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//This class is used to track the robots position while the op-mode is running.
public class Odometry {
  // the IMU sensor in the control hub
  private IMU imu;
  
  //true if you are currently resetting the IMU so you don't use it while it is resetting
  private boolean resettingImu = false;
  // used to set the angle you start at during autonomous
  private double AutoStartAngle = 0;
  
  //the telemetry instance from the main op-mode thread
  private Telemetry telemetry;
  
  //constants for the unit conversions used in update encoders
  private final double ticksPerRevolution = 2000;
  private final double mmPerRevolution = 48 * Math.PI;
  private final double mmPerInch = 25.4;
  
  //the distance the odometry pods are away from the center of the robot so you can correct error when you turn.
  private final static double radiusX = 6.08;
  private final static double radiusY = 7.73886;
  
  // make this a static double if you want to use FTC dashboard for it. used incase odometry is slightly off.
  private final double correctionFactor = 1.0;
  
  // used in the odometry math to store the previous iterations robot position.
  private double initX, initY;
  private double odometryPodOldX = 0, odometryPodOldY = 0;
  private double odometryPodX = 0, odometryPodY = 0;
  
  //where the center of the robot is now
  private double robotX, robotY; // in inches
  private double robotAngle; // in radians
  
  // used in the odometry math to store the previous iteration of the robot angle.
  private double oldAngle;
  
  /**
   * the contructor for this class, used to set some variables
   * @param initX
   * @param initY
   * @param robotAngle
   * @param telemetry
   */
  public Odometry(double initX, double initY, double robotAngle, Telemetry telemetry, HardwareMap hardwareMap){
    this.initX = initX;
    this.initY = initY;
    robotX = initX;
    robotY = initY;
    this.robotAngle = robotAngle;
    oldAngle = robotAngle;
    this.telemetry = telemetry;
    imu = hardwareMap.get(IMU.class, "imu");
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    imu.initialize(new IMU.Parameters(orientationOnRobot));


  }
  
  // this does all of the math to recalculate where to robot is.
  public void updateOdometry() {
    //updates the sensor input before we use that data
    updateIMU();
    updateEncoders();
    
    // how much the robot has moved forward / sideways relative to the robot
    double dForward, dSideways;
    
    // the difference in how much we have rotated
    double deltaRotation = robotAngle - oldAngle;
    deltaRotation = Utlities.wrap(deltaRotation);
    
    // corrects for the error caused by the robot turning.
    dForward = odometryPodY - radiusY * deltaRotation;
    dSideways = odometryPodX - radiusX * deltaRotation;
    
    //calculates our new position
    robotX = (initX - dForward * Math.sin(robotAngle) + dSideways * Math.cos(robotAngle));
    robotY = (initY + dForward * Math.cos(robotAngle) + dSideways * Math.sin(robotAngle));
    
    // refreshes the old variables to reflect our new calculations.
    initX = robotX;
    initY = robotY;
    oldAngle = robotAngle;
  }
  
  // updates the yaw value from the IMU
  private void updateIMU() {
    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
    if (orientation.getRoll(AngleUnit.DEGREES) == 0 && orientation.getPitch(AngleUnit.DEGREES) == 0
            && orientation.getYaw(AngleUnit.DEGREES) == 0) {
      
      // In case of an IMU failure, we can re-init it to automatically fix it
      if (!resettingImu) {
        telemetry.addData("IMU failed?", "Re-initializing!");
        resettingImu = true;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
      }
    } else {
      resettingImu = false;
    }
    AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
    
    robotAngle = orientation.getYaw(AngleUnit.RADIANS);
    robotAngle += AutoStartAngle;
  }
  
  /**
   * for when you need to override the tracking data
   * @param x new input x in inches
   * @param y new input y in inches
   * @param angle new input angle in radians
   */
  public void overridePosition(double x, double y, double angle){
    robotX = x;
    robotY = y;
    robotAngle = angle;
  }
  
  //unit conversion to translate encoder ticks to inches of robot movement
  private void updateEncoders() {
    //TODO this is temp so you can assign the values from the odometry pods to these variables.
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
  
  public double getRobotAngle(){return robotAngle;}
  
  public double getRobotX(){return robotX;}
  
  public double getRobotY(){return robotY;}
  
  // used to check if you are currently resetting the IMU
  public boolean isResetingIMU(){
    return resettingImu;
  }
  
  // triggers an IMU Reset
  public void IMUReset() {
    imu.resetYaw();
  }
  


}
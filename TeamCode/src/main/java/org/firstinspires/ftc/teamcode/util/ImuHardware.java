package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ImuHardware
{
   LinearOpMode mOpMode;

   // The IMU sensor object
   IMU imu;

   double globalAngle;

   YawPitchRollAngles lastOrientation;

   public ImuHardware(LinearOpMode opmode)
   {
      mOpMode = opmode;

      globalAngle = 0;

      init();
   }

   public IMU getImu()
   {
      return imu;
   }

   void init()
   {
      // Retrieve and initialize the IMU.
      // This sample expects the IMU to be in a REV Hub and named "imu".
      imu = mOpMode.hardwareMap.get(IMU.class, "imu");

      /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
       *
       * Two input parameters are required to fully specify the Orientation.
       * The first parameter specifies the direction the printed logo on the Hub is pointing.
       * The second parameter specifies the direction the USB connector on the Hub is pointing.
       * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
       */

      /* The next two lines define Hub orientation.
       * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
       *
       * To Do:  EDIT these two lines to match YOUR mounting configuration.
       */
      RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
      RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

      RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

      // Now initialize the IMU with this mounting orientation
      // Note: if you choose two conflicting directions, this initialization will cause a code exception.
      imu.initialize(new IMU.Parameters(orientationOnRobot));

   }



   public void resetAngle()
   {
      //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      globalAngle = 0;
      lastOrientation = imu.getRobotYawPitchRollAngles();
   }

   public double getHeading()
   {
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

      return orientation.getYaw(AngleUnit.DEGREES);

   }

   /**
    * Get current cumulative angle rotation from last reset.
    * @return Angle in degrees. + = left, - = right from zero point.
    */
   public double getAngle()
   {
      // We experimentally determined the Z axis is the axis we want to use for heading angle.
      // We have to process the angle because the imu works in euler angles so the Z axis is
      // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
      // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

      // Retrieve Rotational Angles and Velocities
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

      //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      double yaw = orientation.getYaw(AngleUnit.DEGREES);

      double deltaAngle = yaw - lastOrientation.getYaw(AngleUnit.DEGREES);

      if (deltaAngle < -180)
         deltaAngle += 360;
      else if (deltaAngle > 180)
         deltaAngle -= 360;

      globalAngle += deltaAngle;

      lastOrientation = orientation;

      return globalAngle;
   }

}

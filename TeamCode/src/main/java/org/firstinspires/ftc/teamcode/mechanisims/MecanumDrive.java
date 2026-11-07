package org.firstinspires.ftc.teamcode.mechanisims;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareNames;

@Disabled
public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

     public MecanumDrive(HardwareMap hwMap) {
         frontLeftMotor = hwMap.get(DcMotor.class, HardwareNames.DRIVE_MOTOR_FRONT_LEFT);
         backLeftMotor = hwMap.get(DcMotor.class, HardwareNames.DRIVE_MOTOR_REAR_LEFT);
         frontRightMotor = hwMap.get(DcMotor.class, HardwareNames.DRIVE_MOTOR_FRONT_RIGHT);
         backRightMotor = hwMap.get(DcMotor.class, HardwareNames.DRIVE_MOTOR_REAR_RIGHT);

         backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
         frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

         frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         imu = hwMap.get(IMU.class, HardwareNames.IMU);

         RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                 RevHubOrientationOnRobot.LogoFacingDirection.UP,
                 RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

         imu.initialize(new IMU.Parameters(RevOrientation));

     }

     public void resetIMU () {
         imu.resetYaw();
     }
     public void drive(double forward, double strafe, double rotate) {
         double frontLeftPower = forward + strafe + rotate;
         double backLeftPower = forward - strafe + rotate;
         double frontRightPower = forward - strafe - rotate;
         double backRightPower = forward + strafe - rotate;

         double maxPower = 1.0;
         double maxSpeed = 1.0;

         maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
         maxPower = Math.max(maxPower, Math.abs(backLeftPower));
         maxPower = Math.max(maxPower, Math.abs(frontRightPower));
         maxPower = Math.max(maxPower, Math.abs(backRightPower));

         frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
         backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
         frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
         backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));

     }

     public void driveFieldRelative(double forward, double strafe, double rotate) {
         double theta = Math.atan2(forward, strafe);
         double r = Math.hypot(strafe, forward);

         theta = AngleUnit.normalizeRadians(theta -
                 imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

         double newForward = r * Math.sin(theta);
         double newStrafe = r * Math.cos(theta);

         this.drive(newForward, newStrafe, rotate);
     }

}
// lable up cord right
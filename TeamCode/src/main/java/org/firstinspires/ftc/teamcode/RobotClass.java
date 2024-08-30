package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drivetrain.AbstractDrivetrain;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.AbstractOmniDrivetrain;
import org.firstinspires.ftc.teamcode.Drivetrain.SixWheelDrivetrain;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotClass {
    public final AbstractOmniDrivetrain drivetrain;
    public final AbstractDrivetrain sixDrivetrain;

    public final IMU imu;
    public OpenCvWebcam camera1;

    public SparkFunOTOS opticalSensor;
    HardwareMap hwmap;

    public static final int kFrontLeft = 0;
    public static final int kBackLeft = 1;
    public static final int kBackRight = 2;
    public static final int kFrontRight = 3;

    public RobotClass(HardwareMap hwmap){
        this.hwmap = hwmap;

        imu = hwmap.get(IMU.class, "imu");

        drivetrain = new MecanumDrive(hwmap.get(DcMotor.class, "frontLeft"),
                                    hwmap.get(DcMotor.class, "backLeft"),
                                    hwmap.get(DcMotor.class, "backRight"),
                                    hwmap.get(DcMotor.class, "frontRight"));

        sixDrivetrain = new SixWheelDrivetrain(hwmap.get(DcMotor.class, "frontLeft"),
                                            hwmap.get(DcMotor.class, "backLeft"),
                                            hwmap.get(DcMotor.class, "backRight"),
                                            hwmap.get(DcMotor.class, "frontRight"));
        //opticalSensor = hwmap.get(SparkFunOTOS.class, "opticalSensor");
        //WebcamName camera1Name = hwmap.get(WebcamName.class, "Webcam 1");
        //camera1 = OpenCvCameraFactory.getInstance().createWebcam(camera1Name);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    public double getHeading(){
        Orientation Theta = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        return Theta.thirdAngle;

    }
    public void resetIMU(){
        imu.resetYaw();
    }

}

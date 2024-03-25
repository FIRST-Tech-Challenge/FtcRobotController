package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.AbstractOmniDrivetrain;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotClass {
    public final AbstractOmniDrivetrain drivetrain;

    public final IMU imu;
    public OpenCvWebcam camera1;
    HardwareMap hwmap;

    public RobotClass(HardwareMap hwmap){
        this.hwmap = hwmap;

        imu = hwmap.get(IMU.class, "imu");

        drivetrain = new MecanumDrive(hwmap.get(DcMotor.class, "frontLeft"),
                                    hwmap.get(DcMotor.class, "frontRight"),
                                    hwmap.get(DcMotor.class, "backLeft"),
                                    hwmap.get(DcMotor.class, "backRight"));

        WebcamName camera1Name = hwmap.get(WebcamName.class, "Webcam 1");
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(camera1Name);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double getHeading(){
        Orientation Theta = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return Theta.thirdAngle;
    }
    public void resetIMU(){
        imu.resetYaw();
    }

}

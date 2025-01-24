package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class IMU {

    BHI260IMU imu;

    public IMU(HardwareMap hardwareMap)
    {
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        imu.initialize(
                new com.qualcomm.robotcore.hardware.IMU.Parameters(
                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                                     RevHubOrientationOnRobot.UsbFacingDirection.FORWARD) ));
    }

    public void SetYaw()
    {
        imu.resetYaw();
    }

    public double getAngle() {

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();


        return robotOrientation.getYaw(AngleUnit.RADIANS);
    }

    public float getAngularVelocity(char angle) {

        AngularVelocity myRobotAngularVelocity;

        myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        float zRotationRate = myRobotAngularVelocity.zRotationRate;
        float xRotationRate = myRobotAngularVelocity.xRotationRate;
        //float yRotationRate = myRobotAngularVelocity.yRotationRate;


        if(angle == 'z')
            return zRotationRate;
        if(angle == 'x')
            return xRotationRate;
//        if(angle == 'y')   //robot is probably not flipping over so no need for it
//            return yRotationRate;
        else
            return 0;

    }

}

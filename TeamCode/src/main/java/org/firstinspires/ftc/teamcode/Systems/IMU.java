package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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

    public double getAngle(char angle) {

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
//        double pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        //double roll  = robotOrientation.getRoll(AngleUnit.DEGREES); // robot will most likely not flip over

        if(angle == 'y')
            return yaw;
//        if(angle == 'p')
//            return pitch;
//        if(angle == 'r')   //robot is probably not flipping over so no need for roll or pitch
//            return roll;
        else
            return 0;

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

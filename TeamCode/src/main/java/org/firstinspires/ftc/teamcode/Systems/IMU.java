package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class IMU {

    BHI260IMU imu;

    public IMU()
    {
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        imu.initialize(
                new com.qualcomm.robotcore.hardware.IMU.Parameters(
                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                                     RevHubOrientationOnRobot.UsbFacingDirection.FORWARD) ));
    }

    public double getAngle(double angle) {

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        //double roll  = robotOrientation.getRoll(AngleUnit.DEGREES); // robot will most likely not flip over

        if(angle == 0)
            return yaw;
        if(angle == 1)
            return pitch;
//        if(angle == 2)   //robot is probably not flipping over so no need for roll
//            return roll;
        else
            return 0;

    }

    public float getAngularVelocity(int angle) {

        AngularVelocity myRobotAngularVelocity;

        myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        float zRotationRate = myRobotAngularVelocity.zRotationRate;
        float xRotationRate = myRobotAngularVelocity.xRotationRate;
        //float yRotationRate = myRobotAngularVelocity.yRotationRate;


        if(angle == 0)
            return zRotationRate;
        if(angle == 1)
            return xRotationRate;
//        if(angle == 2)   //robot is probably not flipping over so no need for roll
//            return yRotationRate;
        else
            return 0;

    }

}

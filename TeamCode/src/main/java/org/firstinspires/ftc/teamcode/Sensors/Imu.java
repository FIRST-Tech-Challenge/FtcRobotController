package org.firstinspires.ftc.teamcode.hardware.Sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Imu{
    private int count = 0;
    private double currentAngle;
    private double lastAngle;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections = RevHubOrientationOnRobot.UsbFacingDirection.values();

    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    private YawPitchRollAngles angles;
    private IMU imu;

    public Imu(IMU imu){
        this.imu = imu;
    }

    public void init(){
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void update(){
       angles = this.imu.getRobotYawPitchRollAngles();
    }

    //degrees
    public double getYawR(){
        return angles.getYaw(AngleUnit.DEGREES);
    }

    public double getPitchR(){
        return angles.getPitch(AngleUnit.DEGREES);
    }

    public double getRollR(){
        return angles.getRoll(AngleUnit.DEGREES);
    }

    public double updateAngleWrapped(){
        currentAngle = getYawR(); //or getPitchR or getRollR
        if(Math.abs(currentAngle - lastAngle) > 180){
            count += Math.signum(lastAngle - currentAngle);
        }
        lastAngle = currentAngle;
        return count * 360 + currentAngle;
    }
}

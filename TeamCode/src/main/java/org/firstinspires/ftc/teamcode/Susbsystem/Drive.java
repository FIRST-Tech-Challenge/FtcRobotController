package org.firstinspires.ftc.teamcode.Susbsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;

public class Drive {

    public static double[] wheelSpeeds = new double[4];

    public static void DriveCartesian(double rotX, double rotY, double rotation){

        wheelSpeeds[RobotClass.kFrontLeft] = (rotY + rotX + rotation);
        wheelSpeeds[RobotClass.kBackLeft] = (rotY - rotX + rotation);
        wheelSpeeds[RobotClass.kFrontRight] = (rotY - rotX - rotation);
        wheelSpeeds[RobotClass.kBackRight] = (rotY + rotX - rotation);
    }

    public static double[] getWheelSpeeds(){
        return wheelSpeeds;
    }
    public static double getWheelSpeed(int wheelId){
        return (int) wheelSpeeds[wheelId];
    }
    public static double[] correctDrift(double[] wheelSpeeds, Telemetry telemetry, double rotationRate){ //milliseconds

        if(rotationRate < -0.1){
            wheelSpeeds[RobotClass.kBackRight] *= (1 + rotationRate);
            wheelSpeeds[RobotClass.kBackLeft] *= (1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontLeft] *= -(1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontRight] *= -(1 + rotationRate);

        }
        if(rotationRate > 0.1){
            wheelSpeeds[RobotClass.kBackRight] *= -(1 + rotationRate);
            wheelSpeeds[RobotClass.kBackLeft] *= -(1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontLeft] *= (1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontRight] *= (1 + rotationRate);

        }

        telemetry.addData("changeInHeadingPerMillisecond", rotationRate);
        return wheelSpeeds;
    }

}

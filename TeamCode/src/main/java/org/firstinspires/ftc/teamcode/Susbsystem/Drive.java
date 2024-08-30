package org.firstinspires.ftc.teamcode.Susbsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;

public class Drive extends RobotClass {
    private static final ElapsedTime stopWatch = new ElapsedTime();
    public Drive(HardwareMap hwmap) {
        super(hwmap);
    }

    public final double[] wheelSpeeds = new double[4];
    public void DriveCartesian(double rotX, double rotY, double rotation){
        wheelSpeeds[kFrontLeft] = (rotY + rotX + rotation);
        wheelSpeeds[kBackLeft] = (rotY - rotX + rotation);
        wheelSpeeds[kFrontRight] = (rotY - rotX - rotation);
        wheelSpeeds[kBackRight] = (rotY + rotX - rotation);
    }

    public double[] getWheelSpeeds(){
        return wheelSpeeds;
    }
    public double getWheelSpeed(int wheelId){
        return (int) wheelSpeeds[wheelId];
    }
    public double[] correctDrift(double[] wheelSpeeds, double oldHeading, double oldTime, Telemetry telemetry){ //milliseconds

        double changeInHeadingPerMillisecond = (getHeading() - oldHeading) / (stopWatch.milliseconds() - oldTime);
        stopWatch.reset();
        if(changeInHeadingPerMillisecond < -0.1){
            wheelSpeeds[kBackRight] *= (1 + changeInHeadingPerMillisecond);
            wheelSpeeds[kBackLeft] *= (1 + changeInHeadingPerMillisecond);
            wheelSpeeds[kFrontLeft] *= -(1 + changeInHeadingPerMillisecond);
            wheelSpeeds[kFrontRight] *= -(1 + changeInHeadingPerMillisecond);

        }
        if(changeInHeadingPerMillisecond > 0.1){
            wheelSpeeds[kBackRight] *= -(1 + changeInHeadingPerMillisecond);
            wheelSpeeds[kBackLeft] *= -(1 + changeInHeadingPerMillisecond);
            wheelSpeeds[kFrontLeft] *= (1 + changeInHeadingPerMillisecond);
            wheelSpeeds[kFrontRight] *= (1 + changeInHeadingPerMillisecond);

        }

        telemetry.addData("changeInHeadingPerMillisecond", changeInHeadingPerMillisecond);
        return wheelSpeeds;

    }
}

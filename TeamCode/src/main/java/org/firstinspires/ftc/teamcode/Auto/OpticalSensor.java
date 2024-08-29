package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;

public class OpticalSensor extends AutoControl {
    RobotClass robot;
    public OpticalSensor(RobotClass robot){
        this.robot = robot;
    }

    public SparkFunOTOS.Pose2D getPos(){
        return robot.opticalSensor.getPosition();
    }
}

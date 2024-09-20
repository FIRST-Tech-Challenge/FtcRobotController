package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;

public class OpticalSensor extends AutoControl {
    RobotClass robot;
    SparkFunOTOS opticalSensor;
    public OpticalSensor(RobotClass robot){
        this.robot = robot;
        opticalSensor = robot.opticalSensor;
        configureOtos();
    }

    public SparkFunOTOS.Pose2D getPos(){
        return opticalSensor.getPosition();

    }
    private void configureOtos(){
        opticalSensor.setLinearUnit(DistanceUnit.INCH);
        opticalSensor.setAngularUnit(AngleUnit.DEGREES);

        //set offset from center of robot IF sensor is not centered
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0,0);
        opticalSensor.setOffset(offset);

        SparkFunOTOS.Pose2D startingPos = new SparkFunOTOS.Pose2D(0,0,0);
        opticalSensor.setPosition(startingPos);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();

        opticalSensor.setAngularScalar(1.0);
        opticalSensor.setLinearScalar(1.0);

        opticalSensor.getVersionInfo(hwVersion, fwVersion);

    }
    
}

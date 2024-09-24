package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.OpticalSensor;
import org.firstinspires.ftc.teamcode.Camera.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.Susbsystem.AutoUtils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

@Autonomous
public class AutoControl extends OpMode{
    public RobotClass robot;
    AprilTagPipeline aprilTagPipeline;
    boolean stop = false;
    AutoUtils autoUtils;
    @Override
    public void init(){
        robot = new RobotClass(hardwareMap);
        autoUtils = new AutoUtils(robot);
        robot.stopAndReset();
        robot.setDirection();
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //aprilTagPipeline = new AprilTagPipeline(robot.webcamName, telemetry);
        //opticalSensor = new OpticalSensor(drive);
    }

    @Override
    public void start(){
        autoUtils.AutoTurn(30, telemetry);
        autoUtils.AutoDrive(20,90);
        autoUtils.AutoTurn(150, telemetry);
        autoUtils.AutoDrive(20,90);
        autoUtils.AutoTurn(270, telemetry);
        autoUtils.AutoDrive(20,90);
        autoUtils.AutoTurn(0, telemetry);


        stop();
    }

    @Override
    public void loop(){
        //aprilTagPipeline.updateAprilTagPipeline();
    }

    @Override
    public void stop(){
        boolean stop = true;
        if(robot == null) return; // ensures that stop() is not called before initialization
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0.0);
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0.0);
        requestOpModeStop();
    }



}
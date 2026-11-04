package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DepollenaterSubsystem {
    Telemetry telemetry;
    Servo Depollenater;

    public static double downPose = .58;
    public DepollenaterSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        Depollenater = hardwareMap.get(Servo.class,"De-pollenater");
    }

    public void depollenaterUp(){
        Depollenater.setPosition(1);
    }

    public void DepollenaterDown(){
        Depollenater.setPosition(downPose);
    }
}

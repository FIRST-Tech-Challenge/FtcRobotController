package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class DroneLauncher implements Subsystem {
    private Servo droneTrigger;
    Telemetry telemetry;
    private double holdPos = 1;
    private double releasePos = 0.1;

    public DroneLauncher(Robot robot, Telemetry telemetry) {
        this.telemetry = telemetry;
        droneTrigger = robot.getServo("droneTrigger");
        hold();
    }
    public void hold(){
        droneTrigger.setPosition(holdPos);
    }
    public void release(){
        droneTrigger.setPosition(releasePos);
    }

    public void update(TelemetryPacket packet){

    }
}

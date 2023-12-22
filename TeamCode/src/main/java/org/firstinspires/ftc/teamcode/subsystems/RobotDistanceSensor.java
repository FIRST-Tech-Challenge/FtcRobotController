package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class RobotDistanceSensor implements Subsystem {
    private DistanceSensor DistanceSensorL;
    private DistanceSensor DistanceSensorR;
    private Telemetry telemetry;
    public double dsL;
    public double dsR;
    private double threashold = 0.65;

    public RobotDistanceSensor(Robot robot, Telemetry telemetry) {
        DistanceSensorL = robot.hardwareMap.get(DistanceSensor.class, "LDist");
        DistanceSensorR = robot.hardwareMap.get(DistanceSensor.class,"RDist");
        this.telemetry = telemetry;
        dsL = 0;
        dsR = 0;

    }
    public double distanceLeft() {
        return DistanceSensorL.getDistance(DistanceUnit.CM);
    }
    public double distanceRight(){
        return DistanceSensorR.getDistance(DistanceUnit.CM);
    }

    @Override
    public void update(TelemetryPacket packet) {

        dsL = distanceLeft();
        dsR = distanceRight();
    }
}
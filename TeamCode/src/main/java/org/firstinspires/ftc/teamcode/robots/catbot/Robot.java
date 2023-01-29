package org.firstinspires.ftc.teamcode.robots.catbot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Robot {
    Telemetry telemetry;
    DriveTrain driveTrain;
    ElevatorNClaw elevatorNClaw;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry=telemetry;
        driveTrain = new DriveTrain(telemetry, hardwareMap);
        elevatorNClaw = new ElevatorNClaw(telemetry, hardwareMap);
        motorInit();
    }
    public void motorInit()
    {
        driveTrain.motorInit();
        elevatorNClaw.motorInit();
    }
}

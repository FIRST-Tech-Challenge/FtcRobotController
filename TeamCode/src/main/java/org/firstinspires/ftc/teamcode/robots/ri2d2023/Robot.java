package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Robot {
    Telemetry telemetry;
    DriveTrain driveTrain;
    Intake intake;
    Outtake outtake;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry=telemetry;
        driveTrain = new DriveTrain(telemetry, hardwareMap);
        intake = new Intake(telemetry, hardwareMap);
        outtake = new Outtake(telemetry, hardwareMap);
        init();
    }
    public void init()
    {
        driveTrain.motorInit();
        intake.init();
        outtake.init();
    }
}

package com.parshwa.drive.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface DriveInterface {
    void move(double froward, double strafe, double rotate, double speed);
    void move(double froward, double strafe, double rotate);
    void init(HardwareMap hardwareMap, Telemetry telemetry, DriveModes drivermode);
    void init(HardwareMap hardwareMap, Telemetry telemetry);
}

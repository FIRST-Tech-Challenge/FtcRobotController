package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.settings.DeviceNames;

public class Robot {
    // Gamepads
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    
    // Telemetry
    public Telemetry telemetry;
    
    // Hardware Map
    public HardwareMap hardwareMap;
    
    // Drive Train
    public DriveTrain driveTrain;

    // Device Names
    public DeviceNames deviceNames = new DeviceNames();

    public Robot(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
}

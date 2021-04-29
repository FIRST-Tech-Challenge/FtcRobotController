package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.drivetrain.wheels.WheelTypes;
import org.firstinspires.ftc.teamcode.settings.DeviceNames;

public class Robot {
    // Gamepads
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    
    // Telemetry
    public Telemetry telemetry;
    private boolean useTelemetry;
    
    // Hardware Map
    public HardwareMap hardwareMap;
    
    // Drive Train
    public DriveTrain driveTrain;

    // Device Names
    public DeviceNames deviceNames = new DeviceNames();

    // Time
    public ElapsedTime timeSinceInstantiation = new ElapsedTime();

    // Status
    public RobotStatus status;

    public Robot(boolean useTelemetry, WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.useTelemetry = useTelemetry;

        // Initializing
        status = RobotStatus.INITIALIZING;
        if (useTelemetry) telemetry.addData("Robot Status","Initializing");

        driveTrain = new DriveTrain(false, wheelType, this);


        // Initialized
        status = RobotStatus.INITIALIZED;
        if (useTelemetry) telemetry.addData("Robot Status","Initialized");
    }

    public Robot(WheelTypes wheelType, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this(true, wheelType, gamepad1,gamepad2,telemetry,hardwareMap);
    }

    public void start() {
        status = RobotStatus.RUNNING;
        if (useTelemetry) telemetry.addData("Robot Status","Running");
    }

    public void stop() {
        // Stopping
        status = RobotStatus.STOPPING;
        if (useTelemetry) telemetry.addData("Robot Status","Stopping");

        driveTrain.stop();

        // Stopped
        status = RobotStatus.STOPPED;
        if (useTelemetry) telemetry.addData("Robot Status","Stopped");
    }

    public void setDrivePowerModifier(double powerModifier) {
        driveTrain.setPowerModifier(powerModifier);
    }
}

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPart;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPartSettings;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.DriveTrainSettings;
import org.firstinspires.ftc.teamcode.settings.DeviceNames;

import java.util.ArrayList;

/**
 * The entirety of the robot. Has many subsystems called "Robot Parts"
 * @see RobotPart
 * @see RobotPartSettings
 * @author 22jmiller
 */
public class Robot {
    // Gamepads
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    
    // Telemetry
    public Telemetry telemetry;
    private boolean useTelemetry;
    
    // Hardware Map
    public HardwareMap hardwareMap;
    
    // Robot Parts
    public ArrayList<RobotPart> robotParts;

    // Device Names
    public static final DeviceNames deviceNames = new DeviceNames();

    // Time
    public ElapsedTime timeSinceInstantiation = new ElapsedTime();

    // Status
    public RobotStatus status;

    /**
     *
     * @param useTelemetry Use telemetry?
     * @param robotPartSettings List of settings for robot parts, these settings will create robot parts
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     * @param telemetry Telemetry
     * @param hardwareMap Hardware Map
     */
    public Robot(boolean useTelemetry, ArrayList<RobotPartSettings> robotPartSettings, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.useTelemetry = useTelemetry;

        robotParts = new ArrayList<RobotPart>();

        // Initializing
        status = RobotStatus.INITIALIZING;
        if (useTelemetry) telemetry.addData("Robot Status","Initializing");

        // -- Start of Initialization --
        for (RobotPartSettings robotPartSetting : robotPartSettings) {
            robotParts.add(robotPartSetting.create(this));
        }

        // Initialized
        status = RobotStatus.INITIALIZED;
        if (useTelemetry) telemetry.addData("Robot Status","Initialized");
    }

    public Robot(ArrayList<RobotPartSettings> robotPartSettings, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this(true, robotPartSettings, gamepad1,gamepad2,telemetry,hardwareMap);
    }

    public void start() {
        for (RobotPart robotPart : robotParts) {
            robotPart.start();
        }

        status = RobotStatus.RUNNING;
        if (useTelemetry) telemetry.addData("Robot Status","Running");
    }

    public void stop() {
        // Stopping
        status = RobotStatus.STOPPING;
        if (useTelemetry) telemetry.addData("Robot Status","Stopping");

        for (RobotPart robotPart : robotParts) {
            robotPart.stop();
        }

        // Stopped
        status = RobotStatus.STOPPED;
        if (useTelemetry) telemetry.addData("Robot Status","Stopped");
    }

    public RobotPart getRobotPart(Class className) {
        for (RobotPart robotPart : robotParts) {
            if (className.isAssignableFrom(robotPart.getClass())) return robotPart;
        }
        return null;
    }

    public boolean hasRobotPart(Class className) {
        if (getRobotPart(className) != null) return true;
        return false;
    }

    public RobotPart getRobotPart(int pos) {
        return robotParts.get(pos);
    }
}

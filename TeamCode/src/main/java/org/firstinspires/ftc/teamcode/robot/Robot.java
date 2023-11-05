package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.CachingServo;
import org.firstinspires.ftc.teamcode.hardware.Encoder;

import java.util.ArrayList;
import java.util.List;

public class Robot {

    public static final String TAG = "Robot";

    public HardwareMap hardwareMap; //TODO: change back to private
    private LinearOpMode opMode;

    private List<Subsystem> subsystems;
    private List<Listener> listeners;
    private List<Command> commands;

    private List<LynxModule> hubs;

    private FtcDashboard dashboard;

    private boolean stopped = false;

    public interface Listener {
        void update();
    }

    public interface Target {
        boolean reached();
    }

    public Robot (LinearOpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        subsystems = new ArrayList<>();
        listeners = new ArrayList<>();
        commands = new ArrayList<>();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        dashboard = FtcDashboard.getInstance();
    }

    public void update() {
        for (LynxModule module : hubs) {
            module.clearBulkCache();
        }

        TelemetryPacket packet = new TelemetryPacket();

        for (Subsystem subsystem : subsystems) {
            subsystem.update(packet);
        }

        for (Listener listener : listeners) {
            listener.update();
        }

        ArrayList<Command> commandsToRemove = new ArrayList<>();
        for (Command command : commands) {
            command.update();
            if (command.isCompleted()) {
                command.stop();
                commandsToRemove.add(command);
            }
        }
        commands.removeAll(commandsToRemove);

        dashboard.sendTelemetryPacket(packet);
    }

    public void runUntil(Target target) {
        while (!opMode.isStopRequested() && !target.reached() && !stopped) {
            update();
        }
    }

    public void runUntilStop() {
        runUntil(() -> false);
    }

    public void runUntilCommandsFinished() {
        runUntil(() -> commands.isEmpty());
    }

    public void stop() {
        stopped = true;
        for (Command command: commands) {
            command.stop();
            commands.remove(command);
        }
    }

    public DcMotorEx getMotor(String deviceName) {
        CachingDcMotorEx motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, deviceName));
        listeners.add(motor);
        return motor;
    }

    public Servo getServo(String deviceName) {
        CachingServo servo = new CachingServo(hardwareMap.get(Servo.class, deviceName));
        listeners.add(servo);
        return servo;
    }

    public VoltageSensor getVoltageSensor() {
        return hardwareMap.voltageSensor.iterator().next();
    }
    /*
    public DistanceSensor getDistanceSensor(String deviceName) {
        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, deviceName);
        //listeners.add(sensor);
        return sensor;
    }
    */
    public Encoder getEncoder(String deviceName) {
        return new Encoder(getMotor(deviceName));
    }

    public BNO055IMU getIMU(String deviceName) {
        return hardwareMap.get(BNO055IMU.class, deviceName);
    }

    public AnalogSensor getAnalogSensor(String deviceName) {
        return hardwareMap.get(AnalogSensor.class, deviceName);
    }

    public void registerSubsystem(Subsystem subsystem) {
        if (!subsystems.contains(subsystem)) subsystems.add(subsystem);
    }

    public void addListener(Listener listener) {
        if (!listeners.contains(listener)) listeners.add(listener);
    }

    public void addCommand(Command command) {
        addCommands(command);
    }

    public void addCommands(Command... commands) {
        for (Command command : commands) {
            this.commands.add(command);
            command.start();
        }
    }

    public void runCommand(Command command) {
        addCommand(command);
        runUntilCommandsFinished();
    }

    public void runCommands(Command... commands) {
        addCommands(commands);
        runUntilCommandsFinished();
    }

}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opMode.templates.OpModeTemplate;
import org.firstinspires.ftc.teamcode.util.TeamColor;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

public class RobotController {
    public final GamepadEx driverController;
    public final GamepadEx actionController;

    private final HashMap<Class<? extends Subsystem>, Subsystem> subsystemHashMap = new HashMap<>();
    private final OpModeTemplate opModeTemplate;

    private final HardwareMap hardwareMap;
    private final MultipleTelemetry telemetry;
    private final TeamColor teamColor;

    public RobotController(Class<? extends OpModeTemplate> opModeTemplateClazz, HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor) {
        this.hardwareMap = hMap;
        this.teamColor = teamColor;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.driverController = new GamepadEx(driverController);
        this.actionController = new GamepadEx(actionController);

        FtcDashboard.getInstance().stopCameraStream();
        CommandScheduler.getInstance().reset();

        try {
            Constructor<? extends OpModeTemplate> opModeTemplateConstructor = opModeTemplateClazz.getDeclaredConstructor();
            opModeTemplateConstructor.setAccessible(true);
            this.opModeTemplate = opModeTemplateConstructor.newInstance();
        } catch (NoSuchMethodException | InvocationTargetException | IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
    }

    public void initSubSystems() {
        for(Class<? extends Subsystem> subsystem : this.opModeTemplate.getSubsystems()) {
            try {
                subsystemHashMap.put(subsystem, subsystem.getDeclaredConstructor(HardwareMap.class).newInstance(this.hardwareMap));
            } catch (InvocationTargetException | IllegalAccessException | InstantiationException | NoSuchMethodException e) {
                telemetry.addData("RobotController", String.format("failed to initialize the '%s' subsystem.", subsystem.getSimpleName()));
            }
        }

        this.opModeTemplate.createControls(this.driverController, this.actionController, this);

        telemetry.update();

        this.opModeTemplate.init(this);
    }

    public <T extends Subsystem> T getSubsystem(Class<T> clazz) {
        return (T) this.subsystemHashMap.get(clazz);
    }

    public TeamColor getTeamColor() {
        return teamColor;
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }
}

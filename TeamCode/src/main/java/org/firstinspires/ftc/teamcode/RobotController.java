package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystem;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

import top.symple.symplegraphdisplay.SympleGraphDisplay;

public class RobotController {
    public final GamepadEx driverController;
    public final GamepadEx actionController;

    private final HashMap<Class<? extends SympleSubsystem>, SympleSubsystem> subsystemHashMap = new HashMap<>();
    private final OpModeTemplate opModeTemplate;

    private final HardwareMap hardwareMap;
    private final MultipleTelemetry telemetry;
    private final TeamColor teamColor;
    private final DataLogger dataLogger;
    private final RobotPositionManager robotPositionManager;

    public RobotController(Class<? extends OpModeTemplate> opModeTemplateClazz, HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor, boolean logData) {
        this.hardwareMap = hMap;
        this.teamColor = teamColor;
        this.robotPositionManager = new RobotPositionManager(this);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.dataLogger = new DataLogger(opModeTemplateClazz.getSimpleName(), !logData);

        this.dataLogger.addData(DataLogger.DataType.INFO, String.format("RobotController: starting on the '%s' team, with '%s' as the op mode template", teamColor.name(), opModeTemplateClazz.getSimpleName()));

        this.driverController = new GamepadEx(driverController);
        this.actionController = new GamepadEx(actionController);

        this.dataLogger.addData(DataLogger.DataType.INFO, "RobotController: resetting robot");
        FtcDashboard.getInstance().stopCameraStream();
        SympleGraphDisplay.getInstance().reset();
        SympleGraphDisplay.getInstance().setUpdateTime(0.05);
        CommandScheduler.getInstance().reset();

        this.dataLogger.addData(DataLogger.DataType.INFO, String.format("RobotController: trying to create the op template (%s)", opModeTemplateClazz.getSimpleName()));
        try {
            Constructor<? extends OpModeTemplate> opModeTemplateConstructor = opModeTemplateClazz.getDeclaredConstructor();
            opModeTemplateConstructor.setAccessible(true);
            this.opModeTemplate = opModeTemplateConstructor.newInstance();
        } catch (NoSuchMethodException | InvocationTargetException | IllegalAccessException | InstantiationException e) {
            this.dataLogger.addData(DataLogger.DataType.ERROR, "RobotController: failed to create op mode template");
            this.dataLogger.addThrowable(e);
            throw new RuntimeException(e);
        }
    }

    public void initSubSystems() {
        this.dataLogger.addData(DataLogger.DataType.INFO, "RobotController: Starting to initialize subsystems");
        for(Class<? extends SympleSubsystem> subsystem : this.opModeTemplate.getSubsystems()) {
            this.dataLogger.addData(DataLogger.DataType.INFO, "RobotController: initializing '" + subsystem.getSimpleName() + "' sub system");
            try {
                subsystemHashMap.put(subsystem, subsystem.getDeclaredConstructor(RobotController.class).newInstance(this));
            } catch (InvocationTargetException | IllegalAccessException | InstantiationException | NoSuchMethodException e) {
                String err = String.format("failed to initialize the '%s' sub system.", subsystem.getSimpleName());
                telemetry.addData("RobotController", err);
                this.dataLogger.addData(DataLogger.DataType.WARN, "RobotController: " + err);
                this.dataLogger.addThrowable(e);
            }
        }

        this.dataLogger.addData(DataLogger.DataType.INFO, "RobotController: creating controls");
        this.opModeTemplate.createControls(this.driverController, this.actionController, this);

        telemetry.update();

        this.opModeTemplate.init(this);
    }

    @SuppressWarnings("unchecked")
    public <T extends SympleSubsystem> T getSubsystem(Class<T> clazz) {
        return (T) this.subsystemHashMap.get(clazz);
    }

    public void run() {
        SympleGraphDisplay.getInstance().run();
    }

    public TeamColor getTeamColor() {
        return teamColor;
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public DataLogger getDataLogger() {
        return dataLogger;
    }

    public RobotPositionManager getRobotPositionManager() {
        return robotPositionManager;
    }
}

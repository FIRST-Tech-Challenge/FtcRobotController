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
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

import top.symple.symplegraphdisplay.GraphSettings;
import top.symple.symplegraphdisplay.SympleGraphDisplay;

public abstract class RobotControllerBase {
    private static final GraphSettings DEFAULT_SYMPLE_GRAPH_DISPLAY_SETTINGS = new GraphSettings();

    public final GamepadEx driverController;
    public final GamepadEx actionController;

    private final HardwareMap hardwareMap;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public RobotControllerBase(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData) {
        this.hardwareMap = hMap;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.dataLogger = new DataLogger(logFilePrefix, !logData);

        this.dataLogger.addData(DataLogger.DataType.INFO, "RobotController: Initializing...");

        this.driverController = new GamepadEx(driverController);
        this.actionController = new GamepadEx(actionController);

        // Reset the robot state
        this.dataLogger.addData(DataLogger.DataType.INFO, "RobotController: resetting robot");
        FtcDashboard.getInstance().stopCameraStream();
        SympleGraphDisplay.getInstance().reset();
        SympleGraphDisplay.getInstance().setSetting(DEFAULT_SYMPLE_GRAPH_DISPLAY_SETTINGS);
        CommandScheduler.getInstance().reset();
        RobotPositionManager.init(hardwareMap);
    }

    public abstract void createKeyBindings();
    public abstract void initialize();
    public abstract void initializeLoop();
    public abstract void postInitialize();
    public abstract void run();
    public abstract void postRun();

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public DataLogger getDataLogger() {
        return dataLogger;
    }

    protected static abstract class Builder {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        protected Gamepad driverController;
        protected Gamepad actionController;
        protected String logFilePrefix;
        protected boolean logData;

        public Builder() {
            this.logData = true;
            this.logFilePrefix = "RobotController";
        }

        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            this.hardwareMap = opMode.hardwareMap;
            this.telemetry = opMode.telemetry;
            this.driverController = opMode.gamepad1;
            this.actionController = opMode.gamepad2;
            return this;
        }

        public abstract RobotControllerBase build();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.MecanumArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class TeleOpRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;

    private TeleOpRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor, String logFilePrefix, boolean logData) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        if(teamColor == null) {
            RuntimeException exception = new RuntimeException("Team color cannot be null!");
            this.getDataLogger().addThrowable(exception);
            throw exception;
        }

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
    }

    @Override
    public void createKeyBindings() {

    }

    @Override
    public void initialize() {
        this.mecanumDriveSubsystem.setDefaultCommand(new MecanumArcadeDriveCommand(this.mecanumDriveSubsystem, this.driverController));
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {

    }

    @Override
    public void run() {

    }

    @Override
    public void postRun() {

    }

    public static class Builder extends RobotControllerBase.Builder {
        private TeamColor teamColor;

        public Builder() {
            this.logFilePrefix = "TeleOp";
        }

        public Builder teamColor(TeamColor value) {
            this.teamColor = value;
            return this;
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        @Override
        public TeleOpRobotController build() {
            return new TeleOpRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.teamColor, this.logFilePrefix, this.logData);
        }
    }
}

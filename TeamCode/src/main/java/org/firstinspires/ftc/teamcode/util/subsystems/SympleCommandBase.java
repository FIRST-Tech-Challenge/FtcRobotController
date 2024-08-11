package org.firstinspires.ftc.teamcode.util.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.util.DataLogger;

public class SympleCommandBase<T extends SympleSubsystem> extends CommandBase {
    protected final T subsystem;

    protected SympleCommandBase(T subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getDataLoggerPrefix() + ": command started");
    }

    @Override
    public void end(boolean interrupted) {
        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getDataLoggerPrefix() + ": command ended");
    }

    protected MultipleTelemetry getTelemetry() {
        return subsystem.getTelemetry();
    }

    protected DataLogger getDataLogger() {
        return subsystem.getDataLogger();
    }

    protected String getDataLoggerPrefix() {
        return this.getClass().getSimpleName() + ": ";
    }
}

package org.firstinspires.ftc.teamcode.util.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.DataLogger;

public class SympleSubSystemBase extends SubsystemBase implements SympleSubsystem {
    protected final RobotController robotController;
    protected SympleSubSystemBase(RobotController robotController) {
        this.robotController = robotController;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return this.robotController.getTelemetry();
    }

    @Override
    public DataLogger getDataLogger() {
        return this.robotController.getDataLogger();
    }
}
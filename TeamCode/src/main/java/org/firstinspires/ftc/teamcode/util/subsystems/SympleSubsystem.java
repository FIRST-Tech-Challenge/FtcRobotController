package org.firstinspires.ftc.teamcode.util.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.DataLogger;

public interface SympleSubsystem extends Subsystem {
    MultipleTelemetry getTelemetry();
    DataLogger getDataLogger();
}

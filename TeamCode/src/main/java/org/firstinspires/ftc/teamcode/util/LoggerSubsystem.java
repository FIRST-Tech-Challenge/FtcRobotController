package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public interface LoggerSubsystem {
    MultipleTelemetry getTelemetry();
    DataLogger getDataLogger();
}

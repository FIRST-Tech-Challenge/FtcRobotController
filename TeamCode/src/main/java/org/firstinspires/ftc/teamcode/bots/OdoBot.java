package org.firstinspires.ftc.teamcode.bots;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;

public interface OdoBot {
    BotCalibConfig getCalibConfig();
    double getLeftOdometer();
    double getRightOdometer();
    Telemetry getTelemetry();
}

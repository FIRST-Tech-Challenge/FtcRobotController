package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.datalogger.HydraDatalogger;

public class HydraOpMode {
    public Telemetry mTelemetry;
    public HardwareMap mHardwareMap;
    public HydraDatalogger mLogger;
    public HydraOpMode(Telemetry telemetry, HardwareMap hardwareMap, HydraDatalogger logger) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mLogger = logger;
    }
}

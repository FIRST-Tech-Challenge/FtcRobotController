package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.datalogger.HydraDriveDatalogger;
import org.firstinspires.ftc.teamcode.datalogger.HydraObjDetDatalogger;

public class HydraOpMode {
    public Telemetry mTelemetry;
    public HardwareMap mHardwareMap;
    public HydraDriveDatalogger mDriveLogger;
    public HydraObjDetDatalogger mObjLogger;
    public HydraOpMode(Telemetry telemetry, HardwareMap hardwareMap, HydraDriveDatalogger driveLogger,
                       HydraObjDetDatalogger objDetDatalogger) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mDriveLogger = driveLogger;
        mObjLogger = objDetDatalogger;
    }
}

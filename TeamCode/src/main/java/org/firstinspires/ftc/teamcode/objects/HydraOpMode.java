package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HydraOpMode {
    public Telemetry mTelemetry;
    public HardwareMap mHardwareMap;
    public HydraOpMode(Telemetry telemetry, HardwareMap hardwareMap) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
    }
}

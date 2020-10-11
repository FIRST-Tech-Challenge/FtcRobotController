package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface SkystoneVisionProvider {

    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry, boolean enableTelemetry, Viewpoint viewpoint, boolean redAlliance);

    public void shutdownVision();

    public SkystoneTargetInfo detectSkystone();

    public void reset();
}

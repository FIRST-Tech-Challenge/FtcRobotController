package org.firstinspires.ftc.teamcode.vision.dummy;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.SkystoneTargetInfo;
import org.firstinspires.ftc.teamcode.vision.SkystoneVisionProvider;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;

public abstract class AbstractDummyVisionIntegration implements VisionProvider, SkystoneVisionProvider {

    @Override
    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry, boolean enableTelemetry, Viewpoint viewpoint) {

    }

    @Override
    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry, boolean enableTelemetry, Viewpoint viewpoint, boolean redAlliance) {

    }

    @Override
    public abstract GoldPos detect();

    @Override
    public abstract SkystoneTargetInfo detectSkystone();

    @Override
    public void shutdownVision() {}

    @Override
    public void reset() {}

}

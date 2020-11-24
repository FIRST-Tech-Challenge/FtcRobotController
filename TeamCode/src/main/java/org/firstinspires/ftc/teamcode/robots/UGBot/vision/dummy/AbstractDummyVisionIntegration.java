package org.firstinspires.ftc.teamcode.robots.UGBot.vision.dummy;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProvider;

public abstract class AbstractDummyVisionIntegration implements VisionProvider {
    @Override
    public void initializeVision(HardwareMap hardwareMap, Viewpoint viewpoint) {

    }

    @Override
    public abstract StackHeight detect();

    @Override
    public void shutdownVision() {}

    @Override
    public void reset() {}

}

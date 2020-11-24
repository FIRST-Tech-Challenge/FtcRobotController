package org.firstinspires.ftc.teamcode.robots.UGBot.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;

public interface VisionProvider {
    public void initializeVision(HardwareMap hardwareMap, Viewpoint viewpoint);

    public void shutdownVision();

    public StackHeight detect();

    public void reset();
}

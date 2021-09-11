package org.firstinspires.ftc.teamcode.robots.goodBot.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.Viewpoint;

public interface VisionProvider {
    public void initializeVision(HardwareMap hardwareMap, Viewpoint viewpoint, boolean enableDashboard);

    public void shutdownVision();

    public StackHeight detect();

    public void reset();
}

package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.TelemetryProvider;
import org.opencv.core.Mat;

public interface VisionProvider extends TelemetryProvider {
    void initializeVision(HardwareMap hardwareMap);

    void shutdownVision();

    Position getPosition();

    void reset();

    boolean canSendDashboardImage();

    Mat getDashboardImage();

    void update();
}

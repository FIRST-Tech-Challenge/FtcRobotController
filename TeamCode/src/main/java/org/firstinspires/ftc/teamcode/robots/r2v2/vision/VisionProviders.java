package org.firstinspires.ftc.teamcode.robots.r2v2.vision;

import org.firstinspires.ftc.teamcode.robots.r2v2.vision.provider.AprilTagProvider;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.provider.R2V2ConeDetectorProvider;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.provider.OpenCVProvider;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.provider.dummy.LeftDummyProvider;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.provider.dummy.MiddleDummyProvider;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.provider.dummy.RightDummyProvider;

public class VisionProviders {
    public static final Class<? extends VisionProvider>[] VISION_PROVIDERS =
            new Class[]{AprilTagProvider.class, R2V2ConeDetectorProvider.class , OpenCVProvider.class, LeftDummyProvider.class, MiddleDummyProvider.class, RightDummyProvider.class,};


    public static final int DEFAULT_PROVIDER_INDEX = 0;
}

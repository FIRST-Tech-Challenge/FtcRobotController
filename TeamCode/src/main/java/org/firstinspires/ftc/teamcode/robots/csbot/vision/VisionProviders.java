package org.firstinspires.ftc.teamcode.robots.csbot.vision;

import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.AprilTagProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.DPRGCanDetectorProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.OpenCVProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.dummy.LeftDummyProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.dummy.MiddleDummyProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.dummy.RightDummyProvider;

public class VisionProviders {
    public static final Class<? extends VisionProvider>[] VISION_PROVIDERS =
            new Class[]{AprilTagProvider.class, DPRGCanDetectorProvider.class , OpenCVProvider.class, LeftDummyProvider.class, MiddleDummyProvider.class, RightDummyProvider.class,};


    public static final int DEFAULT_PROVIDER_INDEX = 0;
}

package org.firstinspires.ftc.teamcode.robots.gruntbuggly.vision;

import org.firstinspires.ftc.teamcode.robots.gruntbuggly.vision.provider.OpenCVProvider;
import org.firstinspires.ftc.teamcode.robots.gruntbuggly.vision.provider.dummy.LeftDummyProvider;
import org.firstinspires.ftc.teamcode.robots.gruntbuggly.vision.provider.dummy.MiddleDummyProvider;
import org.firstinspires.ftc.teamcode.robots.gruntbuggly.vision.provider.dummy.RightDummyProvider;

public class VisionProviders {
    public static final Class<? extends VisionProvider>[] VISION_PROVIDERS =
            new Class[]{OpenCVProvider.class, LeftDummyProvider.class, MiddleDummyProvider.class, RightDummyProvider.class,};


    public static final int DEFAULT_PROVIDER_INDEX = 0;
}

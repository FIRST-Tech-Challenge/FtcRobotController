package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers.OpenCVProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers.dummy.LeftDummyProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers.dummy.MiddleDummyProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers.dummy.RightDummyProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers.TensorflowProvider;

public class VisionProviders {
    public static final Class<? extends VisionProvider>[] VISION_PROVIDERS =
            new Class[]{OpenCVProvider.class, TensorflowProvider.class, LeftDummyProvider.class, MiddleDummyProvider.class, RightDummyProvider.class,};


    public static final int DEFAULT_PROVIDER_INDEX = 0;
    public static final Class<? extends VisionProvider> DEFAULT_PROVIDER = OpenCVProvider.class;
}

package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.dogecv.DogeCVIntegration;
import org.firstinspires.ftc.teamcode.vision.dummy.LeftDummyVisionIntegration;
import org.firstinspires.ftc.teamcode.vision.dummy.MiddleDummyVisionIntegration;
import org.firstinspires.ftc.teamcode.vision.dummy.RightDummyVisionIntegration;

public final class VisionProvidersSkystone {
    private VisionProvidersSkystone() { throw new RuntimeException("Utility Class"); }

    public static final Class<? extends SkystoneVisionProvider>[] visionProviders =
            new Class[]{SkystoneOpenCVIntegration.class, SkystoneGripIntegration.class, SkystoneGripPipeline.class, MiddleDummyVisionIntegration.class,
                    LeftDummyVisionIntegration.class, RightDummyVisionIntegration.class};


    public static final Class<? extends SkystoneVisionProvider> defaultProvider = MiddleDummyVisionIntegration.class;

}

package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.dogecv.DogeCVIntegration;
import org.firstinspires.ftc.teamcode.vision.dummy.LeftDummyVisionIntegration;
import org.firstinspires.ftc.teamcode.vision.dummy.MiddleDummyVisionIntegration;
import org.firstinspires.ftc.teamcode.vision.dummy.RightDummyVisionIntegration;

public final class VisionProvidersRoverRuckus {
    private VisionProvidersRoverRuckus() { throw new RuntimeException("Utility Class"); }

    public static final Class<? extends VisionProvider>[] visionProviders =
            new Class[]{DogeCVIntegration.class, MiddleDummyVisionIntegration.class,
                    LeftDummyVisionIntegration.class, RightDummyVisionIntegration.class};


    public static final Class<? extends VisionProvider> defaultProvider = MiddleDummyVisionIntegration.class;

}

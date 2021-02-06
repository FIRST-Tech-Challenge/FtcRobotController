package org.firstinspires.ftc.teamcode.robots.UGBot.vision;

import org.firstinspires.ftc.teamcode.robots.UGBot.vision.dummy.FourDummyVisionIntegration;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.dummy.OneDummyVisionIntegration;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.dummy.ZeroDummyVisionIntegration;

public class VisionProviders {
    private VisionProviders() { throw new RuntimeException("Utility Class"); }

    public static final Class<? extends VisionProvider>[] visionProviders =
            new Class[]{OpenCVIntegration.class, TensorflowIntegration.class, ZeroDummyVisionIntegration.class, OneDummyVisionIntegration.class, FourDummyVisionIntegration.class,};


    public static final Class<? extends VisionProvider> defaultProvider = OneDummyVisionIntegration.class;
}

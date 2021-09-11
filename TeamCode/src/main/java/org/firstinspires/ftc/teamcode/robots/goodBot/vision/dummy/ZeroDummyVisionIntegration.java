package org.firstinspires.ftc.teamcode.robots.goodBot.vision.dummy;

import org.firstinspires.ftc.teamcode.robots.goodBot.vision.StackHeight;

public class ZeroDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public StackHeight detect() {
        return StackHeight.ZERO;
    }

}

package org.firstinspires.ftc.teamcode.robots.reach.vision.dummy;

import org.firstinspires.ftc.teamcode.robots.reach.vision.StackHeight;

public class ZeroDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public StackHeight detect() {
        return StackHeight.ZERO;
    }

}

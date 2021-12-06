package org.firstinspires.ftc.teamcode.robots.reach.vision.dummy;

import org.firstinspires.ftc.teamcode.robots.reach.vision.StackHeight;

public class FourDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public StackHeight detect() {
        return StackHeight.FOUR;
    }

}

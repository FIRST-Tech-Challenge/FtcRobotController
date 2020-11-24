package org.firstinspires.ftc.teamcode.robots.UGBot.vision.dummy;

import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;

public class ZeroDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public StackHeight detect() {
        return StackHeight.ZERO;
    }

}

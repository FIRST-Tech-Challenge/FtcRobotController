package org.firstinspires.ftc.teamcode.robots.UGBot.vision.dummy;

import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;

public class FourDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public StackHeight detect() {
        return StackHeight.FOUR;
    }

}

package org.firstinspires.ftc.teamcode.robots.conceptTrikeBot.vision.dummy;

import org.firstinspires.ftc.teamcode.robots.conceptTrikeBot.vision.StackHeight;

public class FourDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public StackHeight detect() {
        return StackHeight.FOUR;
    }

}

package org.firstinspires.ftc.teamcode.vision.dummy;

import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.SkystoneTargetInfo;
import org.firstinspires.ftc.teamcode.vision.StonePos;

public class MiddleDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public GoldPos detect() {
        return GoldPos.MIDDLE;
    }

    @Override
    public SkystoneTargetInfo detectSkystone() {
        return new SkystoneTargetInfo() {{ setQuarryPosition(StonePos.MIDDLE);}};
    }

}

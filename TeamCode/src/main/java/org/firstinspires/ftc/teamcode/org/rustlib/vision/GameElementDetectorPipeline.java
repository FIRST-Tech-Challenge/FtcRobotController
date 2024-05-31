package org.firstinspires.ftc.teamcode.org.rustlib.vision;

import org.firstinspires.ftc.teamcode.org.rustlib.core.RobotBase.GameElementLocation;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class GameElementDetectorPipeline extends OpenCvPipeline {
    public abstract GameElementLocation getGameElementLocation();
}

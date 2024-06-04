package org.rustlib.vision;

import org.openftc.easyopencv.OpenCvPipeline;
import org.rustlib.core.RobotBase.GameElementLocation;

public abstract class GameElementDetectorPipeline extends OpenCvPipeline {
    public abstract GameElementLocation getGameElementLocation();
}

package org.firstinspires.ftc.teamcode.CVRec;

import org.openftc.easyopencv.OpenCvPipeline;

public abstract class CVPipelineBase extends OpenCvPipeline {
    protected volatile RingStackSize stackSize = RingStackSize.Undefined;
    protected volatile int meanVal;

    public RingStackSize getStackSize() {
        return stackSize;
    }

    public int getMeanVal() {
        return meanVal;
    }
}

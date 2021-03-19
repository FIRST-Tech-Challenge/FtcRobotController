package org.firstinspires.ftc.teamcode.CVRec;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class CVPipelineBase extends OpenCvPipeline {
    protected volatile RingStackSize stackSize = RingStackSize.Undefined;
    private List<CVRoi> targets;
    protected volatile int meanVal;

    protected static final int QUAD_MAX = 108;
    protected static final int SINGLE_MAX = 118;

    private int resolutionX;
    private int resolutionY;

    protected CVPipelineBase(int resX, int resY){
        setResolutionX(resX);
        setResolutionY(resY);
        targets = Collections.synchronizedList(new ArrayList<CVRoi>());
    }

    public RingStackSize getStackSize() {
        return stackSize;
    }

    public int getMeanVal() {
        return meanVal;
    }

    public int getResolutionX() {
        return resolutionX;
    }

    public void setResolutionX(int resolutionX) {
        this.resolutionX = resolutionX;
    }

    public int getResolutionY() {
        return resolutionY;
    }

    public void setResolutionY(int resolutionY) {
        this.resolutionY = resolutionY;
    }

    public List<CVRoi> getTargets() {
        return targets;
    }
}

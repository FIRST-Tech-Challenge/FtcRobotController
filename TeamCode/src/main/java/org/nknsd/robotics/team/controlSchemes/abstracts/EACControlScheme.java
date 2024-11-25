package org.nknsd.robotics.team.controlSchemes.abstracts;

import org.nknsd.robotics.framework.NKNControlScheme;

import java.util.concurrent.Callable;

public abstract class EACControlScheme extends NKNControlScheme {
    public boolean controlEAC = true;
    public abstract Callable swapEACcontrol();
    public abstract Callable sampleGrab();
    public abstract Callable sampleNeutral();
    public abstract Callable sampleRelease();
    public abstract Callable sampleUp();
    public abstract Callable sampleDown();
    public abstract Callable sampleExtend();
    public abstract Callable sampleRetract();
}

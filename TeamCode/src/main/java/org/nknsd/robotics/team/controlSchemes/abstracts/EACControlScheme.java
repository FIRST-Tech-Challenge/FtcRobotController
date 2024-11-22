package org.nknsd.robotics.team.controlSchemes.abstracts;

import org.nknsd.robotics.framework.NKNControlScheme;

import java.util.concurrent.Callable;

public abstract class EACControlScheme extends NKNControlScheme {
    public abstract Callable servoGrab();
    public abstract Callable servoNeutral();
    public abstract Callable servoRelease();
    public abstract Callable armUp();
    public abstract Callable armDown();
    public abstract Callable armExtend();
    public abstract Callable armRetract();
}

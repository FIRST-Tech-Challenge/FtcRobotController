package org.nknsd.robotics.team.controlSchemes.abstracts;

import org.nknsd.robotics.framework.NKNControlScheme;
import org.nknsd.robotics.team.components.ExtensionHandler;

import java.util.concurrent.Callable;

public abstract class EACControlScheme extends NKNControlScheme {
    public boolean controlEAC = true;
    public ExtensionHandler extensionHandler;

    public abstract Callable<Boolean> swapEACcontrol();

    public abstract Callable<Boolean> sampleGrab();

    public abstract Callable<Boolean> sampleNeutral();

    public abstract Callable<Boolean> sampleRelease();

    public abstract Callable<Boolean> sampleUp();

    public abstract Callable<Boolean> sampleDown();

    public abstract Callable<Boolean> sampleExtend();

    public abstract Callable<Boolean> sampleRetract();

    public void linkExtensionHandler(ExtensionHandler extensionHandler) {
        this.extensionHandler = extensionHandler;
    }
}

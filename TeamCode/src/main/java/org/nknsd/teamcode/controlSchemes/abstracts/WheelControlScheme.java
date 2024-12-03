package org.nknsd.teamcode.controlSchemes.abstracts;

import org.nknsd.teamcode.frameworks.NKNControlScheme;
import org.nknsd.teamcode.components.utility.GamePadHandler;

import java.util.concurrent.Callable;

public abstract class WheelControlScheme extends NKNControlScheme {
    public abstract Callable<Boolean> gearUp();
    public abstract Callable<Boolean> gearDown();
    public abstract Callable<Boolean> resetAngle();
    public Callable<Boolean> initDisableAutoFix() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return GamePadHandler.GamepadButtons.Y.detect(gamePadHandler.getGamePad1());
            }
        };
    }
}
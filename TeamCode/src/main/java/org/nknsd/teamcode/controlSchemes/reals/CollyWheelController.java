package org.nknsd.teamcode.controlSchemes.reals;

import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.controlSchemes.abstracts.WheelControlScheme;

import java.util.concurrent.Callable;

public class CollyWheelController extends WheelControlScheme {
    private boolean delaySpeedChangesUp = false;
    private boolean delaySpeedChangesDown = false;

    @Override
    public Callable<Boolean> gearUp() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamePadHandler.getGamePad1());

                if (!delaySpeedChangesUp && button) {
                    delaySpeedChangesUp = true;
                    return true;
                } else if (!button) {
                    delaySpeedChangesUp = false;
                }

                return false;
            }
        };
    }

    @Override
    public Callable<Boolean> gearDown() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamePadHandler.getGamePad1());

                if (!delaySpeedChangesDown && button) {
                    delaySpeedChangesDown = true;
                    return true;
                } else if (!button) {
                    delaySpeedChangesDown = false;
                }

                return false;
            }
        };
    }

    @Override
    public Callable<Boolean> resetAngle() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return GamePadHandler.GamepadButtons.BACK.detect(gamePadHandler.getGamePad1());
            }
        };
    }
}

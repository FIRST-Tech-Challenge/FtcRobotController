package org.nknsd.robotics.team.controlSchemes.reals;

import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.controlSchemes.abstracts.WheelControlScheme;

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

package org.nknsd.robotics.team.controlSchemes.reals;

import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.controlSchemes.abstracts.WheelControlScheme;

import java.util.concurrent.Callable;

public class CollyWheelController extends WheelControlScheme {

    @Override
    public Callable<Boolean> gearUp() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamepad1);
            }
        };
    }

    @Override
    public Callable<Boolean> gearDown() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamepad1);
            }
        };
    }

    @Override
    public Callable<Boolean> resetAngle() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return GamePadHandler.GamepadButtons.BACK.detect(gamepad1);
            }
        };
    }
}

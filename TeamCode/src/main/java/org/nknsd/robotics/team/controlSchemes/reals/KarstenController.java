package org.nknsd.robotics.team.controlSchemes.reals;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.controlSchemes.abstracts.EACControlScheme;

import java.util.concurrent.Callable;


public class KarstenController extends EACControlScheme {
    @Override
    public Callable servoGrab() {
        return new Callable() {@Override public Object call() throws Exception {return GamePadHandler.GamepadButtons.A.detect(gamepad2);}};
    }

    @Override
    public Callable servoNeutral() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return !(GamePadHandler.GamepadButtons.A.detect(gamepad2) && (GamePadHandler.GamepadButtons.B.detect(gamepad2)));
            }
        };
    }

    @Override
    public Callable servoRelease() {
        return new Callable() {@Override public Object call() throws Exception {return GamePadHandler.GamepadButtons.B.detect(gamepad2);}};
    }

    @Override
    public Callable armUp() {
        return new Callable() {@Override public Object call() throws Exception {return GamePadHandler.GamepadButtons.DPAD_UP.detect(gamepad2);}};
    }

    @Override
    public Callable armDown() {
        return new Callable() {@Override public Object call() throws Exception {return GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamepad2);}};
    }

    @Override
    public Callable armExtend() {
        return new Callable() {@Override public Object call() throws Exception {return GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamepad2);}};
    }

    @Override
    public Callable armRetract() {
        return new Callable() {@Override public Object call() throws Exception {return GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamepad2);}};
    }
}

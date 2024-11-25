package org.nknsd.robotics.team.controlSchemes.reals;

import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.controlSchemes.abstracts.EACControlScheme;

import java.util.concurrent.Callable;


public class KarstenEACController extends EACControlScheme {
    boolean controlEACDelay = false;
    @Override
    public Callable swapEACcontrol() {
        return new Callable() {

            @Override
            public Object call() throws Exception {
                if (GamePadHandler.GamepadButtons.BACK.detect(gamepad2)){
                    if(controlEACDelay = false){
                        controlEAC = !controlEAC;
                    }
                    controlEACDelay = true;
                } else {
                    controlEACDelay = false;
                }
                return controlEAC;
            }
        };
    }

    @Override
    public Callable sampleGrab() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return GamePadHandler.GamepadButtons.A.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable sampleNeutral() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return !(GamePadHandler.GamepadButtons.A.detect(gamepad2) && (GamePadHandler.GamepadButtons.B.detect(gamepad2)) && controlEAC);
            }
        };
    }

    @Override
    public Callable sampleRelease() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return GamePadHandler.GamepadButtons.B.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable sampleUp() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return GamePadHandler.GamepadButtons.DPAD_UP.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable sampleDown() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable sampleExtend() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable sampleRetract() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamepad2) && controlEAC;
            }
        };
    }
}

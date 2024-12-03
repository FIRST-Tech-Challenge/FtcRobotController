package org.nknsd.robotics.team.controlSchemes.reals;

import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.controlSchemes.abstracts.EACControlScheme;

import java.util.concurrent.Callable;


public class KarstenEACController extends EACControlScheme {
    boolean controlEACDelay = false;
    @Override
    public Callable<Boolean> swapEACcontrol() {
        return new Callable<Boolean>() {

            @Override
            public Boolean call() {
                if (GamePadHandler.GamepadButtons.BACK.detect(gamepad2)){
                    if(!controlEACDelay){
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
    public Callable<Boolean> sampleGrab() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return GamePadHandler.GamepadButtons.A.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> sampleNeutral() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return !(GamePadHandler.GamepadButtons.A.detect(gamepad2) && (GamePadHandler.GamepadButtons.B.detect(gamepad2)) && controlEAC);
            }
        };
    }

    @Override
    public Callable<Boolean> sampleRelease() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return GamePadHandler.GamepadButtons.B.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> sampleUp() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return GamePadHandler.GamepadButtons.DPAD_UP.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> sampleDown() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> sampleExtend() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamepad2) && controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> sampleRetract() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamepad2) && controlEAC;
            }
        };
    }
}

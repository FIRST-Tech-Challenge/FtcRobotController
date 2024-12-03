package org.nknsd.robotics.team.controlSchemes.reals;

import org.nknsd.robotics.team.components.ExtensionHandler;
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
                if (GamePadHandler.GamepadButtons.BACK.detect(gamePadHandler.getGamePad2())){
                    if(!controlEACDelay){
                        controlEAC = !controlEAC;
                        controlEACDelay = true;
                        return true;
                    }
                } else {
                    controlEACDelay = false;
                }
                return false;
            }
        };
    }

    @Override
    public Callable<Boolean> sampleGrab() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                try {
                    if (extensionHandler.targetPosition() == ExtensionHandler.ExtensionPositions.COLLECT && !sampleRelease().call()) {
                        return true;
                    }
                } catch (Exception ignore) {}

                return GamePadHandler.GamepadButtons.A.detect(gamePadHandler.getGamePad2()) && controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> sampleNeutral() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return !(GamePadHandler.GamepadButtons.A.detect(gamePadHandler.getGamePad2()) || (GamePadHandler.GamepadButtons.B.detect(gamePadHandler.getGamePad2())) && controlEAC);
            }
        };
    }

    @Override
    public Callable<Boolean> sampleRelease() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return GamePadHandler.GamepadButtons.B.detect(gamePadHandler.getGamePad2()) && controlEAC;
            }
        };
    }

    private boolean delayRotChangeUp = false;
    @Override
    public Callable<Boolean> sampleUp() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                boolean button = GamePadHandler.GamepadButtons.DPAD_UP.detect(gamePadHandler.getGamePad2());

                if (!delayRotChangeUp && button) {
                    delayRotChangeUp = true;
                    return controlEAC;
                } else if (!button) {
                    delayRotChangeUp = false;
                }

                return false;
            }
        };
    }

    private boolean delayRotChangeDown = false;
    @Override
    public Callable<Boolean> sampleDown() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                boolean button = GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamePadHandler.getGamePad2());

                if (!delayRotChangeDown && button) {
                    delayRotChangeDown = true;
                    return controlEAC;
                } else if (!button) {
                    delayRotChangeDown = false;
                }

                return false;
            }
        };
    }

    private boolean delayExtChangeOut = false;
    @Override
    public Callable<Boolean> sampleExtend() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                boolean button = GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamePadHandler.getGamePad2());

                if (!delayExtChangeOut && button) {
                    delayExtChangeOut = true;
                    return controlEAC;
                } else if (!button) {
                    delayExtChangeOut = false;
                }

                return false;
            }
        };
    }

    private boolean delayExtChangeIn = false;
    @Override
    public Callable<Boolean> sampleRetract() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                boolean button = GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamePadHandler.getGamePad2());

                if (!delayExtChangeIn && button) {
                    delayExtChangeIn = true;
                    return controlEAC;
                } else if (!button) {
                    delayExtChangeIn = false;
                }

                return false;
            }
        };
    }
}

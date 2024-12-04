package org.nknsd.teamcode.controlSchemes.reals;

import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.controlSchemes.abstracts.SpecimenControlScheme;

import java.util.concurrent.Callable;

public class XandreSpecimenController extends SpecimenControlScheme {
    @Override
    public Callable<Boolean> specimenGrab() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return (GamePadHandler.GamepadButtons.A.detect(gamePadHandler.getGamePad2()) && !eacControlScheme.controlEAC);
            }
        };
    }

    @Override
    public Callable<Boolean> specimenRelease() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return (GamePadHandler.GamepadButtons.B.detect(gamePadHandler.getGamePad2()) && !eacControlScheme.controlEAC);
            }
        };
    }

    private boolean delayRotChangeFor = false;
    @Override
    public Callable<Boolean> specimenForward() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamePadHandler.getGamePad2());

                if (!delayRotChangeFor && button) {
                    delayRotChangeFor = true;
                    return !eacControlScheme.controlEAC;
                } else if (!button) {
                    delayRotChangeFor = false;
                }

                return false;
            }
        };
    }

    private boolean delayRotChangeBack = false;
    @Override
    public Callable<Boolean> specimenBackwards() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.DPAD_UP.detect(gamePadHandler.getGamePad2());

                if (!delayRotChangeBack && button) {
                    delayRotChangeBack = true;
                    return !eacControlScheme.controlEAC;
                } else if (!button) {
                    delayRotChangeBack = false;
                }

                return false;
            }
        };
    }

    private boolean delayRaise = false;
    @Override
    public Callable<Boolean> specimenRaise() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.DPAD_RIGHT.detect(gamePadHandler.getGamePad2());

                if (!delayRaise && button) {
                    delayRaise = true;
                    return !eacControlScheme.controlEAC;
                } else if (!button) {
                    delayRaise = false;
                }

                return false;
            }
        };
    }

    private boolean delayLower = false;
    @Override
    public Callable<Boolean> specimenLower() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.DPAD_LEFT.detect(gamePadHandler.getGamePad2());

                if (!delayLower && button) {
                    delayLower = true;
                    return !eacControlScheme.controlEAC;
                } else if (!button) {
                    delayLower = false;
                }

                return false;
            }
        };
    }
}

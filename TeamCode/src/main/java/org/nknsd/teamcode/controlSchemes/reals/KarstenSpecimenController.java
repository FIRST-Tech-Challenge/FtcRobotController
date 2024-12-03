package org.nknsd.teamcode.controlSchemes.reals;

import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.controlSchemes.abstracts.SpecimenControlScheme;

import java.util.concurrent.Callable;

public class KarstenSpecimenController extends SpecimenControlScheme {
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

    @Override
    public Callable<Boolean> specimenForward() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return (GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> specimenBackwards() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return (GamePadHandler.GamepadButtons.DPAD_UP.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> specimenRaise() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return (GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }

    @Override
    public Callable<Boolean> specimenLower() {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return (GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }
}

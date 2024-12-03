package org.nknsd.robotics.team.controlSchemes.reals;

import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.controlSchemes.abstracts.SpecimenControlScheme;

import java.util.concurrent.Callable;

public class KarstenSpecimenController extends SpecimenControlScheme {
    @Override
    public Callable specimenGrab() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return (GamePadHandler.GamepadButtons.A.detect(gamePadHandler.getGamePad2()) && !eacControlScheme.controlEAC);
            }
        };
    }

    @Override
    public Callable specimenRelease() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return (GamePadHandler.GamepadButtons.B.detect(gamePadHandler.getGamePad2()) && !eacControlScheme.controlEAC);
            }
        };
    }

    @Override
    public Callable specimenForward() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return (GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }

    @Override
    public Callable specimenBackwards() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return (GamePadHandler.GamepadButtons.DPAD_UP.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }

    @Override
    public Callable specimenRaise() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return (GamePadHandler.GamepadButtons.RIGHT_BUMPER.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }

    @Override
    public Callable specimenLower() {
        return new Callable() {
            @Override
            public Object call() throws Exception {
                return (GamePadHandler.GamepadButtons.LEFT_BUMPER.detect(gamePadHandler.getGamePad2())) && !eacControlScheme.controlEAC;
            }
        };
    }
}

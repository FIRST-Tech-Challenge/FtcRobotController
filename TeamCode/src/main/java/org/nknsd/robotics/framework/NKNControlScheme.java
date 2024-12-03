package org.nknsd.robotics.framework;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.nknsd.robotics.team.components.GamePadHandler;

public abstract class NKNControlScheme {
    public GamePadHandler gamePadHandler;

    public void link(GamePadHandler gamePadHandler) {
        this.gamePadHandler = gamePadHandler;
    }
}

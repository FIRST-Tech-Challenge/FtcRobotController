package org.nknsd.robotics.framework;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.nknsd.robotics.team.components.GamePadHandler;

public abstract class NKNControlScheme {
    GamePadHandler gamePadHandler;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    void link(GamePadHandler gamePadHandler, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamePadHandler = gamePadHandler;
        this.gamepad2 = gamepad2;
        this.gamepad1 = gamepad1;
    }
}

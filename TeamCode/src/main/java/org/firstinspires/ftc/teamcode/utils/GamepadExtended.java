package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class GamepadExtended {
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public ButtonPriority priority = new ButtonPriority();

    public GamepadExtended(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public abstract void main();
}

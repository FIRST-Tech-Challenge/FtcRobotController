package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadExEx extends GamepadEx {
    Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 600)
            .build();

    public GamepadExEx(Gamepad gp) {
        super(gp);
    }

    public void rumble() {
        gamepad.runRumbleEffect(rumbleEffect);
    }
}

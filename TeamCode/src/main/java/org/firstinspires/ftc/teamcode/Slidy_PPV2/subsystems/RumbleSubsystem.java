package org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleSubsystem extends SubsystemBase {
    private Gamepad gamepad;
    private Gamepad.RumbleEffect rumbleEffect;

    public RumbleSubsystem(Gamepad gamepad) {
        this.gamepad = gamepad;
        rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 800)
                .build();
    }

    public void rumble() {
        gamepad.runRumbleEffect(rumbleEffect);
    }
}

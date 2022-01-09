package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;

/**
 * intake, not an extension of ControllerToggleableTool
 */
public class ControllerIntake extends AutoIntake {
    private final GamepadEx toolGamepad;
    private final ToggleButtonReader reader;

    public ControllerIntake(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(map);
        this.reader = new ToggleButtonReader(toolGamepad, GamepadKeys.Button.X);
        this.toolGamepad = toolGamepad;
    }

    public void update() {
        reader.readValue();
        if (noObject()) {
            if (reader.getState()) {
                if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
                    this.forward();
                } else {
                    this.backward();
                }
            } else {
                this.stop();
            }
        } else if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
            this.forward();
        } else {
            this.stop();
        }
    }
}

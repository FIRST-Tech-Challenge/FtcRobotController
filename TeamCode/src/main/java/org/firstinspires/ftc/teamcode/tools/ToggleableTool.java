package org.firstinspires.ftc.teamcode.tools;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

public abstract class ToggleableTool<T extends DcMotorSimple> {
    private final T motor;
    private final ToggleButtonReader reader;
    public void update() {
        reader.readValue();
        if (reader.getState()) {
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }
    }
    public ToggleableTool(@NonNull HardwareMap map, GamepadEx toolGamepad, Class<T> tClass, String name, GamepadKeys.Button button) {
        this.motor = map.get(tClass, name);
        this.reader = new ToggleButtonReader(toolGamepad, button);
    }
}

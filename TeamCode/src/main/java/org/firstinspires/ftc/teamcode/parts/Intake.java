package org.firstinspires.ftc.teamcode.parts;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.thread.event.thread.EventThread;

import androidx.annotation.NonNull;

/**
 * intake, extension of ToggleableTool
 */
public class Intake extends ToggleableTool<DcMotor> {
    public Intake(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(eventThread, map, toolGamepad, DcMotor.class, "intake", GamepadKeys.Button.X, -1);
    }
}

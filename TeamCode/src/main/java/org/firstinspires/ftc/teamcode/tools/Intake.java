package org.firstinspires.ftc.teamcode.tools;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import androidx.annotation.NonNull;

public class Intake extends ToggleableTool<DcMotor> {
    public Intake(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(map, toolGamepad, DcMotor.class, "intake", GamepadKeys.Button.X);
    }
}

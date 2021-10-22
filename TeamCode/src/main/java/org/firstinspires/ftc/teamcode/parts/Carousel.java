package org.firstinspires.ftc.teamcode.parts;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import androidx.annotation.NonNull;
/**
 * carousel spinner, extension of ToggleableTool
 */
public class Carousel extends ToggleableTool<CRServo> {
    public Carousel(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(map, toolGamepad, CRServo.class, "spinner", GamepadKeys.Button.B, -1);
    }
}

package org.firstinspires.ftc.teamcode.core.robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.core.thread.thread.EventThread;
import androidx.annotation.NonNull;
/**
 * carousel spinner, extension of ToggleableTool
 */
public class Carousel extends ToggleableTool<CRServo> {
    public Carousel(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(eventThread, map, toolGamepad, CRServo.class, "spinner", GamepadKeys.Button.B, -1);
    }
}

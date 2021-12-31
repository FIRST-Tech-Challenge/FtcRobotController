package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import androidx.annotation.NonNull;
/**
 * carousel spinner, extension of ControllerToggleableTool
 */
public class ControllerCarousel extends ControllerToggleableTool<CRServo> {
    public ControllerCarousel(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(eventThread, map, toolGamepad, CRServo.class, "spinner", GamepadKeys.Button.B, -1);
    }
}

package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import androidx.annotation.NonNull;
/**
 * carousel spinner, extension of ControllerToggleableTool
 */
public class ControllerCarousel extends ControllerToggleableTool<DcMotor> {
    public ControllerCarousel(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad, double power) {
        super(eventThread, map, toolGamepad, DcMotor.class, "spinner", GamepadKeys.Button.B, power * AutoCarousel.CarouselSpeedConfig.speed);
    }
}

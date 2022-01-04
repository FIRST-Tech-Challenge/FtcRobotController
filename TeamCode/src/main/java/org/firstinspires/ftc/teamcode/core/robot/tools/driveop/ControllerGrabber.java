package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoGrabber;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedIndefinitelyEvent;

import androidx.annotation.NonNull;

public class ControllerGrabber extends AutoGrabber {
    public ControllerGrabber(@NonNull EventThread eventThread, @NonNull HardwareMap hardwareMap, GamepadEx gamepadEx) {
        super(hardwareMap);
        eventThread.addEvent(new RunWhenOutputChangedIndefinitelyEvent(this::toggle, () -> gamepadEx.getButton(GamepadKeys.Button.B)));
    }
}

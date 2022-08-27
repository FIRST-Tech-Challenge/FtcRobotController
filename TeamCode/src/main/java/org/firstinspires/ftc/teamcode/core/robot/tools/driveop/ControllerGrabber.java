package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoGrabber;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

import androidx.annotation.NonNull;

@Deprecated
public class ControllerGrabber extends AutoGrabber {
    final Thread thread;
    public ControllerGrabber(@NonNull EventThread eventThread, @NonNull HardwareMap hardwareMap, GamepadEx gamepadEx) {
        super(hardwareMap);
        this.thread = new Thread(() -> {
            final ToggleButtonReader reader = new ToggleButtonReader(gamepadEx, GamepadKeys.Button.DPAD_UP);
            while (!eventThread.isInterrupted()) {
                reader.readValue();
                if (reader.wasJustReleased()) toggle();
            }
        });
        thread.setPriority(5);
    }

    public void init() {
        thread.start();
    }
}

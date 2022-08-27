package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.HeadlessToggleableTool;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

/**
 * simple button push toggleable tool
 */
public abstract class ControllerToggleableTool<T extends DcMotorSimple> extends HeadlessToggleableTool<T> {
    protected final ButtonReader reader;

    /**
     * @param eventThread local instance of eventThread
     * @param map         pass this through, this will be handled by user opmode. hardwaremap instance.
     * @param toolGamepad same as above, instance of GamepadEx from FtcLib
     * @param tClass      Either DcMotor or CRServo, any extension of DcMotorSimple
     * @param name        Hardware map name of tool motor/CRServo
     * @param button      button to be pushed for toggle, uses GamepadKeys.Button
     * @param power       power motor should be set to upon toggle
     */
    public ControllerToggleableTool(@NonNull EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad, Class<T> tClass, String name, GamepadKeys.Button button, double power) {
        super(map, tClass, name, power);
        this.reader = new ButtonReader(toolGamepad, button);
        makeEvent(eventThread, reader);
    }

    private void run() {
        reader.readValue();
        if (reader.wasJustReleased()) {
            if (currentState) {
                this.off();
            } else {
                this.on();
            }
        }
    }

    protected void makeEvent(@NonNull EventThread eventThread, ButtonReader reader) {
        final Thread thread = new Thread(() -> {
            while (!eventThread.isInterrupted()) {
                run();
            }
        });
        thread.setPriority(Thread.NORM_PRIORITY);
        thread.start();
    }
}

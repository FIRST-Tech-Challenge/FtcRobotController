package org.firstinspires.ftc.teamcode.core.robot;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.api.RunListenerIndefinitelyEvent;

/**
 * simple button push toggleable tool
 */
public abstract class ToggleableTool<T extends DcMotorSimple>{
    protected final T motor;
    protected final double power;
    protected final ButtonReader reader;
    protected boolean toggle = false;

    /**
     * @param eventThread local instance of eventThread
     * @param map         pass this through, this will be handled by user opmode. hardwaremap instance.
     * @param toolGamepad same as above, instance of GamepadEx from FtcLib
     * @param tClass      Either DcMotor or CRServo, any extension of DcMotorSimple
     * @param name        Hardware map name of tool motor/CRServo
     * @param button      button to be pushed for toggle, uses GamepadKeys.Button
     * @param power       power motor should be set to upon toggle
     */
    public ToggleableTool(@NonNull EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad, Class<T> tClass, String name, GamepadKeys.Button button, double power) {
        this.motor = map.get(tClass, name);
        this.reader = new ButtonReader(toolGamepad, button);
        this.power = power;
        eventThread.addEvent(new RunListenerIndefinitelyEvent(this::run) {
            @Override
            public boolean shouldRun() {
                reader.readValue();
                return reader.wasJustReleased();
            }
        });
    }

    protected void run() {
        toggle = !toggle;
        motor.setPower(toggle ? power : 0);
    }

    protected void makeEvent(@NonNull EventThread eventThread, ButtonReader reader) {
        eventThread.addEvent(new RunListenerIndefinitelyEvent(this::run) {
            @Override
            public boolean shouldRun() {
                reader.readValue();
                return reader.wasJustReleased();
            }
        });
    }
}

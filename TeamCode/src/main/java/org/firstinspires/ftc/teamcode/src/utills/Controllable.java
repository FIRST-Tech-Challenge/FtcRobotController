package org.firstinspires.ftc.teamcode.src.utills;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Controllable<T> {

    /**
     * Allows control of subsystems through the gamepads
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     */
    @Nullable
    T gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) throws InterruptedException;


    /**
     * Stops the controllable object from moving.
     */
    void halt();
}

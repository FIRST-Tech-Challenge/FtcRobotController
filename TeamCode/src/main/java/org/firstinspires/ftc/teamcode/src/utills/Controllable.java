package org.firstinspires.ftc.teamcode.src.utills;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Controllable {

    /**
     * Allows control of subsystems through the gamepads
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     */
    Object gamepadControl(Gamepad gamepad1, Gamepad gamepad2);
}

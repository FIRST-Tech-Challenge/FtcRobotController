package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;

/** Shared control mappings; reads the current gamepad state on every call. */
public final class GamepadConfig {
    public static final double DRIVE_DEADBAND = 0.05;
    // Turn input multiplier: 0.0 disables turning, 1.0 uses full turn power.
    public static final double TURN_LIMIT = 0.5;
    private final Gamepad gamepad;

    public GamepadConfig(@NotNull final Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public double getForward() {
        return -gamepad.left_stick_y;
    }

    public double getStrafe() {
        return gamepad.left_stick_x;
    }

    public double getTurn() {
        return gamepad.right_stick_x;
    }

    public boolean isIntakeHeld() {
        return gamepad.left_bumper;
    }

    public boolean isShootHeld() {
        return gamepad.right_bumper;
    }
}

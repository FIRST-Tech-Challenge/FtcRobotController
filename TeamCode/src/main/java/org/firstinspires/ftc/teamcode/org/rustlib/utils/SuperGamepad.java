package org.firstinspires.ftc.teamcode.org.rustlib.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Trigger;

import java.util.function.DoubleSupplier;

public class SuperGamepad {
    private double triggerDeadZone = 0.05;
    private double joystickDeadZone = 0.05;
    public Trigger a;
    public Trigger b;
    public Trigger x;
    public Trigger y;
    public final Trigger leftBumper;
    public final Trigger rightBumper;
    public final Trigger leftTrigger;
    public final Trigger rightTrigger;
    public final Trigger dpadLeft;
    public final Trigger dpadRight;
    public final Trigger dpadUp;
    public final Trigger dpadDown;
    public final Trigger circle;
    public final Trigger cross;
    public final Trigger gamepadActive;
    public final Trigger options;
    public final DoubleSupplier leftStickX;
    public final DoubleSupplier leftStickY;
    public final DoubleSupplier rightStickX;
    public final DoubleSupplier rightStickY;

    public SuperGamepad(Gamepad gamepad) {
        a = new Trigger(() -> gamepad.a);
        b = new Trigger(() -> gamepad.b);
        x = new Trigger(() -> gamepad.x);
        y = new Trigger(() -> gamepad.y);
        leftBumper = new Trigger(() -> gamepad.left_bumper);
        rightBumper = new Trigger(() -> gamepad.right_bumper);
        leftTrigger = new Trigger(() -> gamepad.left_trigger > triggerDeadZone);
        rightTrigger = new Trigger(() -> gamepad.right_trigger > triggerDeadZone);
        dpadLeft = new Trigger(() -> gamepad.dpad_left);
        dpadRight = new Trigger(() -> gamepad.dpad_right);
        dpadUp = new Trigger(() -> gamepad.dpad_up);
        dpadDown = new Trigger(() -> gamepad.dpad_down);
        circle = new Trigger(() -> gamepad.circle);
        cross = new Trigger(() -> gamepad.cross);
        options = new Trigger(() -> gamepad.options);
        gamepadActive = new Trigger(() -> !gamepad.atRest());
        leftStickX = () -> Math.abs(gamepad.left_stick_x) > joystickDeadZone ? gamepad.left_stick_x : 0;
        leftStickY = () -> Math.abs(gamepad.left_stick_y) > joystickDeadZone ? gamepad.left_stick_y : 0;
        rightStickX = () -> Math.abs(gamepad.right_stick_x) > joystickDeadZone ? gamepad.right_stick_x : 0;
        rightStickY = () -> Math.abs(gamepad.right_stick_y) > joystickDeadZone ? gamepad.right_stick_y : 0;
    }

    public SuperGamepad setTriggerDeadZone(double triggerDeadZone) {
        this.triggerDeadZone = triggerDeadZone;
        return this;
    }

    public SuperGamepad setJoystickDeadZone(double joystickDeadZone) {
        this.joystickDeadZone = joystickDeadZone;
        return this;
    }
}

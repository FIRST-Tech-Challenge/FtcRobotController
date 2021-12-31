package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A controller-less tool with two states, on and off.
 */
public abstract class HeadlessToggleableTool<T extends DcMotorSimple> {
    protected final T motor;
    protected final double power;
    protected boolean currentState = false;

    /**
     * Creates a new toggleable tool with a motor and on power.
     * @param motor The motor that will be used for the tool.
     * @param power The power that the tool shall run at when on.
     */
    public HeadlessToggleableTool(T motor, double power) {
        this.motor = motor;
        this.power = power;
    }

    /**
     * Creates a new toggleable tool with a motor specified by hardware map, class, and name, and the on power.
     * @param hardwareMap The hardware map to get the motor that will be used for the tool from.
     * @param tClass The class of the motor that will be used for the tool.
     * @param name The name of the motor that will be used for the tool.
     * @param power The power that the tool shall run at when on.
     */
    public HeadlessToggleableTool(@NonNull HardwareMap hardwareMap, Class<T> tClass, String name, double power) {
        this(hardwareMap.get(tClass, name), power);
    }

    public void on() {
        this.motor.setPower(power);
        currentState = true;
    }

    public void off() {
        this.motor.setPower(0);
        currentState = false;
    }

    public boolean isOn() {
        return currentState;
    }
}

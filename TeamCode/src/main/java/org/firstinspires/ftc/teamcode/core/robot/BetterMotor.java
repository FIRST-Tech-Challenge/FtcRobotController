package org.firstinspires.ftc.teamcode.core.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedIndefinitelyEvent;

import androidx.annotation.NonNull;

public class BetterMotor {
    private double power;
    private DcMotor motor;
    private EventThread eventThread;

    public BetterMotor(EventThread eventThread, @NonNull HardwareMap hardwareMap, String name, double power) {
        this.eventThread = eventThread;
        this.motor = hardwareMap.get(DcMotor.class, name);
        this.power = power;
    }
    public void init() {
        motor.setPower(power);
        eventThread.addEvent(new RunWhenOutputChangedIndefinitelyEvent(() -> motor.setPower(power), this::getPower));
    }
    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }
}

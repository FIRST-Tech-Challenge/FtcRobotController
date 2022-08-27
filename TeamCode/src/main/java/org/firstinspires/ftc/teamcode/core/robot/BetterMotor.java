package org.firstinspires.ftc.teamcode.core.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.old.types.impl.RunWhenOutputChangedIndefinitelyEvent;
import org.firstinspires.ftc.teamcode.core.thread.old.types.impl.TimedEvent;

import androidx.annotation.NonNull;

public class BetterMotor {
    private double power;
    public DcMotorEx motor;
    private EventThread eventThread;

    public BetterMotor(EventThread eventThread, @NonNull HardwareMap hardwareMap, String name, double power) {
        this.eventThread = eventThread;
        this.motor = hardwareMap.get(DcMotorEx.class, name);
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

    public void runForMs(long milliseconds, double power, double returnPower) {
        setPower(power);
        eventThread.addEvent(new TimedEvent(() -> setPower(returnPower), milliseconds));
    }

    public void runForMs(long milliseconds, double power) {
        runForMs(milliseconds, power, getPower());
    }
}

package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.TelemetryManager;

public class ActionBuilder {
    // Builder
    public ActionBuilder() { list = new ArrayList<>(); }
    // Return a list to be used by the state machine
    public ArrayList<Action> getList() { return list; }

    public ActionBuilder resetMotorEncoder(DcMotor motor) {
        ActionFunction function = () -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return true;
        };
        String name =
                "stop and reset " + motor.getDeviceName();
        return add(name, function);
    }
    public ActionBuilder setMotorPosition(DcMotor motor, int targetPosition, double power) {
        ActionFunction function = () -> {
            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
            return true;
        };
        String name =
                "set motor " + motor.getDeviceName() + " to target pos " + targetPosition + " with power" + power;
        return add(name, function);
    }
    public ActionBuilder startMotor(DcMotor motor, double power) {
        ActionFunction function = () -> {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(power);
            return true;
        };
        String name =
                "start motor " + motor.getDeviceName() + " with power" + power;
        return add(name, function);
    }

    public ActionBuilder servoRunToPosition(Servo servo, double targetPosition) {
        ActionFunction function = () -> {
            servo.setPosition(targetPosition);
            return true;
        };
        String name =
                "set servo " + servo.getDeviceName() + " to target pos " + targetPosition;
        return add(name, function);
    }

    public ActionBuilder waitForMotorAbovePosition(DcMotor motor, int expectedPosition) {
        ActionFunction function = () -> {
            return motor.getCurrentPosition() >= expectedPosition*0.95;
        };
        String name = "test if motor " + motor.getDeviceName() +
                " is at or above target pos " + expectedPosition;
        return add(name, function);
    }

    public ActionBuilder waitForMotorBelowPosition(DcMotor motor, int expectedPosition) {
        ActionFunction function = () -> {
            return motor.getCurrentPosition() <= expectedPosition*0.95;
        };
        String name = "test if motor " + motor.getDeviceName() +
                " is at or below target pos " + expectedPosition;
        return add(name, function);
    }

    public ActionBuilder resetTimer(ElapsedTime timer) {
        ActionFunction function = () -> { timer.reset(); return true;};
        String name = "reset timer " + timer.toString();
        return add(name, function);
    }

    public ActionBuilder waitUntil(ElapsedTime timer, double targetTime) {
        ActionFunction function = () -> timer.milliseconds() >= targetTime;
        String name = "wait for timer " + timer.toString() + " to exceed " + targetTime;
        return add(name, function);
    }
    public ActionBuilder addLine(String text) {
        ActionFunction function = () -> {TelemetryManager.getTelemetry().addLine(text); TelemetryManager.getTelemetry().update(); return true;};
        String name = "";
        return add(name, function);
    }


    private ActionBuilder add(String name, ActionFunction function) {
        list.add(new Action(name, function));
        return this;
    }

    private ArrayList<Action> list;
}

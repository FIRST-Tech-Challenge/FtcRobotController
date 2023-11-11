package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.Motor; // Adjust this according to your package structure
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ActionBuilder {
    // Builder
    ActionBuilder() { list = new ArrayList<Action>(); }
    // Return a list to be used by the state machine
    ArrayList<Action> getList() { return list; }

    ActionBuilder setMotorPosition(Motor motor, int targetPosition) {
        ActionFunction function = () -> {
            motor.setTargetPosition(targetPosition);
            motor.setPower(1.0);
            return true;
        };
        String name =
                "set motor " + motor.getName() + " to target pos " + targetPosition;
        return add(name, function);
    }

    ActionBuilder waitForMotorAbovePosition(Motor motor, int expectedPosition) {
        ActionFunction function = () -> {
            return motor.getCurrentPosition() >= expectedPosition;
        };
        String name = "test if motor " + motor.getName() +
                " is at or above target pos " + expectedPosition;
        return add(name, function);
    }

    ActionBuilder resetTimer(Timer timer) {
        ActionFunction function = () -> { timer.reset(); return true;};
        String name = "reset timer " + timer.getName();
        return add(name, function);
    }

    ActionBuilder waitUntil(Timer timer, int elapsedTime) {
        ActionFunction function = () -> { return timer.getElapsedTime() >= elapsedTime; };
        String name = "wait for timer " + timer.getName() + " to exceed " + elapsedTime;
        return add(name, function);
    }

    private ActionBuilder add(String name, ActionFunction function) {
        list.add(new Action(name, function));
        return this;
    }

    private ArrayList<Action> list;
}

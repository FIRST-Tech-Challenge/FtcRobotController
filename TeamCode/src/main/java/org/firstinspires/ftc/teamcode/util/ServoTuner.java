package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

import java.util.ArrayList;

/**
 * Created by djfig on 1/7/2018.
 */

@TeleOp(name = "Servo Tuner")
public class ServoTuner extends OpMode {

    GamepadActions gamepadActions;
    ArrayList<Servo> servos;
    int currentIndex = 0;

    Servo activeServo;

    @Override
    public void init() {
        gamepadActions = new GamepadActions();
        servos = new ArrayList<>();
        for (HardwareDevice device : hardwareMap.getAll(Servo.class)) {
            Servo servo = (Servo) device;
            servos.add(servo);
        }
        if (servos.size() > 0) {
            activeServo = servos.get(currentIndex);
        }
    }

    @Override
    public void loop() {
        gamepadActions.update(gamepad1,gamepad2);

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_left)) {
            currentIndex -= 1;
            if (currentIndex < 0) {
                currentIndex = servos.size() - 1;
            }
            activeServo = servos.get(currentIndex);
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_right)) {
            currentIndex += 1;
            if (currentIndex > servos.size() - 1) {
                currentIndex = 0;
            }
            activeServo = servos.get(currentIndex);
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_up)) {
            activeServo.setPosition(activeServo.getPosition() + 0.05);
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_down)) {
            activeServo.setPosition(activeServo.getPosition() - 0.05);
        }

        if (activeServo != null) {
            double current_position = activeServo.getPosition();
            double new_position = activeServo.getPosition() + gamepad1.right_stick_y / 100;
            if (current_position != new_position) {
                activeServo.setPosition(new_position);
            }

        }

        if (activeServo != null) {
            telemetry.addData("Servo", activeServo.getDeviceName());
            telemetry.addData("Port", activeServo.getPortNumber());
            telemetry.addData("Position", activeServo.getPosition());
        }
    }
}
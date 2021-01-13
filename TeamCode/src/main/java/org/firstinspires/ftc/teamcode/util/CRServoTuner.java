package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;

import java.util.ArrayList;

/**
 * Created by djfig on 1/7/2018.
 */

@TeleOp(name = "CRServo Tuner")
public class CRServoTuner extends OpMode {

    GamepadActions gamepadActions;
    ArrayList<CRServo> servos;
    int currentIndex = 0;

    CRServo activeServo;

    @Override
    public void init() {
        gamepadActions = new GamepadActions();
        servos = new ArrayList<>();
        for (HardwareDevice device : hardwareMap.getAll(CRServo.class)) {
            CRServo servo = (CRServo) device;
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

        if (activeServo != null) {
            activeServo.setPower(gamepad1.right_stick_y);
        }

        if (activeServo != null) {
            telemetry.addData("Motor", activeServo.getDeviceName());
            telemetry.addData("Port", activeServo.getPortNumber());
        }
    }
}
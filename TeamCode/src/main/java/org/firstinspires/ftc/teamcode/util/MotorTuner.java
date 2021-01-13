package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;

import java.util.ArrayList;

/**
 * Created by djfig on 1/7/2018.
 */

@TeleOp(name = "Motor Tuner")
public class MotorTuner extends OpMode {

    GamepadActions gamepadActions;
    ArrayList<DcMotor> motors;
    int currentIndex = 0;

    DcMotor activeMotor;

    @Override
    public void init() {
        gamepadActions = new GamepadActions();
        motors = new ArrayList<>();
        for (HardwareDevice device : hardwareMap.getAll(DcMotor.class)) {
            DcMotor motor = (DcMotor) device;
            motors.add(motor);
        }
        if (motors.size() > 0) {
            activeMotor = motors.get(currentIndex);
        }
    }

    @Override
    public void loop() {
        gamepadActions.update(gamepad1,gamepad2);

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_left)) {
            currentIndex -= 1;
            if (currentIndex < 0) {
                currentIndex = motors.size() - 1;
            }
            activeMotor = motors.get(currentIndex);
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_right)) {
            currentIndex += 1;
            if (currentIndex > motors.size() - 1) {
                currentIndex = 0;
            }
            activeMotor = motors.get(currentIndex);
        }

        if (activeMotor != null) {
            activeMotor.setPower(gamepad1.right_stick_y);
        }

        if (activeMotor != null) {
            telemetry.addData("Motor", activeMotor.getDeviceName());
            telemetry.addData("Port", activeMotor.getPortNumber());
            telemetry.addData("Position", activeMotor.getCurrentPosition());
        }
    }
}
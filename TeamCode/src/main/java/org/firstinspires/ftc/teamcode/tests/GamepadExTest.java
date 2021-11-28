package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.gamepad.GamepadEx;

@TeleOp(name = "GamepadEx Test", group = "Tests")
public class GamepadExTest extends OpMode {
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        gamepadEx = new GamepadEx(gamepad1);
        telemetry.addData("A pressed:", false);
        telemetry.addData("B toggled:", false);
        telemetry.addData("Right trigger toggled:", false);
        telemetry.update();
    }

    @Override
    public void start() {
        // Add an a button
        gamepadEx.add("a", gamepadEx.new AStandardButton() {
            @Override
            public void run(boolean value) {
                telemetry.addData("A pressed:", value);
            }
        });
        // Add a b toggle button
        gamepadEx.add("b", gamepadEx.new BToggleButton() {
            @Override
            public void onToggle(boolean value) {
                telemetry.addData("B toggled:", value);
            }
        });
        // Add a right toggle trigger
        gamepadEx.add("rightTrigger", gamepadEx.new RightTriggerToggleButton() {
            @Override
            public void onToggle(boolean value) {
                telemetry.addData("Right trigger toggled:", value);
            }
        });
    }

    @Override
    public void loop() {
        gamepadEx.update();
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GamepadynImpl.TestActions.MOVEMENT;
import static org.firstinspires.ftc.teamcode.GamepadynImpl.TestActions.ROTATION;
import static org.firstinspires.ftc.teamcode.GamepadynImpl.TestActions.CLAW;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

import computer.living.gamepadyn.ActionDescriptor;
import computer.living.gamepadyn.Gamepadyn;
import computer.living.gamepadyn.Tak;
import computer.living.gamepadyn.ftc.InputSystemFtc;

@TeleOp
public class GamepadynImpl extends OpMode {

    enum TestActions {
        MOVEMENT,
        ROTATION,
        CLAW
    }

    Gamepadyn<TestActions> gamepadyn;

    @Override
    public void init() {
        // in Java 9, you can do this more easily.
        gamepadyn = new Gamepadyn<>(new InputSystemFtc(this),
            Tak.makeActionMap(Arrays.asList(
                Tak.a(MOVEMENT, 2),
                Tak.a(ROTATION, 1),
                Tak.d(CLAW)
            ))
        );
    }

    @Override
    public void loop() {
        gamepadyn.update();
    }
}

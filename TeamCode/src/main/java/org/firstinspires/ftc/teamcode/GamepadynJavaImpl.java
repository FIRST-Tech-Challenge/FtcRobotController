package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GamepadynJavaImpl.TestAction.DEBUG_ACTION;
import static org.firstinspires.ftc.teamcode.GamepadynJavaImpl.TestAction.MOVEMENT;
import static org.firstinspires.ftc.teamcode.GamepadynJavaImpl.TestAction.ROTATION;
import static org.firstinspires.ftc.teamcode.GamepadynJavaImpl.TestAction.CLAW;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;
import java.util.Objects;

import computer.living.gamepadyn.Gamepadyn;
import computer.living.gamepadyn.Player;
import computer.living.gamepadyn.Tak;
import computer.living.gamepadyn.ftc.InputSystemFtc;

@TeleOp
public class GamepadynJavaImpl extends OpMode {

    enum TestAction {
        MOVEMENT,
        ROTATION,
        CLAW,
        DEBUG_ACTION
    }

    Gamepadyn<TestAction> gamepadyn;

    @Override
    public void init() {
        // in Java 9, you can do this more easily.
        gamepadyn = new Gamepadyn<>(new InputSystemFtc(this),
            Tak.makeActionMap(Arrays.asList(
                Tak.a(MOVEMENT, 2),
                Tak.a(ROTATION, 1),
                Tak.d(CLAW),
                Tak.d(DEBUG_ACTION)
            ))
        );
    }

    @Override
    public void start() {

        // There's a bit of boilerplate here because of how Java treats nullability.
        // Gamepadyn was designed for Kotlin, but built to also work with Java.
        // It's much easier in Kotlin.

        // Get a reference to the player (FTC Player 1)
        Player<TestAction> p0 = gamepadyn.getPlayer(0);
        assert p0 != null;

        // Get the event corresponding to DEBUG_ACTION and add a lambda function as a listener to it.
        Objects.requireNonNull(p0.getEventDigital(DEBUG_ACTION)).addListener(it -> {
            telemetry.addLine("Button " + ((it.digitalData) ? "pressed" : "released") + "!");
        });

    }

    @Override
    public void loop() {
        gamepadyn.update();
        telemetry.update();
    }
}

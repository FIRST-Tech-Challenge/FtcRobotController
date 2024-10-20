package org.firstinspires.ftc.teamcode.utils.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GameController extends Controller {
    private final Controller previousController = new Controller();

    public GameController(Gamepad gamepad) {
        super(gamepad);
    }

    public boolean pressed(Button button) {
        return button(button) && !previousController.button(button);
    }

    public boolean release(Button button) {
        return !button(button) && previousController.button(button);
    }

    @Override
    public void update(Gamepad gamepad) {
        previousController.update(super.currentGamepad);
        super.update(gamepad);
    }
}

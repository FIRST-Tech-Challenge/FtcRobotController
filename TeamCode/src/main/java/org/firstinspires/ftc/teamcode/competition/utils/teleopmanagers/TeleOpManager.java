package org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class TeleOpManager {

    private final Gamepad GAMEPAD1, GAMEPAD2;
    private final GamepadFunctions FUNCTION1, FUNCTION2;

    /**
     * Creates a new manager
     * @param gamepad1 First gamepad
     * @param function1 First gamepad's functions
     * @param gamepad2 Second gamepad
     * @param function2 Second gamepad's functions
     */
    public TeleOpManager(Gamepad gamepad1, GamepadFunctions function1, Gamepad gamepad2, GamepadFunctions function2) {
        GAMEPAD1 = gamepad1;
        GAMEPAD2 = gamepad2;
        FUNCTION1 = function1;
        FUNCTION2 = function2;
    }

    /**
     * The code the manager should run
     */
    public abstract void main();

    /**
     * The method to call when stopping the manager
     */
    public abstract void stop();

    public Gamepad getGamepad1() {
        return GAMEPAD1;
    }

    public Gamepad getGamepad2() {
        return GAMEPAD2;
    }

    public GamepadFunctions getGamepad1Functions() {
        return FUNCTION1;
    }

    public GamepadFunctions getGamepad2Functions() {
        return FUNCTION2;
    }

}

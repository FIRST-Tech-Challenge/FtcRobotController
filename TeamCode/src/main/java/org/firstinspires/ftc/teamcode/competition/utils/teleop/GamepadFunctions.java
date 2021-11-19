package org.firstinspires.ftc.teamcode.competition.utils.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadFunctions {

    private final Gamepad GAMEPAD;
    private boolean f1, f2, f3, f4, f5, f6;

    /**
     * Makes a new object containing the functions associated with a gamepad
     * @param gamepad The gamepad
     * @param f1 Whether the gamepad is allowed to execute function 1
     * @param f2 Whether the gamepad is allowed to execute function 2
     * @param f3 Whether the gamepad is allowed to execute function 3
     * @param f4 Whether the gamepad is allowed to execute function 4
     * @param f5 Whether the gamepad is allowed to execute function 5
     * @param f6 Whether the gamepad is allowed to execute function 6
     */
    public GamepadFunctions(Gamepad gamepad, boolean f1, boolean f2, boolean f3, boolean f4, boolean f5, boolean f6) {
        GAMEPAD = gamepad;
        this.f1 = f1;
        this.f2 = f2;
        this.f3 = f3;
        this.f4 = f4;
        this.f5 = f5;
        this.f6 = f6;
    }

    public Gamepad getGamepad() {
        return GAMEPAD;
    }

    public boolean hasF1() {
        return f1;
    }

    public void setF1(boolean f1) {
        this.f1 = f1;
    }

    public boolean hasF2() {
        return f2;
    }

    public void setF2(boolean f2) {
        this.f2 = f2;
    }

    public boolean hasF3() {
        return f3;
    }

    public void setF3(boolean f3) {
        this.f3 = f3;
    }

    public boolean hasF4() {
        return f4;
    }

    public void setF4(boolean f4) {
        this.f4 = f4;
    }

    public boolean hasF5() {
        return f5;
    }

    public void setF5(boolean f5) {
        this.f5 = f5;
    }

    public boolean hasF6() {
        return f6;
    }

    public void setF6(boolean f6) {
        this.f6 = f6;
    }
}
